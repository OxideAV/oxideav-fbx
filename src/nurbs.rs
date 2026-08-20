//! B-spline / NURBS evaluation + tessellation engine.
//!
//! FBX carries free-form geometry as `Geometry` elements whose §6
//! prop2 subtype is `"NurbsCurve"` / `"NurbsSurface"` /
//! `"TrimNurbsSurface"` / `"Boundary"` / `"Line"`
//! (`docs/3d/fbx/fbx-binary-properties70.md` §6 point 3). The staged
//! docs enumerate those subtype *names* only — the per-subtype wire
//! payload grammar (the record names + layouts for knot vectors,
//! control points, orders, forms and weights) is **not** staged, and
//! no staged fixture contains such a geometry, so the decode-side
//! join from `FbxDocument` records into this module is gated on that
//! grammar being staged (see [`crate::geometry_kind`], which surfaces
//! the subtype discriminator in the meantime).
//!
//! What this module provides *now* is the format-independent half:
//! a validated typed model for rational B-spline (NURBS) curves and
//! tensor-product surfaces, evaluation via the Cox–de Boor recursion,
//! first derivatives, and tessellators emitting
//! [`oxideav_mesh3d::Primitive`] values at a configurable resolution.
//! Everything here is textbook-standard numerical mathematics
//! implemented from the definition of the B-spline basis
//!
//! ```text
//! N_{i,0}(t) = 1 if U_i <= t < U_{i+1} else 0
//! N_{i,p}(t) = (t - U_i)/(U_{i+p} - U_i) · N_{i,p-1}(t)
//!            + (U_{i+p+1} - t)/(U_{i+p+1} - U_{i+1}) · N_{i+1,p-1}(t)
//! ```
//!
//! with the usual `0/0 := 0` convention for repeated knots, and the
//! rational (homogeneous-coordinate) extension
//! `C(t) = Σ N_{i,p}(t) w_i P_i / Σ N_{i,p}(t) w_i`.
//!
//! # Forms
//!
//! [`NurbsForm`] models the three standard CAD curve/surface forms:
//!
//! - **Open** — the parameter domain is `[U_p, U_n]` (clamped and
//!   unclamped knot vectors both accepted); the ends are unrelated.
//! - **Closed** — geometrically closed by authorship (coincident end
//!   points); evaluation is identical to `Open`, the variant carries
//!   the declared intent.
//! - **Periodic** — the control polygon wraps with `C^{p-1}`
//!   continuity: the effective control array is the authored one
//!   extended by its own first `degree` entries, and the knot vector
//!   (supplied for the *extended* array) must repeat with a constant
//!   period so that shifting the parameter by one period reproduces
//!   the same point.
//!
//! Whether these map 1:1 onto the FBX wire "form" vocabulary is part
//! of the unstaged payload grammar above; they are defined here in
//! their generic B-spline sense.
//!
//! # Totality
//!
//! Construction validates everything (finite values, knot ordering,
//! array lengths, strictly positive weights, non-degenerate domain),
//! so every constructed value evaluates totally: no panic, no NaN
//! from in-domain parameters. Out-of-domain parameters are clamped
//! (`Open` / `Closed`) or wrapped (`Periodic`). Tessellation
//! resolutions are capped ([`MAX_TESSELLATION_VERTICES`]) so a
//! hostile resolution cannot balloon memory.

use oxideav_mesh3d::{Error, Indices, Primitive, Result, Topology};

/// Hard cap on the number of vertices any single tessellation call
/// will emit. Guards against hostile / absurd resolutions turning
/// into a memory bomb. `(res_u + 1) * (res_v + 1)` for surfaces,
/// `res + 1` for curves.
pub const MAX_TESSELLATION_VERTICES: usize = 1 << 20;

/// Relative tolerance used when validating periodic knot vectors.
const PERIODIC_KNOT_TOL: f64 = 1e-9;

/// Curve / surface-direction form. See the module docs for the exact
/// semantics of each variant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NurbsForm {
    /// Ends unrelated; domain `[U_degree, U_n]`.
    Open,
    /// Geometrically closed by authorship; evaluation identical to
    /// [`NurbsForm::Open`].
    Closed,
    /// Control polygon wraps with `C^{p-1}` continuity; the parameter
    /// wraps modulo the domain period.
    Periodic,
}

impl NurbsForm {
    fn wraps(self) -> bool {
        matches!(self, Self::Periodic)
    }
}

/// Validated rational B-spline curve in 3-space.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
    degree: usize,
    control_points: Vec<[f64; 3]>,
    weights: Vec<f64>,
    knots: Vec<f64>,
    form: NurbsForm,
    /// Homogeneous control points `[x·w, y·w, z·w, w]`, periodic
    /// extension (first `degree` entries re-appended) already applied.
    hpoints: Vec<[f64; 4]>,
}

impl NurbsCurve {
    /// Build a validated curve.
    ///
    /// - `degree >= 1`, `control_points.len() >= degree + 1`.
    /// - `weights`: `None` = non-rational (all `1.0`); otherwise one
    ///   strictly positive finite weight per control point.
    /// - `knots`: non-decreasing finite values.
    ///   `Open` / `Closed`: length `n + degree + 1` where
    ///   `n = control_points.len()`.
    ///   `Periodic`: length `n + 2·degree + 1` — the knot vector of
    ///   the internally *extended* control array — and periodic
    ///   (`U[i+n] - U[i]` constant).
    /// - The domain must be non-degenerate.
    pub fn new(
        degree: usize,
        control_points: Vec<[f64; 3]>,
        weights: Option<Vec<f64>>,
        knots: Vec<f64>,
        form: NurbsForm,
    ) -> Result<Self> {
        let n = control_points.len();
        let weights = validate_common(degree, n, &control_points, weights)?;

        let ext = if form.wraps() { degree } else { 0 };
        let expected_knots = n + ext + degree + 1;
        if knots.len() != expected_knots {
            return Err(Error::invalid(format!(
                "nurbs curve: knot vector length {} != expected {expected_knots} \
                 (n = {n}, degree = {degree}, form = {form:?})",
                knots.len()
            )));
        }
        validate_knots(&knots, degree, n + ext)?;
        if form.wraps() {
            validate_periodic_knots(&knots, degree, n)?;
        }

        let mut hpoints: Vec<[f64; 4]> = control_points
            .iter()
            .zip(&weights)
            .map(|(p, &w)| [p[0] * w, p[1] * w, p[2] * w, w])
            .collect();
        for i in 0..ext {
            let h = hpoints[i];
            hpoints.push(h);
        }

        Ok(Self {
            degree,
            control_points,
            weights,
            knots,
            form,
            hpoints,
        })
    }

    /// Periodic curve with an internally synthesized uniform knot
    /// vector (`U_i = i`), the common wrap-with-`C^{p-1}`-continuity
    /// closed form. Domain `[degree, degree + n]`, period `n`.
    pub fn periodic_uniform(
        degree: usize,
        control_points: Vec<[f64; 3]>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self> {
        let n = control_points.len();
        let len = n
            .checked_add(2 * degree + 1)
            .ok_or_else(|| Error::invalid("nurbs curve: control-point count overflow"))?;
        let knots = (0..len).map(|i| i as f64).collect();
        Self::new(degree, control_points, weights, knots, NurbsForm::Periodic)
    }

    /// Polynomial degree.
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Authored control points (periodic extension not included).
    pub fn control_points(&self) -> &[[f64; 3]] {
        &self.control_points
    }

    /// One weight per authored control point (all `1.0` when the
    /// curve was built non-rational).
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// The knot vector exactly as supplied / synthesized.
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// Declared form.
    pub fn form(&self) -> NurbsForm {
        self.form
    }

    /// Valid parameter interval `(start, end)`.
    pub fn domain(&self) -> (f64, f64) {
        let n = self.hpoints.len();
        (self.knots[self.degree], self.knots[n])
    }

    fn param(&self, t: f64) -> f64 {
        let (lo, hi) = self.domain();
        if self.form.wraps() {
            wrap_param(t, lo, hi)
        } else {
            t.clamp(lo, hi)
        }
    }

    /// Point on the curve. Out-of-domain `t` clamps (`Open` /
    /// `Closed`) or wraps (`Periodic`).
    pub fn evaluate(&self, t: f64) -> [f64; 3] {
        let t = self.param(t);
        let p = self.degree;
        let span = find_span(&self.knots, p, self.hpoints.len(), t);
        let basis = basis_funcs(&self.knots, span, p, t);
        let mut acc = [0.0f64; 4];
        for (j, &b) in basis.iter().enumerate() {
            let h = self.hpoints[span - p + j];
            for (a, hv) in acc.iter_mut().zip(h) {
                *a += b * hv;
            }
        }
        dehomogenize(acc)
    }

    /// First derivative `dC/dt`. Same out-of-domain handling as
    /// [`evaluate`](Self::evaluate).
    pub fn derivative(&self, t: f64) -> [f64; 3] {
        let t = self.param(t);
        let p = self.degree;
        let span = find_span(&self.knots, p, self.hpoints.len(), t);
        let (basis, dbasis) = basis_funcs_and_derivs(&self.knots, span, p, t);
        let mut a = [0.0f64; 4];
        let mut da = [0.0f64; 4];
        for j in 0..=p {
            let h = self.hpoints[span - p + j];
            for k in 0..4 {
                a[k] += basis[j] * h[k];
                da[k] += dbasis[j] * h[k];
            }
        }
        rational_derivative(a, da)
    }
}

/// Validated rational tensor-product B-spline surface in 3-space.
///
/// The control grid is stored **u-fastest**: the point at grid
/// coordinate `(u_idx, v_idx)` lives at `control_points[v_idx * nu +
/// u_idx]`.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
    degree_u: usize,
    degree_v: usize,
    nu: usize,
    nv: usize,
    control_points: Vec<[f64; 3]>,
    weights: Vec<f64>,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    form_u: NurbsForm,
    form_v: NurbsForm,
    ext_nu: usize,
    ext_nv: usize,
    /// Homogeneous control grid, periodic extensions applied,
    /// u-fastest with row stride `ext_nu`.
    hpoints: Vec<[f64; 4]>,
}

impl NurbsSurface {
    /// Build a validated surface.
    ///
    /// - `nu × nv` control grid (u-fastest), `control_points.len() ==
    ///   nu * nv`, `nu >= degree_u + 1`, `nv >= degree_v + 1`,
    ///   degrees `>= 1`.
    /// - `weights`: `None` = non-rational; otherwise one strictly
    ///   positive finite weight per grid point (same layout).
    /// - Knot vectors as for [`NurbsCurve::new`], per direction.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        nu: usize,
        nv: usize,
        control_points: Vec<[f64; 3]>,
        weights: Option<Vec<f64>>,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        form_u: NurbsForm,
        form_v: NurbsForm,
    ) -> Result<Self> {
        let expected = nu.checked_mul(nv).ok_or_else(|| {
            Error::invalid("nurbs surface: control-grid dimension product overflows")
        })?;
        if control_points.len() != expected {
            return Err(Error::invalid(format!(
                "nurbs surface: {} control points != nu * nv = {nu} * {nv} = {expected}",
                control_points.len()
            )));
        }
        // Per-direction count checks ride on the shared validator by
        // treating each direction as a curve of that many points.
        let weights = validate_common(degree_u, nu, &control_points, weights)?;
        if degree_v < 1 {
            return Err(Error::invalid("nurbs surface: degree_v must be >= 1"));
        }
        if nv < degree_v + 1 {
            return Err(Error::invalid(format!(
                "nurbs surface: nv = {nv} control rows need at least degree_v + 1 = {}",
                degree_v + 1
            )));
        }

        let ext_u = if form_u.wraps() { degree_u } else { 0 };
        let ext_v = if form_v.wraps() { degree_v } else { 0 };
        for (name, knots, degree, count, ext, form) in [
            ("u", &knots_u, degree_u, nu, ext_u, form_u),
            ("v", &knots_v, degree_v, nv, ext_v, form_v),
        ] {
            let expected_knots = count + ext + degree + 1;
            if knots.len() != expected_knots {
                return Err(Error::invalid(format!(
                    "nurbs surface: {name}-knot vector length {} != expected {expected_knots} \
                     (count = {count}, degree = {degree}, form = {form:?})",
                    knots.len()
                )));
            }
            validate_knots(knots, degree, count + ext)?;
            if form.wraps() {
                validate_periodic_knots(knots, degree, count)?;
            }
        }

        let ext_nu = nu + ext_u;
        let ext_nv = nv + ext_v;
        let mut hpoints = vec![[0.0f64; 4]; ext_nu * ext_nv];
        for vv in 0..ext_nv {
            let sv = vv % nv;
            for uu in 0..ext_nu {
                let su = uu % nu;
                let p = control_points[sv * nu + su];
                let w = weights[sv * nu + su];
                hpoints[vv * ext_nu + uu] = [p[0] * w, p[1] * w, p[2] * w, w];
            }
        }

        Ok(Self {
            degree_u,
            degree_v,
            nu,
            nv,
            control_points,
            weights,
            knots_u,
            knots_v,
            form_u,
            form_v,
            ext_nu,
            ext_nv,
            hpoints,
        })
    }

    /// Degrees `(u, v)`.
    pub fn degrees(&self) -> (usize, usize) {
        (self.degree_u, self.degree_v)
    }

    /// Authored grid dimensions `(nu, nv)` (periodic extension not
    /// included).
    pub fn grid_size(&self) -> (usize, usize) {
        (self.nu, self.nv)
    }

    /// Authored control grid, u-fastest.
    pub fn control_points(&self) -> &[[f64; 3]] {
        &self.control_points
    }

    /// One weight per authored grid point.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Knot vectors `(u, v)` exactly as supplied.
    pub fn knots(&self) -> (&[f64], &[f64]) {
        (&self.knots_u, &self.knots_v)
    }

    /// Declared forms `(u, v)`.
    pub fn forms(&self) -> (NurbsForm, NurbsForm) {
        (self.form_u, self.form_v)
    }

    /// Valid parameter interval in `u`.
    pub fn domain_u(&self) -> (f64, f64) {
        (self.knots_u[self.degree_u], self.knots_u[self.ext_nu])
    }

    /// Valid parameter interval in `v`.
    pub fn domain_v(&self) -> (f64, f64) {
        (self.knots_v[self.degree_v], self.knots_v[self.ext_nv])
    }

    fn params(&self, u: f64, v: f64) -> (f64, f64) {
        let (ulo, uhi) = self.domain_u();
        let (vlo, vhi) = self.domain_v();
        let u = if self.form_u.wraps() {
            wrap_param(u, ulo, uhi)
        } else {
            u.clamp(ulo, uhi)
        };
        let v = if self.form_v.wraps() {
            wrap_param(v, vlo, vhi)
        } else {
            v.clamp(vlo, vhi)
        };
        (u, v)
    }

    /// Point on the surface. Out-of-domain parameters clamp (`Open` /
    /// `Closed`) or wrap (`Periodic`) per direction.
    pub fn evaluate(&self, u: f64, v: f64) -> [f64; 3] {
        let (s, _, _) = self.derivatives(u, v);
        s
    }

    /// `(S, ∂S/∂u, ∂S/∂v)` at `(u, v)`.
    pub fn derivatives(&self, u: f64, v: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let (u, v) = self.params(u, v);
        let pu = self.degree_u;
        let pv = self.degree_v;
        let span_u = find_span(&self.knots_u, pu, self.ext_nu, u);
        let span_v = find_span(&self.knots_v, pv, self.ext_nv, v);
        let (bu, dbu) = basis_funcs_and_derivs(&self.knots_u, span_u, pu, u);
        let (bv, dbv) = basis_funcs_and_derivs(&self.knots_v, span_v, pv, v);

        let mut a = [0.0f64; 4];
        let mut au = [0.0f64; 4];
        let mut av = [0.0f64; 4];
        for jv in 0..=pv {
            let row = (span_v - pv + jv) * self.ext_nu + span_u - pu;
            for ju in 0..=pu {
                let h = self.hpoints[row + ju];
                for k in 0..4 {
                    a[k] += bu[ju] * bv[jv] * h[k];
                    au[k] += dbu[ju] * bv[jv] * h[k];
                    av[k] += bu[ju] * dbv[jv] * h[k];
                }
            }
        }
        let s = dehomogenize(a);
        let su = rational_derivative(a, au);
        let sv = rational_derivative(a, av);
        (s, su, sv)
    }

    /// Unit surface normal `normalize(∂S/∂u × ∂S/∂v)` at `(u, v)`.
    ///
    /// At degenerate parameterization points (a collapsed pole row, a
    /// zero-length partial) the cross product vanishes; the parameters
    /// are then nudged a small step towards the domain interior and
    /// the normal re-derived there, which recovers the limit normal
    /// for the common collapsed-edge cases. If the normal is still
    /// degenerate after the nudge, `[0.0, 0.0, 1.0]` is returned.
    pub fn normal(&self, u: f64, v: f64) -> [f64; 3] {
        let (u, v) = self.params(u, v);
        if let Some(n) = self.normal_raw(u, v) {
            return n;
        }
        // Nudge towards the domain midpoint by a small fraction of
        // the domain span and retry.
        let (ulo, uhi) = self.domain_u();
        let (vlo, vhi) = self.domain_v();
        let nudge_u = (uhi - ulo) * 1e-4;
        let nudge_v = (vhi - vlo) * 1e-4;
        let un = if u - ulo < uhi - u {
            u + nudge_u
        } else {
            u - nudge_u
        };
        let vn = if v - vlo < vhi - v {
            v + nudge_v
        } else {
            v - nudge_v
        };
        for (cu, cv) in [(un, v), (u, vn), (un, vn)] {
            if let Some(n) = self.normal_raw(cu, cv) {
                return n;
            }
        }
        [0.0, 0.0, 1.0]
    }

    fn normal_raw(&self, u: f64, v: f64) -> Option<[f64; 3]> {
        let (_, su, sv) = self.derivatives(u, v);
        let n = cross(su, sv);
        normalize(n)
    }
}

/// Tessellation resolution knobs. `resolution_u` / `resolution_v` are
/// segment counts per direction (so a surface emits `(resolution_u +
/// 1) * (resolution_v + 1)` grid vertices); both default to 16.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TessellationOptions {
    pub resolution_u: u32,
    pub resolution_v: u32,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self {
            resolution_u: 16,
            resolution_v: 16,
        }
    }
}

impl TessellationOptions {
    /// Uniform resolution in both directions.
    pub fn uniform(resolution: u32) -> Self {
        Self {
            resolution_u: resolution,
            resolution_v: resolution,
        }
    }
}

impl NurbsCurve {
    /// Sample the curve into a line primitive.
    ///
    /// `resolution` is the segment count (`>= 1`, capped by
    /// [`MAX_TESSELLATION_VERTICES`]). `Open` / `Closed` curves emit
    /// `resolution + 1` vertices as a [`Topology::LineStrip`];
    /// `Periodic` curves emit `resolution` vertices as a
    /// [`Topology::LineLoop`] (the seam vertex is not duplicated).
    /// `uvs[0]` carries the normalized parameter as `[t, 0]`.
    pub fn tessellate(&self, resolution: u32) -> Result<Primitive> {
        if resolution == 0 {
            return Err(Error::invalid("nurbs curve: tessellation resolution 0"));
        }
        let res = resolution as usize;
        let count = if self.form.wraps() { res } else { res + 1 };
        if count > MAX_TESSELLATION_VERTICES {
            return Err(Error::invalid(format!(
                "nurbs curve: tessellation of {count} vertices exceeds the \
                 {MAX_TESSELLATION_VERTICES}-vertex cap"
            )));
        }
        let (lo, hi) = self.domain();
        let topology = if self.form.wraps() {
            Topology::LineLoop
        } else {
            Topology::LineStrip
        };
        let mut prim = Primitive::new(topology);
        let mut uv = Vec::with_capacity(count);
        for i in 0..count {
            let f = i as f64 / res as f64;
            let t = lo + (hi - lo) * f;
            let p = self.evaluate(t);
            prim.positions.push([p[0] as f32, p[1] as f32, p[2] as f32]);
            uv.push([f as f32, 0.0]);
        }
        prim.uvs.push(uv);
        Ok(prim)
    }
}

impl NurbsSurface {
    /// Tessellate the surface into an indexed triangle primitive.
    ///
    /// Samples a regular `(resolution_u + 1) × (resolution_v + 1)`
    /// parameter grid over the valid domain (periodic directions
    /// include the seam vertex twice so the UV layout stays a clean
    /// `0..=1` rectangle), emitting positions, analytic normals
    /// (`∂S/∂u × ∂S/∂v`, with the degenerate-pole nudge described on
    /// [`NurbsSurface::normal`]), normalized-parameter UVs and a
    /// `2 · resolution_u · resolution_v`-triangle index buffer wound
    /// counter-clockwise around that normal.
    pub fn tessellate(&self, options: &TessellationOptions) -> Result<Primitive> {
        let ru = options.resolution_u as usize;
        let rv = options.resolution_v as usize;
        if ru == 0 || rv == 0 {
            return Err(Error::invalid("nurbs surface: tessellation resolution 0"));
        }
        let count = (ru + 1)
            .checked_mul(rv + 1)
            .filter(|&c| c <= MAX_TESSELLATION_VERTICES)
            .ok_or_else(|| {
                Error::invalid(format!(
                    "nurbs surface: tessellation of ({} + 1) x ({} + 1) vertices exceeds \
                     the {MAX_TESSELLATION_VERTICES}-vertex cap",
                    options.resolution_u, options.resolution_v
                ))
            })?;
        let (ulo, uhi) = self.domain_u();
        let (vlo, vhi) = self.domain_v();

        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions.reserve(count);
        let mut normals = Vec::with_capacity(count);
        let mut uv = Vec::with_capacity(count);
        for j in 0..=rv {
            let fv = j as f64 / rv as f64;
            let v = vlo + (vhi - vlo) * fv;
            for i in 0..=ru {
                let fu = i as f64 / ru as f64;
                let u = ulo + (uhi - ulo) * fu;
                let p = self.evaluate(u, v);
                let n = self.normal(u, v);
                prim.positions.push([p[0] as f32, p[1] as f32, p[2] as f32]);
                normals.push([n[0] as f32, n[1] as f32, n[2] as f32]);
                uv.push([fu as f32, fv as f32]);
            }
        }
        prim.normals = Some(normals);
        prim.uvs.push(uv);

        let mut indices = Vec::with_capacity(ru * rv * 6);
        let stride = (ru + 1) as u32;
        for j in 0..rv as u32 {
            for i in 0..ru as u32 {
                let v00 = j * stride + i;
                let v10 = v00 + 1;
                let v01 = v00 + stride;
                let v11 = v01 + 1;
                indices.extend_from_slice(&[v00, v10, v11, v00, v11, v01]);
            }
        }
        prim.indices = Some(Indices::U32(indices));
        Ok(prim)
    }
}

// ---------------------------------------------------------------------
// Shared validation + basis machinery.
// ---------------------------------------------------------------------

/// Degree / count / point-finiteness / weight validation shared by
/// curves and surfaces. Returns the effective weight vector (all
/// `1.0` when `weights` is `None`).
fn validate_common(
    degree: usize,
    count: usize,
    points: &[[f64; 3]],
    weights: Option<Vec<f64>>,
) -> Result<Vec<f64>> {
    if degree < 1 {
        return Err(Error::invalid("nurbs: degree must be >= 1"));
    }
    if count < degree + 1 {
        return Err(Error::invalid(format!(
            "nurbs: {count} control points need at least degree + 1 = {}",
            degree + 1
        )));
    }
    if let Some(p) = points.iter().flatten().find(|c| !c.is_finite()) {
        return Err(Error::invalid(format!(
            "nurbs: non-finite control-point coordinate {p}"
        )));
    }
    match weights {
        None => Ok(vec![1.0; points.len()]),
        Some(w) => {
            if w.len() != points.len() {
                return Err(Error::invalid(format!(
                    "nurbs: {} weights for {} control points",
                    w.len(),
                    points.len()
                )));
            }
            if let Some(bad) = w.iter().find(|&&x| !(x.is_finite() && x > 0.0)) {
                return Err(Error::invalid(format!(
                    "nurbs: weight {bad} is not finite and strictly positive"
                )));
            }
            Ok(w)
        }
    }
}

/// Knot-vector validation: finite, non-decreasing, non-degenerate
/// domain `U[degree] < U[n]` (`n` = effective control-point count).
fn validate_knots(knots: &[f64], degree: usize, n: usize) -> Result<()> {
    if let Some(bad) = knots.iter().find(|k| !k.is_finite()) {
        return Err(Error::invalid(format!("nurbs: non-finite knot {bad}")));
    }
    if let Some(w) = knots.windows(2).find(|w| w[0] > w[1]) {
        return Err(Error::invalid(format!(
            "nurbs: decreasing knot pair {} > {}",
            w[0], w[1]
        )));
    }
    if knots[degree] >= knots[n] {
        return Err(Error::invalid(format!(
            "nurbs: degenerate parameter domain [{}, {}]",
            knots[degree], knots[n]
        )));
    }
    Ok(())
}

/// Periodicity validation for a `Periodic`-form knot vector: with
/// `n` authored control points the knots (length `n + 2·degree + 1`)
/// must satisfy `U[i + n] == U[i] + T` for every applicable `i`,
/// where `T = U[n + degree] - U[degree]` is the domain period.
fn validate_periodic_knots(knots: &[f64], degree: usize, n: usize) -> Result<()> {
    let period = knots[n + degree] - knots[degree];
    let scale = period.abs().max(1.0);
    for i in 0..knots.len() - n {
        let diff = knots[i + n] - knots[i] - period;
        if diff.abs() > PERIODIC_KNOT_TOL * scale {
            return Err(Error::invalid(format!(
                "nurbs: periodic form requires U[i + n] = U[i] + period; \
                 U[{}] - U[{i}] = {} but the period is {period}",
                i + n,
                knots[i + n] - knots[i]
            )));
        }
    }
    Ok(())
}

/// Wrap `t` into `[lo, hi)` by the domain period.
fn wrap_param(t: f64, lo: f64, hi: f64) -> f64 {
    let period = hi - lo;
    let mut f = (t - lo) % period;
    if f < 0.0 {
        f += period;
    }
    lo + f
}

/// Index `k` of the knot span containing `t`: `U[k] <= t < U[k+1]`
/// with `k` in `[degree, n - 1]` (`n` = effective control count).
/// `t` must already be clamped/wrapped into the domain; `t == U[n]`
/// resolves to the last non-empty span.
fn find_span(knots: &[f64], degree: usize, n: usize, t: f64) -> usize {
    let hi = knots[n];
    if t >= hi {
        let mut k = n - 1;
        while knots[k] >= hi {
            k -= 1;
        }
        return k;
    }
    let mut lo = degree;
    let mut up = n - 1;
    while lo < up {
        let mid = (lo + up).div_ceil(2);
        if knots[mid] <= t {
            lo = mid;
        } else {
            up = mid - 1;
        }
    }
    lo
}

/// The `degree + 1` non-zero basis values `N_{span-degree+j, degree}(t)`
/// for `j = 0..=degree`, via the standard triangular scheme derived
/// from the Cox–de Boor recursion (module docs), with the `0/0 := 0`
/// convention for repeated knots.
fn basis_funcs(knots: &[f64], span: usize, degree: usize, t: f64) -> Vec<f64> {
    let mut n = vec![0.0f64; degree + 1];
    let mut left = vec![0.0f64; degree + 1];
    let mut right = vec![0.0f64; degree + 1];
    n[0] = 1.0;
    for d in 1..=degree {
        left[d] = t - knots[span + 1 - d];
        right[d] = knots[span + d] - t;
        let mut saved = 0.0;
        for r in 0..d {
            let denom = right[r + 1] + left[d - r];
            let temp = if denom != 0.0 { n[r] / denom } else { 0.0 };
            n[r] = saved + right[r + 1] * temp;
            saved = left[d - r] * temp;
        }
        n[d] = saved;
    }
    n
}

/// Basis values plus their first derivatives, from the definitional
/// derivative identity
/// `N'_{i,p} = p · ( N_{i,p-1}/(U_{i+p} - U_i)
///               −  N_{i+1,p-1}/(U_{i+p+1} - U_{i+1}) )`
/// (each quotient `0` when its denominator vanishes).
fn basis_funcs_and_derivs(
    knots: &[f64],
    span: usize,
    degree: usize,
    t: f64,
) -> (Vec<f64>, Vec<f64>) {
    let values = basis_funcs(knots, span, degree, t);
    let mut derivs = vec![0.0f64; degree + 1];
    if degree >= 1 {
        // Non-zero degree-(p-1) basis at t: N_{span-p+1 .. span, p-1}.
        let lower = basis_funcs(knots, span, degree - 1, t);
        let p = degree as f64;
        for (j, d) in derivs.iter_mut().enumerate() {
            let i = span - degree + j;
            let mut val = 0.0;
            if j >= 1 {
                let denom = knots[i + degree] - knots[i];
                if denom != 0.0 {
                    val += lower[j - 1] / denom;
                }
            }
            if j < degree {
                let denom = knots[i + degree + 1] - knots[i + 1];
                if denom != 0.0 {
                    val -= lower[j] / denom;
                }
            }
            *d = p * val;
        }
    }
    (values, derivs)
}

/// `[xw, yw, zw, w] -> [x, y, z]`. `w > 0` is guaranteed by weight
/// validation (the basis is non-negative and sums to 1, so the
/// denominator is at least the smallest weight in the span).
fn dehomogenize(a: [f64; 4]) -> [f64; 3] {
    [a[0] / a[3], a[1] / a[3], a[2] / a[3]]
}

/// Rational first derivative from the homogeneous value `a` and its
/// parametric derivative `da` via the quotient rule:
/// `C' = (A' − w'·C) / w`.
fn rational_derivative(a: [f64; 4], da: [f64; 4]) -> [f64; 3] {
    let c = dehomogenize(a);
    [
        (da[0] - da[3] * c[0]) / a[3],
        (da[1] - da[3] * c[1]) / a[3],
        (da[2] - da[3] * c[2]) / a[3],
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `None` when the vector is too short to normalize meaningfully.
fn normalize(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        return None;
    }
    Some([v[0] / len, v[1] / len, v[2] / len])
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQRT2_2: f64 = std::f64::consts::FRAC_1_SQRT_2;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn approx3(a: [f64; 3], b: [f64; 3], tol: f64) -> bool {
        a.iter().zip(b).all(|(&x, y)| approx(x, y, tol))
    }

    /// Rational quadratic quarter circle in the xy unit circle, from
    /// (1,0,0) to (0,1,0): the standard conic-as-NURBS construction
    /// (middle control point at the tangent intersection with weight
    /// cos(θ/2) = √2/2 for a 90° arc).
    fn quarter_circle() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            Some(vec![1.0, SQRT2_2, 1.0]),
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            NurbsForm::Open,
        )
        .expect("valid quarter circle")
    }

    /// Full xy unit circle as four rational quadratic arcs — 9
    /// control points around the enclosing square, weights
    /// alternating 1 / √2/2, double interior knots.
    fn full_circle() -> NurbsCurve {
        let w = SQRT2_2;
        NurbsCurve::new(
            2,
            vec![
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [-1.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [-1.0, -1.0, 0.0],
                [0.0, -1.0, 0.0],
                [1.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
            ],
            Some(vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0]),
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
            NurbsForm::Closed,
        )
        .expect("valid full circle")
    }

    // ---- validation ------------------------------------------------

    #[test]
    fn rejects_degree_zero() {
        let e = NurbsCurve::new(
            0,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_too_few_control_points() {
        let e = NurbsCurve::new(
            3,
            vec![[0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            None,
            vec![0.0; 7],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_wrong_knot_count() {
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            None,
            vec![0.0, 0.0, 1.0, 1.0, 2.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_decreasing_knots() {
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            None,
            vec![0.0, 1.0, 0.5, 2.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_non_finite_knot() {
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            None,
            vec![0.0, 0.0, f64::NAN, 1.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_non_finite_control_point() {
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [f64::INFINITY, 0.0, 0.0]],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_degenerate_domain() {
        // All knots equal: U[degree] == U[n].
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            None,
            vec![1.0, 1.0, 1.0, 1.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_nonpositive_and_nonfinite_weights() {
        for w in [0.0, -1.0, f64::NAN] {
            let e = NurbsCurve::new(
                1,
                vec![[0.0; 3], [1.0, 0.0, 0.0]],
                Some(vec![1.0, w]),
                vec![0.0, 0.0, 1.0, 1.0],
                NurbsForm::Open,
            );
            assert!(e.is_err(), "weight {w} must be rejected");
        }
    }

    #[test]
    fn rejects_weight_count_mismatch() {
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0]],
            Some(vec![1.0]),
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    #[test]
    fn rejects_aperiodic_knots_on_periodic_form() {
        // Length is right (3 + 2·1 + 1 = 6) but the shifts are not
        // constant-period.
        let e = NurbsCurve::new(
            1,
            vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            None,
            vec![0.0, 1.0, 2.0, 3.0, 4.0, 9.0],
            NurbsForm::Periodic,
        );
        assert!(e.is_err());
    }

    #[test]
    fn surface_rejects_grid_mismatch() {
        let e = NurbsSurface::new(
            1,
            1,
            2,
            2,
            vec![[0.0; 3]; 3],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
            NurbsForm::Open,
        );
        assert!(e.is_err());
    }

    // ---- basis fundamentals ---------------------------------------

    #[test]
    fn basis_partition_of_unity() {
        // Arbitrary clamped knot vector with an interior double knot;
        // the non-zero basis values must sum to 1 everywhere in the
        // domain (definitional property of the B-spline basis).
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0];
        let degree = 3;
        let n = knots.len() - degree - 1; // 7 control points
        for i in 0..=300 {
            let t = 3.0 * i as f64 / 300.0;
            let span = find_span(&knots, degree, n, t);
            let b = basis_funcs(&knots, span, degree, t);
            let sum: f64 = b.iter().sum();
            assert!(approx(sum, 1.0, 1e-12), "sum {sum} at t = {t}");
            assert!(b.iter().all(|&x| x >= -1e-12), "negative basis at t = {t}");
        }
    }

    #[test]
    fn basis_derivatives_sum_to_zero() {
        // d/dt of a partition of unity is 0.
        let knots = [0.0, 0.0, 0.0, 1.0, 2.0, 4.0, 4.0, 4.0];
        let degree = 2;
        let n = knots.len() - degree - 1;
        for i in 1..40 {
            let t = 4.0 * i as f64 / 40.0;
            let span = find_span(&knots, degree, n, t);
            let (_, db) = basis_funcs_and_derivs(&knots, span, degree, t);
            let sum: f64 = db.iter().sum();
            assert!(approx(sum, 0.0, 1e-10), "derivative sum {sum} at t = {t}");
        }
    }

    #[test]
    fn degree_one_curve_is_the_polyline() {
        let pts = vec![[0.0, 0.0, 0.0], [1.0, 2.0, 0.0], [3.0, 1.0, -1.0]];
        let c = NurbsCurve::new(
            1,
            pts.clone(),
            None,
            vec![0.0, 0.0, 1.0, 2.0, 2.0],
            NurbsForm::Open,
        )
        .unwrap();
        assert!(approx3(c.evaluate(0.0), pts[0], 1e-15));
        assert!(approx3(c.evaluate(1.0), pts[1], 1e-15));
        assert!(approx3(c.evaluate(2.0), pts[2], 1e-15));
        assert!(approx3(c.evaluate(0.5), [0.5, 1.0, 0.0], 1e-15));
        assert!(approx3(c.evaluate(1.5), [2.0, 1.5, -0.5], 1e-15));
    }

    #[test]
    fn clamped_cubic_hits_endpoints_and_midpoint() {
        // Single cubic Bézier segment as a clamped B-spline; the
        // midpoint value is the definitional weighted sum with basis
        // (1/8, 3/8, 3/8, 1/8).
        let pts = [
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ];
        let c = NurbsCurve::new(
            3,
            pts.to_vec(),
            None,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            NurbsForm::Open,
        )
        .unwrap();
        assert!(approx3(c.evaluate(0.0), pts[0], 1e-15));
        assert!(approx3(c.evaluate(1.0), pts[3], 1e-15));
        let mid = c.evaluate(0.5);
        assert!(approx3(mid, [0.5, 0.75, 0.0], 1e-15), "mid = {mid:?}");
    }

    #[test]
    fn out_of_domain_parameters_clamp() {
        let c = quarter_circle();
        assert!(approx3(c.evaluate(-5.0), c.evaluate(0.0), 0.0));
        assert!(approx3(c.evaluate(42.0), c.evaluate(1.0), 0.0));
    }

    // ---- rational conics ------------------------------------------

    #[test]
    fn quarter_circle_lies_on_the_unit_circle() {
        let c = quarter_circle();
        for i in 0..=100 {
            let t = i as f64 / 100.0;
            let p = c.evaluate(t);
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!(approx(r, 1.0, 1e-12), "radius {r} at t = {t}");
            assert!(p[0] >= -1e-12 && p[1] >= -1e-12, "wrong quadrant at {t}");
        }
    }

    #[test]
    fn quarter_circle_tangent_is_perpendicular_to_radius() {
        let c = quarter_circle();
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let p = c.evaluate(t);
            let d = c.derivative(t);
            let dot = p[0] * d[0] + p[1] * d[1];
            assert!(approx(dot, 0.0, 1e-12), "radial·tangent {dot} at t = {t}");
            let speed = (d[0] * d[0] + d[1] * d[1]).sqrt();
            assert!(speed > 0.1, "vanishing tangent at t = {t}");
        }
    }

    #[test]
    fn full_circle_lies_on_the_unit_circle() {
        let c = full_circle();
        for i in 0..=400 {
            let t = 4.0 * i as f64 / 400.0;
            let p = c.evaluate(t);
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!(approx(r, 1.0, 1e-12), "radius {r} at t = {t}");
        }
        // Closure: same point at both domain ends.
        assert!(approx3(c.evaluate(0.0), c.evaluate(4.0), 1e-15));
    }

    // ---- periodic form --------------------------------------------

    #[test]
    fn periodic_uniform_wraps_and_is_smooth_at_the_seam() {
        // Square control polygon, cubic periodic — a rounded-square
        // loop. The parameter must wrap by the period and the seam
        // must be C^1 (derivative continuous).
        let c = NurbsCurve::periodic_uniform(
            3,
            vec![
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            None,
        )
        .unwrap();
        let (lo, hi) = c.domain();
        let period = hi - lo;
        assert!(approx(period, 4.0, 1e-15));
        assert!(approx3(c.evaluate(lo), c.evaluate(hi), 1e-12));
        // Wrap: one full period later is the same point.
        assert!(approx3(
            c.evaluate(lo + 1.3),
            c.evaluate(lo + 1.3 + period),
            1e-12
        ));
        // Seam smoothness.
        let d0 = c.derivative(lo);
        let d1 = c.derivative(hi);
        assert!(approx3(d0, d1, 1e-9), "seam derivative {d0:?} vs {d1:?}");
    }

    #[test]
    fn interior_full_multiplicity_knot_stays_total() {
        // Interior knot with multiplicity degree + 1 creates an
        // (empty-span) kink; evaluation must stay total and finite
        // across it under the 0/0 := 0 convention.
        let c = NurbsCurve::new(
            2,
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [2.0, 1.0, 0.0],
                [2.0, 2.0, 0.0],
                [3.0, 2.0, 0.0],
            ],
            None,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0],
            NurbsForm::Open,
        )
        .unwrap();
        for i in 0..=200 {
            let t = 2.0 * i as f64 / 200.0;
            let p = c.evaluate(t);
            assert!(p.iter().all(|c| c.is_finite()), "non-finite at t = {t}");
        }
        // Multiplicity degree + 1 splits the curve into two
        // independent segments; the span lookup is right-continuous,
        // so t = 1 lands on the right segment's start control point.
        assert!(approx3(c.evaluate(1.0), [2.0, 1.0, 0.0], 1e-15));
        // The left limit is the left segment's end control point.
        assert!(approx3(c.evaluate(1.0 - 1e-12), [1.0, 1.0, 0.0], 1e-9));
    }

    // ---- surfaces --------------------------------------------------

    #[test]
    fn bilinear_patch_is_the_plane() {
        // Degree 1×1 grid on the plane z = 2x + 3y.
        let s = NurbsSurface::new(
            1,
            1,
            2,
            2,
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 2.0],
                [0.0, 1.0, 3.0],
                [1.0, 1.0, 5.0],
            ],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
            NurbsForm::Open,
        )
        .unwrap();
        for i in 0..=10 {
            for j in 0..=10 {
                let u = i as f64 / 10.0;
                let v = j as f64 / 10.0;
                let p = s.evaluate(u, v);
                assert!(
                    approx(p[2], 2.0 * p[0] + 3.0 * p[1], 1e-12),
                    "off-plane at ({u}, {v}): {p:?}"
                );
            }
        }
        // Normal ∝ (-2, -3, 1) everywhere.
        let expect = normalize([-2.0, -3.0, 1.0]).unwrap();
        let n = s.normal(0.3, 0.7);
        assert!(approx3(n, expect, 1e-12), "normal {n:?}");
    }

    #[test]
    fn surface_derivatives_match_finite_differences() {
        // Biquadratic non-rational patch; analytic partials vs
        // central finite differences.
        let mut pts = Vec::new();
        for j in 0..3 {
            for i in 0..3 {
                let x = i as f64;
                let y = j as f64;
                pts.push([x, y, (x * x - y * y) * 0.25 + x * y * 0.1]);
            }
        }
        let knots = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let s = NurbsSurface::new(
            2,
            2,
            3,
            3,
            pts,
            None,
            knots.clone(),
            knots,
            NurbsForm::Open,
            NurbsForm::Open,
        )
        .unwrap();
        let h = 1e-6;
        for &(u, v) in &[(0.25, 0.5), (0.5, 0.25), (0.7, 0.7)] {
            let (_, su, sv) = s.derivatives(u, v);
            let fd_u: Vec<f64> = (0..3)
                .map(|k| (s.evaluate(u + h, v)[k] - s.evaluate(u - h, v)[k]) / (2.0 * h))
                .collect();
            let fd_v: Vec<f64> = (0..3)
                .map(|k| (s.evaluate(u, v + h)[k] - s.evaluate(u, v - h)[k]) / (2.0 * h))
                .collect();
            for k in 0..3 {
                assert!(approx(su[k], fd_u[k], 1e-5), "su[{k}] at ({u}, {v})");
                assert!(approx(sv[k], fd_v[k], 1e-5), "sv[{k}] at ({u}, {v})");
            }
        }
    }

    // ---- tessellation ----------------------------------------------

    #[test]
    fn open_curve_tessellates_to_a_line_strip() {
        let c = quarter_circle();
        let prim = c.tessellate(8).unwrap();
        assert_eq!(prim.topology, Topology::LineStrip);
        assert_eq!(prim.positions.len(), 9);
        assert_eq!(prim.uvs.len(), 1);
        assert_eq!(prim.uvs[0].len(), 9);
        assert!(approx(prim.uvs[0][0][0] as f64, 0.0, 0.0));
        assert!(approx(prim.uvs[0][8][0] as f64, 1.0, 0.0));
        for p in &prim.positions {
            let r = ((p[0] as f64).powi(2) + (p[1] as f64).powi(2)).sqrt();
            assert!(approx(r, 1.0, 1e-6), "radius {r}");
        }
    }

    #[test]
    fn periodic_curve_tessellates_to_a_line_loop_without_seam_duplicate() {
        let c = NurbsCurve::periodic_uniform(
            2,
            vec![
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            None,
        )
        .unwrap();
        let prim = c.tessellate(12).unwrap();
        assert_eq!(prim.topology, Topology::LineLoop);
        assert_eq!(prim.positions.len(), 12);
        // First and last samples are distinct (the loop closes them).
        assert_ne!(prim.positions[0], prim.positions[11]);
    }

    #[test]
    fn surface_tessellation_shape_and_indices() {
        let s = NurbsSurface::new(
            1,
            1,
            2,
            2,
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
            ],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
            NurbsForm::Open,
        )
        .unwrap();
        let prim = s
            .tessellate(&TessellationOptions {
                resolution_u: 4,
                resolution_v: 3,
            })
            .unwrap();
        assert_eq!(prim.topology, Topology::Triangles);
        assert_eq!(prim.positions.len(), 5 * 4);
        assert_eq!(prim.normals.as_ref().unwrap().len(), 20);
        assert_eq!(prim.uvs[0].len(), 20);
        let Some(Indices::U32(idx)) = &prim.indices else {
            panic!("expected U32 indices");
        };
        assert_eq!(idx.len(), 4 * 3 * 2 * 3);
        assert!(idx.iter().all(|&i| (i as usize) < prim.positions.len()));
        // Flat z = 0 patch: every normal is +z (CCW winding).
        for n in prim.normals.as_ref().unwrap() {
            assert!(approx3(
                [n[0] as f64, n[1] as f64, n[2] as f64],
                [0.0, 0.0, 1.0],
                1e-6
            ));
        }
    }

    #[test]
    fn tessellation_rejects_resolution_zero_and_bombs() {
        let c = quarter_circle();
        assert!(c.tessellate(0).is_err());
        assert!(c.tessellate(u32::MAX).is_err());

        let s = NurbsSurface::new(
            1,
            1,
            2,
            2,
            vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
            None,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            NurbsForm::Open,
            NurbsForm::Open,
        )
        .unwrap();
        assert!(s.tessellate(&TessellationOptions::uniform(0)).is_err());
        assert!(s
            .tessellate(&TessellationOptions {
                resolution_u: 1 << 16,
                resolution_v: 1 << 16,
            })
            .is_err());
        // Overflow-shaped product must error, not wrap.
        assert!(s
            .tessellate(&TessellationOptions {
                resolution_u: u32::MAX,
                resolution_v: u32::MAX,
            })
            .is_err());
    }
}
