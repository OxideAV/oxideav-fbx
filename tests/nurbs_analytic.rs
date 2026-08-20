//! Analytic ground-truth + hostile-input validation for the
//! B-spline / NURBS engine in `oxideav_fbx::nurbs`.
//!
//! Everything here checks the tessellator output against closed-form
//! geometry that rational B-splines represent *exactly* (in f64;
//! tolerances account for the f32 cast the `Primitive` buffers
//! carry): a cylinder (rational quadratic circle × line), a full
//! sphere of revolution (rational quadratic semicircle profile ×
//! circle — the collapsed pole rows also exercise the
//! degenerate-normal nudge), a quadratic plane patch, and a periodic
//! tube seam. A fixed-seed generative sweep then hammers random valid
//! curves / surfaces for totality (finite outputs, exact buffer
//! shapes, no panic).

use oxideav_fbx::nurbs::{NurbsCurve, NurbsForm, NurbsSurface, TessellationOptions};
use oxideav_mesh3d::{Indices, Topology};

const W: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// 9-control-point rational quadratic full circle of radius `r` at
/// height `z` (four 90° arcs, double interior knots) — the standard
/// conic-as-NURBS construction.
fn circle_points(r: f64, z: f64) -> Vec<[f64; 3]> {
    vec![
        [r, 0.0, z],
        [r, r, z],
        [0.0, r, z],
        [-r, r, z],
        [-r, 0.0, z],
        [-r, -r, z],
        [0.0, -r, z],
        [r, -r, z],
        [r, 0.0, z],
    ]
}

fn circle_weights() -> Vec<f64> {
    vec![1.0, W, 1.0, W, 1.0, W, 1.0, W, 1.0]
}

fn circle_knots() -> Vec<f64> {
    vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0]
}

#[test]
fn cylinder_patch_is_exactly_cylindrical() {
    // u = full circle (closed, rational quadratic), v = line segment
    // z: 0 -> 5. Grid is u-fastest: row v0 then row v1.
    let nu = 9;
    let nv = 2;
    let mut pts = circle_points(1.0, 0.0);
    pts.extend(circle_points(1.0, 5.0));
    let mut w = circle_weights();
    w.extend(circle_weights());
    let s = NurbsSurface::new(
        2,
        1,
        nu,
        nv,
        pts,
        Some(w),
        circle_knots(),
        vec![0.0, 0.0, 1.0, 1.0],
        NurbsForm::Closed,
        NurbsForm::Open,
    )
    .expect("valid cylinder");

    let prim = s
        .tessellate(&TessellationOptions {
            resolution_u: 48,
            resolution_v: 4,
        })
        .expect("tessellates");
    assert_eq!(prim.topology, Topology::Triangles);
    assert_eq!(prim.positions.len(), 49 * 5);

    let normals = prim.normals.as_ref().expect("normals");
    for (p, n) in prim.positions.iter().zip(normals) {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        let r = (x * x + y * y).sqrt();
        assert!((r - 1.0).abs() < 1e-5, "off-cylinder radius {r}");
        assert!((0.0..=5.0 + 1e-4).contains(&z), "z out of range: {z}");
        // Outward radial normal (u CCW × v +z).
        let dot = (n[0] as f64) * x + (n[1] as f64) * y;
        assert!(dot > 0.999, "non-radial normal dot = {dot}");
        assert!((n[2] as f64).abs() < 1e-4, "normal has z component");
    }

    let Some(Indices::U32(idx)) = &prim.indices else {
        panic!("expected U32 indices");
    };
    assert_eq!(idx.len(), 48 * 4 * 2 * 3);
    assert!(idx.iter().all(|&i| (i as usize) < prim.positions.len()));
}

#[test]
fn sphere_of_revolution_is_exactly_spherical() {
    // u = full revolution circle; v = semicircle profile in the xz
    // plane, south pole -> north pole (two 90° arcs). Grid point
    // (i, j) = profile radius rotated by the circle control point,
    // weight = w_circle · w_profile — the standard
    // surface-of-revolution construction. This ordering makes the
    // `∂S/∂u × ∂S/∂v` normal point outward.
    let profile: [([f64; 2], f64); 5] = [
        ([0.0, -1.0], 1.0),
        ([1.0, -1.0], W),
        ([1.0, 0.0], 1.0),
        ([1.0, 1.0], W),
        ([0.0, 1.0], 1.0),
    ];
    let circle = circle_points(1.0, 0.0);
    let cw = circle_weights();

    let nu = circle.len(); // 9
    let nv = profile.len(); // 5
    let mut pts = Vec::with_capacity(nu * nv);
    let mut wts = Vec::with_capacity(nu * nv);
    for (prof, pw) in &profile {
        let (r, z) = (prof[0], prof[1]);
        for (c, w) in circle.iter().zip(&cw) {
            pts.push([r * c[0], r * c[1], z]);
            wts.push(pw * w);
        }
    }
    let s = NurbsSurface::new(
        2,
        2,
        nu,
        nv,
        pts,
        Some(wts),
        circle_knots(),
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0],
        NurbsForm::Closed,
        NurbsForm::Open,
    )
    .expect("valid sphere");

    // Every evaluated point sits on the unit sphere (f64 check first,
    // tighter than the f32 buffers).
    for i in 0..=20 {
        for j in 0..=20 {
            let u = 4.0 * i as f64 / 20.0;
            let v = 2.0 * j as f64 / 20.0;
            let p = s.evaluate(u, v);
            let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
            assert!((r - 1.0).abs() < 1e-12, "off-sphere r = {r} at ({u}, {v})");
        }
    }

    let prim = s
        .tessellate(&TessellationOptions::uniform(24))
        .expect("tessellates");
    let normals = prim.normals.as_ref().expect("normals");
    assert_eq!(normals.len(), prim.positions.len());
    for (p, n) in prim.positions.iter().zip(normals) {
        let r = ((p[0] as f64).powi(2) + (p[1] as f64).powi(2) + (p[2] as f64).powi(2)).sqrt();
        assert!((r - 1.0).abs() < 1e-5, "off-sphere vertex r = {r}");
        // Unit normals everywhere — including the collapsed pole rows,
        // which go through the degenerate-normal nudge.
        let nlen = ((n[0] as f64).powi(2) + (n[1] as f64).powi(2) + (n[2] as f64).powi(2)).sqrt();
        assert!((nlen - 1.0).abs() < 1e-4, "non-unit normal {nlen}");
        assert!(n.iter().all(|c| c.is_finite()), "non-finite normal");
    }

    // Pole rows: position is the pole, nudged normal points along ∓z.
    let south = s.normal(1.3, 0.0);
    assert!(south[2] < -0.99, "south-pole normal {south:?}");
    let north = s.normal(2.6, 2.0);
    assert!(north[2] > 0.99, "north-pole normal {north:?}");
}

#[test]
fn quadratic_patch_matches_its_polynomial() {
    // Non-rational biquadratic patch interpolating z = x·y over a
    // clamped [0,1]² domain: control z_ij = x_i · y_j reproduces the
    // bilinear-in-control polynomial exactly for the tensor Bézier
    // (control abscissae 0, 0.5, 1 with clamped single-segment
    // knots), since z = x·y is degree (1,1) <= (2,2).
    let xs = [0.0, 0.5, 1.0];
    let mut pts = Vec::new();
    for y in xs {
        for x in xs {
            pts.push([x, y, x * y]);
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
    .expect("valid patch");
    let prim = s
        .tessellate(&TessellationOptions::uniform(16))
        .expect("tessellates");
    for p in &prim.positions {
        let (x, y, z) = (p[0] as f64, p[1] as f64, p[2] as f64);
        assert!((z - x * y).abs() < 1e-5, "z != x·y at ({x}, {y}): {z}");
    }
    // UVs span the unit square corners.
    let uv = &prim.uvs[0];
    assert_eq!(uv[0], [0.0, 0.0]);
    assert_eq!(*uv.last().unwrap(), [1.0, 1.0]);
}

#[test]
fn periodic_tube_seam_is_watertight() {
    // Periodic (uniform-knot) square loop swept along z. The
    // tessellation duplicates the seam vertex for clean UVs; the
    // duplicated positions must agree exactly and the surface must
    // wrap by its period.
    let loop_pts = [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]];
    let nu = 4;
    let nv = 2;
    let mut pts = Vec::new();
    for z in [0.0, 2.0] {
        for p in loop_pts {
            pts.push([p[0], p[1], z]);
        }
    }
    // Uniform periodic knots for the extended u direction:
    // nu + 2·degree + 1 = 4 + 4 + 1 = 9 values.
    let knots_u: Vec<f64> = (0..9).map(|i| i as f64).collect();
    let s = NurbsSurface::new(
        2,
        1,
        nu,
        nv,
        pts,
        None,
        knots_u,
        vec![0.0, 0.0, 1.0, 1.0],
        NurbsForm::Periodic,
        NurbsForm::Open,
    )
    .expect("valid tube");

    let (ulo, uhi) = s.domain_u();
    for j in 0..=8 {
        let v = j as f64 / 8.0;
        let a = s.evaluate(ulo, v);
        let b = s.evaluate(uhi, v);
        for k in 0..3 {
            assert!((a[k] - b[k]).abs() < 1e-12, "seam gap at v = {v}");
        }
        // Wrapping: a full period past an interior parameter is the
        // same point.
        let mid = ulo + 1.234;
        let c = s.evaluate(mid, v);
        let d = s.evaluate(mid + (uhi - ulo), v);
        for k in 0..3 {
            assert!((c[k] - d[k]).abs() < 1e-12, "period wrap broken at v = {v}");
        }
    }

    let prim = s
        .tessellate(&TessellationOptions {
            resolution_u: 12,
            resolution_v: 2,
        })
        .expect("tessellates");
    // Row stride 13: first and last vertex of every row coincide.
    for row in 0..=2usize {
        let first = prim.positions[row * 13];
        let last = prim.positions[row * 13 + 12];
        assert_eq!(first, last, "seam vertices diverge on row {row}");
    }
}

#[test]
fn curve_tessellation_survives_extreme_but_legal_shapes() {
    // Tiny domain span, huge coordinates, minimum resolution.
    let c = NurbsCurve::new(
        1,
        vec![[1e12, -1e12, 3.0], [1e12 + 1.0, -1e12, 3.0]],
        None,
        vec![0.0, 0.0, 1e-9, 1e-9],
        NurbsForm::Open,
    )
    .expect("valid degenerate-ish line");
    let prim = c.tessellate(1).expect("tessellates");
    assert_eq!(prim.topology, Topology::LineStrip);
    assert_eq!(prim.positions.len(), 2);
    assert!(prim
        .positions
        .iter()
        .all(|p| p.iter().all(|c| c.is_finite())));
}

// ---------------------------------------------------------------------
// Fixed-seed generative totality sweep.
// ---------------------------------------------------------------------

/// Minimal deterministic LCG (constants from the classic 64-bit
/// linear congruential parameterization) — no external dependency,
/// fixed seed, replayable.
struct Lcg(u64);

impl Lcg {
    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0
    }

    /// Uniform-ish f64 in [0, 1).
    fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: usize, hi: usize) -> usize {
        lo + (self.next_u64() as usize) % (hi - lo + 1)
    }
}

fn random_knots(rng: &mut Lcg, degree: usize, n: usize, clamped: bool) -> Vec<f64> {
    let len = n + degree + 1;
    let mut knots = Vec::with_capacity(len);
    let mut t = rng.f64() * 2.0 - 1.0;
    for i in 0..len {
        // Occasional zero increments produce repeated interior knots.
        let step = if rng.f64() < 0.25 {
            0.0
        } else {
            rng.f64() + 0.01
        };
        if i > 0 {
            t += step;
        }
        knots.push(t);
    }
    if clamped {
        for i in 0..=degree {
            knots[i] = knots[degree];
            knots[len - 1 - i] = knots[len - 1 - degree];
        }
    }
    // Guarantee a non-degenerate domain.
    if knots[degree] >= knots[n] {
        knots[n] = knots[degree] + 1.0;
        for i in n + 1..len {
            if knots[i] < knots[i - 1] {
                knots[i] = knots[i - 1];
            }
        }
    }
    knots
}

fn random_points(rng: &mut Lcg, count: usize) -> Vec<[f64; 3]> {
    (0..count)
        .map(|_| {
            [
                rng.f64() * 20.0 - 10.0,
                rng.f64() * 20.0 - 10.0,
                rng.f64() * 20.0 - 10.0,
            ]
        })
        .collect()
}

fn random_weights(rng: &mut Lcg, count: usize) -> Option<Vec<f64>> {
    if rng.f64() < 0.5 {
        None
    } else {
        Some((0..count).map(|_| 0.1 + rng.f64() * 9.9).collect())
    }
}

#[test]
fn generative_curve_sweep_is_total() {
    let mut rng = Lcg(0x0449_f00d);
    for iter in 0..200 {
        let degree = rng.range(1, 3);
        let n = rng.range(degree + 1, degree + 6);
        let curve = if rng.f64() < 0.3 {
            NurbsCurve::periodic_uniform(degree, random_points(&mut rng, n), {
                random_weights(&mut rng, n)
            })
        } else {
            let clamped = rng.f64() < 0.5;
            NurbsCurve::new(
                degree,
                random_points(&mut rng, n),
                random_weights(&mut rng, n),
                random_knots(&mut rng, degree, n, clamped),
                NurbsForm::Open,
            )
        };
        let curve = curve.unwrap_or_else(|e| panic!("iter {iter}: construction failed: {e}"));
        let (lo, hi) = curve.domain();
        for k in 0..24 {
            // Sweep across and beyond the domain (clamp/wrap paths).
            let t = lo + (hi - lo) * (k as f64 / 20.0 - 0.1);
            let p = curve.evaluate(t);
            let d = curve.derivative(t);
            assert!(
                p.iter().chain(&d).all(|c| c.is_finite()),
                "iter {iter}: non-finite output at t = {t}"
            );
        }
        let prim = curve
            .tessellate(7)
            .unwrap_or_else(|e| panic!("iter {iter}: tessellation failed: {e}"));
        let expect = match prim.topology {
            Topology::LineLoop => 7,
            _ => 8,
        };
        assert_eq!(prim.positions.len(), expect, "iter {iter}");
        assert!(prim
            .positions
            .iter()
            .all(|p| p.iter().all(|c| c.is_finite())));
    }
}

#[test]
fn generative_surface_sweep_is_total() {
    let mut rng = Lcg(0x0449_cafe);
    for iter in 0..80 {
        let du = rng.range(1, 3);
        let dv = rng.range(1, 3);
        let nu = rng.range(du + 1, du + 4);
        let nv = rng.range(dv + 1, dv + 4);
        let clamp_u = rng.f64() < 0.5;
        let clamp_v = rng.f64() < 0.5;
        let s = NurbsSurface::new(
            du,
            dv,
            nu,
            nv,
            random_points(&mut rng, nu * nv),
            random_weights(&mut rng, nu * nv),
            random_knots(&mut rng, du, nu, clamp_u),
            random_knots(&mut rng, dv, nv, clamp_v),
            NurbsForm::Open,
            NurbsForm::Open,
        )
        .unwrap_or_else(|e| panic!("iter {iter}: construction failed: {e}"));
        let (ulo, uhi) = s.domain_u();
        let (vlo, vhi) = s.domain_v();
        for k in 0..10 {
            let u = ulo + (uhi - ulo) * (k as f64 / 8.0 - 0.1);
            let v = vlo + (vhi - vlo) * (k as f64 / 8.0 - 0.1);
            let (p, su, sv) = s.derivatives(u, v);
            let n = s.normal(u, v);
            assert!(
                p.iter()
                    .chain(&su)
                    .chain(&sv)
                    .chain(&n)
                    .all(|c| c.is_finite()),
                "iter {iter}: non-finite output at ({u}, {v})"
            );
        }
        let prim = s
            .tessellate(&TessellationOptions::uniform(5))
            .unwrap_or_else(|e| panic!("iter {iter}: tessellation failed: {e}"));
        assert_eq!(prim.positions.len(), 36, "iter {iter}");
        assert_eq!(prim.normals.as_ref().unwrap().len(), 36);
        assert_eq!(prim.uvs[0].len(), 36);
        let Some(Indices::U32(idx)) = &prim.indices else {
            panic!("iter {iter}: expected U32 indices");
        };
        assert_eq!(idx.len(), 5 * 5 * 2 * 3);
        assert!(idx.iter().all(|&i| (i as usize) < 36));
    }
}
