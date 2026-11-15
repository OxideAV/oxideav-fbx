//! `Geometry` elements of subtype `"NurbsCurve"` — the wire record
//! set, a typed FBX-side record, the join into the
//! [`crate::nurbs`] evaluation engine, and the scene-model surface.
//!
//! # Wire grammar
//!
//! Per `docs/3d/fbx/fixtures/nurbs-curves-ascii-v7200/notes.md` and
//! `docs/3d/fbx/blender-export-sweeps-2026-09.md` §5.2, a
//! `Geometry: <id>, "<name>::", "NurbsCurve"` element carries, in file
//! order:
//!
//! | Record | Type | Meaning |
//! |--------|------|---------|
//! | `GeometryVersion` | int | `124` observed |
//! | `Type` | string | `"NurbsCurve"` (repeats the subtype) |
//! | `NurbsCurveVersion` | int | `100` observed |
//! | `Order` | int | order = degree + 1 (3 and 4 observed) |
//! | `Dimension` | int | `3` observed |
//! | `Form` | string | `"Open"` / `"Closed"` / `"Periodic"` |
//! | `Rational` | int (bool) | `1` / `0` |
//! | `Points` | float array | four values per control point, `x, y, z, w` — also when `Rational` is `0` (every `w = 1`) |
//! | `KnotVector` | float array | `K` knots |
//!
//! and attaches `OO` directly to a `Model` whose own subtype is
//! `"NurbsCurve"` (no `NodeAttribute` in between).
//!
//! # Knot counts and the effective control array
//!
//! The staged sample pins three knot-count relations (`n` = authored
//! control-point count):
//!
//! | Form | `K` |
//! |------|-----|
//! | `Open` | `n + order` |
//! | `Closed` | `n + order + 1` |
//! | `Periodic` | `n + 2·order − 1` |
//!
//! A B-spline of order `k` over `m` control points has exactly
//! `m + k` knots, so the knot vector itself fixes the size of the
//! control array the curve is evaluated over: `m = K − order`. For
//! `Open`, `m = n`. For `Closed`, `m = n + 1` — the array is the
//! authored one followed by its first point, which (with the staged
//! sample's clamped knot ends) makes the curve start and end on the
//! same point, i.e. geometrically closed. For `Periodic`,
//! `m = n + degree` — the authored array followed by its first
//! `degree` points, exactly the wrap [`crate::nurbs::NurbsCurve`]
//! applies to its [`NurbsForm::Periodic`] form (whose knot-length
//! rule `n + 2·degree + 1` is the same number). This module derives
//! nothing beyond those counts.
//!
//! # Weights, dimension, the other families
//!
//! `docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §3 (fixtures
//! `fixtures/fbxsdk-nurbs/`, binary 7400 / 7500 and ASCII) pins the
//! rest of the grammar:
//!
//! - **Weights (§3.2)**: the vendor tessellator evaluates a curve as
//!   Σ Nᵢ·(xᵢ, yᵢ, zᵢ) / Σ Nᵢ·wᵢ — the stored `x, y, z` are
//!   already-weighted (homogeneous) coordinates.
//!   [`FbxNurbsCurve::to_curve`] follows it. The surface convention is
//!   not determined by the staged material (§5 item 11);
//!   [`FbxNurbsSurface::to_surface`] reads surface points the same way.
//! - **`Dimension`**: the stride is four values also for `Dimension 2`;
//!   the third value is stored and written back unchanged, and
//!   evaluation of a 2-D curve ignores it.
//! - **`NurbsSurface`** (§3.3) — [`FbxNurbsSurface`]; **`Line`** (§3.4)
//!   — [`FbxLine`]; **`TrimNurbsSurface`** with its **`Boundary`**
//!   loops and edge curves (§3.5) — [`FbxTrimSurface`]. The older
//!   `Nurb` / `Patch` families (§3.6) are carried verbatim
//!   ([`crate::geometry_tree`]): the meaning of `Nurb` multiplicities
//!   is not staged.
//! - An `Open` curve whose knot vector is not clamped is evaluated as
//!   the ordinary B-spline over `[U[order−1], U[n]]`; the vendor
//!   tessellator's reading of that case is not staged (§5 item 12).
//!
//! # Scene model
//!
//! `oxideav-mesh3d` has no curve type, so a decoded `NurbsCurve`
//! geometry becomes an [`oxideav_mesh3d::Mesh`] whose single
//! [`oxideav_mesh3d::Primitive`] is the curve sampled as a
//! `LineStrip` (`Open` / `Closed`) or `LineLoop` (`Periodic`), and the
//! exact wire record set rides on that primitive's
//! `extras[`[`NURBS_CURVE_KEY`]`]` so the encoder re-emits the
//! `NurbsCurve` element (not a polygon `Mesh`) byte-for-byte in value.
//! [`FbxNurbsCurve::from_primitive`] reads the typed record back.

use oxideav_mesh3d::{Error, Indices, Mesh, Primitive, Result, Topology};
use serde_json::Value;

use crate::binary::{FbxNode, FbxProperty};
use crate::nurbs::{
    MAX_TESSELLATION_VERTICES, NurbsCurve, NurbsForm, NurbsSurface, TessellationOptions,
};

/// `Primitive::extras` key carrying a `NurbsCurve` geometry's wire
/// record set: `{ "records": [P…], "body": [leaf…] }` in the
/// [`crate::properties70::own_records_json`] /
/// [`crate::properties70::body_json`] shapes.
pub const NURBS_CURVE_KEY: &str = "fbx:nurbs_curve";

/// Line segments emitted per non-empty knot span when a decoded curve
/// is sampled into the scene model.
pub const SEGMENTS_PER_SPAN: u32 = 16;

/// Typed `NurbsCurve` geometry record (see the module docs for the
/// wire grammar).
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FbxNurbsCurve {
    /// `Order` — degree + 1.
    pub order: u32,
    /// `Dimension` — `3` on every staged sample.
    pub dimension: u32,
    /// `Form`.
    pub form: NurbsForm,
    /// `Rational`.
    pub rational: bool,
    /// `Points` as `[x, y, z, w]` per authored control point.
    pub points: Vec<[f64; 4]>,
    /// `KnotVector`.
    pub knots: Vec<f64>,
}

impl FbxNurbsCurve {
    /// A curve with `Dimension 3`; `Rational` is set when any point's
    /// `w` differs from `1`.
    pub fn new(order: u32, form: NurbsForm, points: Vec<[f64; 4]>, knots: Vec<f64>) -> Self {
        let rational = points.iter().any(|p| p[3] != 1.0);
        Self {
            order,
            dimension: 3,
            form,
            rational,
            points,
            knots,
        }
    }

    /// Builder-style `Dimension` setter.
    pub fn with_dimension(mut self, dimension: u32) -> Self {
        self.dimension = dimension;
        self
    }

    /// Builder-style `Rational` setter.
    pub fn with_rational(mut self, rational: bool) -> Self {
        self.rational = rational;
        self
    }

    /// The `KnotVector` length the staged relations predict for this
    /// form, order and point count (module docs table).
    pub fn expected_knot_count(&self) -> usize {
        let n = self.points.len();
        let k = self.order as usize;
        match self.form {
            NurbsForm::Open => n + k,
            NurbsForm::Closed => n + k + 1,
            NurbsForm::Periodic => (n + 2 * k).saturating_sub(1),
        }
    }

    /// Parse a `Geometry` element of subtype `"NurbsCurve"`. `None`
    /// when a required record is missing or malformed (unknown `Form`
    /// string, `Points` not a multiple of four values, …).
    pub fn from_element(element: &FbxNode) -> Option<Self> {
        let int = |name: &str| -> Option<i64> { element.child(name)?.properties.first()?.as_i64() };
        let order = u32::try_from(int("Order")?).ok()?;
        let dimension = u32::try_from(int("Dimension").unwrap_or(3)).ok()?;
        let form = match element.child("Form")?.properties.first()?.as_str()? {
            "Open" => NurbsForm::Open,
            "Closed" => NurbsForm::Closed,
            "Periodic" => NurbsForm::Periodic,
            _ => return None,
        };
        let rational = int("Rational").unwrap_or(0) != 0;
        let flat = float_array(element.child("Points")?)?;
        if flat.len() % 4 != 0 {
            return None;
        }
        let points = flat.as_chunks::<4>().0.to_vec();
        let knots = float_array(element.child("KnotVector")?)?;
        Some(Self {
            order,
            dimension,
            form,
            rational,
            points,
            knots,
        })
    }

    /// Build the `Geometry` element in the staged record order.
    pub fn to_element(&self, id: i64, name: &str) -> FbxNode {
        let mut full = name.as_bytes().to_vec();
        full.extend_from_slice(b"\x00\x01Geometry");
        FbxNode {
            name: "Geometry".to_string(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(full),
                FbxProperty::String(b"NurbsCurve".to_vec()),
            ],
            children: self.body(),
        }
    }

    /// The element body (every leaf, staged order).
    fn body(&self) -> Vec<FbxNode> {
        let leaf = |name: &str, p: FbxProperty| FbxNode {
            name: name.to_string(),
            properties: vec![p],
            children: Vec::new(),
        };
        let form = match self.form {
            NurbsForm::Open => "Open",
            NurbsForm::Closed => "Closed",
            NurbsForm::Periodic => "Periodic",
        };
        vec![
            leaf("GeometryVersion", FbxProperty::I32(124)),
            leaf("Type", FbxProperty::String(b"NurbsCurve".to_vec())),
            leaf("NurbsCurveVersion", FbxProperty::I32(100)),
            leaf("Order", FbxProperty::I32(self.order as i32)),
            leaf("Dimension", FbxProperty::I32(self.dimension as i32)),
            leaf("Form", FbxProperty::String(form.as_bytes().to_vec())),
            leaf("Rational", FbxProperty::I32(i32::from(self.rational))),
            leaf(
                "Points",
                FbxProperty::F64Array(self.points.iter().flatten().copied().collect()),
            ),
            leaf("KnotVector", FbxProperty::F64Array(self.knots.clone())),
        ]
    }

    /// The validated evaluation-engine curve (module docs: effective
    /// control array; `x, y, z` are homogeneous coordinates divided by
    /// the blended `w`; a `Dimension 2` curve lies in the `z = 0`
    /// plane).
    pub fn to_curve(&self) -> Result<NurbsCurve> {
        if self.order < 2 {
            return Err(Error::invalid(format!(
                "fbx nurbs curve: order {} < 2",
                self.order
            )));
        }
        let degree = self.order as usize - 1;
        if self.knots.len() != self.expected_knot_count() {
            return Err(Error::invalid(format!(
                "fbx nurbs curve: {} knots, the {:?} form with {} points of order {} needs {}",
                self.knots.len(),
                self.form,
                self.points.len(),
                self.order,
                self.expected_knot_count()
            )));
        }
        let mut pts: Vec<[f64; 4]> = self.points.clone();
        if self.form == NurbsForm::Closed {
            // `m = K − order = n + 1`: the first point closes the array.
            let first = *pts
                .first()
                .ok_or_else(|| Error::invalid("fbx nurbs curve: no control points"))?;
            pts.push(first);
        }
        let planar = self.dimension == 2;
        let (control, weights) = split_homogeneous(&pts, planar);
        NurbsCurve::new(degree, control, weights, self.knots.clone(), self.form)
    }

    /// Sample the curve into a line primitive, `segments_per_span`
    /// segments per non-empty knot span of the evaluation domain
    /// (capped at [`MAX_TESSELLATION_VERTICES`]).
    pub fn tessellate(&self, segments_per_span: u32) -> Result<Primitive> {
        let curve = self.to_curve()?;
        let (lo, hi) = curve.domain();
        let spans = curve
            .knots()
            .windows(2)
            .filter(|w| w[0] >= lo && w[1] <= hi && w[1] > w[0])
            .count()
            .max(1);
        let segments = (spans as u64 * u64::from(segments_per_span.max(1)))
            .min(MAX_TESSELLATION_VERTICES as u64 - 1) as u32;
        curve.tessellate(segments)
    }

    /// The typed record back off a primitive carrying
    /// `extras[`[`NURBS_CURVE_KEY`]`]`.
    pub fn from_primitive(prim: &Primitive) -> Option<Self> {
        let element = element_from_extras(prim.extras.get(NURBS_CURVE_KEY)?, 0, "")?;
        Self::from_element(&element)
    }

    /// A scene-model mesh for this curve: one sampled line primitive
    /// ([`SEGMENTS_PER_SPAN`]) carrying the record on
    /// `extras[`[`NURBS_CURVE_KEY`]`]`, so the encoder writes it back
    /// as a `NurbsCurve` geometry. A curve that fails validation gets
    /// an empty `LineStrip` primitive (the record still rides along).
    pub fn to_mesh(&self, name: Option<String>) -> Mesh {
        let element = self.to_element(0, "");
        mesh_for_element(&element, name, &NurbsTessellation::default())
    }
}

/// Decode-side entry: the scene-model mesh for one `NurbsCurve`
/// `Geometry` element (its own records verbatim on the primitive's
/// extras, the sampled line when the record set validates).
pub(crate) fn mesh_for_element(
    element: &FbxNode,
    name: Option<String>,
    how: &NurbsTessellation,
) -> Mesh {
    let mut prim = FbxNurbsCurve::from_element(element)
        .and_then(|c| c.tessellate_with(how).ok())
        .unwrap_or_else(|| Primitive::new(Topology::LineStrip));
    prim.extras.insert(
        NURBS_CURVE_KEY.to_string(),
        serde_json::json!({
            "records": crate::properties70::own_records_json(element),
            "body": crate::properties70::body_json(element),
        }),
    );
    let mut mesh = Mesh::new(name);
    mesh.primitives.push(prim);
    mesh
}

/// Encode-side entry: the `NurbsCurve` `Geometry` element a
/// primitive's `extras[`[`NURBS_CURVE_KEY`]`]` describes, re-emitted
/// verbatim (`Properties70` first when the source carried one, then
/// every body leaf in source order).
pub(crate) fn element_from_extras(v: &Value, id: i64, name: &str) -> Option<FbxNode> {
    let body = v.get("body")?.as_array()?;
    let mut children = Vec::new();
    if let Some(records) = v.get("records").and_then(Value::as_array)
        && !records.is_empty()
    {
        children.push(FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: records
                .iter()
                .filter_map(crate::properties70::json_to_p_record)
                .collect(),
        });
    }
    children.extend(
        body.iter()
            .filter_map(crate::properties70::json_to_body_leaf),
    );
    let mut full = name.as_bytes().to_vec();
    full.extend_from_slice(b"\x00\x01Geometry");
    Some(FbxNode {
        name: "Geometry".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(full),
            FbxProperty::String(b"NurbsCurve".to_vec()),
        ],
        children,
    })
}

/// A numeric array leaf read loosely: the ASCII form narrows an
/// all-integer list (`KnotVector: *10 { a: 0,0,0,1,… }`) to an integer
/// array, the binary form carries `d` / `f`.
fn float_array(n: &FbxNode) -> Option<Vec<f64>> {
    Some(match n.properties.first()? {
        FbxProperty::F64Array(a) => a.clone(),
        FbxProperty::F32Array(a) => a.iter().map(|v| f64::from(*v)).collect(),
        FbxProperty::I32Array(a) => a.iter().map(|v| f64::from(*v)).collect(),
        FbxProperty::I64Array(a) => a.iter().map(|v| *v as f64).collect(),
        _ => return None,
    })
}

/// Split stored `[x, y, z, w]` points into the engine's Cartesian
/// control points and weights: `x, y, z` are homogeneous coordinates
/// (module docs), so the Cartesian point is `(x, y, z) / w`. `None`
/// weights when every `w` is 1. `planar` zeroes `z`.
fn split_homogeneous(pts: &[[f64; 4]], planar: bool) -> (Vec<[f64; 3]>, Option<Vec<f64>>) {
    let rational = pts.iter().any(|p| p[3] != 1.0);
    let control = pts
        .iter()
        .map(|p| {
            let w = if rational && p[3] != 0.0 { p[3] } else { 1.0 };
            [p[0] / w, p[1] / w, if planar { 0.0 } else { p[2] / w }]
        })
        .collect();
    (
        control,
        rational.then(|| pts.iter().map(|p| p[3]).collect()),
    )
}

fn form_from_str(s: &str) -> Option<NurbsForm> {
    match s {
        "Open" => Some(NurbsForm::Open),
        "Closed" => Some(NurbsForm::Closed),
        "Periodic" => Some(NurbsForm::Periodic),
        _ => None,
    }
}

fn form_str(f: NurbsForm) -> &'static str {
    match f {
        NurbsForm::Open => "Open",
        NurbsForm::Closed => "Closed",
        NurbsForm::Periodic => "Periodic",
    }
}

/// Knot count for a form, order `k` and point count `n` (§3.2 table).
fn knot_count(form: NurbsForm, k: usize, n: usize) -> usize {
    match form {
        NurbsForm::Open => n + k,
        NurbsForm::Closed => n + k + 1,
        NurbsForm::Periodic => (n + 2 * k).saturating_sub(1),
    }
}

fn leaf(name: &str, properties: Vec<FbxProperty>) -> FbxNode {
    FbxNode {
        name: name.to_string(),
        properties,
        children: Vec::new(),
    }
}

fn geometry_element(id: i64, name: &str, subtype: &str, children: Vec<FbxNode>) -> FbxNode {
    let mut full = name.as_bytes().to_vec();
    full.extend_from_slice(b"\x00\x01Geometry");
    FbxNode {
        name: "Geometry".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(full),
            FbxProperty::String(subtype.as_bytes().to_vec()),
        ],
        children,
    }
}

fn ints_of(element: &FbxNode, name: &str) -> Option<Vec<i64>> {
    element
        .child(name)?
        .properties
        .iter()
        .map(FbxProperty::as_i64)
        .collect()
}

/// How a decoded NURBS record is sampled into the scene model.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct NurbsTessellation {
    /// Segments per non-empty knot span when no tolerance is set and
    /// the record carries no `Step` of its own.
    pub segments_per_span: u32,
    /// Chord tolerance in model units; when set it decides the
    /// resolution ([`NurbsCurve::tessellate_within`] /
    /// [`crate::nurbs::NurbsSurface::tessellate_within`]).
    pub tolerance: Option<f64>,
}

impl Default for NurbsTessellation {
    fn default() -> Self {
        Self {
            segments_per_span: SEGMENTS_PER_SPAN,
            tolerance: None,
        }
    }
}

impl NurbsTessellation {
    /// The defaults: [`SEGMENTS_PER_SPAN`], no tolerance.
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder-style setter for
    /// [`segments_per_span`](Self::segments_per_span).
    pub fn with_segments_per_span(mut self, segments_per_span: u32) -> Self {
        self.segments_per_span = segments_per_span;
        self
    }

    /// Builder-style setter for [`tolerance`](Self::tolerance).
    pub fn with_tolerance(mut self, tolerance: f64) -> Self {
        self.tolerance = Some(tolerance);
        self
    }
}

impl FbxNurbsCurve {
    /// Sample the curve per `how`: within its tolerance when one is
    /// set, else [`FbxNurbsCurve::tessellate`] at its per-span count.
    pub fn tessellate_with(&self, how: &NurbsTessellation) -> Result<Primitive> {
        match how.tolerance {
            Some(tol) => self.to_curve()?.tessellate_within(tol),
            None => self.tessellate(how.segments_per_span),
        }
    }
}

/// Typed `NurbsSurface` geometry record (§3.3). Pairs are `[U, V]`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FbxNurbsSurface {
    /// First `SurfaceDisplay` value: 0 raw, 1 low without normals,
    /// 2 low, 3 high without normals, 4 high (the default).
    pub surface_mode: i32,
    /// `Step` (repeated as the 2nd and 3rd `SurfaceDisplay` values).
    pub step: [u32; 2],
    /// `NurbsSurfaceOrder` — degree + 1 per direction.
    pub order: [u32; 2],
    /// `Dimensions` — control-point count per direction.
    pub dimensions: [u32; 2],
    /// `Form` per direction.
    pub form: [NurbsForm; 2],
    /// `Points` as `[x, y, z, w]`, U fastest (flat index `v·nU + u`).
    pub points: Vec<[f64; 4]>,
    /// `KnotVectorU`.
    pub knots_u: Vec<f64>,
    /// `KnotVectorV`.
    pub knots_v: Vec<f64>,
    /// `FlipNormals`.
    pub flip_normals: bool,
}

impl FbxNurbsSurface {
    /// A surface with the default display mode (4), no steps and
    /// unflipped normals.
    pub fn new(
        order: [u32; 2],
        dimensions: [u32; 2],
        form: [NurbsForm; 2],
        points: Vec<[f64; 4]>,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
    ) -> Self {
        Self {
            surface_mode: 4,
            step: [0, 0],
            order,
            dimensions,
            form,
            points,
            knots_u,
            knots_v,
            flip_normals: false,
        }
    }

    /// Builder-style `Step` setter.
    pub fn with_step(mut self, step: [u32; 2]) -> Self {
        self.step = step;
        self
    }

    /// Builder-style surface-mode setter.
    pub fn with_surface_mode(mut self, surface_mode: i32) -> Self {
        self.surface_mode = surface_mode;
        self
    }

    /// Builder-style `FlipNormals` setter.
    pub fn with_flip_normals(mut self, flip_normals: bool) -> Self {
        self.flip_normals = flip_normals;
        self
    }

    /// Knot counts the §3.2 table predicts, `[U, V]`.
    pub fn expected_knot_counts(&self) -> [usize; 2] {
        [0, 1].map(|d| {
            knot_count(
                self.form[d],
                self.order[d] as usize,
                self.dimensions[d] as usize,
            )
        })
    }

    /// Parse a `Geometry` element of sub-type `"NurbsSurface"`.
    pub fn from_element(element: &FbxNode) -> Option<Self> {
        let pair = |name: &str| -> Option<[u32; 2]> {
            let v = ints_of(element, name)?;
            Some([
                u32::try_from(*v.first()?).ok()?,
                u32::try_from(*v.get(1)?).ok()?,
            ])
        };
        let display = ints_of(element, "SurfaceDisplay").unwrap_or_default();
        let forms = &element.child("Form")?.properties;
        let form = [
            form_from_str(forms.first()?.as_str()?)?,
            form_from_str(forms.get(1)?.as_str()?)?,
        ];
        let flat = float_array(element.child("Points")?)?;
        if flat.len() % 4 != 0 {
            return None;
        }
        let dimensions = pair("Dimensions")?;
        let step = pair("Step").unwrap_or_else(|| {
            [1, 2].map(|i| {
                display
                    .get(i)
                    .and_then(|v| u32::try_from(*v).ok())
                    .unwrap_or(0)
            })
        });
        Some(Self {
            surface_mode: display.first().copied().unwrap_or(4) as i32,
            step,
            order: pair("NurbsSurfaceOrder")?,
            dimensions,
            form,
            points: flat.as_chunks::<4>().0.to_vec(),
            knots_u: float_array(element.child("KnotVectorU")?)?,
            knots_v: float_array(element.child("KnotVectorV")?)?,
            flip_normals: ints_of(element, "FlipNormals")
                .and_then(|v| v.first().copied())
                .unwrap_or(0)
                != 0,
        })
    }

    /// The element body in the §3.3 record order.
    pub fn to_body(&self) -> Vec<FbxNode> {
        let i = |v: u32| FbxProperty::I32(v as i32);
        let s = |v: &str| FbxProperty::String(v.as_bytes().to_vec());
        vec![
            leaf("Type", vec![s("NurbsSurface")]),
            leaf("NurbsSurfaceVersion", vec![FbxProperty::I32(100)]),
            leaf(
                "SurfaceDisplay",
                vec![
                    FbxProperty::I32(self.surface_mode),
                    i(self.step[0]),
                    i(self.step[1]),
                ],
            ),
            leaf(
                "NurbsSurfaceOrder",
                vec![i(self.order[0]), i(self.order[1])],
            ),
            leaf(
                "Dimensions",
                vec![i(self.dimensions[0]), i(self.dimensions[1])],
            ),
            leaf("Step", vec![i(self.step[0]), i(self.step[1])]),
            leaf(
                "Form",
                vec![s(form_str(self.form[0])), s(form_str(self.form[1]))],
            ),
            leaf(
                "Points",
                vec![FbxProperty::F64Array(
                    self.points.iter().flatten().copied().collect(),
                )],
            ),
            leaf(
                "KnotVectorU",
                vec![FbxProperty::F64Array(self.knots_u.clone())],
            ),
            leaf(
                "KnotVectorV",
                vec![FbxProperty::F64Array(self.knots_v.clone())],
            ),
            leaf("GeometryVersion", vec![FbxProperty::I32(124)]),
            leaf(
                "FlipNormals",
                vec![FbxProperty::I32(i32::from(self.flip_normals))],
            ),
        ]
    }

    /// The `Geometry` element.
    pub fn to_element(&self, id: i64, name: &str) -> FbxNode {
        geometry_element(id, name, "NurbsSurface", self.to_body())
    }

    /// The validated evaluation-engine surface: per direction a
    /// `Closed` form is the authored row closed by its first point, a
    /// `Periodic` form wraps (§3.2 "what the extra knots mean", applied
    /// per direction); points are homogeneous (module docs).
    pub fn to_surface(&self) -> Result<NurbsSurface> {
        let [nu, nv] = self.dimensions.map(|d| d as usize);
        if self.order[0] < 2 || self.order[1] < 2 {
            return Err(Error::invalid("fbx nurbs surface: order < 2"));
        }
        if nu.checked_mul(nv) != Some(self.points.len()) || self.points.is_empty() {
            return Err(Error::invalid(format!(
                "fbx nurbs surface: {} points for a {nu} x {nv} grid",
                self.points.len()
            )));
        }
        let expected = self.expected_knot_counts();
        if self.knots_u.len() != expected[0] || self.knots_v.len() != expected[1] {
            return Err(Error::invalid(format!(
                "fbx nurbs surface: {} / {} knots, the forms need {} / {}",
                self.knots_u.len(),
                self.knots_v.len(),
                expected[0],
                expected[1]
            )));
        }
        let eu = nu + usize::from(self.form[0] == NurbsForm::Closed);
        let ev = nv + usize::from(self.form[1] == NurbsForm::Closed);
        let mut grid = Vec::with_capacity(eu * ev);
        for v in 0..ev {
            for u in 0..eu {
                grid.push(self.points[(v % nv) * nu + (u % nu)]);
            }
        }
        let (control, weights) = split_homogeneous(&grid, false);
        NurbsSurface::new(
            self.order[0] as usize - 1,
            self.order[1] as usize - 1,
            eu,
            ev,
            control,
            weights,
            self.knots_u.clone(),
            self.knots_v.clone(),
            self.form[0],
            self.form[1],
        )
    }

    /// Grid resolution `(u, v)` for `how`: from its tolerance, else the
    /// record's own `Step` per span where it is set, else
    /// `segments_per_span`.
    fn resolution(&self, surface: &NurbsSurface, how: &NurbsTessellation) -> (u32, u32) {
        if let Some(tol) = how.tolerance.filter(|t| t.is_finite() && *t > 0.0) {
            return surface.resolution_within(tol);
        }
        let (su, sv) = surface.span_counts();
        // A per-span count beyond `MAX_SEGMENTS_PER_SPAN` (a damaged
        // or hostile `Step`) is held to it.
        let per = |d: usize| -> u64 {
            u64::from(
                if self.step[d] > 0 {
                    self.step[d]
                } else {
                    how.segments_per_span.max(1)
                }
                .min(crate::nurbs::MAX_SEGMENTS_PER_SPAN),
            )
        };
        let cap = |spans: usize, per: u64| (spans as u64 * per).clamp(1, 1023) as u32;
        (cap(su, per(0)), cap(sv, per(1)))
    }

    /// Tessellate into an indexed triangle primitive (positions,
    /// normals, parameter UVs). `FlipNormals` reverses the winding and
    /// the normals.
    pub fn tessellate(&self, how: &NurbsTessellation) -> Result<Primitive> {
        let surface = self.to_surface()?;
        let (resolution_u, resolution_v) = self.resolution(&surface, how);
        let mut prim = surface.tessellate(
            &TessellationOptions::default()
                .with_resolution_u(resolution_u)
                .with_resolution_v(resolution_v),
        )?;
        if self.flip_normals {
            flip_primitive(&mut prim);
        }
        Ok(prim)
    }
}

/// Reverse triangle winding and negate normals.
fn flip_primitive(prim: &mut Primitive) {
    if let Some(Indices::U32(idx)) = &mut prim.indices {
        for tri in idx.as_chunks_mut::<3>().0 {
            tri.swap(1, 2);
        }
    }
    if let Some(normals) = &mut prim.normals {
        for n in normals {
            *n = [-n[0], -n[1], -n[2]];
        }
    }
}

/// Typed `Line` geometry record (§3.4): points with stride three and an
/// index list in which the last index of each polyline is stored as
/// its bitwise complement.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FbxLine {
    /// `Points`.
    pub points: Vec<[f64; 3]>,
    /// `PointsIndex` exactly as stored (end marks complemented).
    pub points_index: Vec<i32>,
}

impl FbxLine {
    /// A line set from explicit polylines (each one's last index gets
    /// the end mark).
    pub fn from_polylines(points: Vec<[f64; 3]>, polylines: &[Vec<u32>]) -> Self {
        let mut points_index = Vec::new();
        for line in polylines {
            for (i, idx) in line.iter().enumerate() {
                let v = *idx as i32;
                points_index.push(if i + 1 == line.len() { !v } else { v });
            }
        }
        Self {
            points,
            points_index,
        }
    }

    /// The polylines as point indices: every run closed by an end mark,
    /// then — when the list ends without one — the trailing run (the
    /// staged material does not say whether such a run is drawn, §5
    /// item 14; it is surfaced so no point is lost, see
    /// [`FbxLine::has_unmarked_tail`]).
    pub fn polylines(&self) -> Vec<Vec<u32>> {
        let mut out = Vec::new();
        let mut run: Vec<u32> = Vec::new();
        for &raw in &self.points_index {
            if raw < 0 {
                run.push(!raw as u32);
                out.push(std::mem::take(&mut run));
            } else {
                run.push(raw as u32);
            }
        }
        if !run.is_empty() {
            out.push(run);
        }
        out
    }

    /// `true` when the index list ends without an end mark.
    pub fn has_unmarked_tail(&self) -> bool {
        self.points_index.last().is_some_and(|v| *v >= 0)
    }

    /// Parse a `Geometry` element of sub-type `"Line"`.
    pub fn from_element(element: &FbxNode) -> Option<Self> {
        let flat = float_array(element.child("Points")?)?;
        if flat.len() % 3 != 0 {
            return None;
        }
        let points_index = match element.child("PointsIndex")?.properties.first()? {
            FbxProperty::I32Array(a) => a.clone(),
            FbxProperty::I64Array(a) => a.iter().map(|v| *v as i32).collect(),
            _ => return None,
        };
        Some(Self {
            points: flat.as_chunks::<3>().0.to_vec(),
            points_index,
        })
    }

    /// The element body in the §3.4 record order.
    pub fn to_body(&self) -> Vec<FbxNode> {
        vec![
            leaf("GeometryVersion", vec![FbxProperty::I32(124)]),
            leaf("Type", vec![FbxProperty::String(b"Line".to_vec())]),
            leaf("LineVersion", vec![FbxProperty::I32(100)]),
            leaf(
                "Points",
                vec![FbxProperty::F64Array(
                    self.points.iter().flatten().copied().collect(),
                )],
            ),
            leaf(
                "PointsIndex",
                vec![FbxProperty::I32Array(self.points_index.clone())],
            ),
        ]
    }

    /// The `Geometry` element.
    pub fn to_element(&self, id: i64, name: &str) -> FbxNode {
        geometry_element(id, name, "Line", self.to_body())
    }

    /// One `LineStrip` primitive per polyline (indices out of range are
    /// dropped).
    pub fn to_primitives(&self) -> Vec<Primitive> {
        self.polylines()
            .into_iter()
            .map(|line| {
                let mut prim = Primitive::new(Topology::LineStrip);
                prim.positions = line
                    .iter()
                    .filter_map(|i| self.points.get(*i as usize))
                    .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
                    .collect();
                prim
            })
            .collect()
    }
}

/// One `Boundary` of a trimmed surface (§3.5): a closed loop of edge
/// curves in the surface's parameter plane.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FbxBoundary {
    /// The `OuterBoundary` property: this loop starts a trim region.
    pub outer: bool,
    /// The edge curves in loop (connection) order.
    pub edges: Vec<FbxNurbsCurve>,
}

impl FbxBoundary {
    /// A boundary from its edge curves.
    pub fn new(outer: bool, edges: Vec<FbxNurbsCurve>) -> Self {
        Self { outer, edges }
    }

    /// The `Geometry` element (§3.5: `Properties70` with
    /// `OuterBoundary` on an outer boundary only).
    pub fn to_element(&self, id: i64, name: &str) -> FbxNode {
        let mut children = Vec::new();
        if self.outer {
            children.push(FbxNode {
                name: "Properties70".to_string(),
                properties: Vec::new(),
                children: vec![leaf(
                    "P",
                    vec![
                        FbxProperty::String(b"OuterBoundary".to_vec()),
                        FbxProperty::String(b"bool".to_vec()),
                        FbxProperty::String(Vec::new()),
                        FbxProperty::String(Vec::new()),
                        FbxProperty::I32(1),
                    ],
                )],
            });
        }
        children.extend([
            leaf("GeometryVersion", vec![FbxProperty::I32(124)]),
            leaf("Type", vec![FbxProperty::String(b"Boundary".to_vec())]),
            leaf("BoundaryVersion", vec![FbxProperty::I32(100)]),
        ]);
        geometry_element(id, name, "Boundary", children)
    }

    /// `true` when the element's `Properties70` sets `OuterBoundary`.
    pub fn element_is_outer(element: &FbxNode) -> bool {
        crate::properties70::PropertyMap::from_element(element)
            .as_i64("OuterBoundary")
            .is_some_and(|v| v != 0)
    }

    /// The loop as a polyline of `(u, v)` parameter points: every edge
    /// sampled per `how` and joined in order.
    pub fn loop_points(&self, how: &NurbsTessellation) -> Vec<[f64; 2]> {
        let mut out: Vec<[f64; 2]> = Vec::new();
        for edge in &self.edges {
            let Ok(prim) = edge.tessellate_with(how) else {
                continue;
            };
            for p in &prim.positions {
                let q = [f64::from(p[0]), f64::from(p[1])];
                if out.last() != Some(&q) {
                    out.push(q);
                }
            }
        }
        if out.len() > 1 && out.first() == out.last() {
            out.pop();
        }
        out
    }
}

/// One trim region: an outer loop and the holes cut out of it, as
/// indices into [`FbxTrimSurface::boundaries`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrimRegion {
    /// The region's outer boundary.
    pub outer: usize,
    /// Its holes.
    pub holes: Vec<usize>,
}

/// Typed `TrimNurbsSurface` (§3.5): the untrimmed surface and its
/// boundaries. The trim object itself carries only `FlipNormals`.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FbxTrimSurface {
    /// `FlipNormals` of the trim object.
    pub flip_normals: bool,
    /// The untrimmed surface.
    pub surface: FbxNurbsSurface,
    /// The boundaries in connection order.
    pub boundaries: Vec<FbxBoundary>,
}

impl FbxTrimSurface {
    /// A trimmed surface.
    pub fn new(surface: FbxNurbsSurface, boundaries: Vec<FbxBoundary>) -> Self {
        Self {
            flip_normals: false,
            surface,
            boundaries,
        }
    }

    /// Builder-style `FlipNormals` setter.
    pub fn with_flip_normals(mut self, flip_normals: bool) -> Self {
        self.flip_normals = flip_normals;
        self
    }

    /// The trim regions (§3.5): they are not stored — each boundary
    /// flagged outer starts a region, and the unflagged boundaries
    /// after it are that region's holes. (An unflagged boundary with no
    /// flagged one before it starts a region of its own.)
    pub fn regions(&self) -> Vec<TrimRegion> {
        let mut out: Vec<TrimRegion> = Vec::new();
        for (i, b) in self.boundaries.iter().enumerate() {
            match out.last_mut() {
                Some(region) if !b.outer => region.holes.push(i),
                _ => out.push(TrimRegion {
                    outer: i,
                    holes: Vec::new(),
                }),
            }
        }
        out
    }

    /// The `TrimNurbsSurface` element body (§3.5).
    pub fn to_body(&self) -> Vec<FbxNode> {
        vec![
            leaf("TrimmedNurbVersion", vec![FbxProperty::I32(100)]),
            leaf(
                "Type",
                vec![FbxProperty::String(b"TrimNurbsSurface".to_vec())],
            ),
            leaf(
                "FlipNormals",
                vec![FbxProperty::I32(i32::from(self.flip_normals))],
            ),
            leaf("GeometryVersion", vec![FbxProperty::I32(124)]),
        ]
    }

    /// The `TrimNurbsSurface` element (boundaries and the untrimmed
    /// surface are separate objects — see [`crate::geometry_tree`]).
    pub fn to_element(&self, id: i64, name: &str) -> FbxNode {
        geometry_element(id, name, "TrimNurbsSurface", self.to_body())
    }

    /// `true` when parameter point `(u, v)` lies in a trim region:
    /// inside a region's outer loop and outside all of its holes
    /// (even–odd crossing rule). Boundary coordinates are read as the
    /// surface's own parameter values (§3.5: the edge curves are
    /// "authored in the surface's parameter plane").
    pub fn contains(loops: &[Vec<[f64; 2]>], regions: &[TrimRegion], u: f64, v: f64) -> bool {
        regions.iter().any(|r| {
            inside_loop(&loops[r.outer], u, v)
                && !r.holes.iter().any(|h| inside_loop(&loops[*h], u, v))
        })
    }

    /// Tessellate the trimmed surface: the untrimmed surface's grid
    /// with every triangle outside the trim regions removed. A triangle
    /// the boundary crosses is split in four (three levels deep) and
    /// its pieces are kept by where their centres fall, so the trimmed
    /// edge follows the loops to 1/8 of a grid cell.
    pub fn tessellate(&self, how: &NurbsTessellation) -> Result<Primitive> {
        let surface = self.surface.to_surface()?;
        let mut prim = self.surface.tessellate(how)?;
        let loops: Vec<Vec<[f64; 2]>> =
            self.boundaries.iter().map(|b| b.loop_points(how)).collect();
        let regions = self.regions();
        let (ulo, uhi) = surface.domain_u();
        let (vlo, vhi) = surface.domain_v();
        let uvs: Vec<[f64; 2]> = prim
            .uvs
            .first()
            .map(|uv| {
                uv.iter()
                    .map(|p| {
                        [
                            ulo + (uhi - ulo) * f64::from(p[0]),
                            vlo + (vhi - vlo) * f64::from(p[1]),
                        ]
                    })
                    .collect()
            })
            .unwrap_or_default();
        let Some(Indices::U32(indices)) = prim.indices.take() else {
            return Ok(prim);
        };
        let mut mesher = TrimMesher {
            surface: &surface,
            loops: &loops,
            regions: &regions,
            domain: [ulo, uhi, vlo, vhi],
            flip: self.surface.flip_normals,
            prim: &mut prim,
            params: uvs,
            kept: Vec::with_capacity(indices.len()),
        };
        for tri in indices.as_chunks::<3>().0 {
            mesher.triangle(*tri, 3);
        }
        let kept = std::mem::take(&mut mesher.kept);
        prim.indices = Some(Indices::U32(kept));
        if self.flip_normals {
            flip_primitive(&mut prim);
        }
        Ok(prim)
    }
}

/// Even–odd point-in-polygon test.
fn inside_loop(poly: &[[f64; 2]], u: f64, v: f64) -> bool {
    let mut inside = false;
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > v) != (b[1] > v) && u < (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Working state of [`FbxTrimSurface::tessellate`].
struct TrimMesher<'a> {
    surface: &'a NurbsSurface,
    loops: &'a [Vec<[f64; 2]>],
    regions: &'a [TrimRegion],
    /// `[ulo, uhi, vlo, vhi]`.
    domain: [f64; 4],
    flip: bool,
    prim: &'a mut Primitive,
    /// `(u, v)` of every vertex.
    params: Vec<[f64; 2]>,
    kept: Vec<u32>,
}

impl TrimMesher<'_> {
    fn inside(&self, p: [f64; 2]) -> bool {
        FbxTrimSurface::contains(self.loops, self.regions, p[0], p[1])
    }

    /// A new vertex at the parameter mid-point of `a` and `b`.
    fn mid_vertex(&mut self, a: u32, b: u32) -> u32 {
        let (pa, pb) = (self.params[a as usize], self.params[b as usize]);
        let p = [0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1])];
        let pos = self.surface.evaluate(p[0], p[1]);
        let mut n = self.surface.normal(p[0], p[1]);
        if self.flip {
            n = [-n[0], -n[1], -n[2]];
        }
        let [ulo, uhi, vlo, vhi] = self.domain;
        self.prim
            .positions
            .push([pos[0] as f32, pos[1] as f32, pos[2] as f32]);
        if let Some(normals) = &mut self.prim.normals {
            normals.push([n[0] as f32, n[1] as f32, n[2] as f32]);
        }
        if let Some(uv) = self.prim.uvs.first_mut() {
            uv.push([
                ((p[0] - ulo) / (uhi - ulo)) as f32,
                ((p[1] - vlo) / (vhi - vlo)) as f32,
            ]);
        }
        self.params.push(p);
        (self.params.len() - 1) as u32
    }

    fn triangle(&mut self, tri: [u32; 3], depth: u32) {
        let p = tri.map(|i| self.params[i as usize]);
        let flags = p.map(|q| self.inside(q));
        let centre = [
            (p[0][0] + p[1][0] + p[2][0]) / 3.0,
            (p[0][1] + p[1][1] + p[2][1]) / 3.0,
        ];
        let centre_in = self.inside(centre);
        let uniform = flags.iter().all(|f| *f == centre_in);
        if uniform || depth == 0 {
            if centre_in {
                self.kept.extend_from_slice(&tri);
            }
            return;
        }
        if self.params.len() + 3 > MAX_TESSELLATION_VERTICES {
            if centre_in {
                self.kept.extend_from_slice(&tri);
            }
            return;
        }
        let ab = self.mid_vertex(tri[0], tri[1]);
        let bc = self.mid_vertex(tri[1], tri[2]);
        let ca = self.mid_vertex(tri[2], tri[0]);
        for t in [
            [tri[0], ab, ca],
            [ab, tri[1], bc],
            [ca, bc, tri[2]],
            [ab, bc, ca],
        ] {
            self.triangle(t, depth - 1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_quadratic() -> FbxNurbsCurve {
        FbxNurbsCurve::new(
            3,
            NurbsForm::Open,
            vec![
                [0.0, 0.0, 0.0, 1.0],
                [1.0, 1.0, 0.0, 1.0],
                [2.0, 0.0, 0.0, 1.0],
            ],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        )
    }

    #[test]
    fn element_round_trips_through_the_typed_record() {
        let c = open_quadratic();
        let el = c.to_element(7, "arc");
        assert_eq!(FbxNurbsCurve::from_element(&el), Some(c.clone()));
        assert!(!c.rational);
        assert_eq!(c.expected_knot_count(), 6);
    }

    #[test]
    fn clamped_open_curve_hits_its_end_points() {
        let curve = open_quadratic().to_curve().unwrap();
        let (lo, hi) = curve.domain();
        assert_eq!(curve.evaluate(lo), [0.0, 0.0, 0.0]);
        assert_eq!(curve.evaluate(hi), [2.0, 0.0, 0.0]);
        // Quadratic Bézier midpoint: (P0 + 2·P1 + P2) / 4.
        let mid = curve.evaluate(0.5);
        assert!((mid[0] - 1.0).abs() < 1e-12 && (mid[1] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn closed_form_wraps_the_first_point() {
        let c = FbxNurbsCurve::new(
            3,
            NurbsForm::Closed,
            vec![
                [0.0, 0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0, 1.0],
                [1.0, 1.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 1.0],
            ],
            vec![0.0, 0.0, 0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0],
        );
        assert_eq!(c.expected_knot_count(), 8);
        // 9 knots supplied — wrong for the observed relation.
        assert!(c.to_curve().is_err());
        let c = FbxNurbsCurve {
            knots: vec![0.0, 0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0, 1.0],
            ..c
        };
        let curve = c.to_curve().unwrap();
        let (lo, hi) = curve.domain();
        assert_eq!(curve.evaluate(lo), curve.evaluate(hi));
    }

    #[test]
    fn unknown_form_or_ragged_points_do_not_type() {
        let mut el = open_quadratic().to_element(1, "");
        for c in &mut el.children {
            if c.name == "Form" {
                c.properties = vec![FbxProperty::String(b"Spiral".to_vec())];
            }
        }
        assert!(FbxNurbsCurve::from_element(&el).is_none());
        let mut el = open_quadratic().to_element(1, "");
        for c in &mut el.children {
            if c.name == "Points" {
                c.properties = vec![FbxProperty::F64Array(vec![0.0; 5])];
            }
        }
        assert!(FbxNurbsCurve::from_element(&el).is_none());
    }

    #[test]
    fn mesh_surface_round_trips_the_record() {
        let c = open_quadratic();
        let mesh = c.to_mesh(Some("arc".into()));
        let prim = &mesh.primitives[0];
        assert_eq!(prim.topology, Topology::LineStrip);
        assert_eq!(prim.positions.len(), SEGMENTS_PER_SPAN as usize + 1);
        assert_eq!(FbxNurbsCurve::from_primitive(prim), Some(c));
    }

    #[test]
    fn invalid_curve_keeps_its_record_on_an_empty_primitive() {
        let c = FbxNurbsCurve::new(3, NurbsForm::Open, vec![[0.0, 0.0, 0.0, 1.0]], vec![0.0]);
        let mesh = c.to_mesh(None);
        assert!(mesh.primitives[0].positions.is_empty());
        assert_eq!(FbxNurbsCurve::from_primitive(&mesh.primitives[0]), Some(c));
    }
}
