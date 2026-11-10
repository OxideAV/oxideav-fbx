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
//! # What stays unpinned
//!
//! - Whether a rational curve's `x, y, z` are Cartesian or
//!   pre-multiplied by `w`: every staged point has `w = 1`, so the two
//!   readings coincide on the only evidence. [`FbxNurbsCurve::to_curve`]
//!   reads them as Cartesian with `w` as the weight.
//! - `Dimension` values other than `3`.
//! - Any binary-form `NurbsCurve` (the staged sample is ASCII v7200);
//!   the writer emits the same records with `d` arrays and `I`
//!   integers, the type codes the binary form uses for the
//!   corresponding records on `Geometry{Mesh}`.
//! - `NurbsSurface`, `TrimNurbsSurface`, `Line` and `Boundary` (no
//!   staged sample of any; their discriminators still surface through
//!   [`crate::geometry_kind`]).
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

use oxideav_mesh3d::{Error, Mesh, Primitive, Result, Topology};
use serde_json::Value;

use crate::binary::{FbxNode, FbxProperty};
use crate::nurbs::{MAX_TESSELLATION_VERTICES, NurbsCurve, NurbsForm};

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
    /// control array, weights read from `w` when `Rational`).
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
        let control = pts.iter().map(|p| [p[0], p[1], p[2]]).collect();
        let weights = self
            .rational
            .then(|| pts.iter().map(|p| p[3]).collect::<Vec<_>>());
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
        mesh_for_element(&element, name)
    }
}

/// Decode-side entry: the scene-model mesh for one `NurbsCurve`
/// `Geometry` element (its own records verbatim on the primitive's
/// extras, the sampled line when the record set validates).
pub(crate) fn mesh_for_element(element: &FbxNode, name: Option<String>) -> Mesh {
    let mut prim = FbxNurbsCurve::from_element(element)
        .and_then(|c| c.tessellate(SEGMENTS_PER_SPAN).ok())
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
