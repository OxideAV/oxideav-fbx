//! `GlobalSettings` element — scene-wide axis / unit / time / ambient
//! settings surfaced onto [`oxideav_mesh3d::Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-binary-properties70.md` §4 (Properties70 grammar
//! sample) and the cubes-ascii-v7500.fbx fixture, the FBX top-level
//! `GlobalSettings` node carries a single `Properties70` block whose
//! `P` records expose scene-wide configuration:
//!
//! ```text
//! GlobalSettings:  {
//!   Version: 1000
//!   Properties70:  {
//!     P: "UpAxis", "int", "Integer", "", 1
//!     P: "UpAxisSign", "int", "Integer", "", 1
//!     P: "FrontAxis", "int", "Integer", "", 2
//!     P: "FrontAxisSign", "int", "Integer", "", 1
//!     P: "CoordAxis", "int", "Integer", "", 0
//!     P: "CoordAxisSign", "int", "Integer", "", 1
//!     P: "OriginalUpAxis", "int", "Integer", "", 1
//!     P: "OriginalUpAxisSign", "int", "Integer", "", 1
//!     P: "UnitScaleFactor", "double", "Number", "", 1
//!     P: "OriginalUnitScaleFactor", "double", "Number", "", 1
//!     P: "AmbientColor", "ColorRGB", "Color", "", 0,0,0
//!     P: "DefaultCamera", "KString", "", "", "Producer Perspective"
//!     P: "TimeMode", "enum", "", "", 11
//!     P: "TimeProtocol", "enum", "", "", 2
//!     P: "SnapOnFrameMode", "enum", "", "", 0
//!     P: "TimeSpanStart", "KTime", "Time", "", 1924423250
//!     P: "TimeSpanStop", "KTime", "Time", "", 384884650000
//!     P: "CustomFrameRate", "double", "Number", "", -1
//!     ...
//!   }
//! }
//! ```
//!
//! This module decodes that block via the existing
//! [`crate::properties70::PropertyMap`] machinery and surfaces the
//! results onto [`oxideav_mesh3d::Scene3D`] in two forms:
//!
//! 1. **Every well-known P-record** is stashed verbatim onto
//!    `Scene3D::extras` keyed `"fbx:<name>"` (the raw int / double /
//!    string / vec3 value) so a downstream consumer can apply
//!    exporter-specific auto-conversion without re-walking the
//!    document.
//! 2. **`UnitScaleFactor`** is translated to [`oxideav_mesh3d::Unit`]
//!    for the two canonical values. The FBX de-facto default is
//!    centimetres, where `UnitScaleFactor = 100.0`; a value of `1.0`
//!    denotes metre units. The mapping `100.0 → Centimetres` /
//!    `1.0 → Metres` follows directly (the `box-binary-v7400.fbx`
//!    fixture ships `UnitScaleFactor = 100.0`, confirming the
//!    centimetre convention). Other
//!    values fall back to the default `Unit::Metres` and the raw
//!    factor stays available on `extras["fbx:unit_scale_factor"]` for
//!    callers that need the literal exporter-side value.
//!
//! # Axis integers → typed [`oxideav_mesh3d::Axis`]
//!
//! Per `docs/3d/fbx/fbx-node-transform-chain.md` §4a the six axis
//! `"int"` records are three `(axis, sign)` pairs with **`0 = X`,
//! `1 = Y`, `2 = Z`** and the `*Sign` sibling carrying `+1` / `−1` as
//! a separate plain integer (pinned from the staged fixture bytes:
//! the three ASCII fixtures are Maya Y-up / Z-front / X-right and
//! their `UpAxis = 1` / `FrontAxis = 2` / `CoordAxis = 0` values are
//! mutually distinct and exhaust `{0, 1, 2}`). [`axis_from_ints`]
//! implements that table, and `extract_global_settings` now sets
//! [`Scene3D::up_axis`] / [`Scene3D::front_axis`] from the decoded
//! `UpAxis` / `FrontAxis` pairs (an absent `*Sign` record defaults to
//! `+1`, the only observed value). The FBX `FrontAxis` semantics are
//! surfaced literally — the doc's *"which axis points towards the
//! viewer"* — so a Maya export decodes as `front_axis = PosZ`. Axis
//! ints outside the documented `{0, 1, 2}` table (or a sign outside
//! `{+1, −1}`) leave the scene fields at the [`Scene3D::new`]
//! defaults; the raw ints always stay on `Scene3D::extras`. The §4a
//! structural fact — the triple declares three *distinct* axes
//! exhausting `{0, 1, 2}` — is enforced as a coherence guard:
//! `UpAxis == FrontAxis` leaves both typed fields at their defaults
//! with `extras["fbx:axis_convention_inconsistent"] =
//! "up_front_equal"`, and a `CoordAxis` colliding with a
//! self-consistent up/front pair keeps up/front typed but surfaces
//! `"coord_axis_collision"`.
//!
//! # No coordinate-system / unit-scale auto-conversion
//!
//! Per the README "Lacks" tail, coordinate-system / unit-scale
//! auto-conversion is explicitly deferred — files travel with their
//! author's axis convention and downstream consumers handle
//! re-orientation per the surfaced metadata. This module only
//! *decodes* the settings; it does not transform the geometry.

use std::collections::HashMap;

use oxideav_mesh3d::{Axis, Scene3D, Unit};
use serde_json::Value;

use crate::binary::FbxDocument;
use crate::properties70::PropertyMap;

/// FBX top-level node name for the global-settings element. Sibling
/// of `Objects`, `Connections`, `Documents`, etc. (per
/// `docs/3d/fbx/fbx-ascii-grammar.md` §7 top-level section list).
pub const GLOBAL_SETTINGS_NODE: &str = "GlobalSettings";

/// Decode `GlobalSettings` from `doc` and surface the well-known
/// P-records onto `scene`.
///
/// Returns the number of records the function recognised from the
/// fixture-grounded list (zero when the document has no `GlobalSettings`
/// node). The caller's `scene` is mutated in place — see the module
/// doc for the two-form surface (`extras` + `unit`).
pub fn extract_global_settings(doc: &FbxDocument, scene: &mut Scene3D) -> usize {
    let Some(gs) = doc.root.child(GLOBAL_SETTINGS_NODE) else {
        return 0;
    };
    let props = PropertyMap::from_element(gs);
    if props.is_empty() {
        return 0;
    }
    let mut extras = HashMap::new();
    let mut recognised = 0usize;

    // Integer-typed records (UpAxis / FrontAxis / CoordAxis triples +
    // their *Sign companions + Original* variants, plus the enum-typed
    // TimeMode / TimeProtocol / SnapOnFrameMode / CurrentTimeMarker).
    for name in [
        "UpAxis",
        "UpAxisSign",
        "FrontAxis",
        "FrontAxisSign",
        "CoordAxis",
        "CoordAxisSign",
        "OriginalUpAxis",
        "OriginalUpAxisSign",
        "TimeMode",
        "TimeProtocol",
        "SnapOnFrameMode",
        "CurrentTimeMarker",
    ] {
        if let Some(v) = props.as_i32(name) {
            extras.insert(extras_key(name), Value::Number(v.into()));
            recognised += 1;
        }
    }

    // Long-typed (`KTime`) records. The Time-span pair stays as i64 to
    // preserve every tick (`KTIME_TICKS_PER_SECOND ≈ 4.6e10`, well
    // outside f32 range) — downstream consumers can convert to seconds
    // with the same constant the animation module uses.
    for name in ["TimeSpanStart", "TimeSpanStop"] {
        if let Some(v) = ktime_long(&props, name) {
            extras.insert(extras_key(name), Value::Number(v.into()));
            recognised += 1;
        }
    }

    // Double-typed records.
    for name in [
        "UnitScaleFactor",
        "OriginalUnitScaleFactor",
        "CustomFrameRate",
    ] {
        if let Some(v) = props.as_f64(name) {
            extras.insert(extras_key(name), f64_value(v));
            recognised += 1;
        }
    }

    // String-typed (KString) records.
    for name in ["DefaultCamera"] {
        if let Some(s) = props.as_str(name) {
            extras.insert(extras_key(name), Value::String(s.to_owned()));
            recognised += 1;
        }
    }

    // Vec3-typed records (ColorRGB / Vector3D).
    for name in ["AmbientColor"] {
        if let Some(v) = props.as_vec3(name) {
            let arr = Value::Array(vec![f64_value(v[0]), f64_value(v[1]), f64_value(v[2])]);
            extras.insert(extras_key(name), arr);
            recognised += 1;
        }
    }

    // Translate `UnitScaleFactor` to `Scene3D::unit` for the two
    // values whose semantics are canonical for FBX: factor 100 →
    // centimetres (the de-facto default), factor 1 → metres. Any other
    // value leaves `scene.unit` at the [`Scene3D::new`] default; the
    // raw factor stays on extras.
    if let Some(f) = props.as_f64("UnitScaleFactor") {
        if let Some(unit) = unit_from_scale_factor(f) {
            scene.unit = unit;
        }
    }

    // Axis convention → typed `Scene3D::up_axis` / `front_axis` per
    // the `docs/3d/fbx/fbx-node-transform-chain.md` §4a integer table
    // (`0 = X`, `1 = Y`, `2 = Z`; signs are separate `+1` / `−1`
    // ints; an absent `*Sign` record defaults to `+1`, the only
    // observed value). Out-of-table values leave the `Scene3D::new`
    // defaults — the raw ints are already on `extras` above.
    //
    // §4a's structural fact — the triple declares three *distinct*
    // axes exhausting `{0, 1, 2}` (up / front / "the remaining
    // (right) axis") — is enforced as a coherence guard: a file
    // claiming `UpAxis == FrontAxis` is geometrically incoherent, so
    // neither typed field is set and
    // `extras["fbx:axis_convention_inconsistent"] = "up_front_equal"`
    // marks the raw-only fallback. A `CoordAxis` colliding with a
    // self-consistent up/front pair still types up/front (they alone
    // determine the frame) but surfaces the
    // `"coord_axis_collision"` marker.
    let in_table = |name: &str| {
        props
            .as_i32(name)
            .map(i64::from)
            .filter(|v| (0..=2).contains(v))
    };
    let (up_i, front_i, coord_i) = (
        in_table("UpAxis"),
        in_table("FrontAxis"),
        in_table("CoordAxis"),
    );
    let up_front_clash = matches!((up_i, front_i), (Some(a), Some(b)) if a == b);
    if up_front_clash {
        extras.insert(
            "fbx:axis_convention_inconsistent".to_string(),
            Value::String("up_front_equal".to_string()),
        );
    } else {
        if let Some(axis) = typed_axis(&props, "UpAxis", "UpAxisSign") {
            scene.up_axis = axis;
        }
        if let Some(axis) = typed_axis(&props, "FrontAxis", "FrontAxisSign") {
            scene.front_axis = axis;
        }
        if coord_i.is_some() && (coord_i == up_i || coord_i == front_i) {
            extras.insert(
                "fbx:axis_convention_inconsistent".to_string(),
                Value::String("coord_axis_collision".to_string()),
            );
        }
    }

    // The whole record set verbatim (`fbx:global_settings_records`)
    // — the writer merges its typed records into it by name, so
    // records outside the recognised set (`TimeMarker`, vendor
    // additions) and the producer's record order survive re-encode.
    let records = crate::properties70::own_records_json(gs);
    if !records.is_empty() {
        extras.insert(
            "fbx:global_settings_records".to_string(),
            Value::Array(records),
        );
    }

    // Merge into the scene's extras (preserves any prior entry).
    for (k, v) in extras {
        scene.extras.entry(k).or_insert(v);
    }

    recognised
}

/// Translate the FBX `UnitScaleFactor` P-record value to a typed
/// [`Unit`].
///
/// Only the two canonical values are translated:
/// `100.0` → [`Unit::Centimetres`] (the de-facto FBX default) and
/// `1.0` → [`Unit::Metres`] (the Blender "FBX Units Scale" preset).
/// Other values return `None` so the caller can decide whether to fall
/// back to the [`Scene3D::new`] default or read the raw factor from
/// `extras["fbx:unit_scale_factor"]` and scale geometry itself.
pub fn unit_from_scale_factor(f: f64) -> Option<Unit> {
    // Tolerance around the two documented values; exporters write the
    // exact double either way (no observed jitter), but using a small
    // epsilon protects against an `int 1` literal that the JSON
    // round-trip might bring through as `1.0000000000000002`.
    if (f - 100.0).abs() < 1e-6 {
        return Some(Unit::Centimetres);
    }
    if (f - 1.0).abs() < 1e-6 {
        return Some(Unit::Metres);
    }
    None
}

/// Map an FBX `(axis, sign)` integer pair to a typed [`Axis`] variant
/// per the `docs/3d/fbx/fbx-node-transform-chain.md` §4a table:
/// **`0 = X`, `1 = Y`, `2 = Z`**, with the sign a separate plain
/// integer `+1` / `−1`. Returns `None` for any value outside those
/// tables, so a caller can fall back to its own default rather than
/// guess.
pub fn axis_from_ints(axis: i64, sign: i64) -> Option<Axis> {
    Some(match (axis, sign) {
        (0, 1) => Axis::PosX,
        (0, -1) => Axis::NegX,
        (1, 1) => Axis::PosY,
        (1, -1) => Axis::NegY,
        (2, 1) => Axis::PosZ,
        (2, -1) => Axis::NegZ,
        _ => None?,
    })
}

/// Inverse of [`axis_from_ints`] — the `(axis, sign)` integer pair
/// for a typed [`Axis`] under the same §4a table.
pub fn axis_to_ints(axis: Axis) -> (i32, i32) {
    match axis {
        Axis::PosX => (0, 1),
        Axis::NegX => (0, -1),
        Axis::PosY => (1, 1),
        Axis::NegY => (1, -1),
        Axis::PosZ => (2, 1),
        Axis::NegZ => (2, -1),
    }
}

/// Read one `(axis, sign)` record pair as a typed [`Axis`]. The axis
/// record must be present and in-table; the sign record defaults to
/// `+1` when absent (the only observed value — every staged fixture
/// writes all three `*Sign` records as `1`).
fn typed_axis(props: &PropertyMap, axis_name: &str, sign_name: &str) -> Option<Axis> {
    let axis = i64::from(props.as_i32(axis_name)?);
    let sign = props.as_i32(sign_name).map_or(1, i64::from);
    axis_from_ints(axis, sign)
}

/// Pull a `KTime` value from the [`PropertyMap`].
///
/// Thin alias around [`PropertyMap::as_i64`], which preserves the
/// underlying int64 payload exactly (the `as_f64` path would lose
/// precision near the 2^53 boundary). Per
/// `docs/3d/fbx/fbx-binary-properties70.md` §4, the `KTime` typeName
/// is wire-encoded as `L` (int64).
fn ktime_long(props: &PropertyMap, name: &str) -> Option<i64> {
    props.as_i64(name)
}

/// Format an `extras` key as `"fbx:<snake_case_name>"` so the result
/// matches the convention the rest of the crate uses
/// (`fbx:bind_pose`, `fbx:shading_model`, `fbx:light_type`,
/// `fbx:camera_resolution`).
fn extras_key(p_record_name: &str) -> String {
    let mut out = String::from("fbx:");
    let mut prev_lower = false;
    for ch in p_record_name.chars() {
        if ch.is_ascii_uppercase() {
            if prev_lower {
                out.push('_');
            }
            for lo in ch.to_lowercase() {
                out.push(lo);
            }
            prev_lower = false;
        } else {
            out.push(ch);
            prev_lower = ch.is_ascii_lowercase() || ch.is_ascii_digit();
        }
    }
    out
}

/// Build a `serde_json::Value::Number` from an `f64`, falling back to
/// `Null` when the value is NaN / ±inf (which the JSON number grammar
/// can't represent).
fn f64_value(v: f64) -> Value {
    serde_json::Number::from_f64(v)
        .map(Value::Number)
        .unwrap_or(Value::Null)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::{FbxNode, FbxProperty};

    /// Build a `Properties70` `P` record with the given name, typeName,
    /// and trailing value props. Mirrors the fixture-grounded shape
    /// (`docs/3d/fbx/fbx-binary-properties70.md` §4).
    fn p(name: &str, type_name: &str, values: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(type_name.as_bytes().to_vec()),
            FbxProperty::String(b"".to_vec()),
            FbxProperty::String(b"".to_vec()),
        ];
        props.extend(values);
        FbxNode {
            name: "P".to_string(),
            properties: props,
            children: vec![],
        }
    }

    /// Wrap a list of P-record children in a `GlobalSettings`
    /// `Properties70` element and put it under a synthetic root.
    fn doc_with_globals(p_records: Vec<FbxNode>) -> FbxDocument {
        let props70 = FbxNode {
            name: "Properties70".to_string(),
            properties: vec![],
            children: p_records,
        };
        let global_settings = FbxNode {
            name: GLOBAL_SETTINGS_NODE.to_string(),
            properties: vec![],
            children: vec![props70],
        };
        FbxDocument {
            version: 7400,
            root: FbxNode {
                name: "".to_string(),
                properties: vec![],
                children: vec![global_settings],
            },
        }
    }

    #[test]
    fn missing_global_settings_returns_zero() {
        let doc = FbxDocument {
            version: 7400,
            root: FbxNode {
                name: "".to_string(),
                properties: vec![],
                children: vec![],
            },
        };
        let mut scene = Scene3D::new();
        assert_eq!(extract_global_settings(&doc, &mut scene), 0);
        assert!(scene.extras.is_empty());
    }

    #[test]
    fn empty_properties70_returns_zero() {
        let doc = doc_with_globals(vec![]);
        let mut scene = Scene3D::new();
        assert_eq!(extract_global_settings(&doc, &mut scene), 0);
        assert!(scene.extras.is_empty());
    }

    #[test]
    fn up_axis_int_surfaces_to_extras() {
        let doc = doc_with_globals(vec![p("UpAxis", "int", vec![FbxProperty::I32(1)])]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        assert_eq!(
            scene.extras.get("fbx:up_axis"),
            Some(&Value::Number(1.into()))
        );
    }

    #[test]
    fn extras_key_camelcase_to_snake_case() {
        assert_eq!(extras_key("UpAxis"), "fbx:up_axis");
        assert_eq!(extras_key("UpAxisSign"), "fbx:up_axis_sign");
        assert_eq!(extras_key("UnitScaleFactor"), "fbx:unit_scale_factor");
        assert_eq!(
            extras_key("OriginalUnitScaleFactor"),
            "fbx:original_unit_scale_factor"
        );
        assert_eq!(extras_key("AmbientColor"), "fbx:ambient_color");
        assert_eq!(extras_key("CustomFrameRate"), "fbx:custom_frame_rate");
        assert_eq!(extras_key("DefaultCamera"), "fbx:default_camera");
    }

    #[test]
    fn unit_scale_factor_100_maps_to_centimetres() {
        let doc = doc_with_globals(vec![p(
            "UnitScaleFactor",
            "double",
            vec![FbxProperty::F64(100.0)],
        )]);
        let mut scene = Scene3D::new();
        assert_eq!(scene.unit, Unit::Metres);
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        assert_eq!(scene.unit, Unit::Centimetres);
        let stored = scene.extras.get("fbx:unit_scale_factor").unwrap();
        assert_eq!(stored.as_f64(), Some(100.0));
    }

    #[test]
    fn unit_scale_factor_1_maps_to_metres() {
        let doc = doc_with_globals(vec![p(
            "UnitScaleFactor",
            "double",
            vec![FbxProperty::F64(1.0)],
        )]);
        let mut scene = Scene3D::new();
        scene.unit = Unit::Inches; // sentinel, should be overwritten
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        assert_eq!(scene.unit, Unit::Metres);
    }

    #[test]
    fn unit_scale_factor_unknown_leaves_unit_unchanged() {
        // Inches FBX uses UnitScaleFactor = 2.54 (centimeters per
        // inch). Without an explicit mapping in the docs we leave
        // scene.unit alone — the raw factor stays on extras.
        let doc = doc_with_globals(vec![p(
            "UnitScaleFactor",
            "double",
            vec![FbxProperty::F64(2.54)],
        )]);
        let mut scene = Scene3D::new();
        let original = scene.unit;
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        assert_eq!(scene.unit, original);
        let stored = scene.extras.get("fbx:unit_scale_factor").unwrap();
        assert_eq!(stored.as_f64(), Some(2.54));
    }

    #[test]
    fn ambient_color_vec3_surfaces_as_json_array() {
        let doc = doc_with_globals(vec![p(
            "AmbientColor",
            "ColorRGB",
            vec![
                FbxProperty::F64(0.1),
                FbxProperty::F64(0.2),
                FbxProperty::F64(0.3),
            ],
        )]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        let arr = scene.extras.get("fbx:ambient_color").unwrap();
        let xs = arr.as_array().unwrap();
        assert_eq!(xs.len(), 3);
        assert!((xs[0].as_f64().unwrap() - 0.1).abs() < 1e-12);
        assert!((xs[1].as_f64().unwrap() - 0.2).abs() < 1e-12);
        assert!((xs[2].as_f64().unwrap() - 0.3).abs() < 1e-12);
    }

    #[test]
    fn time_span_keeps_i64_precision() {
        // KTime stores ticks at `46_186_158_000` per second — a
        // full-day TimeSpanStop (~ 4e15 ticks) is well beyond the
        // f64-exact int range.
        let big_ticks: i64 = 4_000_000_000_000_000;
        let doc = doc_with_globals(vec![
            p(
                "TimeSpanStart",
                "KTime",
                vec![FbxProperty::I64(1_924_423_250)],
            ),
            p("TimeSpanStop", "KTime", vec![FbxProperty::I64(big_ticks)]),
        ]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 2);
        let start = scene
            .extras
            .get("fbx:time_span_start")
            .unwrap()
            .as_i64()
            .unwrap();
        assert_eq!(start, 1_924_423_250);
        let stop = scene
            .extras
            .get("fbx:time_span_stop")
            .unwrap()
            .as_i64()
            .unwrap();
        assert_eq!(stop, big_ticks);
    }

    #[test]
    fn default_camera_string_surfaces() {
        let doc = doc_with_globals(vec![p(
            "DefaultCamera",
            "KString",
            vec![FbxProperty::String(b"Producer Perspective".to_vec())],
        )]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        assert_eq!(
            scene.extras.get("fbx:default_camera"),
            Some(&Value::String("Producer Perspective".to_string()))
        );
    }

    #[test]
    fn custom_frame_rate_negative_one_surfaces() {
        // CustomFrameRate is `-1` in the cubes fixture (no custom rate
        // — fall back to TimeMode).
        let doc = doc_with_globals(vec![p(
            "CustomFrameRate",
            "double",
            vec![FbxProperty::F64(-1.0)],
        )]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 1);
        let v = scene
            .extras
            .get("fbx:custom_frame_rate")
            .unwrap()
            .as_f64()
            .unwrap();
        assert!((v - -1.0).abs() < 1e-9);
    }

    #[test]
    fn full_fixture_p_record_set_decodes() {
        // Mirrors the cubes-ascii-v7500.fbx GlobalSettings block.
        // Exercises every documented branch in one pass.
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(1)]),
            p("UpAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(2)]),
            p("FrontAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("CoordAxis", "int", vec![FbxProperty::I32(0)]),
            p("CoordAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("OriginalUpAxis", "int", vec![FbxProperty::I32(1)]),
            p("OriginalUpAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("UnitScaleFactor", "double", vec![FbxProperty::F64(1.0)]),
            p(
                "OriginalUnitScaleFactor",
                "double",
                vec![FbxProperty::F64(1.0)],
            ),
            p(
                "AmbientColor",
                "ColorRGB",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "DefaultCamera",
                "KString",
                vec![FbxProperty::String(b"Producer Perspective".to_vec())],
            ),
            p("TimeMode", "enum", vec![FbxProperty::I32(11)]),
            p("TimeProtocol", "enum", vec![FbxProperty::I32(2)]),
            p("SnapOnFrameMode", "enum", vec![FbxProperty::I32(0)]),
            p(
                "TimeSpanStart",
                "KTime",
                vec![FbxProperty::I64(1_924_423_250)],
            ),
            p(
                "TimeSpanStop",
                "KTime",
                vec![FbxProperty::I64(384_884_650_000)],
            ),
            p("CustomFrameRate", "double", vec![FbxProperty::F64(-1.0)]),
            p("CurrentTimeMarker", "int", vec![FbxProperty::I32(-1)]),
        ]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 19);
        // Spot-check every documented bucket type.
        assert_eq!(scene.unit, Unit::Metres); // factor 1.0
        assert!(scene.extras.contains_key("fbx:up_axis"));
        assert!(scene.extras.contains_key("fbx:front_axis"));
        assert!(scene.extras.contains_key("fbx:coord_axis"));
        assert!(scene.extras.contains_key("fbx:original_up_axis"));
        assert!(scene.extras.contains_key("fbx:unit_scale_factor"));
        assert!(scene.extras.contains_key("fbx:original_unit_scale_factor"));
        assert!(scene.extras.contains_key("fbx:ambient_color"));
        assert!(scene.extras.contains_key("fbx:default_camera"));
        assert!(scene.extras.contains_key("fbx:time_mode"));
        assert!(scene.extras.contains_key("fbx:time_protocol"));
        assert!(scene.extras.contains_key("fbx:snap_on_frame_mode"));
        assert!(scene.extras.contains_key("fbx:time_span_start"));
        assert!(scene.extras.contains_key("fbx:time_span_stop"));
        assert!(scene.extras.contains_key("fbx:custom_frame_rate"));
        assert!(scene.extras.contains_key("fbx:current_time_marker"));
    }

    #[test]
    fn unrecognised_record_names_get_no_typed_key() {
        // P-records this crate doesn't recognise get no typed
        // `fbx:<snake_case>` key (so a future round can opt-in to
        // more names without an extras-key collision) — they ride
        // only on the verbatim `fbx:global_settings_records` set the
        // writer re-emits.
        let doc = doc_with_globals(vec![p("SomeFutureField", "int", vec![FbxProperty::I32(7)])]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 0);
        assert_eq!(scene.extras.len(), 1);
        let raw = scene.extras["fbx:global_settings_records"]
            .as_array()
            .unwrap();
        assert_eq!(raw.len(), 1);
        assert_eq!(raw[0]["name"].as_str(), Some("SomeFutureField"));
    }

    #[test]
    fn extract_does_not_clobber_prior_extras_entry() {
        // If a downstream pre-populates `Scene3D::extras` with a key
        // colliding with the GlobalSettings naming, our walker must
        // not overwrite it.
        let doc = doc_with_globals(vec![p("UpAxis", "int", vec![FbxProperty::I32(1)])]);
        let mut scene = Scene3D::new();
        scene
            .extras
            .insert("fbx:up_axis".to_string(), Value::String("preset".into()));
        let n = extract_global_settings(&doc, &mut scene);
        // Still recognised, but the value is preserved.
        assert_eq!(n, 1);
        assert_eq!(
            scene.extras.get("fbx:up_axis"),
            Some(&Value::String("preset".into()))
        );
    }

    #[test]
    fn unit_scale_factor_epsilon_tolerated() {
        // Float-rounding around the canonical 100 — still maps to cm.
        let doc = doc_with_globals(vec![p(
            "UnitScaleFactor",
            "double",
            vec![FbxProperty::F64(100.0 + 1e-9)],
        )]);
        let mut scene = Scene3D::new();
        let _ = extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.unit, Unit::Centimetres);
    }

    #[test]
    fn unit_from_scale_factor_unknown_returns_none() {
        assert_eq!(unit_from_scale_factor(2.54), None);
        assert_eq!(unit_from_scale_factor(1000.0), None);
        assert_eq!(unit_from_scale_factor(0.0), None);
    }

    /// The §4a integer table, both directions, all twelve entries.
    #[test]
    fn axis_int_table_round_trips() {
        for axis in [
            Axis::PosX,
            Axis::NegX,
            Axis::PosY,
            Axis::NegY,
            Axis::PosZ,
            Axis::NegZ,
        ] {
            let (i, s) = axis_to_ints(axis);
            assert_eq!(axis_from_ints(i64::from(i), i64::from(s)), Some(axis));
        }
        // Doc §4a pins the assignment itself: 0 = X, 1 = Y, 2 = Z.
        assert_eq!(axis_from_ints(0, 1), Some(Axis::PosX));
        assert_eq!(axis_from_ints(1, 1), Some(Axis::PosY));
        assert_eq!(axis_from_ints(2, 1), Some(Axis::PosZ));
        // Out-of-table axis ints / signs stay None.
        assert_eq!(axis_from_ints(3, 1), None);
        assert_eq!(axis_from_ints(-1, 1), None);
        assert_eq!(axis_from_ints(1, 0), None);
        assert_eq!(axis_from_ints(1, 2), None);
    }

    /// The Maya fixture triple (`UpAxis 1` / `FrontAxis 2` /
    /// `CoordAxis 0`, all signs `+1`) decodes to the typed Y-up /
    /// Z-front convention.
    #[test]
    fn maya_axis_ints_set_typed_scene_axes() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(1)]),
            p("UpAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(2)]),
            p("FrontAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("CoordAxis", "int", vec![FbxProperty::I32(0)]),
            p("CoordAxisSign", "int", vec![FbxProperty::I32(1)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.up_axis, Axis::PosY);
        // FBX `FrontAxis` semantics surfaced literally: the axis that
        // points towards the viewer — `+Z` here, not the mesh3d
        // default `NegZ`.
        assert_eq!(scene.front_axis, Axis::PosZ);
    }

    /// A Z-up / negative-sign pair decodes through the same table.
    #[test]
    fn z_up_negative_sign_decodes() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(2)]),
            p("UpAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxisSign", "int", vec![FbxProperty::I32(-1)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.up_axis, Axis::PosZ);
        assert_eq!(scene.front_axis, Axis::NegY);
    }

    /// An absent `*Sign` record defaults to `+1` (the only observed
    /// value).
    #[test]
    fn missing_sign_record_defaults_positive() {
        let doc = doc_with_globals(vec![p("UpAxis", "int", vec![FbxProperty::I32(0)])]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.up_axis, Axis::PosX);
        // FrontAxis absent entirely → mesh3d default untouched.
        assert_eq!(scene.front_axis, Axis::NegZ);
    }

    /// §4a coherence guard: `UpAxis == FrontAxis` is geometrically
    /// incoherent (the triple declares three *distinct* axes), so
    /// neither typed field is set and the marker surfaces.
    #[test]
    fn equal_up_and_front_axes_stay_untyped_with_marker() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(1)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        // Both typed fields keep the Scene3D::new defaults.
        assert_eq!(scene.up_axis, Axis::PosY);
        assert_eq!(scene.front_axis, Axis::NegZ);
        assert_eq!(
            scene
                .extras
                .get("fbx:axis_convention_inconsistent")
                .and_then(|v| v.as_str()),
            Some("up_front_equal")
        );
        // Raw ints still ride on extras for the consumer.
        assert_eq!(
            scene.extras.get("fbx:front_axis").and_then(|v| v.as_i64()),
            Some(1)
        );
    }

    /// A `CoordAxis` colliding with a self-consistent up/front pair
    /// still types up/front (they alone determine the frame) but
    /// surfaces the collision marker.
    #[test]
    fn coord_axis_collision_keeps_up_front_typed_with_marker() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(2)]),
            p("CoordAxis", "int", vec![FbxProperty::I32(2)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.up_axis, Axis::PosY);
        assert_eq!(scene.front_axis, Axis::PosZ);
        assert_eq!(
            scene
                .extras
                .get("fbx:axis_convention_inconsistent")
                .and_then(|v| v.as_str()),
            Some("coord_axis_collision")
        );
    }

    /// The coherent Maya triple raises no marker.
    #[test]
    fn coherent_axis_triple_raises_no_marker() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(2)]),
            p("CoordAxis", "int", vec![FbxProperty::I32(0)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert!(!scene
            .extras
            .contains_key("fbx:axis_convention_inconsistent"));
        assert_eq!(scene.up_axis, Axis::PosY);
        assert_eq!(scene.front_axis, Axis::PosZ);
    }

    /// Out-of-table ints leave the typed fields at their defaults;
    /// the raw values still ride on extras.
    #[test]
    fn out_of_table_axis_ints_leave_defaults() {
        let doc = doc_with_globals(vec![
            p("UpAxis", "int", vec![FbxProperty::I32(5)]),
            p("UpAxisSign", "int", vec![FbxProperty::I32(1)]),
            p("FrontAxis", "int", vec![FbxProperty::I32(2)]),
            p("FrontAxisSign", "int", vec![FbxProperty::I32(3)]),
        ]);
        let mut scene = Scene3D::new();
        extract_global_settings(&doc, &mut scene);
        assert_eq!(scene.up_axis, Axis::PosY);
        assert_eq!(scene.front_axis, Axis::NegZ);
        assert_eq!(
            scene.extras.get("fbx:up_axis").and_then(|v| v.as_i64()),
            Some(5)
        );
    }
}
