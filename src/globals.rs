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
//! # No axis auto-conversion
//!
//! The `UpAxis` / `FrontAxis` / `CoordAxis` integer enum mapping to
//! the [`oxideav_mesh3d::Axis`] (positive/negative X/Y/Z) variants is
//! **not** documented in the staged clean-room references: the
//! `UpAxis` / `*Sign` integers are observed as `P`-record values but
//! the int → axis-variant table is absent. The raw ints surface on
//! `Scene3D::extras` and
//! `Scene3D::up_axis` / `front_axis` stay at the [`Scene3D::new`]
//! defaults (`PosY` / `NegZ`).
//!
//! # No coordinate-system / unit-scale auto-conversion
//!
//! Per the README "Lacks" tail, coordinate-system / unit-scale
//! auto-conversion is explicitly deferred — files travel with their
//! author's axis convention and downstream consumers handle
//! re-orientation per the surfaced metadata. This module only
//! *decodes* the settings; it does not transform the geometry.

use std::collections::HashMap;

use oxideav_mesh3d::{Scene3D, Unit};
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
    fn unrecognised_record_names_are_ignored() {
        // P-records this round doesn't recognise round-trip through
        // FbxDocument but do not surface to Scene3D::extras (so a
        // future round can opt-in to more names without an extras-key
        // collision).
        let doc = doc_with_globals(vec![p("SomeFutureField", "int", vec![FbxProperty::I32(7)])]);
        let mut scene = Scene3D::new();
        let n = extract_global_settings(&doc, &mut scene);
        assert_eq!(n, 0);
        assert!(scene.extras.is_empty());
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
}
