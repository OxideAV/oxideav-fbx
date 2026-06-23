//! `NodeAttribute` (subtype `"Light"` / `"Camera"`) surfacing onto
//! [`Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-binary-properties70.md` §6, fine-grained subtypes
//! within an FBX `NodeAttribute` record are carried by the third
//! property (the `S`-string subtype discriminator). The two well-known
//! subtypes that map onto the [`oxideav_mesh3d`] scene model are:
//!
//! - `NodeAttribute : id, "Name\x00\x01NodeAttribute", "Light"` —
//!   a punctual light source.
//! - `NodeAttribute : id, "Name\x00\x01NodeAttribute", "Camera"` —
//!   a perspective / orthographic camera.
//!
//! The attribute's parameters live inside the element's `Properties70`
//! `P`-record block per `fbx-binary-properties70.md` §4 (and the
//! ASCII counterpart in `fbx-ascii-grammar.md` §8). The well-known
//! `P`-record property names this round consumes are the FBX-SDK
//! Light / Camera attribute names observed on `NodeAttribute`
//! records:
//!
//! ### Light P-records
//!
//! | FBX `P`-record name | Type      | Maps to                                       |
//! |---------------------|-----------|-----------------------------------------------|
//! | `Color`             | ColorRGB  | `Light::*.color` (per-channel × intensity)    |
//! | `Intensity`         | Number    | `Light::*.intensity` (scaled — see below)     |
//! | `LightType`         | enum      | which `Light` variant                         |
//! | `DecayType`         | enum      | governs `range` (Linear/Quadratic vs None)    |
//! | `DecayStart`        | Number    | optional `range` (only when DecayType != None)|
//! | `InnerAngle`        | Number    | `Light::Spot.inner_cone_angle` (deg → rad / 2)|
//! | `OuterAngle`        | Number    | `Light::Spot.outer_cone_angle` (deg → rad / 2)|
//! | `CastShadows`       | bool      | round-trips through `Node::extras`            |
//!
//! `LightType` enum: `0` = Point, `1` = Directional, `2` = Spot,
//! `3` = Area (mapped to Point — area lights aren't punctual),
//! `4` = Volume (also mapped to Point).
//!
//! The FBX `Intensity` `P`-record value is a DCC-program percentage:
//! the punctual intensity is `0.01 ×` the stored `Intensity` (so a
//! stored `100` is unit intensity). We apply the same 0.01× scale
//! before storing.
//!
//! ### Camera P-records
//!
//! | FBX `P`-record name      | Type     | Maps to                                            |
//! |--------------------------|----------|----------------------------------------------------|
//! | `CameraProjectionType`   | enum     | `0` = Perspective, `1` = Orthographic              |
//! | `FieldOfView`            | Number   | horizontal FoV in degrees → `Camera::Perspective.yfov` after aspect correction |
//! | `FieldOfViewX`           | Number   | horizontal FoV (used when AspectMode = horizontal_and_vertical) |
//! | `FieldOfViewY`           | Number   | vertical   FoV (preferred when present)            |
//! | `AspectWidth`            | Number   | aspect-ratio numerator (paired with `AspectHeight`)|
//! | `AspectHeight`           | Number   | aspect-ratio denominator                           |
//! | `NearPlane`              | Number   | `Camera::*.znear`                                  |
//! | `FarPlane`               | Number   | `Camera::Perspective.zfar` (`Some`) / `Orthographic.zfar`|
//! | `OrthoZoom`              | Number   | `Camera::Orthographic.{xmag, ymag}` half-extent    |
//!
//! glTF (mesh3d's reference shape) stores vertical FoV in radians;
//! FBX stores degrees, and `FieldOfView` is *horizontal*. When only
//! `FieldOfView` is present we derive the vertical angle from the
//! horizontal one via the aspect ratio (the FBX horizontal-aperture
//! convention): `yfov = 2 * atan( tan(xfov/2) / aspect )`.
//!
//! # Connection wiring
//!
//! `NodeAttribute -> Model` `OO` connections bind a `Light` or
//! `Camera` to a scene-graph [`oxideav_mesh3d::Node`] (the `Model` that
//! owns it). Multiple `NodeAttribute` records sharing a single `Model`
//! are rare in practice; this round picks the first matching one of
//! each kind and surfaces it onto `Node::light` / `Node::camera`.
//!
//! # Not surfaced
//!
//! - Decay-curve animation channels — `Color` / `Intensity` /
//!   `FieldOfView` round-trip through the [`crate::FbxDocument`] but
//!   the `AnimationCurveNode` plumbing in [`crate::animation`] only
//!   wires up `Lcl Translation` / `Rotation` / `Scaling` /
//!   `DeformPercent`. Light / camera animation curves are a follow-up
//!   round.
//! - Area-light shape (rectangle vs sphere) — mesh3d's `Light` enum
//!   has no area variant; we map to `Point` and stash
//!   `fbx:light_type = "Area"` in `Node::extras` for downstream
//!   consumers.
//! - Camera resolution (`AspectWidth` / `AspectHeight` in pixels, the
//!   fixed-resolution aspect mode) — only the *ratio* surfaces
//!   on `Camera::Perspective.aspect_ratio`; the absolute resolution
//!   round-trips via `Node::extras["fbx:camera_resolution"]`.
//! - Aperture / film-back metadata (`FilmWidth` / `FilmHeight` /
//!   `FocalLength`) — these don't fit the glTF-style enum and are
//!   left round-tripping through the [`crate::FbxDocument`].

use std::collections::HashMap;

use oxideav_mesh3d::{Camera, Light, NodeId, Scene3D};
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::properties70::PropertyMap;

/// Walk the top-level `Objects` + `Connections` records to populate
/// `Scene3D::lights` / `Scene3D::cameras` and wire them into the
/// matching `Node::light` / `Node::camera` slots.
///
/// `model_nodes` is the per-Model FBX-id → `NodeId` lookup the scene
/// builder produced; `NodeAttribute` records that connect to a
/// `Model` whose id isn't in this map are silently ignored (they may
/// belong to a `Model` we didn't surface, e.g. an unsupported subtype).
pub fn extract_lights_and_cameras(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) {
    // 1) Index every NodeAttribute element by id.
    let mut light_attrs: HashMap<i64, &FbxNode> = HashMap::new();
    let mut camera_attrs: HashMap<i64, &FbxNode> = HashMap::new();
    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            if child.name != "NodeAttribute" {
                continue;
            }
            let id = match child.properties.first().and_then(FbxProperty::as_i64) {
                Some(i) => i,
                None => continue,
            };
            match subtype_string(child).as_deref() {
                Some("Light") => {
                    light_attrs.insert(id, child);
                }
                Some("Camera") => {
                    camera_attrs.insert(id, child);
                }
                _ => {}
            }
        }
    }

    if light_attrs.is_empty() && camera_attrs.is_empty() {
        return;
    }

    // 2) Walk Connections to find NodeAttribute -> Model OO links.
    //    Each NodeAttribute we surface is bound to exactly one Model.
    let mut light_owner: HashMap<i64, i64> = HashMap::new();
    let mut camera_owner: HashMap<i64, i64> = HashMap::new();
    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            if kind != "OO" {
                continue;
            }
            if light_attrs.contains_key(&child_id) && model_nodes.contains_key(&parent_id) {
                light_owner.entry(child_id).or_insert(parent_id);
            } else if camera_attrs.contains_key(&child_id) && model_nodes.contains_key(&parent_id) {
                camera_owner.entry(child_id).or_insert(parent_id);
            }
        }
    }

    // 3) Decode each bound NodeAttribute and attach to the Model's
    //    scene-graph Node.
    for (&attr_id, &model_fid) in &light_owner {
        let Some(node) = light_attrs.get(&attr_id) else {
            continue;
        };
        let (light, kind_tag, extras) = decode_light(node);
        let lid = scene.add_light(light);
        if let Some(&node_id) = model_nodes.get(&model_fid) {
            if let Some(n) = scene.nodes.get_mut(node_id.0 as usize) {
                n.light = Some(lid);
                if let Some(tag) = kind_tag {
                    n.extras
                        .insert("fbx:light_type".to_string(), Value::String(tag));
                }
                for (k, v) in extras {
                    n.extras.insert(k, v);
                }
            }
        }
    }
    for (&attr_id, &model_fid) in &camera_owner {
        let Some(node) = camera_attrs.get(&attr_id) else {
            continue;
        };
        let (camera, extras) = decode_camera(node);
        let cid = scene.add_camera(camera);
        if let Some(&node_id) = model_nodes.get(&model_fid) {
            if let Some(n) = scene.nodes.get_mut(node_id.0 as usize) {
                n.camera = Some(cid);
                for (k, v) in extras {
                    n.extras.insert(k, v);
                }
            }
        }
    }
}

/// Subtype-string extractor — third property of the element, per
/// `docs/3d/fbx/fbx-binary-properties70.md` §5 + §6.
fn subtype_string(node: &FbxNode) -> Option<String> {
    node.properties.get(2)?.as_str().map(str::to_owned)
}

/// Decode a `NodeAttribute : "Light"` element. Returns the typed
/// [`Light`], an optional kind-tag string for `Node::extras`
/// (`"Area"` / `"Volume"` — kinds mesh3d doesn't model so the
/// downstream consumer knows the mapping was lossy), and any extra
/// `extras` entries to attach to the owning node.
fn decode_light(node: &FbxNode) -> (Light, Option<String>, Vec<(String, Value)>) {
    let pm = PropertyMap::from_element(node);
    // Per §6 LightType: 0=Point, 1=Directional, 2=Spot, 3=Area, 4=Volume.
    let light_type = pm.as_i32("LightType").unwrap_or(0);
    let color3 = pm.as_vec3("Color").unwrap_or([1.0, 1.0, 1.0]);
    // FBX `Intensity` is a DCC percentage; punctual intensity is 0.01x.
    let intensity_raw = pm.as_f64("Intensity").unwrap_or(100.0);
    let intensity = (intensity_raw * 0.01) as f32;
    let color: [f32; 3] = [color3[0] as f32, color3[1] as f32, color3[2] as f32];

    // DecayType (0=None, 1=Linear, 2=Quadratic, 3=Cubic — the FBX
    // light decay enum). When non-None we surface DecayStart as
    // `range`; otherwise the light has no cutoff per mesh3d's
    // `range: None == physical inverse-square no cutoff` convention.
    let decay_type = pm.as_i32("DecayType").unwrap_or(2);
    let range = if decay_type != 0 {
        pm.as_f64("DecayStart").map(|v| v as f32)
    } else {
        None
    };

    let mut extras: Vec<(String, Value)> = Vec::new();
    if let Some(b) = pm.as_bool("CastShadows") {
        extras.push(("fbx:cast_shadows".to_string(), Value::Bool(b)));
    }
    if let Some(d) = pm.as_i32("DecayType") {
        extras.push(("fbx:decay_type".to_string(), Value::Number(d.into())));
    }

    let (light, kind_tag) = match light_type {
        // Directional — sun-like.
        1 => (Light::Directional { color, intensity }, None),
        // Spot.
        2 => {
            // FBX stores the full cone angle in degrees
            // (`InnerAngle` / `OuterAngle` P-records); mesh3d wants
            // the half-cone angle in radians (glTF convention).
            let inner_deg = pm.as_f64("InnerAngle").unwrap_or(0.0) as f32;
            let outer_deg = pm.as_f64("OuterAngle").unwrap_or(45.0) as f32;
            let to_half_rad = |deg: f32| (deg * std::f32::consts::PI / 180.0) * 0.5;
            let inner = to_half_rad(inner_deg);
            let mut outer = to_half_rad(outer_deg);
            // glTF requires outer > inner strictly.
            if outer <= inner {
                outer = inner + 1.0e-4;
            }
            (
                Light::Spot {
                    color,
                    intensity,
                    range,
                    inner_cone_angle: inner,
                    outer_cone_angle: outer,
                },
                None,
            )
        }
        // Area (3) / Volume (4) — mesh3d has no area-light variant.
        // Fall back to Point and tag the kind so the consumer knows.
        3 => (
            Light::Point {
                color,
                intensity,
                range,
            },
            Some("Area".to_string()),
        ),
        4 => (
            Light::Point {
                color,
                intensity,
                range,
            },
            Some("Volume".to_string()),
        ),
        // Point (0) and everything unrecognised.
        _ => (
            Light::Point {
                color,
                intensity,
                range,
            },
            None,
        ),
    };
    (light, kind_tag, extras)
}

/// Decode a `NodeAttribute : "Camera"` element.
fn decode_camera(node: &FbxNode) -> (Camera, Vec<(String, Value)>) {
    let pm = PropertyMap::from_element(node);
    // §6 `CameraProjectionType`: 0=Perspective, 1=Orthographic per the
    // FBX SDK projection-mode enum convention.
    let proj = pm.as_i32("CameraProjectionType").unwrap_or(0);
    let near = pm.as_f64("NearPlane").unwrap_or(0.1) as f32;
    let far = pm.as_f64("FarPlane").unwrap_or(1000.0) as f32;
    let aspect_w = pm.as_f64("AspectWidth");
    let aspect_h = pm.as_f64("AspectHeight");
    let aspect_ratio = match (aspect_w, aspect_h) {
        (Some(w), Some(h)) if h > 0.0 => Some((w / h) as f32),
        _ => None,
    };

    let mut extras: Vec<(String, Value)> = Vec::new();
    if let (Some(w), Some(h)) = (aspect_w, aspect_h) {
        // In the fixed-resolution aspect mode both are pixel counts —
        // round-trip the absolute pair so the downstream consumer can
        // decide what to do.
        let arr = Value::Array(vec![
            Value::Number(serde_json::Number::from_f64(w).unwrap_or_else(|| 0.into())),
            Value::Number(serde_json::Number::from_f64(h).unwrap_or_else(|| 0.into())),
        ]);
        extras.push(("fbx:camera_resolution".to_string(), arr));
    }

    if proj == 1 {
        // Orthographic. FBX's OrthoZoom is the vertical half-extent
        // in scene units; mesh3d wants `xmag`/`ymag` half-widths.
        let ortho_zoom = pm.as_f64("OrthoZoom").unwrap_or(1.0) as f32;
        let ar = aspect_ratio.unwrap_or(1.0);
        let xmag = ortho_zoom * ar;
        let ymag = ortho_zoom;
        return (
            Camera::Orthographic {
                xmag,
                ymag,
                znear: near,
                zfar: far,
            },
            extras,
        );
    }

    // Perspective. Prefer `FieldOfViewY` (vertical, matches glTF
    // `yfov` 1:1); fall back to `FieldOfView` (horizontal — derive
    // vertical via aspect ratio, the FBX horizontal-aperture
    // convention); last resort: 60° fallback.
    let yfov_deg = if let Some(yfov) = pm.as_f64("FieldOfViewY") {
        yfov as f32
    } else if let Some(xfov) = pm.as_f64("FieldOfView") {
        let xfov = xfov as f32;
        let ar = aspect_ratio.unwrap_or(16.0 / 9.0);
        let half_x = xfov * std::f32::consts::PI / 360.0;
        let half_y = (half_x.tan() / ar).atan();
        (half_y * 2.0) * 180.0 / std::f32::consts::PI
    } else if let Some(xfov) = pm.as_f64("FieldOfViewX") {
        let xfov = xfov as f32;
        let ar = aspect_ratio.unwrap_or(16.0 / 9.0);
        let half_x = xfov * std::f32::consts::PI / 360.0;
        let half_y = (half_x.tan() / ar).atan();
        (half_y * 2.0) * 180.0 / std::f32::consts::PI
    } else {
        60.0
    };
    let yfov = yfov_deg * std::f32::consts::PI / 180.0;

    // glTF spec §3.10.2.1 — `zfar = None` allows an infinite far plane;
    // we keep the explicit FBX value to preserve round-trip fidelity.
    (
        Camera::Perspective {
            aspect_ratio,
            yfov,
            znear: near,
            zfar: Some(far),
        },
        extras,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(b: &[u8]) -> FbxProperty {
        FbxProperty::String(b.to_vec())
    }

    fn p_record(
        name: &str,
        type_name: &str,
        label: &str,
        flags: &str,
        vals: Vec<FbxProperty>,
    ) -> FbxNode {
        let mut props = vec![
            s(name.as_bytes()),
            s(type_name.as_bytes()),
            s(label.as_bytes()),
            s(flags.as_bytes()),
        ];
        props.extend(vals);
        FbxNode {
            name: "P".into(),
            properties: props,
            children: Vec::new(),
        }
    }

    fn properties70(records: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: records,
        }
    }

    fn node_attribute(id: i64, subtype: &str, props70: FbxNode) -> FbxNode {
        FbxNode {
            name: "NodeAttribute".into(),
            properties: vec![
                FbxProperty::I64(id),
                s(b"NodeAttribute\x00\x01NodeAttribute"),
                s(subtype.as_bytes()),
            ],
            children: vec![props70],
        }
    }

    fn model_node(id: i64, name: &str, subtype: &str) -> FbxNode {
        let display = format!("{name}\x00\x01Model");
        FbxNode {
            name: "Model".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(display.into_bytes()),
                s(subtype.as_bytes()),
            ],
            children: Vec::new(),
        }
    }

    fn c_oo(child: i64, parent: i64) -> FbxNode {
        FbxNode {
            name: "C".into(),
            properties: vec![s(b"OO"), FbxProperty::I64(child), FbxProperty::I64(parent)],
            children: Vec::new(),
        }
    }

    fn build_doc(objects: Vec<FbxNode>, conns: Vec<FbxNode>) -> FbxDocument {
        let root = FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![
                FbxNode {
                    name: "Objects".into(),
                    properties: Vec::new(),
                    children: objects,
                },
                FbxNode {
                    name: "Connections".into(),
                    properties: Vec::new(),
                    children: conns,
                },
            ],
        };
        FbxDocument {
            version: 7500,
            root,
        }
    }

    #[test]
    fn decodes_point_light_with_color_and_intensity() {
        // LightType=0 (Point) + Color + Intensity P-records per
        // fbx-binary-properties70.md §6. Intensity is a DCC
        // percentage, scaled 0.01x to punctual intensity.
        let props70 = properties70(vec![
            p_record(
                "Color",
                "ColorRGB",
                "Color",
                "A",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(0.5),
                    FbxProperty::F64(0.25),
                ],
            ),
            p_record(
                "Intensity",
                "Number",
                "Intensity",
                "A",
                vec![FbxProperty::F64(250.0)],
            ),
            p_record("LightType", "enum", "", "", vec![FbxProperty::I32(0)]),
        ]);
        let attr = node_attribute(100, "Light", props70);
        let model = model_node(200, "Lamp", "Light");
        let doc = build_doc(vec![attr, model], vec![c_oo(100, 200)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new().with_name("Lamp"));
        model_nodes.insert(200, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        assert_eq!(scene.lights.len(), 1);
        match scene.lights[0] {
            Light::Point {
                color, intensity, ..
            } => {
                assert!((color[0] - 1.0).abs() < 1e-5);
                assert!((color[1] - 0.5).abs() < 1e-5);
                assert!((color[2] - 0.25).abs() < 1e-5);
                // 250 × 0.01 = 2.5.
                assert!((intensity - 2.5).abs() < 1e-5);
            }
            other => panic!("expected Point light, got {other:?}"),
        }
        let node = &scene.nodes[nid.0 as usize];
        assert!(node.light.is_some());
    }

    #[test]
    fn decodes_directional_light() {
        let props70 = properties70(vec![
            p_record(
                "Color",
                "ColorRGB",
                "Color",
                "A",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                ],
            ),
            p_record(
                "Intensity",
                "Number",
                "Intensity",
                "A",
                vec![FbxProperty::F64(100.0)],
            ),
            p_record("LightType", "enum", "", "", vec![FbxProperty::I32(1)]),
        ]);
        let attr = node_attribute(101, "Light", props70);
        let model = model_node(201, "Sun", "Light");
        let doc = build_doc(vec![attr, model], vec![c_oo(101, 201)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(201, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        assert!(matches!(scene.lights[0], Light::Directional { .. }));
    }

    #[test]
    fn decodes_spot_light_with_cone_angles() {
        // FBX stores the full cone in degrees (`InnerAngle` /
        // `OuterAngle`). mesh3d Spot wants half-cone in radians.
        let props70 = properties70(vec![
            p_record(
                "Color",
                "ColorRGB",
                "Color",
                "A",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                ],
            ),
            p_record(
                "Intensity",
                "Number",
                "Intensity",
                "A",
                vec![FbxProperty::F64(100.0)],
            ),
            p_record("LightType", "enum", "", "", vec![FbxProperty::I32(2)]),
            p_record(
                "InnerAngle",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(30.0)],
            ),
            p_record(
                "OuterAngle",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(60.0)],
            ),
        ]);
        let attr = node_attribute(102, "Light", props70);
        let model = model_node(202, "Spot", "Light");
        let doc = build_doc(vec![attr, model], vec![c_oo(102, 202)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(202, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        match scene.lights[0] {
            Light::Spot {
                inner_cone_angle,
                outer_cone_angle,
                ..
            } => {
                // 30 deg full -> 15 deg half -> π/12 rad
                let expected_inner = 15.0_f32.to_radians();
                let expected_outer = 30.0_f32.to_radians();
                assert!((inner_cone_angle - expected_inner).abs() < 1e-4);
                assert!((outer_cone_angle - expected_outer).abs() < 1e-4);
                assert!(outer_cone_angle > inner_cone_angle);
            }
            other => panic!("expected Spot light, got {other:?}"),
        }
    }

    #[test]
    fn area_light_falls_back_to_point_with_kind_tag() {
        let props70 = properties70(vec![p_record(
            "LightType",
            "enum",
            "",
            "",
            vec![FbxProperty::I32(3)],
        )]);
        let attr = node_attribute(103, "Light", props70);
        let model = model_node(203, "AreaLamp", "Light");
        let doc = build_doc(vec![attr, model], vec![c_oo(103, 203)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(203, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        assert!(matches!(scene.lights[0], Light::Point { .. }));
        let node = &scene.nodes[nid.0 as usize];
        assert_eq!(
            node.extras.get("fbx:light_type"),
            Some(&Value::String("Area".into()))
        );
    }

    #[test]
    fn decodes_perspective_camera_with_fov_y_priority() {
        // FieldOfViewY directly carries the vertical angle in degrees,
        // matching mesh3d's `yfov` field 1:1 (in radians).
        let props70 = properties70(vec![
            p_record(
                "CameraProjectionType",
                "enum",
                "",
                "",
                vec![FbxProperty::I32(0)],
            ),
            p_record(
                "FieldOfViewY",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(45.0)],
            ),
            p_record("NearPlane", "Number", "", "", vec![FbxProperty::F64(0.1)]),
            p_record("FarPlane", "Number", "", "", vec![FbxProperty::F64(500.0)]),
            p_record(
                "AspectWidth",
                "Number",
                "",
                "",
                vec![FbxProperty::F64(1920.0)],
            ),
            p_record(
                "AspectHeight",
                "Number",
                "",
                "",
                vec![FbxProperty::F64(1080.0)],
            ),
        ]);
        let attr = node_attribute(110, "Camera", props70);
        let model = model_node(210, "Cam", "Camera");
        let doc = build_doc(vec![attr, model], vec![c_oo(110, 210)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new().with_name("Cam"));
        model_nodes.insert(210, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        assert_eq!(scene.cameras.len(), 1);
        match scene.cameras[0] {
            Camera::Perspective {
                yfov,
                znear,
                zfar,
                aspect_ratio,
            } => {
                let expected = 45.0_f32.to_radians();
                assert!((yfov - expected).abs() < 1e-4);
                assert!((znear - 0.1).abs() < 1e-5);
                assert_eq!(zfar, Some(500.0));
                let ar = aspect_ratio.expect("AspectWidth/Height -> ratio");
                assert!((ar - 1920.0 / 1080.0).abs() < 1e-4);
            }
            other => panic!("expected Perspective camera, got {other:?}"),
        }
        let node = &scene.nodes[nid.0 as usize];
        assert!(node.camera.is_some());
        // Absolute resolution stashed in extras for roundtrip.
        assert!(node.extras.contains_key("fbx:camera_resolution"));
    }

    #[test]
    fn perspective_camera_derives_yfov_from_horizontal_fov() {
        // Only FieldOfView (horizontal) is present; we must derive
        // yfov via the aspect ratio (FBX horizontal-aperture mode):
        // yfov = 2 * atan( tan(xfov/2) / aspect ).
        let props70 = properties70(vec![
            p_record(
                "CameraProjectionType",
                "enum",
                "",
                "",
                vec![FbxProperty::I32(0)],
            ),
            p_record(
                "FieldOfView",
                "Number",
                "",
                "A",
                vec![FbxProperty::F64(90.0)],
            ),
            p_record("AspectWidth", "Number", "", "", vec![FbxProperty::F64(2.0)]),
            p_record(
                "AspectHeight",
                "Number",
                "",
                "",
                vec![FbxProperty::F64(1.0)],
            ),
        ]);
        let attr = node_attribute(111, "Camera", props70);
        let model = model_node(211, "Cam", "Camera");
        let doc = build_doc(vec![attr, model], vec![c_oo(111, 211)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(211, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        match scene.cameras[0] {
            Camera::Perspective { yfov, .. } => {
                // xfov=90deg, ar=2:1 → half_x=45deg, tan(half_x)=1,
                // half_y=atan(1/2), yfov=2*atan(0.5) rad.
                let expected = 2.0_f32 * (0.5_f32).atan();
                assert!(
                    (yfov - expected).abs() < 1e-4,
                    "yfov={yfov}, expected={expected}"
                );
            }
            other => panic!("expected Perspective, got {other:?}"),
        }
    }

    #[test]
    fn decodes_orthographic_camera() {
        let props70 = properties70(vec![
            p_record(
                "CameraProjectionType",
                "enum",
                "",
                "",
                vec![FbxProperty::I32(1)],
            ),
            p_record("OrthoZoom", "Number", "", "", vec![FbxProperty::F64(5.0)]),
            p_record("NearPlane", "Number", "", "", vec![FbxProperty::F64(0.5)]),
            p_record("FarPlane", "Number", "", "", vec![FbxProperty::F64(200.0)]),
            p_record(
                "AspectWidth",
                "Number",
                "",
                "",
                vec![FbxProperty::F64(16.0)],
            ),
            p_record(
                "AspectHeight",
                "Number",
                "",
                "",
                vec![FbxProperty::F64(9.0)],
            ),
        ]);
        let attr = node_attribute(120, "Camera", props70);
        let model = model_node(220, "OrthoCam", "Camera");
        let doc = build_doc(vec![attr, model], vec![c_oo(120, 220)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(220, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        match scene.cameras[0] {
            Camera::Orthographic {
                xmag,
                ymag,
                znear,
                zfar,
            } => {
                assert!((ymag - 5.0).abs() < 1e-4);
                assert!((xmag - 5.0 * 16.0 / 9.0).abs() < 1e-4);
                assert!((znear - 0.5).abs() < 1e-5);
                assert!((zfar - 200.0).abs() < 1e-5);
            }
            other => panic!("expected Orthographic, got {other:?}"),
        }
    }

    #[test]
    fn ignores_node_attribute_without_owning_model() {
        // NodeAttribute without an OO connection to a Model -- should
        // surface no light / camera.
        let props70 = properties70(vec![p_record(
            "LightType",
            "enum",
            "",
            "",
            vec![FbxProperty::I32(0)],
        )]);
        let attr = node_attribute(130, "Light", props70);
        let doc = build_doc(vec![attr], vec![]);

        let mut scene = Scene3D::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        // The light arena got an entry only if an owning model was
        // resolved; since none is registered, no surfaced light.
        assert!(scene.lights.is_empty());
        assert!(scene.cameras.is_empty());
    }

    #[test]
    fn ignores_unknown_subtype() {
        // NodeAttribute with subtype "LimbNode" — not a Light, not a
        // Camera; this round skips it.
        let attr = node_attribute(140, "LimbNode", properties70(vec![]));
        let model = model_node(240, "Bone", "LimbNode");
        let doc = build_doc(vec![attr, model], vec![c_oo(140, 240)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(240, nid);

        extract_lights_and_cameras(&doc, &mut scene, &model_nodes);
        assert!(scene.lights.is_empty());
        assert!(scene.cameras.is_empty());
    }
}
