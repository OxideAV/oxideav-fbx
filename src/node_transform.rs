//! `Model` node local-transform decode.
//!
//! Every FBX `Model` element carries its local-to-parent placement in
//! its `Properties70` block as the three `Lcl …` transform P-records
//! (per `docs/3d/fbx/fbx-ascii-grammar.md` §8 typeName enumeration and
//! the cubes-ascii-v7500.fbx fixture's `Model` blocks):
//!
//! ```text
//! P: "Lcl Translation", "Lcl Translation", "", "A", tx, ty, tz
//! P: "Lcl Rotation",    "Lcl Rotation",    "", "A", rx, ry, rz   (XYZ Euler degrees)
//! P: "Lcl Scaling",     "Lcl Scaling",     "", "A", sx, sy, sz
//! ```
//!
//! The scene walker (`crate::scene::build_scene`) creates one
//! [`oxideav_mesh3d::Node`] per `Model` but previously left every node
//! at [`oxideav_mesh3d::Transform::identity`], so an authored
//! placement (the fixture's four cubes each sit at a distinct
//! translation / scale) collapsed to the origin. This module fills the
//! gap: it resolves each `Model`'s `Properties70` against the
//! `ObjectType: "Model"` `PropertyTemplate` defaults (so an
//! exporter-omitted `Lcl Scaling` decodes to the template's `1,1,1`
//! exactly like an explicitly-written record — the same
//! template-resolution path `crate::material` uses) and writes the
//! resulting [`oxideav_mesh3d::Transform::Trs`] onto the node.
//!
//! ## Composition order and the documented chain
//!
//! mesh3d's `Transform::Trs` builds its matrix as `T * R * S` (see
//! [`oxideav_mesh3d::Transform::to_matrix`]). FBX's full node-transform
//! chain additionally composes rotation **offsets** / **pivots**
//! (`RotationOffset` / `RotationPivot` / `ScalingOffset` /
//! `ScalingPivot`), a `PreRotation` / `PostRotation` pair, and a
//! `RotationOrder` enum selecting the Euler axis order — the full
//! `WorldTransform = T * Roff * Rp * Rpre * R * Rpost * Rp⁻¹ * Soff *
//! Sp * S * Sp⁻¹` product. That product's *composition order* and the
//! `RotationOrder` enum-int → axis-order table are **not** documented in
//! the staged `docs/3d/fbx/` references (only the P-record names + the
//! XYZ-order Euler convention the `crate::animation` module already
//! uses). So this module applies the reduced `T * R(XYZ) * S` form
//! **only** when the chain provably reduces to it — every pivot /
//! offset is zero, `PreRotation` / `PostRotation` are zero, and
//! `RotationOrder` is `0` (XYZ, the FBX default). That is the fixture's
//! case and the overwhelmingly common authored case. When any of those
//! "extension" records is non-trivial, the node transform is left at
//! identity and the raw `Lcl` components plus a
//! `Node::extras["fbx:transform_incomplete"]` marker are surfaced so
//! the lossy reduction is detectable and the authored values are
//! recoverable — pending a docs-staging round that documents the full
//! chain composition order + the `RotationOrder` table.

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D, Transform};

use crate::animation::euler_xyz_to_quat;
use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::definitions::Definitions;
use crate::properties70::PropertyMap;

/// Decode each `Model` element's `Lcl Translation` / `Lcl Rotation` /
/// `Lcl Scaling` P-records into the owning scene-graph node's local
/// [`Transform`].
///
/// `model_nodes` maps each `Model` FBX id to the `NodeId`
/// `crate::scene::build_scene` created for it. Returns the number of
/// nodes whose transform was set to a non-identity `Trs`.
pub fn extract_node_transforms(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) -> usize {
    let definitions = Definitions::from_root(&doc.root);
    let model_template = definitions.template_for("Model");

    let mut applied = 0usize;
    let Some(objects) = doc.root.child("Objects") else {
        return 0;
    };
    for child in objects.children_named("Model") {
        let Some(id) = element_id(child) else {
            continue;
        };
        let Some(&nid) = model_nodes.get(&id) else {
            continue;
        };

        // Resolve own records over the `ObjectType: "Model"` template
        // defaults, mirroring the material decoder's resolution path.
        let own = PropertyMap::from_element(child);
        let resolved = match model_template {
            Some(t) => own.with_template_defaults(t),
            None => own,
        };

        let decoded = decode_local_transform(&resolved);
        let Some(node) = scene.nodes.get_mut(nid.0 as usize) else {
            continue;
        };

        match decoded {
            LocalTransform::Trs {
                translation,
                rotation,
                scale,
            } => {
                node.transform = Transform::Trs {
                    translation,
                    rotation,
                    scale,
                };
                if translation != [0.0; 3] || rotation != [0.0, 0.0, 0.0, 1.0] || scale != [1.0; 3]
                {
                    applied += 1;
                }
            }
            LocalTransform::Incomplete {
                translation,
                rotation_euler,
                scale,
                reason,
            } => {
                // Leave the node at identity but record the raw authored
                // components + the reason so nothing is silently dropped.
                node.extras
                    .insert("fbx:lcl_translation".to_string(), json_vec3(translation));
                node.extras
                    .insert("fbx:lcl_rotation".to_string(), json_vec3(rotation_euler));
                node.extras
                    .insert("fbx:lcl_scaling".to_string(), json_vec3(scale));
                node.extras.insert(
                    "fbx:transform_incomplete".to_string(),
                    serde_json::Value::String(reason.to_string()),
                );
            }
        }
    }
    applied
}

/// The decoded local transform of one `Model`.
enum LocalTransform {
    /// The FBX node-transform chain provably reduces to `T * R(XYZ) *
    /// S` — applied directly to the node.
    Trs {
        translation: [f32; 3],
        rotation: [f32; 4],
        scale: [f32; 3],
    },
    /// The chain carries a non-trivial pivot / offset / pre-post
    /// rotation / non-XYZ rotation order whose composition order the
    /// staged docs don't specify. The node stays at identity; the raw
    /// components are surfaced on `extras`.
    Incomplete {
        translation: [f32; 3],
        rotation_euler: [f32; 3],
        scale: [f32; 3],
        reason: &'static str,
    },
}

/// Resolve a `Model`'s effective `Properties70` into a local transform.
fn decode_local_transform(props: &PropertyMap) -> LocalTransform {
    // Template defaults: Lcl Translation 0,0,0; Lcl Rotation 0,0,0;
    // Lcl Scaling 1,1,1 (the cubes fixture's `FbxNode` template).
    let translation = props
        .as_lcl_translation("Lcl Translation")
        .unwrap_or([0.0; 3]);
    let rotation_euler = props.as_lcl_rotation("Lcl Rotation").unwrap_or([0.0; 3]);
    let scale = props
        .as_lcl_scaling("Lcl Scaling")
        .unwrap_or([1.0, 1.0, 1.0]);

    let translation = vec3_f32(translation);
    let rotation_euler = vec3_f32(rotation_euler);
    let scale = vec3_f32(scale);

    // Examine the "extension" records. The chain reduces to T * R * S
    // only when every one of them is trivial.
    if let Some(reason) = non_trivial_extension(props) {
        return LocalTransform::Incomplete {
            translation,
            rotation_euler,
            scale,
            reason,
        };
    }

    let rotation = euler_xyz_to_quat(rotation_euler);
    LocalTransform::Trs {
        translation,
        rotation,
        scale,
    }
}

/// Return `Some(reason)` when any FBX node-transform extension record
/// (pivot / offset / pre-post rotation / non-XYZ rotation order) is
/// non-trivial, so the reduced `T * R * S` form would be lossy.
fn non_trivial_extension(props: &PropertyMap) -> Option<&'static str> {
    // RotationOrder enum: 0 == XYZ (FBX default). Any other order needs
    // a Euler-order table the staged docs don't provide.
    if let Some(order) = props.as_enum("RotationOrder") {
        if order != 0 {
            return Some("rotation_order");
        }
    }
    // Pre/Post-rotation compose around `Lcl Rotation`; non-zero needs
    // the documented chain order.
    if nonzero_vec3(props.as_vector3d("PreRotation")) {
        return Some("pre_rotation");
    }
    if nonzero_vec3(props.as_vector3d("PostRotation")) {
        return Some("post_rotation");
    }
    // Rotation/scaling pivots + offsets translate the transform around
    // a pivot point; non-zero needs the documented chain order.
    for name in [
        "RotationOffset",
        "RotationPivot",
        "ScalingOffset",
        "ScalingPivot",
    ] {
        if nonzero_vec3(props.as_vector3d(name)) {
            return Some("pivot_offset");
        }
    }
    None
}

/// `true` when an optional `Vector3D` is present and not all-zero.
fn nonzero_vec3(v: Option<[f64; 3]>) -> bool {
    match v {
        Some(v) => v.iter().any(|c| *c != 0.0),
        None => false,
    }
}

fn vec3_f32(v: [f64; 3]) -> [f32; 3] {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

fn json_vec3(v: [f32; 3]) -> serde_json::Value {
    serde_json::json!([v[0], v[1], v[2]])
}

/// Read property[0] (the FBX element id) of an `Objects`-child record.
fn element_id(n: &FbxNode) -> Option<i64> {
    n.properties.first().and_then(FbxProperty::as_i64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::FbxProperty;

    /// Build a `P` record node: `P` with `[name, type, label, flags,
    /// values...]` string + numeric properties.
    fn p_vec3(name: &str, type_name: &str, v: [f64; 3]) -> FbxNode {
        FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(name.as_bytes().to_vec()),
                FbxProperty::String(type_name.as_bytes().to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"A".to_vec()),
                FbxProperty::F64(v[0]),
                FbxProperty::F64(v[1]),
                FbxProperty::F64(v[2]),
            ],
            children: vec![],
        }
    }

    fn p_enum(name: &str, value: i32) -> FbxNode {
        FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(name.as_bytes().to_vec()),
                FbxProperty::String(b"enum".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::I32(value),
            ],
            children: vec![],
        }
    }

    fn props70(records: Vec<FbxNode>) -> PropertyMap {
        let node = FbxNode {
            name: "Properties70".to_string(),
            properties: vec![],
            children: records,
        };
        PropertyMap::from_properties70(&node)
    }

    #[test]
    fn pure_trs_decodes_translation_scale() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 2.0, 3.0]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [10.0, 10.0, 10.0]),
        ]);
        match decode_local_transform(&map) {
            LocalTransform::Trs {
                translation,
                rotation,
                scale,
            } => {
                assert_eq!(translation, [1.0, 2.0, 3.0]);
                assert_eq!(scale, [10.0, 10.0, 10.0]);
                // No Lcl Rotation → identity quaternion.
                assert!((rotation[3] - 1.0).abs() < 1e-6);
                assert!(rotation[0].abs() < 1e-6);
            }
            LocalTransform::Incomplete { .. } => panic!("expected pure TRS"),
        }
    }

    #[test]
    fn missing_records_default_to_identity_trs() {
        let map = props70(vec![]);
        match decode_local_transform(&map) {
            LocalTransform::Trs {
                translation,
                rotation,
                scale,
            } => {
                assert_eq!(translation, [0.0, 0.0, 0.0]);
                assert_eq!(scale, [1.0, 1.0, 1.0]);
                assert!((rotation[3] - 1.0).abs() < 1e-6);
            }
            LocalTransform::Incomplete { .. } => panic!("expected identity TRS"),
        }
    }

    #[test]
    fn lcl_rotation_90_about_x_becomes_quat() {
        let map = props70(vec![p_vec3(
            "Lcl Rotation",
            "Lcl Rotation",
            [90.0, 0.0, 0.0],
        )]);
        match decode_local_transform(&map) {
            LocalTransform::Trs { rotation, .. } => {
                let s = std::f32::consts::FRAC_1_SQRT_2;
                assert!((rotation[0] - s).abs() < 1e-5);
                assert!((rotation[3] - s).abs() < 1e-5);
            }
            LocalTransform::Incomplete { .. } => panic!("expected TRS"),
        }
    }

    #[test]
    fn nonzero_pre_rotation_is_incomplete() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 0.0, 0.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 45.0, 0.0]),
        ]);
        match decode_local_transform(&map) {
            LocalTransform::Incomplete {
                translation,
                reason,
                ..
            } => {
                assert_eq!(translation, [1.0, 0.0, 0.0]);
                assert_eq!(reason, "pre_rotation");
            }
            LocalTransform::Trs { .. } => panic!("expected Incomplete for non-zero PreRotation"),
        }
    }

    #[test]
    fn nonzero_pivot_is_incomplete() {
        let map = props70(vec![p_vec3("RotationPivot", "Vector3D", [1.0, 0.0, 0.0])]);
        assert!(matches!(
            decode_local_transform(&map),
            LocalTransform::Incomplete {
                reason: "pivot_offset",
                ..
            }
        ));
    }

    #[test]
    fn non_xyz_rotation_order_is_incomplete() {
        let map = props70(vec![p_enum("RotationOrder", 2)]);
        assert!(matches!(
            decode_local_transform(&map),
            LocalTransform::Incomplete {
                reason: "rotation_order",
                ..
            }
        ));
    }

    #[test]
    fn xyz_rotation_order_zero_stays_trs() {
        let map = props70(vec![
            p_enum("RotationOrder", 0),
            p_vec3("Lcl Translation", "Lcl Translation", [5.0, 0.0, 0.0]),
        ]);
        assert!(matches!(
            decode_local_transform(&map),
            LocalTransform::Trs { .. }
        ));
    }

    #[test]
    fn zero_extension_records_stay_trs() {
        // All the template's zero-valued extension records present but
        // trivial — must still reduce to TRS (the fixture's case).
        let map = props70(vec![
            p_vec3("RotationOffset", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("RotationPivot", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("ScalingOffset", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("ScalingPivot", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("PostRotation", "Vector3D", [0.0, 0.0, 0.0]),
            p_enum("RotationOrder", 0),
            p_vec3("Lcl Translation", "Lcl Translation", [-1.04, 0.99, -1.04]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [10.0, 10.0, 10.0]),
        ]);
        match decode_local_transform(&map) {
            LocalTransform::Trs {
                translation, scale, ..
            } => {
                assert!((translation[0] - (-1.04)).abs() < 1e-5);
                assert_eq!(scale, [10.0, 10.0, 10.0]);
            }
            LocalTransform::Incomplete { .. } => {
                panic!("all-zero extension records must reduce to TRS")
            }
        }
    }
}
