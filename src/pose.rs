//! Bind-pose (`Pose` element, subtype `"BindPose"`) surfacing onto
//! [`Scene3D`].
//!
//! Per `docs/3d/fbx/ufbx/reference.html` §`ufbx_pose` /
//! §`ufbx_bone_pose`, an FBX file records the rest/bind pose of a
//! rigged skeleton as a `Pose` element whose `bone_poses[]` list pairs
//! each bone node with its world-space matrix:
//!
//! - `ufbx_pose.is_bind_pose` — set when the pose is the skeleton's
//!   bind pose (the doc's only documented pose flag).
//! - `ufbx_pose.bone_poses[]` — list of `ufbx_bone_pose`, each
//!   carrying `bone_node` (the node the pose applies to) and
//!   `bone_to_world` ("Matrix from node local space to world space").
//!   The doc notes: *"FBX only stores world transformations so this is
//!   approximated from the parent world transform."*
//!
//! The on-disk FBX 7.x record this maps to (same ufbx-field →
//! PascalCase derivation rounds 1–4 used for `Transform` /
//! `TransformLink` / `Indexes` / `Weights`, all read as direct array
//! sub-records rather than `Properties70` `P`-records) is:
//!
//! ```text
//! Objects {
//!   Pose : i64 id, "Name\x00\x01Pose", "BindPose" {
//!       PoseNode {
//!           Node   : i64 <bone Model id>
//!           Matrix : d[16]  // bone_to_world, row-major
//!       }
//!       PoseNode { ... }   // one per posed bone
//!   }
//! }
//! ```
//!
//! `Matrix` is a direct `d`-array sub-record (16 doubles, row-major),
//! read with the same shape as the deformer module's `Transform` /
//! `TransformLink` 4x4 reads — it does **not** live inside a
//! `Properties70` `P`-record, so this round stays clear of the
//! still-unstaged FBX `P`-record grammar that gates the
//! [`crate::material`] colour-factor decode.
//!
//! # What this round surfaces
//!
//! - One `node.extras["fbx:bind_pose"]` entry (16-element `f64` JSON
//!   array, row-major) per `PoseNode` whose `Node` id resolves to a
//!   scene-graph [`oxideav_mesh3d::Node`]. This round-trips the
//!   bind-pose world matrix for every bone even when the bone is not
//!   part of any [`oxideav_mesh3d::Skeleton`] (e.g. a `Pose` exported
//!   without an accompanying skin deformer).
//! - Inverse-bind refinement: a [`oxideav_mesh3d::Skeleton`] joint
//!   whose cluster did not carry an explicit `TransformLink`
//!   sub-record (the deformer module defaults that slot to identity,
//!   producing an identity inverse-bind) is back-filled from the bind
//!   pose as `inverse(bone_to_world)`. This is exactly the
//!   doc's *"FBX only stores world transformations so this is
//!   approximated"* case — a `Pose`-only rig with no per-cluster link
//!   matrix still gets a usable inverse-bind matrix.
//!
//! # Not surfaced
//!
//! - Non-bind "rest" poses (`is_bind_pose == false`) — the reference
//!   documents only the bind-pose flag's meaning; arbitrary rest poses
//!   round-trip through the [`crate::FbxDocument`] but aren't promoted
//!   onto [`Scene3D`].
//! - `bone_to_parent` (the doc's parent-space approximation) — we
//!   surface only the directly-stored world matrix; deriving the
//!   parent-space form needs the full ancestor chain and is left to a
//!   downstream consumer that already has the resolved scene graph.

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D};
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Identity 4x4 used both as the deformer module's inverse-bind
/// default sentinel (the slot we refine) and as our own safe fallback.
const IDENTITY_4X4: [[f32; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// Walk every `Pose` element (subtype `"BindPose"`) and:
///
/// 1. Stash each posed bone's world matrix into the bone
///    [`oxideav_mesh3d::Node`]'s `extras["fbx:bind_pose"]`.
/// 2. Back-fill any [`oxideav_mesh3d::Skeleton`] inverse-bind matrix
///    that is still the deformer module's identity default with
///    `inverse(bone_to_world)` from the bind pose.
///
/// `model_nodes` is the FBX `Model` id → [`NodeId`] table the scene
/// builder already produced; `PoseNode.Node` references a `Model` id,
/// so a pose entry only surfaces when its bone resolves to a live
/// scene-graph node.
pub fn extract_poses(doc: &FbxDocument, scene: &mut Scene3D, model_nodes: &HashMap<i64, NodeId>) {
    // FBX `Model` id → bind-pose world matrix (row-major). Captured
    // once so we can both write `extras` and refine skeletons.
    let mut bind_pose_world: HashMap<i64, [[f32; 4]; 4]> = HashMap::new();

    if let Some(objects) = doc.root.child("Objects") {
        for pose in objects.children_named("Pose") {
            // Only bind poses are surfaced (ufbx_pose.is_bind_pose).
            // The subtype lives in property[2] per the FBX 7.x element
            // record convention the rest of this crate relies on.
            if subtype(pose).as_deref() != Some("BindPose") {
                continue;
            }
            for pose_node in pose.children_named("PoseNode") {
                let bone_id = match read_node_id(pose_node) {
                    Some(id) => id,
                    None => continue,
                };
                let matrix = match read_4x4(pose_node, "Matrix") {
                    Some(m) => m,
                    None => continue,
                };
                // Last writer wins if a bone appears twice (malformed,
                // but tolerated like the rest of the parser).
                bind_pose_world.insert(bone_id, matrix);
            }
        }
    }

    if bind_pose_world.is_empty() {
        return;
    }

    // 1) Stash each bone's bind-pose world matrix into its node's
    //    extras so the data round-trips even without a skeleton.
    for (&bone_id, matrix) in &bind_pose_world {
        if let Some(&node_id) = model_nodes.get(&bone_id) {
            if let Some(node) = scene.nodes.get_mut(node_id.0 as usize) {
                node.extras
                    .insert("fbx:bind_pose".to_string(), mat4_to_json(matrix));
            }
        }
    }

    // 2) Refine skeleton inverse-bind matrices. A joint whose cluster
    //    omitted `TransformLink` got an identity inverse-bind from the
    //    deformer module (`unwrap_or(IDENTITY_4X4)` on both `Transform`
    //    and `TransformLink` → identity product). For those joints we
    //    substitute `inverse(bone_to_world)` from the bind pose — the
    //    doc's documented "FBX only stores world transformations"
    //    approximation. Joints that already have a real inverse-bind
    //    (cluster carried a link matrix) are left untouched.
    //
    //    Build a NodeId → bind-pose lookup keyed by the bone's scene
    //    node, since `Skeleton::joints` stores NodeIds (not FBX ids).
    let mut bind_pose_by_node: HashMap<NodeId, [[f32; 4]; 4]> = HashMap::new();
    for (&bone_id, matrix) in &bind_pose_world {
        if let Some(&node_id) = model_nodes.get(&bone_id) {
            bind_pose_by_node.insert(node_id, *matrix);
        }
    }

    for skeleton in &mut scene.skeletons {
        if skeleton.inverse_bind_matrices.len() != skeleton.joints.len() {
            continue;
        }
        for (joint_node, ibm) in skeleton
            .joints
            .iter()
            .zip(skeleton.inverse_bind_matrices.iter_mut())
        {
            if !is_identity(ibm) {
                continue;
            }
            if let Some(world) = bind_pose_by_node.get(joint_node) {
                *ibm = invert_affine(world);
            }
        }
    }
}

/// Read the `PoseNode { Node : i64 }` direct child as the referenced
/// `Model` element id.
fn read_node_id(pose_node: &FbxNode) -> Option<i64> {
    let n = pose_node.child("Node")?;
    n.properties.first().and_then(FbxProperty::as_i64)
}

/// Read a named direct child carrying a 16-double `d`-array as a
/// row-major 4x4. Mirrors the deformer module's `Transform` /
/// `TransformLink` reads.
fn read_4x4(node: &FbxNode, name: &str) -> Option<[[f32; 4]; 4]> {
    let n = node.child(name)?;
    let arr: Vec<f64> = match n.properties.first()? {
        FbxProperty::F64Array(a) => a.clone(),
        FbxProperty::F32Array(a) => a.iter().map(|v| *v as f64).collect(),
        _ => return None,
    };
    if arr.len() != 16 {
        return None;
    }
    let mut m = [[0.0f32; 4]; 4];
    for r in 0..4 {
        for c in 0..4 {
            m[r][c] = arr[r * 4 + c] as f32;
        }
    }
    Some(m)
}

fn subtype(n: &FbxNode) -> Option<String> {
    n.properties.get(2)?.as_str().map(str::to_owned)
}

fn is_identity(m: &[[f32; 4]; 4]) -> bool {
    for (r, row) in m.iter().enumerate() {
        for (c, &v) in row.iter().enumerate() {
            let target = if r == c { 1.0 } else { 0.0 };
            if (v - target).abs() > 1e-6 {
                return false;
            }
        }
    }
    true
}

fn mat4_to_json(m: &[[f32; 4]; 4]) -> Value {
    let mut flat = Vec::with_capacity(16);
    for row in m {
        for &v in row {
            flat.push(Value::Number(
                serde_json::Number::from_f64(v as f64)
                    .unwrap_or_else(|| serde_json::Number::from_f64(0.0).unwrap()),
            ));
        }
    }
    Value::Array(flat)
}

/// Invert an affine 4x4 (assumes last row is `[0 0 0 1]`). Falls back
/// to identity on singular input — matching the deformer module's
/// tolerance for malformed bind poses.
fn invert_affine(m: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let l = [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ];
    let t = [m[0][3], m[1][3], m[2][3]];

    let c00 = l[1][1] * l[2][2] - l[1][2] * l[2][1];
    let c01 = -(l[1][0] * l[2][2] - l[1][2] * l[2][0]);
    let c02 = l[1][0] * l[2][1] - l[1][1] * l[2][0];
    let det = l[0][0] * c00 + l[0][1] * c01 + l[0][2] * c02;
    if det.abs() < 1e-12 {
        return IDENTITY_4X4;
    }
    let inv_det = 1.0 / det;
    let c10 = -(l[0][1] * l[2][2] - l[0][2] * l[2][1]);
    let c11 = l[0][0] * l[2][2] - l[0][2] * l[2][0];
    let c12 = -(l[0][0] * l[2][1] - l[0][1] * l[2][0]);
    let c20 = l[0][1] * l[1][2] - l[0][2] * l[1][1];
    let c21 = -(l[0][0] * l[1][2] - l[0][2] * l[1][0]);
    let c22 = l[0][0] * l[1][1] - l[0][1] * l[1][0];
    let li = [
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ];
    let it = [
        -(li[0][0] * t[0] + li[0][1] * t[1] + li[0][2] * t[2]),
        -(li[1][0] * t[0] + li[1][1] * t[1] + li[1][2] * t[2]),
        -(li[2][0] * t[0] + li[2][1] * t[1] + li[2][2] * t[2]),
    ];
    [
        [li[0][0], li[0][1], li[0][2], it[0]],
        [li[1][0], li[1][1], li[1][2], it[1]],
        [li[2][0], li[2][1], li[2][2], it[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::FbxProperty as P;

    /// Helper: build a `PoseNode { Node : id, Matrix : d[16] }`.
    fn pose_node(bone_id: i64, matrix: [f64; 16]) -> FbxNode {
        let node_rec = FbxNode {
            name: "Node".to_string(),
            properties: vec![P::I64(bone_id)],
            children: Vec::new(),
        };
        let mat = FbxNode {
            name: "Matrix".to_string(),
            properties: vec![P::F64Array(matrix.to_vec())],
            children: Vec::new(),
        };
        FbxNode {
            name: "PoseNode".to_string(),
            properties: Vec::new(),
            children: vec![node_rec, mat],
        }
    }

    /// Build a minimal document: `Objects { Pose : "BindPose" { ... } }`.
    fn doc_with_pose(pose: FbxNode) -> FbxDocument {
        let objects = FbxNode {
            name: "Objects".to_string(),
            properties: Vec::new(),
            children: vec![pose],
        };
        let root = FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects],
        };
        FbxDocument {
            version: 7400,
            root,
        }
    }

    fn bind_pose_element(id: i64, subtype: &str, nodes: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: "Pose".to_string(),
            properties: vec![
                P::I64(id),
                P::String(b"Pose\x00\x01Pose".to_vec()),
                P::String(subtype.as_bytes().to_vec()),
            ],
            children: nodes,
        }
    }

    #[test]
    fn read_4x4_reads_row_major_doubles() {
        let m = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let pn = pose_node(7, m);
        let got = read_4x4(&pn, "Matrix").expect("matrix");
        assert_eq!(got[0], [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(got[2], [9.0, 10.0, 11.0, 12.0]);
        assert_eq!(got[3], [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn bind_pose_matrix_lands_in_node_extras() {
        // Bone Model id 100 → scene node 0.
        let mut scene = Scene3D::new();
        let n0 = scene.add_node(oxideav_mesh3d::Node::new());
        let mut model_nodes = HashMap::new();
        model_nodes.insert(100i64, n0);

        // Bind pose with a pure-translation world matrix for bone 100.
        let world = [
            1.0, 0.0, 0.0, 5.0, 0.0, 1.0, 0.0, -3.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let pose = bind_pose_element(900, "BindPose", vec![pose_node(100, world)]);
        let doc = doc_with_pose(pose);

        extract_poses(&doc, &mut scene, &model_nodes);

        let extras = &scene.nodes[n0.0 as usize].extras;
        let v = extras.get("fbx:bind_pose").expect("bind pose extra");
        let arr = v.as_array().expect("array");
        assert_eq!(arr.len(), 16);
        // Row-major: translation column is indices 3, 7, 11.
        assert_eq!(arr[3].as_f64(), Some(5.0));
        assert_eq!(arr[7].as_f64(), Some(-3.0));
        assert_eq!(arr[11].as_f64(), Some(2.0));
    }

    #[test]
    fn non_bind_pose_subtype_is_ignored() {
        let mut scene = Scene3D::new();
        let n0 = scene.add_node(oxideav_mesh3d::Node::new());
        let mut model_nodes = HashMap::new();
        model_nodes.insert(100i64, n0);

        let world = [
            1.0, 0.0, 0.0, 9.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        // Subtype "" (rest pose, not a bind pose) → not surfaced.
        let pose = bind_pose_element(900, "", vec![pose_node(100, world)]);
        let doc = doc_with_pose(pose);

        extract_poses(&doc, &mut scene, &model_nodes);
        assert!(!scene.nodes[n0.0 as usize]
            .extras
            .contains_key("fbx:bind_pose"));
    }

    #[test]
    fn identity_inverse_bind_is_refined_from_bind_pose() {
        // Skeleton with one joint at node 0, inverse-bind defaulted to
        // identity (cluster lacked TransformLink). Bind pose gives the
        // bone a translation world matrix; refinement should make the
        // inverse-bind the inverse translation.
        let mut scene = Scene3D::new();
        let n0 = scene.add_node(oxideav_mesh3d::Node::new());
        let mut skeleton = oxideav_mesh3d::Skeleton::new();
        skeleton.joints.push(n0);
        skeleton.inverse_bind_matrices.push(IDENTITY_4X4);
        scene.add_skeleton(skeleton);

        let mut model_nodes = HashMap::new();
        model_nodes.insert(100i64, n0);

        let world = [
            1.0, 0.0, 0.0, 4.0, 0.0, 1.0, 0.0, 6.0, 0.0, 0.0, 1.0, -2.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let pose = bind_pose_element(900, "BindPose", vec![pose_node(100, world)]);
        let doc = doc_with_pose(pose);

        extract_poses(&doc, &mut scene, &model_nodes);

        let ibm = scene.skeletons[0].inverse_bind_matrices[0];
        // inverse of pure translation (4, 6, -2) is (-4, -6, 2).
        assert!((ibm[0][3] + 4.0).abs() < 1e-6, "{ibm:?}");
        assert!((ibm[1][3] + 6.0).abs() < 1e-6, "{ibm:?}");
        assert!((ibm[2][3] - 2.0).abs() < 1e-6, "{ibm:?}");
    }

    #[test]
    fn real_inverse_bind_is_not_overwritten() {
        // A joint whose inverse-bind is already non-identity (cluster
        // carried a link matrix) must be left untouched even when a
        // bind pose also names the bone.
        let mut scene = Scene3D::new();
        let n0 = scene.add_node(oxideav_mesh3d::Node::new());
        let mut skeleton = oxideav_mesh3d::Skeleton::new();
        skeleton.joints.push(n0);
        let real = [
            [1.0, 0.0, 0.0, 99.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        skeleton.inverse_bind_matrices.push(real);
        scene.add_skeleton(skeleton);

        let mut model_nodes = HashMap::new();
        model_nodes.insert(100i64, n0);

        let world = [
            1.0, 0.0, 0.0, 4.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let pose = bind_pose_element(900, "BindPose", vec![pose_node(100, world)]);
        let doc = doc_with_pose(pose);

        extract_poses(&doc, &mut scene, &model_nodes);

        assert_eq!(scene.skeletons[0].inverse_bind_matrices[0][0][3], 99.0);
    }

    #[test]
    fn no_pose_element_is_a_noop() {
        let mut scene = Scene3D::new();
        let n0 = scene.add_node(oxideav_mesh3d::Node::new());
        let mut model_nodes = HashMap::new();
        model_nodes.insert(100i64, n0);

        let root = FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![FbxNode {
                name: "Objects".to_string(),
                properties: Vec::new(),
                children: Vec::new(),
            }],
        };
        let doc = FbxDocument {
            version: 7400,
            root,
        };
        extract_poses(&doc, &mut scene, &model_nodes);
        assert!(!scene.nodes[n0.0 as usize]
            .extras
            .contains_key("fbx:bind_pose"));
    }
}
