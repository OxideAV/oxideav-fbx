//! Deformer extraction — `Deformer{Skin}` + `Deformer{Cluster}` →
//! [`oxideav_mesh3d::Skeleton`] + [`oxideav_mesh3d::Skin`], and
//! `Deformer{BlendShape}` + `Deformer{BlendShapeChannel}` +
//! `Geometry{Shape}` → [`oxideav_mesh3d::MorphTarget`].
//!
//! An FBX skin deformer is structured as a `Deformer` object tree
//! wired through `Connections` `OO` records (per
//! `docs/3d/fbx/fbx-binary-properties70.md` §5–§7):
//!
//! ```text
//! Deformer (subtype "Skin")  --(OO)-->  Geometry
//!     ^
//!     | (OO)
//!     |
//! Deformer (subtype "Cluster")  --(OO)-->  Model (LimbNode / bone)
//!     - Indexes : i32 array — affected vertex indices
//!     - Weights : f64 array — per-affected-vertex blend weight
//!     - Transform     : 4x4 — geometry-to-world at bind
//!     - TransformLink : 4x4 — bone-to-world at bind
//! ```
//!
//! The inverse-bind (geometry-to-bone) matrix is
//! `inverse(TransformLink) * Transform`, composed from the cluster's
//! two 4×4 bind matrices above.
//!
//! For blend shapes:
//!
//! ```text
//! Deformer (subtype "BlendShape")  --(OO)-->  Geometry
//!     ^
//!     | (OO)
//!     |
//! Deformer (subtype "BlendShapeChannel")  --(OO)-->  shape Geometry (named "Shape")
//!     - DeformPercent : property holding the static weight (0..100)
//! Geometry (subtype "Shape")
//!     - Indexes : i32 array — vertex indices to offset
//!     - Vertices : f64 array — position-delta per indexed vertex
//!     - Normals  : f64 array — optional normal-delta
//! ```
//!
//! # Limitations
//!
//! - Only one [`oxideav_mesh3d::Primitive`] per [`oxideav_mesh3d::Mesh`]
//!   is touched (round 1 only emits one anyway). The skin/morph data
//!   targets that primitive's per-corner buffer.
//! - Skin weights are normalised post-hoc per the doc's *"FBX does
//!   not guarantee that skin weights are normalized"* note. We keep
//!   the **top 4 weights** per vertex (the
//!   [`oxideav_mesh3d::Primitive::weights`] / `joints` slots are
//!   `[f32; 4]` / `[u16; 4]`).
//! - In-between blend keyframes are ignored — the doc explicitly
//!   notes most callers use the convenience `target_shape` (last
//!   keyframe) field instead, and we follow the same simplification:
//!   one [`oxideav_mesh3d::MorphTarget`] per `BlendShapeChannel`,
//!   sourced from the most recent `Shape` connection.
//! - Skinning method (`SKINNING_METHOD_*`) is not surfaced; we always
//!   produce LBS-compatible weight buffers.

use std::collections::HashMap;

use oxideav_mesh3d::{
    AnimationProperty, AnimationTarget, MeshId, MorphTarget, NodeId, Scene3D, Skeleton, Skin,
};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Output of [`extract_deformers`] — the bookkeeping the scene
/// builder needs to wire animation channels to the right
/// [`AnimationTarget`].
#[derive(Debug, Default)]
pub struct DeformerOutput {
    /// FBX `BlendShapeChannel` element id → `AnimationTarget` for the
    /// `MorphWeights` property of the owning mesh's node. Empty when
    /// the file carries no blend-shape deformers, or when the bound
    /// mesh has no node attachment.
    pub channel_targets: HashMap<i64, AnimationTarget>,
}

/// Top-level entry point — walks every `Deformer` element in the
/// document, populates `Scene3D::skeletons` / `Scene3D::skins` /
/// per-primitive `joints` / `weights` / `targets`, and returns the
/// per-channel animation-target table.
pub fn extract_deformers(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    geometry_meshes: &HashMap<i64, MeshId>,
    geometry_corner_indices: &HashMap<i64, Vec<u32>>,
    model_nodes: &HashMap<i64, NodeId>,
    geometry_to_node: &HashMap<i64, NodeId>,
) -> DeformerOutput {
    let mut out = DeformerOutput::default();

    // 1) Index every Deformer by id, classify by subtype.
    let mut skin_deformers: HashMap<i64, &FbxNode> = HashMap::new();
    let mut cluster_deformers: HashMap<i64, &FbxNode> = HashMap::new();
    let mut blend_deformers: HashMap<i64, &FbxNode> = HashMap::new();
    let mut blend_channels: HashMap<i64, &FbxNode> = HashMap::new();
    let mut shape_geometries: HashMap<i64, &FbxNode> = HashMap::new();

    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            match child.name.as_str() {
                "Deformer" => {
                    let id = match element_id(child) {
                        Some(i) => i,
                        None => continue,
                    };
                    match subtype(child).as_deref() {
                        Some("Skin") => {
                            skin_deformers.insert(id, child);
                        }
                        Some("Cluster") => {
                            cluster_deformers.insert(id, child);
                        }
                        Some("BlendShape") => {
                            blend_deformers.insert(id, child);
                        }
                        Some("BlendShapeChannel") => {
                            blend_channels.insert(id, child);
                        }
                        _ => {}
                    }
                }
                "Geometry" if subtype(child).as_deref() == Some("Shape") => {
                    if let Some(id) = element_id(child) {
                        shape_geometries.insert(id, child);
                    }
                }
                _ => {}
            }
        }
    }

    if skin_deformers.is_empty() && blend_deformers.is_empty() {
        return out;
    }

    // 2) Walk Connections to build the deformer adjacency lists.
    //
    //    skin_id        -> geometry_id (OO, child=skin, parent=geom)
    //    cluster_id     -> skin_id     (OO, child=cluster, parent=skin)
    //    cluster_id     -> bone_node   (OO, child=cluster, parent=Model[LimbNode])
    //    blend_id       -> geometry_id (OO, child=blend, parent=geom)
    //    channel_id     -> blend_id    (OO, child=channel, parent=blend)
    //    shape_geom_id  -> channel_id  (OO, child=shape, parent=channel)
    let mut skin_to_geom: HashMap<i64, i64> = HashMap::new();
    let mut clusters_of_skin: HashMap<i64, Vec<i64>> = HashMap::new();
    let mut cluster_to_bone: HashMap<i64, i64> = HashMap::new();
    let mut blend_to_geom: HashMap<i64, i64> = HashMap::new();
    let mut channels_of_blend: HashMap<i64, Vec<i64>> = HashMap::new();
    let mut shapes_of_channel: HashMap<i64, Vec<i64>> = HashMap::new();

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
            // Skin -> Geometry
            if skin_deformers.contains_key(&child_id) && geometry_meshes.contains_key(&parent_id) {
                skin_to_geom.insert(child_id, parent_id);
                continue;
            }
            // Cluster -> Skin
            if cluster_deformers.contains_key(&child_id) && skin_deformers.contains_key(&parent_id)
            {
                clusters_of_skin
                    .entry(parent_id)
                    .or_default()
                    .push(child_id);
                continue;
            }
            // Cluster -> Bone (Model)
            if cluster_deformers.contains_key(&child_id) && model_nodes.contains_key(&parent_id) {
                cluster_to_bone.insert(child_id, parent_id);
                continue;
            }
            // BlendShape -> Geometry
            if blend_deformers.contains_key(&child_id) && geometry_meshes.contains_key(&parent_id) {
                blend_to_geom.insert(child_id, parent_id);
                continue;
            }
            // BlendShapeChannel -> BlendShape
            if blend_channels.contains_key(&child_id) && blend_deformers.contains_key(&parent_id) {
                channels_of_blend
                    .entry(parent_id)
                    .or_default()
                    .push(child_id);
                continue;
            }
            // Shape (Geometry subtype "Shape") -> BlendShapeChannel
            if shape_geometries.contains_key(&child_id) && blend_channels.contains_key(&parent_id) {
                shapes_of_channel
                    .entry(parent_id)
                    .or_default()
                    .push(child_id);
                continue;
            }
        }
    }

    // 3) Materialise skins.
    for (skin_id, geom_id) in &skin_to_geom {
        let mesh_id = match geometry_meshes.get(geom_id) {
            Some(m) => *m,
            None => continue,
        };
        let corner_indices = match geometry_corner_indices.get(geom_id) {
            Some(c) => c.clone(),
            None => continue,
        };
        let n_corners = corner_indices.len();

        let cluster_ids = match clusters_of_skin.get(skin_id) {
            Some(v) => v,
            None => continue,
        };
        if cluster_ids.is_empty() {
            continue;
        }

        let mut skeleton = Skeleton::new();
        // Per-corner accumulator: list of (joint_index, weight).
        let mut corner_weights: Vec<Vec<(u16, f32)>> = vec![Vec::new(); n_corners];

        for (joint_index, cluster_id) in cluster_ids.iter().enumerate() {
            let cluster_node = match cluster_deformers.get(cluster_id) {
                Some(n) => *n,
                None => continue,
            };
            let bone_node = match cluster_to_bone.get(cluster_id) {
                Some(b) => *b,
                None => continue,
            };
            let bone_node_id = match model_nodes.get(&bone_node) {
                Some(&n) => n,
                None => continue,
            };
            skeleton.joints.push(bone_node_id);

            // TransformLink + Transform → inverse-bind matrix.
            let tlink = read_4x4(cluster_node, "TransformLink").unwrap_or(IDENTITY_4X4);
            let trans = read_4x4(cluster_node, "Transform").unwrap_or(IDENTITY_4X4);
            let geom_to_bone = mat_mul(invert_affine(&tlink), trans);
            skeleton.inverse_bind_matrices.push(geom_to_bone);

            // Vertex indices + weights for this cluster.
            let indices = read_i32_array(cluster_node, "Indexes").unwrap_or_default();
            let weights = read_f64_array(cluster_node, "Weights").unwrap_or_default();
            let n_pairs = indices.len().min(weights.len());

            // Build a per-shared-vertex lookup → list of corner positions
            // referencing it. For round 1 the geometry module stored
            // `corner_indices` in shared-vertex space, so we walk it once
            // and accumulate.
            let mut shared_to_corners: HashMap<u32, Vec<usize>> = HashMap::new();
            for (corner_ix, &shared_ix) in corner_indices.iter().enumerate() {
                shared_to_corners
                    .entry(shared_ix)
                    .or_default()
                    .push(corner_ix);
            }
            for k in 0..n_pairs {
                let shared_ix = indices[k];
                if shared_ix < 0 {
                    continue;
                }
                let weight = weights[k] as f32;
                if weight == 0.0 {
                    continue;
                }
                if let Some(corners) = shared_to_corners.get(&(shared_ix as u32)) {
                    for &corner in corners {
                        if joint_index <= u16::MAX as usize {
                            corner_weights[corner].push((joint_index as u16, weight));
                        }
                    }
                }
            }
        }

        if skeleton.joints.is_empty() {
            continue;
        }

        // Pick the top 4 weights per corner, normalise.
        let mut joints_buf: Vec<[u16; 4]> = Vec::with_capacity(n_corners);
        let mut weights_buf: Vec<[f32; 4]> = Vec::with_capacity(n_corners);
        for cw in &mut corner_weights {
            cw.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
            let mut j = [0u16; 4];
            let mut w = [0.0f32; 4];
            for (slot, &(ji, wi)) in cw.iter().take(4).enumerate() {
                j[slot] = ji;
                w[slot] = wi;
            }
            let total: f32 = w.iter().sum();
            if total > f32::EPSILON {
                for slot in &mut w {
                    *slot /= total;
                }
            }
            joints_buf.push(j);
            weights_buf.push(w);
        }

        // Push the skeleton + skin to the scene and attach to the
        // mesh's owning node. Per `elements-deformers.md` the skin is
        // bound to the mesh; the renderer then walks the node tree
        // for the per-frame joint world matrices.
        let skel_id = scene.add_skeleton(skeleton);
        let skin_obj = Skin::new(skel_id);
        let skin_id = scene.add_skin(skin_obj);

        // Wire per-primitive joints + weights buffers (round 1 only
        // emits one primitive per Geometry — the first one is the
        // target).
        let mesh = &mut scene.meshes[mesh_id.0 as usize];
        if let Some(prim) = mesh.primitives.first_mut() {
            // Defensive: guard against length mismatch (should be
            // impossible given the corner-index source).
            if joints_buf.len() == prim.positions.len() {
                prim.joints = Some(joints_buf);
                prim.weights = Some(weights_buf);
            }
        }

        // Find the node bearing this mesh and tag it with the skin id.
        if let Some(&node_id) = geometry_to_node.get(geom_id) {
            if let Some(node) = scene.nodes.get_mut(node_id.0 as usize) {
                node.skin = Some(skin_id);
            }
        }
    }

    // 4) Materialise blend shapes.
    for (blend_id, geom_id) in &blend_to_geom {
        let mesh_id = match geometry_meshes.get(geom_id) {
            Some(m) => *m,
            None => continue,
        };
        let corner_indices = match geometry_corner_indices.get(geom_id) {
            Some(c) => c.clone(),
            None => continue,
        };

        let channels = match channels_of_blend.get(blend_id) {
            Some(v) => v,
            None => continue,
        };

        // For every BlendShapeChannel, take the most-recent Shape
        // (matches the doc's `target_shape` simplification) and emit
        // one MorphTarget on the bound mesh's primitive.
        let n_corners = corner_indices.len();
        for &channel_id in channels {
            let shape_ids = match shapes_of_channel.get(&channel_id) {
                Some(v) => v,
                None => continue,
            };
            let shape_id = match shape_ids.last() {
                Some(s) => *s,
                None => continue,
            };
            let shape_node = match shape_geometries.get(&shape_id) {
                Some(n) => *n,
                None => continue,
            };

            let indexes = read_i32_array(shape_node, "Indexes").unwrap_or_default();
            let raw_pos = read_f64_array(shape_node, "Vertices").unwrap_or_default();
            let raw_norm = read_f64_array(shape_node, "Normals").unwrap_or_default();

            // Build sparse delta maps keyed by shared-vertex index.
            let mut pos_delta: HashMap<u32, [f32; 3]> = HashMap::new();
            for (slot, &shared_ix) in indexes.iter().enumerate() {
                if shared_ix < 0 {
                    continue;
                }
                if slot * 3 + 2 < raw_pos.len() {
                    pos_delta.insert(
                        shared_ix as u32,
                        [
                            raw_pos[slot * 3] as f32,
                            raw_pos[slot * 3 + 1] as f32,
                            raw_pos[slot * 3 + 2] as f32,
                        ],
                    );
                }
            }
            let mut norm_delta: HashMap<u32, [f32; 3]> = HashMap::new();
            if !raw_norm.is_empty() && raw_norm.len() == raw_pos.len() {
                for (slot, &shared_ix) in indexes.iter().enumerate() {
                    if shared_ix < 0 {
                        continue;
                    }
                    if slot * 3 + 2 < raw_norm.len() {
                        norm_delta.insert(
                            shared_ix as u32,
                            [
                                raw_norm[slot * 3] as f32,
                                raw_norm[slot * 3 + 1] as f32,
                                raw_norm[slot * 3 + 2] as f32,
                            ],
                        );
                    }
                }
            }

            // Expand to per-corner.
            let mut tgt = MorphTarget::new();
            let mut pos_buf: Vec<[f32; 3]> = Vec::with_capacity(n_corners);
            for &shared_ix in &corner_indices {
                pos_buf.push(pos_delta.get(&shared_ix).copied().unwrap_or([0.0; 3]));
            }
            tgt.position = Some(pos_buf);
            if !norm_delta.is_empty() {
                let mut norm_buf: Vec<[f32; 3]> = Vec::with_capacity(n_corners);
                for &shared_ix in &corner_indices {
                    norm_buf.push(norm_delta.get(&shared_ix).copied().unwrap_or([0.0; 3]));
                }
                tgt.normal = Some(norm_buf);
            }

            let mesh = &mut scene.meshes[mesh_id.0 as usize];
            if let Some(prim) = mesh.primitives.first_mut() {
                if prim.positions.len() == n_corners {
                    prim.targets.push(tgt);
                    // Default weight 0.0 — animation overrides at
                    // runtime.
                    mesh.weights.push(0.0);

                    // Record this BlendShapeChannel's animation target so
                    // `extract_animations` can wire DeformPercent curves.
                    if let Some(&node_id) = geometry_to_node.get(geom_id) {
                        out.channel_targets.insert(
                            channel_id,
                            AnimationTarget {
                                node: node_id,
                                property: AnimationProperty::MorphWeights,
                            },
                        );
                    }
                }
            }
        }
    }

    out
}

/// Pull a `Properties70` numeric vector from the cluster's nested
/// 4x4 `Transform` / `TransformLink` arrays (16 doubles, row-major).
fn read_4x4(node: &FbxNode, name: &str) -> Option<[[f32; 4]; 4]> {
    let arr = read_f64_array(node, name)?;
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

fn read_f64_array(node: &FbxNode, name: &str) -> Option<Vec<f64>> {
    let n = node.child(name)?;
    match n.properties.first()? {
        FbxProperty::F64Array(a) => Some(a.clone()),
        FbxProperty::F32Array(a) => Some(a.iter().map(|v| *v as f64).collect()),
        _ => None,
    }
}

fn read_i32_array(node: &FbxNode, name: &str) -> Option<Vec<i32>> {
    let n = node.child(name)?;
    match n.properties.first()? {
        FbxProperty::I32Array(a) => Some(a.clone()),
        _ => None,
    }
}

fn element_id(n: &FbxNode) -> Option<i64> {
    n.properties.first().and_then(FbxProperty::as_i64)
}

fn subtype(n: &FbxNode) -> Option<String> {
    n.properties.get(2)?.as_str().map(str::to_owned)
}

const IDENTITY_4X4: [[f32; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// Row-major 4x4 multiply: `out = a * b`.
fn mat_mul(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for r in 0..4 {
        for c in 0..4 {
            let mut s = 0.0f32;
            for k in 0..4 {
                s += a[r][k] * b[k][c];
            }
            out[r][c] = s;
        }
    }
    out
}

/// Invert an affine 4x4 (assumes last row is `[0 0 0 1]`). Falls back
/// to identity on singular input — acceptable since malformed FBX
/// bind poses are rare and downstream rendering will still proceed.
fn invert_affine(m: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    // Decompose into 3x3 linear part L and translation t.
    let l = [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ];
    let t = [m[0][3], m[1][3], m[2][3]];

    // Compute cofactors / determinant for L.
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
    // Adjugate is the transpose of the cofactor matrix.
    let li = [
        [c00 * inv_det, c10 * inv_det, c20 * inv_det],
        [c01 * inv_det, c11 * inv_det, c21 * inv_det],
        [c02 * inv_det, c12 * inv_det, c22 * inv_det],
    ];
    // Inverse translation = -L^-1 * t.
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

    #[test]
    fn invert_identity_is_identity() {
        let m = IDENTITY_4X4;
        let inv = invert_affine(&m);
        for r in 0..4 {
            for c in 0..4 {
                assert!((inv[r][c] - m[r][c]).abs() < 1e-6, "{:?}\nvs\n{:?}", inv, m);
            }
        }
    }

    #[test]
    fn invert_translation() {
        let m = [
            [1.0, 0.0, 0.0, 5.0],
            [0.0, 1.0, 0.0, -3.0],
            [0.0, 0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let inv = invert_affine(&m);
        assert!((inv[0][3] + 5.0).abs() < 1e-6);
        assert!((inv[1][3] - 3.0).abs() < 1e-6);
        assert!((inv[2][3] - -2.0).abs() < 1e-6);
    }

    #[test]
    fn invert_scale_and_compose() {
        let m = [
            [2.0, 0.0, 0.0, 0.0],
            [0.0, 4.0, 0.0, 0.0],
            [0.0, 0.0, 0.5, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let inv = invert_affine(&m);
        let prod = mat_mul(m, inv);
        for (r, row) in prod.iter().enumerate().take(3) {
            for (c, &val) in row.iter().enumerate().take(3) {
                let target = if r == c { 1.0 } else { 0.0 };
                assert!((val - target).abs() < 1e-6);
            }
        }
    }
}
