//! Deformer emission for the [`crate::scene_writer`] encoder — the
//! inverse of [`crate::deformer::extract_deformers`].
//!
//! Rebuilds the FBX deformer object trees the decode path walks (per
//! `docs/3d/fbx/fbx-binary-properties70.md` §5–§7):
//!
//! ```text
//! Deformer (subtype "Skin")     --(OO)-->  Geometry
//! Deformer (subtype "Cluster")  --(OO)-->  Deformer{Skin}
//! Deformer (subtype "Cluster")  --(OO)-->  Model (bone)
//!     - Indexes / Weights / Transform / TransformLink
//!
//! Deformer (subtype "BlendShape")         --(OO)-->  Geometry
//! Deformer (subtype "BlendShapeChannel")  --(OO)-->  Deformer{BlendShape}
//! Geometry (subtype "Shape")              --(OO)-->  BlendShapeChannel
//!     - Indexes / Vertices (position deltas) / Normals (normal deltas)
//! ```
//!
//! # Bind-matrix convention
//!
//! The decode side composes the inverse-bind (geometry-to-bone) matrix
//! as `inverse(TransformLink) * Transform` from the cluster's two 4×4
//! arrays. The writer emits `Transform = inverse_bind` and
//! `TransformLink = identity`, so the decode-side composition
//! reproduces the authored [`oxideav_mesh3d::Skeleton`]
//! `inverse_bind_matrices` entry **exactly** (no matrix inversion
//! round-trips through floating point).
//!
//! # Vertex-index space
//!
//! Cluster `Indexes` and Shape `Indexes` are shared-vertex indices
//! into the Geometry's `Vertices` table. [`crate::scene_writer`]
//! emits one `Vertices` entry per corner with an identity
//! `PolygonVertexIndex`, so shared-vertex index == corner index and
//! the per-corner [`oxideav_mesh3d::Primitive`] `joints` / `weights` /
//! `targets` buffers map 1:1.
//!
//! # Lossy edges
//!
//! - The decode side keeps the **top 4** weights per vertex and
//!   normalises; buffers authored that way (including everything this
//!   crate's own decoder produces) round-trip exactly, and the decode
//!   side re-sorts each corner's joints by descending weight.
//! - `Mesh::weights` (static per-target morph weights) has no FBX
//!   home the decode side reads back — the decode path initialises
//!   every target weight to `0.0` (animation overrides at runtime) —
//!   so non-zero static weights do not survive.

use oxideav_mesh3d::{NodeId, Scene3D};

use crate::binary::{FbxNode, FbxProperty};

/// Output of the deformer emission passes: element records for
/// `Objects` + connection records for `Connections`.
#[derive(Default)]
pub(crate) struct DeformerEmit {
    pub objects: Vec<FbxNode>,
    pub connections: Vec<FbxNode>,
    /// FBX `BlendShapeChannel` element ids per scene node, in
    /// mesh-target order — the [`crate::anim_writer`] MorphWeights
    /// emitter targets these with `DeformPercent` OP connections.
    pub morph_channels: Vec<(NodeId, Vec<i64>)>,
}

/// Emit `Deformer{Skin}` + `Deformer{Cluster}` trees for every node
/// carrying a [`oxideav_mesh3d::Skin`], and
/// `Deformer{BlendShape}` + `BlendShapeChannel` + `Geometry{Shape}`
/// trees for every primitive carrying morph targets.
///
/// `mesh_fbx_id` / `node_fbx_id` resolve scene ids to the FBX element
/// ids [`crate::scene_writer`] allocated; `alloc` hands out fresh ids.
pub(crate) fn build_deformer_objects(
    scene: &Scene3D,
    mesh_fbx_id: impl Fn(usize) -> Option<i64>,
    node_fbx_id: impl Fn(usize) -> Option<i64>,
    mut alloc: impl FnMut() -> i64,
) -> DeformerEmit {
    let mut out = DeformerEmit::default();

    for (ni, node) in scene.nodes.iter().enumerate() {
        let Some(mesh_id) = node.mesh else { continue };
        let Some(geom_fbx) = mesh_fbx_id(mesh_id.0 as usize) else {
            continue;
        };
        let Some(prim) = scene
            .meshes
            .get(mesh_id.0 as usize)
            .and_then(|m| m.primitives.first())
        else {
            continue;
        };

        // ---- Skin ----------------------------------------------------
        if let Some(skel) = node
            .skin
            .and_then(|sid| scene.skins.get(sid.0 as usize))
            .and_then(|skin| scene.skeletons.get(skin.skeleton.0 as usize))
        {
            if let (Some(joints), Some(weights)) = (&prim.joints, &prim.weights) {
                emit_skin(
                    &mut out,
                    geom_fbx,
                    skel.joints.as_slice(),
                    &skel.inverse_bind_matrices,
                    joints,
                    weights,
                    &node_fbx_id,
                    &mut alloc,
                );
            }
        }

        // ---- Blend shapes ---------------------------------------------
        if !prim.targets.is_empty() {
            let blend_fbx = alloc();
            out.objects
                .push(deformer_element(blend_fbx, "", "BlendShape"));
            out.connections.push(conn_oo(blend_fbx, geom_fbx));
            let mut channel_ids = Vec::with_capacity(prim.targets.len());
            for (ti, target) in prim.targets.iter().enumerate() {
                let channel_fbx = alloc();
                out.objects.push(deformer_element(
                    channel_fbx,
                    &format!("Target{ti}"),
                    "BlendShapeChannel",
                ));
                out.connections.push(conn_oo(channel_fbx, blend_fbx));

                let shape_fbx = alloc();
                out.objects
                    .push(shape_geometry(shape_fbx, &format!("Target{ti}"), target));
                out.connections.push(conn_oo(shape_fbx, channel_fbx));
                channel_ids.push(channel_fbx);
            }
            out.morph_channels.push((NodeId(ni as u32), channel_ids));
        }
    }

    out
}

/// Emit one `Deformer{Skin}` + per-joint `Deformer{Cluster}` tree.
#[allow(clippy::too_many_arguments)]
fn emit_skin(
    out: &mut DeformerEmit,
    geom_fbx: i64,
    joints: &[NodeId],
    inverse_binds: &[[[f32; 4]; 4]],
    corner_joints: &[[u16; 4]],
    corner_weights: &[[f32; 4]],
    node_fbx_id: &impl Fn(usize) -> Option<i64>,
    alloc: &mut impl FnMut() -> i64,
) {
    // Per-joint sparse (vertex, weight) lists from the per-corner
    // top-4 buffers.
    let mut per_joint: Vec<Vec<(i32, f64)>> = vec![Vec::new(); joints.len()];
    for (corner, (j4, w4)) in corner_joints.iter().zip(corner_weights).enumerate() {
        for slot in 0..4 {
            let w = w4[slot];
            if w <= 0.0 {
                continue;
            }
            let ji = j4[slot] as usize;
            if let Some(list) = per_joint.get_mut(ji) {
                list.push((corner as i32, w as f64));
            }
        }
    }

    let skin_fbx = alloc();
    out.objects.push(deformer_element(skin_fbx, "", "Skin"));
    out.connections.push(conn_oo(skin_fbx, geom_fbx));

    for (j, bone_nid) in joints.iter().enumerate() {
        // The decode side only materialises a cluster whose bone
        // Model resolves; an unresolvable bone would silently shift
        // every later joint index, so skip the whole skin instead.
        let Some(bone_fbx) = node_fbx_id(bone_nid.0 as usize) else {
            continue;
        };
        let cluster_fbx = alloc();
        let (indexes, weights): (Vec<i32>, Vec<f64>) = per_joint[j].iter().copied().unzip();
        let inverse_bind = inverse_binds.get(j).copied().unwrap_or(IDENTITY_4X4);

        let mut cluster = deformer_element(cluster_fbx, &format!("Joint{j}"), "Cluster");
        cluster.children.push(FbxNode {
            name: "Indexes".to_string(),
            properties: vec![FbxProperty::I32Array(indexes)],
            children: Vec::new(),
        });
        cluster.children.push(FbxNode {
            name: "Weights".to_string(),
            properties: vec![FbxProperty::F64Array(weights)],
            children: Vec::new(),
        });
        // Transform = inverse-bind, TransformLink = identity — the
        // decode-side `inverse(TransformLink) * Transform` composition
        // then reproduces the inverse-bind exactly.
        cluster.children.push(mat16("Transform", inverse_bind));
        cluster.children.push(mat16("TransformLink", IDENTITY_4X4));
        out.objects.push(cluster);

        // Cluster -> Skin OO order defines the decode-side joint
        // index, so it must follow the skeleton's joint order.
        out.connections.push(conn_oo(cluster_fbx, skin_fbx));
        out.connections.push(conn_oo(cluster_fbx, bone_fbx));
    }
}

/// Build a `Geometry` element of subtype `"Shape"` carrying a morph
/// target's sparse deltas (`Indexes` + `Vertices` + optional
/// `Normals`, per the blend-shape tree above).
fn shape_geometry(id: i64, name: &str, target: &oxideav_mesh3d::MorphTarget) -> FbxNode {
    let empty: Vec<[f32; 3]> = Vec::new();
    let pos = target.position.as_ref().unwrap_or(&empty);

    // Sparse form: only vertices with a non-zero position *or* normal
    // delta are listed. The decode side requires the `Normals` array
    // (when present) to be the same length as `Vertices`, so the two
    // share one index set.
    let mut indexes: Vec<i32> = Vec::new();
    let mut vertices: Vec<f64> = Vec::new();
    let mut normals: Vec<f64> = Vec::new();
    let has_normals = target.normal.is_some();
    let n = pos.len().max(target.normal.as_ref().map_or(0, Vec::len));
    for i in 0..n {
        let p = pos.get(i).copied().unwrap_or([0.0; 3]);
        let nrm = target
            .normal
            .as_ref()
            .and_then(|v| v.get(i))
            .copied()
            .unwrap_or([0.0; 3]);
        if p == [0.0; 3] && nrm == [0.0; 3] {
            continue;
        }
        indexes.push(i as i32);
        vertices.extend(p.iter().map(|&c| c as f64));
        if has_normals {
            normals.extend(nrm.iter().map(|&c| c as f64));
        }
    }

    let mut children = vec![
        FbxNode {
            name: "Indexes".to_string(),
            properties: vec![FbxProperty::I32Array(indexes)],
            children: Vec::new(),
        },
        FbxNode {
            name: "Vertices".to_string(),
            properties: vec![FbxProperty::F64Array(vertices)],
            children: Vec::new(),
        },
    ];
    if has_normals {
        children.push(FbxNode {
            name: "Normals".to_string(),
            properties: vec![FbxProperty::F64Array(normals)],
            children: Vec::new(),
        });
    }

    FbxNode {
        name: "Geometry".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(name, "Geometry")),
            FbxProperty::String(b"Shape".to_vec()),
        ],
        children,
    }
}

/// Build a `Deformer` element record with the given subtype
/// discriminator (`"Skin"` / `"Cluster"` / `"BlendShape"` /
/// `"BlendShapeChannel"` — the prop2 string the decode side matches).
fn deformer_element(id: i64, name: &str, subtype: &str) -> FbxNode {
    FbxNode {
        name: "Deformer".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(name, "Deformer")),
            FbxProperty::String(subtype.as_bytes().to_vec()),
        ],
        children: Vec::new(),
    }
}

/// 4×4 row-major matrix as a 16-double `d`-array sub-record.
fn mat16(name: &str, m: [[f32; 4]; 4]) -> FbxNode {
    let mut flat = Vec::with_capacity(16);
    for row in &m {
        for &v in row {
            flat.push(v as f64);
        }
    }
    FbxNode {
        name: name.to_string(),
        properties: vec![FbxProperty::F64Array(flat)],
        children: Vec::new(),
    }
}

const IDENTITY_4X4: [[f32; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// `Name\x00\x01ClassTag` join (binary convention; the decode path
/// splits on the `\x00`).
fn name_class(name: &str, class: &str) -> Vec<u8> {
    let mut v = name.as_bytes().to_vec();
    v.push(0x00);
    v.push(0x01);
    v.extend_from_slice(class.as_bytes());
    v
}

fn conn_oo(child_id: i64, parent_id: i64) -> FbxNode {
    FbxNode {
        name: "C".to_string(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(child_id),
            FbxProperty::I64(parent_id),
        ],
        children: Vec::new(),
    }
}
