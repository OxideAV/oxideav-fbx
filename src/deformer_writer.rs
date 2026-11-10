//! Deformer emission for the [`crate::scene_writer`] encoder — the
//! inverse of [`crate::deformer::extract_deformers`].
//!
//! Rebuilds the FBX deformer object trees the decode path walks (per
//! `docs/3d/fbx/fbx-binary-properties70.md` §5–§7):
//!
//! ```text
//! Deformer (subtype "Skin")     --(OO)-->  Geometry
//!     - Version / Link_DeformAcuracy
//! Deformer (subtype "Cluster")  --(OO)-->  Deformer{Skin}
//! Model (bone)                  --(OO)-->  Deformer (subtype "Cluster")
//!     - Version / UserData / Indexes / Weights / Transform /
//!       TransformLink / TransformAssociateModel
//!
//! Deformer (subtype "BlendShape")         --(OO)-->  Geometry
//! Deformer (subtype "BlendShapeChannel")  --(OO)-->  Deformer{BlendShape}
//! Geometry (subtype "Shape")              --(OO)-->  BlendShapeChannel
//!     - Indexes / Vertices (position deltas) / Normals (normal deltas)
//! ```
//!
//! The bone ↔ cluster edge is written with the bone `Model` as the
//! *child* of its `Cluster` — the direction the staged
//! `skin-anim-binary-v7400.fbx` fixture carries (the decode side
//! accepts either direction). The Skin element is named after the
//! [`oxideav_mesh3d::Skeleton`] and each Cluster after its bone
//! node, and the fixed-value leaves the fixture's producer writes on
//! every element (`Version`, `Link_DeformAcuracy`, `UserData`, an
//! identity `TransformAssociateModel`) are emitted verbatim so a
//! decode → encode → decode of the fixture keeps its wire census.
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
//!
//! # Static morph weights and the channel body
//!
//! Each emitted `BlendShapeChannel` carries the body the staged
//! blend-shape fixtures show (`docs/3d/fbx/fixtures/blender-blendshape/notes.md`,
//! byte pins in `docs/3d/fbx/blender-export-sweeps-2026-09.md` §4.1):
//! `Version: 100`, `DeformPercent: D` — the matching `Mesh::weights`
//! slot × [`DEFORM_PERCENT_SCALE`] (mesh3d 0..1 blend factors → FBX
//! 0..100 percentages), which the decode side divides back — and
//! `FullWeights: d[len(Indexes)]`, one percent value per Shape index
//! in `Indexes` order (round-tripped from [`FULL_WEIGHTS_KEY`], `100`
//! for an index without a recorded weight). The `BlendShape` deformer
//! carries `Version: 100`; each `Geometry{Shape}` carries an empty
//! `Properties70` and `Version: 100` ahead of its `Indexes` /
//! `Vertices` / `Normals`. One Shape per channel: the in-between
//! (multi-Shape channel) grammar is not staged.

use oxideav_mesh3d::{NodeId, Scene3D};

use crate::binary::{FbxNode, FbxProperty};
use crate::deformer::{DEFORM_PERCENT_SCALE, FULL_WEIGHTS_KEY};

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
pub(crate) fn build_deformer_objects<'a>(
    scene: &Scene3D,
    mesh_fbx_id: impl Fn(usize) -> Option<i64>,
    corner_to_vertex: impl Fn(usize) -> Option<&'a [u32]>,
    node_fbx_id: impl Fn(usize) -> Option<i64>,
    mut alloc: impl FnMut() -> i64,
) -> DeformerEmit {
    let mut out = DeformerEmit::default();
    // One `BlendShape` deformer per *geometry*: FBX hangs the
    // deformer (and its channels) off the `Geometry` element, so a
    // mesh shared by several nodes must not receive one deformer per
    // node — that would double the morph-target set on re-decode.
    // The first node with the mesh emits; later nodes reuse the same
    // channel ids (their `morph_channels` entry still binds their
    // own animation curves). Consequence: with divergent per-node
    // static overrides on ONE shared geometry, the first node's
    // effective weights are what the wire carries — FBX cannot
    // express two blend states on one shared geometry.
    let mut mesh_channels: std::collections::HashMap<usize, Vec<i64>> =
        std::collections::HashMap::new();

    for (ni, node) in scene.nodes.iter().enumerate() {
        let Some(mesh_id) = node.mesh else { continue };
        let Some(geom_fbx) = mesh_fbx_id(mesh_id.0 as usize) else {
            continue;
        };
        let Some(mesh) = scene.meshes.get(mesh_id.0 as usize) else {
            continue;
        };
        let Some(prim) = mesh.primitives.first() else {
            continue;
        };
        // Welded geometry: cluster / shape `Indexes` are control-point
        // indices, so per-corner buffers map through the corner →
        // control-point table (corners sharing a control point carry
        // the same value on a decoded mesh; the first one speaks).
        let vertex_of = corner_to_vertex(mesh_id.0 as usize);

        // ---- Skin ----------------------------------------------------
        if let Some(skel) = node
            .skin
            .and_then(|sid| scene.skins.get(sid.0 as usize))
            .and_then(|skin| scene.skeletons.get(skin.skeleton.0 as usize))
            && let (Some(joints), Some(weights)) = (&prim.joints, &prim.weights)
        {
            emit_skin(
                &mut out,
                scene,
                geom_fbx,
                skel,
                joints,
                weights,
                vertex_of,
                &node_fbx_id,
                &mut alloc,
            );
        }

        // ---- Blend shapes ---------------------------------------------
        if !prim.targets.is_empty() {
            if let Some(ids) = mesh_channels.get(&(mesh_id.0 as usize)) {
                // Geometry already carries its deformer — bind this
                // node's animation to the existing channels.
                out.morph_channels.push((NodeId(ni as u32), ids.clone()));
                continue;
            }
            let blend_fbx = alloc();
            let mut blend = deformer_element(blend_fbx, "", "BlendShape");
            blend.children.push(leaf_i32("Version", 100));
            out.objects.push(blend);
            out.connections.push(conn_oo(blend_fbx, geom_fbx));
            // Authored per-slot channel names round-tripped by the
            // decode side: the typed `Mesh::target_names` first, the
            // pre-0.0.6 `Primitive::extras["fbx:morph_target_names"]`
            // side-channel as a fallback for callers still authoring
            // it (empty / absent slots fall back to a positional
            // `Target{ti}` name).
            let slot_names: Vec<String> = if !mesh.target_names.is_empty() {
                mesh.target_names.clone()
            } else {
                prim.extras
                    .get("fbx:morph_target_names")
                    .and_then(|v| v.as_array())
                    .map(|arr| {
                        arr.iter()
                            .map(|v| v.as_str().unwrap_or_default().to_owned())
                            .collect()
                    })
                    .unwrap_or_default()
            };
            let mut channel_ids = Vec::with_capacity(prim.targets.len());
            for (ti, target) in prim.targets.iter().enumerate() {
                let slot_name = match slot_names.get(ti) {
                    Some(n) if !n.is_empty() => n.clone(),
                    _ => format!("Target{ti}"),
                };
                let channel_fbx = alloc();
                let mut channel = deformer_element(channel_fbx, &slot_name, "BlendShapeChannel");
                // Static (rest) weight: the node's *effective* blend
                // state × 100 → the channel's DeformPercent record
                // (see the "Static morph weights" module note).
                // `Scene3D::effective_morph_weights` resolves the
                // static half of the §3.7.4 node > mesh precedence
                // chain, so a `Node::weights` per-instance override
                // lands on this node's own emitted channels (the
                // deformer tree is emitted per node, which is exactly
                // the granularity the override needs); nodes without
                // an override keep the shared `Mesh::weights` rest
                // state.
                let static_weight = scene
                    .effective_morph_weights(NodeId(ni as u32))
                    .and_then(|w| w.get(ti))
                    .copied()
                    .unwrap_or_else(|| mesh.weights.get(ti).copied().unwrap_or(0.0));
                let shape_fbx = alloc();
                let (shape, shape_indexes) =
                    shape_geometry(shape_fbx, &slot_name, target, vertex_of);
                channel.children.push(leaf_i32("Version", 100));
                channel.children.push(FbxNode {
                    name: "DeformPercent".to_string(),
                    properties: vec![FbxProperty::F64(
                        f64::from(static_weight) * DEFORM_PERCENT_SCALE,
                    )],
                    children: Vec::new(),
                });
                channel.children.push(FbxNode {
                    name: "FullWeights".to_string(),
                    properties: vec![FbxProperty::F64Array(full_weights(
                        prim.extras
                            .get(FULL_WEIGHTS_KEY)
                            .and_then(|v| v.as_array())
                            .and_then(|a| a.get(ti)),
                        &shape_indexes,
                    ))],
                    children: Vec::new(),
                });
                out.objects.push(channel);
                out.connections.push(conn_oo(channel_fbx, blend_fbx));
                out.objects.push(shape);
                out.connections.push(conn_oo(shape_fbx, channel_fbx));
                channel_ids.push(channel_fbx);
            }
            mesh_channels.insert(mesh_id.0 as usize, channel_ids.clone());
            out.morph_channels.push((NodeId(ni as u32), channel_ids));
        }
    }

    out
}

/// Emit one `Deformer{Skin}` + per-joint `Deformer{Cluster}` tree.
#[allow(clippy::too_many_arguments)]
fn emit_skin(
    out: &mut DeformerEmit,
    scene: &Scene3D,
    geom_fbx: i64,
    skel: &oxideav_mesh3d::Skeleton,
    corner_joints: &[[u16; 4]],
    corner_weights: &[[f32; 4]],
    vertex_of: Option<&[u32]>,
    node_fbx_id: &impl Fn(usize) -> Option<i64>,
    alloc: &mut impl FnMut() -> i64,
) {
    // Per-joint sparse (vertex, weight) lists from the per-corner
    // top-4 buffers.
    let joints = skel.joints.as_slice();
    let inverse_binds = skel.inverse_bind_matrices.as_slice();
    let mut per_joint: Vec<Vec<(i32, f64)>> = vec![Vec::new(); joints.len()];
    let mut seen: Vec<std::collections::HashSet<u32>> = vec![Default::default(); joints.len()];
    for (corner, (j4, w4)) in corner_joints.iter().zip(corner_weights).enumerate() {
        let index = match vertex_of {
            Some(map) => match map.get(corner) {
                Some(v) => *v,
                None => continue,
            },
            None => corner as u32,
        };
        for slot in 0..4 {
            let w = w4[slot];
            if w <= 0.0 {
                continue;
            }
            let ji = j4[slot] as usize;
            if let (Some(list), Some(seen)) = (per_joint.get_mut(ji), seen.get_mut(ji))
                && seen.insert(index)
            {
                list.push((index as i32, w as f64));
            }
        }
    }

    let skin_fbx = alloc();
    let mut skin = deformer_element(skin_fbx, skel.name.as_deref().unwrap_or(""), "Skin");
    skin.children.push(leaf_i32("Version", 101));
    skin.children.push(FbxNode {
        name: "Link_DeformAcuracy".to_string(),
        properties: vec![FbxProperty::F64(50.0)],
        children: Vec::new(),
    });
    out.objects.push(skin);
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

        let cluster_name = scene
            .nodes
            .get(bone_nid.0 as usize)
            .and_then(|n| n.name.clone())
            .unwrap_or_else(|| format!("Joint{j}"));
        let mut cluster = deformer_element(cluster_fbx, &cluster_name, "Cluster");
        cluster.children.push(leaf_i32("Version", 100));
        cluster.children.push(FbxNode {
            name: "UserData".to_string(),
            properties: vec![
                FbxProperty::String(Vec::new()),
                FbxProperty::String(Vec::new()),
            ],
            children: Vec::new(),
        });
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
        cluster
            .children
            .push(mat16("TransformAssociateModel", IDENTITY_4X4));
        out.objects.push(cluster);

        // Cluster -> Skin OO order defines the decode-side joint
        // index, so it must follow the skeleton's joint order. The
        // bone edge is Model -> Cluster (fixture-observed direction).
        out.connections.push(conn_oo(cluster_fbx, skin_fbx));
        out.connections.push(conn_oo(bone_fbx, cluster_fbx));
    }
}

/// Build a `Geometry` element of subtype `"Shape"` carrying a morph
/// target's sparse deltas (`Indexes` + `Vertices` + optional
/// `Normals`, per the blend-shape tree above).
fn shape_geometry(
    id: i64,
    name: &str,
    target: &oxideav_mesh3d::MorphTarget,
    vertex_of: Option<&[u32]>,
) -> (FbxNode, Vec<i32>) {
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
    let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
    let mut rows: Vec<(i32, [f32; 3], [f32; 3])> = Vec::new();
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
        // Welded geometry: the corner's control point (first corner
        // of a shared control point speaks for it).
        let index = match vertex_of {
            Some(map) => match map.get(i) {
                Some(v) => *v,
                None => continue,
            },
            None => i as u32,
        };
        if !seen.insert(index) {
            continue;
        }
        rows.push((index as i32, p, nrm));
    }
    // Ascending `Indexes` (the order the staged Shape bodies carry).
    rows.sort_by_key(|r| r.0);
    for (index, p, nrm) in rows {
        indexes.push(index);
        vertices.extend(p.iter().map(|&c| c as f64));
        if has_normals {
            normals.extend(nrm.iter().map(|&c| c as f64));
        }
    }

    let mut children = vec![
        FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: Vec::new(),
        },
        leaf_i32("Version", 100),
        FbxNode {
            name: "Indexes".to_string(),
            properties: vec![FbxProperty::I32Array(indexes.clone())],
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

    (
        FbxNode {
            name: "Geometry".to_string(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(name_class(name, "Geometry")),
                FbxProperty::String(b"Shape".to_vec()),
            ],
            children,
        },
        indexes,
    )
}

/// A channel's `FullWeights` array for the Shape `indexes` being
/// written: each index's round-tripped weight (the slot's
/// [`FULL_WEIGHTS_KEY`] entry, matched by vertex index), else `100.0`
/// — the full-influence value of the pinned percent scale.
fn full_weights(entry: Option<&serde_json::Value>, indexes: &[i32]) -> Vec<f64> {
    let mut table: std::collections::HashMap<i64, f64> = std::collections::HashMap::new();
    if let Some(e) = entry
        && let (Some(ix), Some(w)) = (
            e.get("indexes").and_then(|v| v.as_array()),
            e.get("weights").and_then(|v| v.as_array()),
        )
    {
        for (i, w) in ix.iter().zip(w) {
            if let (Some(i), Some(w)) = (i.as_i64(), w.as_f64()) {
                table.entry(i).or_insert(w);
            }
        }
    }
    indexes
        .iter()
        .map(|i| table.get(&i64::from(*i)).copied().unwrap_or(100.0))
        .collect()
}

/// Build a `Deformer` element record with the given subtype
/// discriminator (`"Skin"` / `"Cluster"` / `"BlendShape"` /
/// `"BlendShapeChannel"` — the prop2 string the decode side matches).
fn deformer_element(id: i64, name: &str, subtype: &str) -> FbxNode {
    // Class tag: `SubDeformer` for the per-bone / per-shape children
    // (`Cluster`, `BlendShapeChannel`), `Deformer` for their parents —
    // the tags every staged deformer carries
    // (`fixtures/skin-anim-binary-v7400.fbx`,
    // `fixtures/blender-blendshape/`, sweep doc §4.1).
    let class = match subtype {
        "Cluster" | "BlendShapeChannel" => "SubDeformer",
        _ => "Deformer",
    };
    FbxNode {
        name: "Deformer".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(name, class)),
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

fn leaf_i32(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: name.to_string(),
        properties: vec![FbxProperty::I32(v)],
        children: Vec::new(),
    }
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
