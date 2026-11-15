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
//!     - DeformPercent : static weight (0..100) — a body leaf `D`
//!                       (staged fixtures) or a `Properties70` record
//!     - FullWeights   : d[len(Indexes)] per-vertex percent weights
//! Geometry (subtype "Shape")
//!     - Indexes : i32 array — vertex indices to offset
//!     - Vertices : f64 array — position-delta per indexed vertex
//!     - Normals  : f64 array — optional normal-delta
//! ```
//!
//! Each `BlendShapeChannel` becomes one [`oxideav_mesh3d::MorphTarget`]
//! on the bound mesh's primitive, and the channel's **static**
//! `DeformPercent` (0..100; the body leaf, or a `Properties70` record
//! when one is present) decodes — divided by
//! [`DEFORM_PERCENT_SCALE`] — into the matching
//! `oxideav_mesh3d::Mesh::weights` slot, so an authored rest blend
//! state survives into the typed scene (mesh3d weights are the direct
//! glTF-style §3.7.2.2 blend factors: `1.0` = fully applied target).
//! An absent record decodes as `0.0`; the `FbxBlendShapeChannel`
//! `PropertyTemplate` body is unobserved in the staged docs
//! (`docs/3d/fbx/fbx-property-templates.md` §5 residual), so no
//! template default is resolved.
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
//! - **In-between shapes** (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md`
//!   §4): a channel may have several `Shape` geometries connected, and
//!   `FullWeights` then holds one float64 per shape — the channel
//!   percent at which that shape is fully applied, in connection
//!   order, unsorted. The shape at 100 % becomes the
//!   [`oxideav_mesh3d::MorphTarget`]'s own deltas and every other
//!   station an entry of `MorphTarget::inbetweens` at
//!   `full_weight / 100` (each shape stores its delta against the base
//!   mesh, which is exactly the in-between model). When no shape sits
//!   at 100 % the deltas at weight 1 are read off the station table.
//!   How the vendor blends between stations is **not** staged (§4.5,
//!   §5 item 15); the typed model's reading —
//!   `MorphTarget::at_weight`, piecewise linear through the stations
//!   and the base mesh at 0 % — is what a consumer gets.
//!   [`BLEND_CHANNELS_KEY`] carries what the typed model cannot (shape
//!   names and order, a second shape at one percent, a channel without
//!   shapes). Another producer's per-vertex `FullWeights`
//!   (`blender-export-sweeps-2026-09.md` §4.1) is told apart by its
//!   length and stays on [`FULL_WEIGHTS_KEY`].
//! - Skinning method (`SKINNING_METHOD_*`) is not surfaced; we always
//!   produce LBS-compatible weight buffers.

use std::collections::HashMap;

use oxideav_mesh3d::{
    AnimationProperty, AnimationTarget, MeshId, MorphTarget, NodeId, Scene3D, Skeleton, Skin,
};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::properties70::PropertyMap;

/// FBX `DeformPercent` is a percentage (0..100, per the blend-shape
/// tree above); `oxideav_mesh3d` morph weights are the direct
/// glTF-style §3.7.2.2 blend factors (`1.0` = fully applied target).
/// Divide on decode, multiply on encode.
pub const DEFORM_PERCENT_SCALE: f64 = 100.0;

/// `Primitive::extras` key: one entry per morph-target slot (aligned
/// with `Primitive::targets`) holding the slot's `BlendShapeChannel`
/// `FullWeights` array as `{ "indexes": [i32…], "weights": [f64…] }`
/// — `weights[k]` paired with the Shape's `indexes[k]`, in percent —
/// or `null` for a channel without the record. Absent when no channel
/// carries one.
///
/// Byte pins (`docs/3d/fbx/blender-export-sweeps-2026-09.md` §4.1):
/// typecode `d`, length = `len(Indexes)`, `Indexes` order, percent
/// scale (100 = full, a vertex-group weight × 100 otherwise). The
/// staged producer's rendition is per-vertex; whether other producers
/// give the array a per-`Shape` (in-between station) meaning is not
/// pinned, so the values are carried, not applied to the deltas.
pub const FULL_WEIGHTS_KEY: &str = "fbx:blend_full_weights";

/// `Primitive::extras` key: one entry per morph-target slot (aligned
/// with `Primitive::targets`) describing the slot's
/// `BlendShapeChannel` and its target shapes:
///
/// ```json
/// { "deformer": "BS",            // the owning BlendShape's name
///   "records": [P…],             // the channel's own Properties70
///   "station_table": true,       // FullWeights is one percent per shape
///   "shapes": [                  // connection order
///     { "name": "A_full100", "full_weight": 100.0, "properties70": false,
///       "role": "primary" | "inbetween" | "extra",
///       "indexes": [2] } ] }
/// ```
///
/// `role` says where the shape went in the typed model: the target's
/// own deltas (`primary`, the shape at 100 %), one of
/// `MorphTarget::inbetweens` at `full_weight / 100` (`inbetween`), or
/// nowhere (`extra` — a second shape at one percent, or one at 0 %;
/// its `vertices` / `normals` arrays ride along verbatim). The writer
/// re-emits the channel and every shape from it.
pub const BLEND_CHANNELS_KEY: &str = "fbx:blend_channels";

/// One `BlendShapeChannel`'s animation binding: which scene node's
/// `MorphWeights` property its `DeformPercent` curves drive, and which
/// morph-target slot (index into the bound mesh's
/// `oxideav_mesh3d::Mesh::weights` / per-primitive `targets`) the
/// channel occupies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MorphChannelBinding {
    /// The `MorphWeights` target on the mesh-bearing node.
    pub target: AnimationTarget,
    /// The channel's morph-target slot on the bound mesh.
    pub target_index: usize,
}

/// Output of [`extract_deformers`] — the bookkeeping the scene
/// builder needs to wire animation channels to the right
/// [`AnimationTarget`].
#[derive(Debug, Default)]
pub struct DeformerOutput {
    /// FBX `BlendShapeChannel` element id → the channel's
    /// [`MorphChannelBinding`]. Empty when the file carries no
    /// blend-shape deformers, or when the bound mesh has no node
    /// attachment.
    pub channel_targets: HashMap<i64, MorphChannelBinding>,
    /// Per mesh-bearing node: the static (rest) morph weights in
    /// target order — the decoded `DeformPercent` records divided by
    /// [`DEFORM_PERCENT_SCALE`], mirroring the bound mesh's
    /// `oxideav_mesh3d::Mesh::weights`. The animation module uses
    /// these to fill unanimated slots of a multi-target
    /// `MorphWeights` sampler.
    pub node_morph_statics: HashMap<NodeId, Vec<f32>>,
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
    // Document order of the Skin deformers — skeleton / skin ids are
    // positional, so materialisation walks them deterministically.
    let mut skin_order: Vec<i64> = Vec::new();
    let mut cluster_deformers: HashMap<i64, &FbxNode> = HashMap::new();
    let mut blend_deformers: HashMap<i64, &FbxNode> = HashMap::new();
    // Document order of the BlendShape deformers — morph-target slots
    // are positional (mesh.weights / prim.targets indices), so the
    // materialisation below must walk deformers deterministically.
    let mut blend_order: Vec<i64> = Vec::new();
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
                            if skin_deformers.insert(id, child).is_none() {
                                skin_order.push(id);
                            }
                        }
                        Some("Cluster") => {
                            cluster_deformers.insert(id, child);
                        }
                        Some("BlendShape") => {
                            if blend_deformers.insert(id, child).is_none() {
                                blend_order.push(id);
                            }
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
            // Cluster <-> Bone (Model). Both directions are accepted:
            // the staged skin-anim-binary-v7400.fbx fixture wires the
            // bone Model as the *child* of its Cluster
            // (`C: "OO", <model>, <cluster>`), while the
            // Cluster-as-child form is the mirror image; either way
            // the pair binds the cluster to exactly one bone.
            if cluster_deformers.contains_key(&child_id) && model_nodes.contains_key(&parent_id) {
                cluster_to_bone.insert(child_id, parent_id);
                continue;
            }
            if model_nodes.contains_key(&child_id) && cluster_deformers.contains_key(&parent_id) {
                cluster_to_bone.entry(parent_id).or_insert(child_id);
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
    for skin_id in &skin_order {
        let Some(geom_id) = skin_to_geom.get(skin_id) else {
            continue;
        };
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
        // The Skin element's display name (`Armature::Deformer` →
        // `Armature`) is the skeleton's name; the writer emits it
        // back on the Skin element.
        if let Some(name) = skin_deformers
            .get(skin_id)
            .and_then(|n| display_name(n))
            .filter(|n| !n.is_empty())
        {
            skeleton.name = Some(name);
        }
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
        if let Some(&node_id) = geometry_to_node.get(geom_id)
            && let Some(node) = scene.nodes.get_mut(node_id.0 as usize)
        {
            node.skin = Some(skin_id);
        }
    }

    // 4) Materialise blend shapes — in document order (`blend_order`),
    //    since morph-target slots are positional.
    for blend_id in &blend_order {
        let geom_id = match blend_to_geom.get(blend_id) {
            Some(g) => g,
            None => continue,
        };
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
        // (in-between stations are a staged-docs gap — module note)
        // and emit one MorphTarget on the bound mesh's primitive. The
        // channel's authored display name (object-header prop, split
        // at the `\x00\x01` class join) is collected per slot for the
        // typed `Mesh::target_names` surface below —
        // [`oxideav_mesh3d::MorphTarget`] has no name field.
        let n_corners = corner_indices.len();
        let mut slot_names: Vec<String> = Vec::new();
        for &channel_id in channels {
            let channel_node = blend_channels.get(&channel_id).copied();
            // Every `Shape` connected to the channel, in connection
            // order, with the percent at which it is fully applied.
            let shape_nodes: Vec<&FbxNode> = shapes_of_channel
                .get(&channel_id)
                .into_iter()
                .flatten()
                .filter_map(|id| shape_geometries.get(id).copied())
                .collect();
            let full_weights_record = channel_node.and_then(|c| read_f64_array(c, "FullWeights"));
            let stations = shape_stations(&shape_nodes, full_weights_record.as_deref());
            let expand = |sparse: &HashMap<u32, [f32; 3]>| -> Vec<[f32; 3]> {
                corner_indices
                    .iter()
                    .map(|ix| sparse.get(ix).copied().unwrap_or([0.0; 3]))
                    .collect()
            };
            let has_normals = |s: &SparseShape| s.normals.values().any(|n| *n != [0.0; 3]);

            // The primary deltas (the station at 100 %), the in-between
            // stations, and the per-shape record entries.
            let mut tgt = MorphTarget::new();
            let mut shape_entries: Vec<serde_json::Value> = Vec::new();
            let mut primary: Option<&SparseShape> = None;
            let mut seen_weights: Vec<f32> = Vec::new();
            for st in &stations {
                let w = (st.full_weight / DEFORM_PERCENT_SCALE) as f32;
                let role = if w == 1.0 && primary.is_none() {
                    primary = Some(&st.shape);
                    "primary"
                } else if w.is_finite() && w != 0.0 && w != 1.0 && !seen_weights.contains(&w) {
                    seen_weights.push(w);
                    let mut ib = oxideav_mesh3d::Inbetween::new(w)
                        .with_position(expand(&st.shape.positions));
                    if !st.name.is_empty() {
                        ib = ib.with_name(st.name.clone());
                    }
                    if has_normals(&st.shape) {
                        ib = ib.with_normal(expand(&st.shape.normals));
                    }
                    tgt.inbetweens.push(ib);
                    "inbetween"
                } else {
                    // A second shape at one percent, or one at 0 %:
                    // not a station of the piecewise reading — carried
                    // verbatim.
                    "extra"
                };
                let mut entry = serde_json::json!({
                    "name": st.name,
                    "full_weight": st.full_weight,
                    "properties70": st.shape.has_properties70,
                    "role": role,
                    "indexes": st.shape.indexes,
                });
                if role == "extra" {
                    entry["vertices"] = serde_json::json!(st.shape.raw_vertices);
                    entry["normals"] = serde_json::json!(st.shape.raw_normals);
                }
                shape_entries.push(entry);
            }
            match primary {
                Some(shape) => {
                    tgt.position = Some(expand(&shape.positions));
                    if has_normals(shape) {
                        tgt.normal = Some(expand(&shape.normals));
                    }
                }
                None => {
                    // No shape sits at 100 %: the deltas at weight 1
                    // are read off the piecewise-linear station table
                    // (the base mesh at 0 % included), so the typed
                    // target evaluates to the same shape at every
                    // weight.
                    let (pos, norm) = station_value_at_one(&tgt.inbetweens, n_corners);
                    tgt.position = Some(pos);
                    tgt.normal = norm;
                }
            }
            // The index list of the shape the legacy per-vertex
            // `FullWeights` reading pairs with (the last connected).
            let indexes: Vec<i32> = stations
                .last()
                .map(|st| st.shape.indexes.clone())
                .unwrap_or_default();
            let per_shape_table = full_weights_record
                .as_ref()
                .is_some_and(|w| w.len() == shape_nodes.len());
            // One shape listing one control point with a one-element
            // `FullWeights` fits both renditions. The channel says
            // which producer wrote it: a non-zero percent held only by
            // the body leaf, with no `Properties70`, is the per-vertex
            // producer's channel form (the per-shape producer keeps
            // the percent in a property, §4.3).
            let ambiguous = shape_nodes.len() == 1 && indexes.len() == 1;
            let leaf_only_percent = channel_node.is_some_and(|c| {
                c.child("Properties70").is_none()
                    && c.child("DeformPercent")
                        .and_then(|n| n.properties.first())
                        .and_then(FbxProperty::as_f64_loose)
                        .is_some_and(|v| v != 0.0)
            });
            let station_layout =
                shape_nodes.is_empty() || (per_shape_table && !(ambiguous && leaf_only_percent));
            let blend_entry = serde_json::json!({
                "deformer": blend_deformers
                    .get(blend_id)
                    .copied()
                    .and_then(display_name)
                    .unwrap_or_default(),
                "records": channel_node
                    .map(crate::properties70::own_records_json)
                    .unwrap_or_default(),
                "station_table": station_layout,
                "shapes": shape_entries,
            });

            // The channel's static (rest) weight: its own
            // `DeformPercent` `Properties70` record, 0..100 on the
            // wire → 0..1 mesh3d blend factor. Absent → 0.0 (no
            // template default — the `FbxBlendShapeChannel` body is
            // a staged-docs residual).
            let channel_node = blend_channels.get(&channel_id).copied();
            let static_weight = channel_node
                .and_then(static_deform_percent)
                .map_or(0.0f32, |v| (v / DEFORM_PERCENT_SCALE) as f32);
            // `FullWeights` (`d[len(Indexes)]`, element k paired with
            // `Indexes[k]`, percent) — kept per slot, keyed by the
            // Shape's own `Indexes`, so the writer re-emits it against
            // whatever index set it writes.
            let full_weights = full_weights_record
                .as_ref()
                .filter(|w| !per_shape_table || w.len() == indexes.len())
                .map(|w| {
                    serde_json::json!({
                        "indexes": indexes,
                        "weights": w,
                    })
                })
                .unwrap_or(serde_json::Value::Null);

            let mesh = &mut scene.meshes[mesh_id.0 as usize];
            if let Some(prim) = mesh.primitives.first_mut()
                && prim.positions.len() == n_corners
            {
                // This channel's positional morph-target slot.
                let target_index = prim.targets.len();
                prim.targets.push(tgt);
                // Per-slot `FullWeights` table, aligned with
                // `targets` (null for a channel without the record;
                // the key is dropped again below when every slot is
                // null).
                let fw = prim
                    .extras
                    .entry(FULL_WEIGHTS_KEY.to_string())
                    .or_insert_with(|| serde_json::Value::Array(Vec::new()));
                if let Some(arr) = fw.as_array_mut() {
                    arr.resize(target_index, serde_json::Value::Null);
                    arr.push(full_weights);
                }
                // Per-slot channel record ([`BLEND_CHANNELS_KEY`]).
                let bc = prim
                    .extras
                    .entry(BLEND_CHANNELS_KEY.to_string())
                    .or_insert_with(|| serde_json::Value::Array(Vec::new()));
                if let Some(arr) = bc.as_array_mut() {
                    arr.resize(target_index, serde_json::Value::Null);
                    arr.push(blend_entry);
                }
                mesh.weights.push(static_weight);
                slot_names.push(channel_node.and_then(display_name).unwrap_or_default());

                // Record this BlendShapeChannel's animation target so
                // `extract_animations` can wire DeformPercent curves,
                // and mirror the static weight per node so unanimated
                // slots of a multi-target sampler hold their rest
                // value.
                if let Some(&node_id) = geometry_to_node.get(geom_id) {
                    out.channel_targets.insert(
                        channel_id,
                        MorphChannelBinding {
                            target: AnimationTarget {
                                node: node_id,
                                property: AnimationProperty::MorphWeights,
                            },
                            target_index,
                        },
                    );
                    out.node_morph_statics
                        .entry(node_id)
                        .or_default()
                        .push(static_weight);
                }
            }
        }

        // Surface the channels' authored display names, appended in
        // slot order onto the bound mesh's typed
        // `Mesh::target_names` (append — a geometry may carry several
        // BlendShape deformers, each contributing slots; names are
        // pushed exactly when a target is, so the mesh3d validate
        // length rule holds).
        if !slot_names.is_empty() {
            scene.meshes[mesh_id.0 as usize]
                .target_names
                .extend(slot_names);
        }
        if let Some(prim) = scene.meshes[mesh_id.0 as usize].primitives.first_mut()
            && prim
                .extras
                .get(FULL_WEIGHTS_KEY)
                .and_then(serde_json::Value::as_array)
                .is_some_and(|a| a.iter().all(serde_json::Value::is_null))
        {
            prim.extras.remove(FULL_WEIGHTS_KEY);
        }
    }

    out
}

/// One `Shape` geometry's sparse deltas.
struct SparseShape {
    /// `Indexes` as stored.
    indexes: Vec<i32>,
    /// Position delta per listed control point.
    positions: HashMap<u32, [f32; 3]>,
    /// Normal delta per listed control point (empty when the array is
    /// absent or its length disagrees with `Vertices`).
    normals: HashMap<u32, [f32; 3]>,
    /// `Vertices` as stored.
    raw_vertices: Vec<f64>,
    /// `Normals` as stored.
    raw_normals: Vec<f64>,
    /// The element carries a `Properties70` node (empty on every
    /// staged shape that has one).
    has_properties70: bool,
}

impl SparseShape {
    fn read(node: &FbxNode) -> Self {
        let indexes = read_i32_array(node, "Indexes").unwrap_or_default();
        let raw_vertices = read_f64_array(node, "Vertices").unwrap_or_default();
        let raw_normals = read_f64_array(node, "Normals").unwrap_or_default();
        let sparse = |raw: &[f64]| -> HashMap<u32, [f32; 3]> {
            let mut out = HashMap::new();
            for (slot, &ix) in indexes.iter().enumerate() {
                if ix >= 0 && slot * 3 + 2 < raw.len() {
                    out.insert(
                        ix as u32,
                        [
                            raw[slot * 3] as f32,
                            raw[slot * 3 + 1] as f32,
                            raw[slot * 3 + 2] as f32,
                        ],
                    );
                }
            }
            out
        };
        let positions = sparse(&raw_vertices);
        let normals = if !raw_normals.is_empty() && raw_normals.len() == raw_vertices.len() {
            sparse(&raw_normals)
        } else {
            HashMap::new()
        };
        Self {
            positions,
            normals,
            has_properties70: node.child("Properties70").is_some(),
            indexes,
            raw_vertices,
            raw_normals,
        }
    }
}

/// One target shape of a channel with the percent at which it is fully
/// applied.
struct ShapeStation {
    name: String,
    full_weight: f64,
    shape: SparseShape,
}

/// Pair a channel's shapes (connection order) with their full-weight
/// percents (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §4.2).
///
/// `FullWeights` is the per-shape table when its length equals the
/// shape count: element *k* belongs to the *k*-th connected shape,
/// unsorted. Otherwise the record is another producer's per-vertex
/// rendition (`blender-export-sweeps-2026-09.md` §4.1) or absent, and
/// a lone shape sits at 100 %; with several shapes and a table of
/// another length, a missing entry reads 0 as the vendor reader does —
/// and with no record at all every shape reads 100.
fn shape_stations(shapes: &[&FbxNode], full_weights: Option<&[f64]>) -> Vec<ShapeStation> {
    let table = full_weights.filter(|w| w.len() == shapes.len());
    shapes
        .iter()
        .enumerate()
        .map(|(k, node)| ShapeStation {
            name: display_name(node).unwrap_or_default(),
            full_weight: match (table, full_weights) {
                (Some(t), _) => t[k],
                (None, Some(w)) if shapes.len() > 1 => w.get(k).copied().unwrap_or(0.0),
                _ => DEFORM_PERCENT_SCALE,
            },
            shape: SparseShape::read(node),
        })
        .collect()
}

/// `(weight, position deltas, normal deltas)` of one station.
type Station<'a> = (f32, &'a [[f32; 3]], Option<&'a [[f32; 3]]>);

/// The deltas at channel weight 1 for a target whose stations are its
/// in-betweens plus the base mesh at weight 0 — the piecewise-linear
/// reading [`oxideav_mesh3d::MorphTarget::at_weight`] applies, with
/// the nearest segment extrapolating beyond the outermost stations.
fn station_value_at_one(
    inbetweens: &[oxideav_mesh3d::Inbetween],
    n: usize,
) -> (Vec<[f32; 3]>, Option<Vec<[f32; 3]>>) {
    let zeros = vec![[0.0f32; 3]; n];
    // (weight, position, normal) with the base mesh at 0.
    let mut stations: Vec<Station<'_>> = vec![(0.0, &zeros, None)];
    for ib in inbetweens {
        if let Some(p) = &ib.position {
            stations.push((ib.weight, p, ib.normal.as_deref()));
        }
    }
    stations.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    if stations.len() < 2 {
        return (zeros, None);
    }
    // The segment containing 1, else the nearest one.
    let hi = stations
        .iter()
        .position(|s| s.0 >= 1.0)
        .unwrap_or(stations.len() - 1)
        .max(1);
    let (a, b) = (&stations[hi - 1], &stations[hi]);
    let f = if b.0 != a.0 {
        (1.0 - a.0) / (b.0 - a.0)
    } else {
        0.0
    };
    let mix = |x: &[[f32; 3]], y: &[[f32; 3]]| -> Vec<[f32; 3]> {
        (0..n)
            .map(|i| {
                let (p, q) = (
                    x.get(i).copied().unwrap_or([0.0; 3]),
                    y.get(i).copied().unwrap_or([0.0; 3]),
                );
                [
                    p[0] + (q[0] - p[0]) * f,
                    p[1] + (q[1] - p[1]) * f,
                    p[2] + (q[2] - p[2]) * f,
                ]
            })
            .collect()
    };
    let normal =
        (a.2.is_some() || b.2.is_some()).then(|| mix(a.2.unwrap_or(&zeros), b.2.unwrap_or(&zeros)));
    (mix(a.1, b.1), normal)
}

/// A channel's static `DeformPercent` (0..100): the `Properties70`
/// record when present, else the body leaf `DeformPercent: D` — the
/// form the staged blend-shape fixtures carry
/// (`docs/3d/fbx/fixtures/blender-blendshape/notes.md`, "`BlendShapeChannel`
/// body": `Version: 100`, `DeformPercent: D`, `FullWeights: d[…]`, no
/// `Properties70`).
fn static_deform_percent(channel: &FbxNode) -> Option<f64> {
    PropertyMap::from_element(channel)
        .as_f64("DeformPercent")
        .or_else(
            || match channel.child("DeformPercent")?.properties.first()? {
                FbxProperty::F64(v) => Some(*v),
                FbxProperty::F32(v) => Some(f64::from(*v)),
                p => p.as_i64().map(|v| v as f64),
            },
        )
}

/// The element's authored display name — the object-header name
/// property with the trailing `\x00\x01Class` join (binary form)
/// stripped. ASCII-form names carry the `Class::Name` prefix already
/// split by the parser into the same wire shape.
fn display_name(n: &FbxNode) -> Option<String> {
    let raw = n.properties.get(1)?.as_str()?;
    Some(raw.split('\u{0}').next().unwrap_or(raw).to_owned())
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
