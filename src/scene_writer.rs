//! `Scene3D` → [`FbxDocument`] encoder (the inverse of
//! [`crate::scene::build_scene`]).
//!
//! Builds a fresh [`FbxDocument`] node tree from an
//! [`oxideav_mesh3d::Scene3D`], emitting the top-level `Objects` /
//! `Connections` records the binary + ASCII front-ends already read.
//! [`crate::writer::write_document`] then serialises that document to
//! bytes (and [`crate::ascii_writer::write_ascii_document`] to text),
//! so this module is the missing half of the
//! [`oxideav_mesh3d::Mesh3DEncoder`] surface.
//!
//! # Node tree shape
//!
//! The emitted document mirrors the grammar in
//! `docs/3d/fbx/fbx-binary-properties70.md` §5–§7 +
//! `docs/3d/fbx/fbx-ascii-grammar.md` §7b–§7d:
//!
//! ```text
//! FBXHeaderExtension { FBXVersion: <version> }
//! GlobalSettings { Properties70 { ... } }        (when scene carries axis/unit extras)
//! Documents { Count; Document { Properties70; RootNode: 0 } }
//! References { }                                 (observed empty; §7 section set)
//! Definitions { ObjectType: "Geometry"/"Model"/"Material" { Count } }
//! Objects {
//!   Geometry : <id>, "<name>\x00\x01Geometry", "Mesh" {
//!       Vertices: *N { d-array }
//!       PolygonVertexIndex: *M { i-array }       (per-corner; last index of each
//!                                                 triangle bit-NOT'd per §6)
//!       LayerElementNormal { ... }               (when the primitive carries normals)
//!       LayerElementUV { ... }                   (when the primitive carries UV set 0)
//!   }
//!   Model : <id>, "<name>\x00\x01Model", "Mesh" {
//!       Properties70 { P: "Lcl Translation"/"Lcl Rotation"/"Lcl Scaling" ... }
//!   }
//!   Material : <id>, "<name>\x00\x01Material", "" {
//!       Properties70 { P: "DiffuseColor"/"Opacity"/"EmissiveColor"/... }
//!   }
//! }
//! Connections {
//!   C: "OO", <geometry_id>, <model_id>           (Geometry → Model)
//!   C: "OO", <model_id>, <parent_model_id|0>     (Model → parent / root)
//!   C: "OO", <material_id>, <model_id>            (Material → Model)
//! }
//! ```
//!
//! # Geometry vertex layout — per-corner, no dedup
//!
//! [`oxideav_mesh3d::Primitive`] stores per-corner attribute buffers
//! (one position / normal / uv per triangle corner), which is the
//! *output* of [`crate::geometry`]'s fan-triangulation + layer flatten.
//! Rather than re-derive a shared-vertex `Vertices` table (which would
//! require welding identical corners and risks changing the decoded
//! geometry), this writer emits **one `Vertices` entry per corner** and
//! a `PolygonVertexIndex` of sequential triangles
//! `[0, 1, ~2, 3, 4, ~5, …]`. The decode path's fan triangulation of a
//! 3-corner polygon is the identity, so a `Scene3D` → bytes → `Scene3D`
//! round-trip reproduces every corner position exactly. Normals / UVs
//! ride along as `ByPolygonVertex` / `Direct` layers, the mapping the
//! [`crate::geometry`] puller flattens 1:1.
//!
//! # Lossy edges (documented, not silently dropped)
//!
//! - **Rotation** round-trips through an XYZ-Euler `Lcl Rotation`
//!   record. mesh3d stores rotation as a quaternion; the FBX P-record
//!   is degrees-Euler, so the value passes through a quat→Euler→quat
//!   conversion that is exact for axis-aligned rotations and within
//!   float tolerance otherwise (the same convention
//!   [`crate::node_transform`] decodes). A node stored as a raw
//!   [`oxideav_mesh3d::Transform::Matrix`] is decomposed to TRS first.
//! - **Index buffers** are flattened to per-corner positions, so an
//!   indexed mesh re-expands on decode (positions are exact; the shared
//!   index topology is not preserved — mesh3d's decode side already
//!   produces per-corner buffers, so this is symmetric).

use oxideav_mesh3d::{AlphaMode, Indices, Material, Mesh, Node, Primitive, Scene3D, Transform};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Default file-format version the encoder targets when the caller
/// doesn't override it. 7400 selects the 32-bit Node Record layout
/// (the most broadly accepted form; pre-7500 per Gessler's
/// version-dependent-quirks table).
pub const DEFAULT_ENCODE_VERSION: u32 = 7400;

/// FBX-id allocation base. Real exporters use 64-bit hash-like ids;
/// for a freshly-built document any distinct non-zero i64s work, since
/// the only consumer is our own `Connections` graph. We start at a
/// high constant so the ids never collide with the `0` document-root
/// sentinel and stay visually distinct in a hex dump.
const ID_BASE: i64 = 1_000_000;

/// Tunable knobs for [`encode_scene_with_options`].
#[derive(Clone, Debug)]
pub struct SceneEncodeOptions {
    /// File-format version written into the header + used to pick the
    /// 32-bit vs 64-bit Node Record layout. Defaults to
    /// [`DEFAULT_ENCODE_VERSION`].
    pub version: u32,
    /// Emit a `LayerElementNormal` record for primitives that carry
    /// per-corner normals. Default `true`.
    pub emit_normals: bool,
    /// Emit one `LayerElementUV` record per UV set the mesh's
    /// primitives carry (every set — the first is the primary channel,
    /// the rest additional channels, matching the decode side's
    /// document-order `Primitive::uvs` surfacing). Default `true`.
    pub emit_uvs: bool,
    /// Emit one `LayerElementColor` record per vertex-colour set the
    /// mesh's primitives carry (RGBA `Colors` `d`-array, one record
    /// per `Primitive::colors` entry in order). Default `true`.
    pub emit_colors: bool,
    /// Emit a `LayerElementTangent` record (xyz `Tangents` + `w`
    /// handedness-sign `TangentsW`) for primitives that carry the
    /// canonical glTF-style `Primitive::tangents` slot. Default `true`.
    pub emit_tangents: bool,
    /// Emit the binary-only top-level provenance siblings of
    /// `FBXHeaderExtension` (`FileId` / `CreationTime` / `Creator`,
    /// `fbx-binary-properties70.md` §3c) from the `fbx:file_*`
    /// extras. The ASCII form has no such records — every staged
    /// ASCII fixture carries exactly the eight §7 sections — so the
    /// ASCII encoder path turns this off. Default `true`.
    pub binary_provenance: bool,
}

impl Default for SceneEncodeOptions {
    fn default() -> Self {
        Self {
            version: DEFAULT_ENCODE_VERSION,
            emit_normals: true,
            emit_uvs: true,
            emit_colors: true,
            emit_tangents: true,
            binary_provenance: true,
        }
    }
}

impl SceneEncodeOptions {
    /// Builder-style version override.
    pub fn version(mut self, version: u32) -> Self {
        self.version = version;
        self
    }
}

/// Build an [`FbxDocument`] from a [`Scene3D`] with default options.
pub fn encode_scene(scene: &Scene3D) -> FbxDocument {
    encode_scene_with_options(scene, &SceneEncodeOptions::default())
}

/// Build an [`FbxDocument`] from a [`Scene3D`], parameterised by
/// [`SceneEncodeOptions`].
pub fn encode_scene_with_options(scene: &Scene3D, opts: &SceneEncodeOptions) -> FbxDocument {
    let mut alloc = IdAllocator::new();

    // FBX id per mesh / node / material / texture, allocated up-front
    // so the Connections pass can reference them.
    let mesh_ids: Vec<i64> = (0..scene.meshes.len()).map(|_| alloc.next()).collect();
    let node_ids: Vec<i64> = (0..scene.nodes.len()).map(|_| alloc.next()).collect();
    let material_ids: Vec<i64> = (0..scene.materials.len()).map(|_| alloc.next()).collect();
    let texture_ids: Vec<i64> = (0..scene.textures.len()).map(|_| alloc.next()).collect();
    // A `Video` element backs each emitted embedded texture; one id per
    // texture slot (only used when the texture carries embedded bytes).
    let video_ids: Vec<i64> = (0..scene.textures.len()).map(|_| alloc.next()).collect();

    let mut objects = FbxNode {
        name: "Objects".to_string(),
        properties: Vec::new(),
        children: Vec::new(),
    };
    let mut connections = FbxNode {
        name: "Connections".to_string(),
        properties: Vec::new(),
        children: Vec::new(),
    };

    // -- Geometry records (one per mesh) --------------------------------
    for (mi, mesh) in scene.meshes.iter().enumerate() {
        let geom = build_geometry(mesh, mesh_ids[mi], opts);
        objects.children.push(geom);
    }

    // -- Material records (one per material) ----------------------------
    for (xi, mat) in scene.materials.iter().enumerate() {
        let node = build_material(mat, material_ids[xi]);
        objects.children.push(node);
    }

    // -- Texture / Video records + OP wiring ----------------------------
    // Each `Scene3D::Texture` referenced by a material slot becomes a
    // `Texture` element. When the texture carries embedded bytes (an
    // `AssetSource` blob) a `Video` element + `Video.Content` R-blob is
    // emitted and OO-connected (the self-contained-FBX shape); otherwise
    // the external URI lands on `RelativeFilename` / `FileName`. The
    // `Texture -> Material(prop_name)` OP connection wires the texture
    // back into the typed PBR slot the decode path reads (§7).
    // Material index → first mesh drawing it, so the emitted `UVSet`
    // KString can name the UV channel the reference samples with the
    // same label the geometry's `LayerElementUV` `Name` leaf carries
    // (the join the decode side's `resolve_uv_set_index` reads).
    let mut mesh_of_material: Vec<Option<usize>> = vec![None; scene.materials.len()];
    let note_material = |mat_idx: usize, mi: usize, map: &mut Vec<Option<usize>>| {
        if let Some(slot) = map.get_mut(mat_idx) {
            if slot.is_none() {
                *slot = Some(mi);
            }
        }
    };
    for (mi, mesh) in scene.meshes.iter().enumerate() {
        for prim in &mesh.primitives {
            if let Some(mid) = prim.material {
                note_material(mid.0 as usize, mi, &mut mesh_of_material);
            }
            // Multi-material primitives: every slot-table entry draws
            // this mesh too (`fbx:material_slots`, MaterialId.0 per
            // slot in connection order).
            if let Some(slots) = prim
                .extras
                .get("fbx:material_slots")
                .and_then(|v| v.as_array())
            {
                for v in slots {
                    if let Some(id) = v.as_u64() {
                        note_material(id as usize, mi, &mut mesh_of_material);
                    }
                }
            }
        }
    }
    // Every `Scene3D::Texture` becomes one `Texture` element, in
    // texture-index order (the decode side assigns `TextureId`s in
    // document order, so this keeps the ids stable across a round
    // trip). A texture referenced by no material — the *orphan*
    // embedded texture the staged texture-video-ascii-v7500.fbx
    // fixture carries — is emitted too, with default reference
    // settings. The first material slot referencing a texture
    // supplies its `UVSet` / placement records (a texture shared by
    // several slots carries the first reference's; divergent
    // per-slot transforms on one shared texture are a documented
    // lossy edge).
    let mut first_ref: Vec<Option<(usize, oxideav_mesh3d::TextureRef)>> =
        vec![None; scene.textures.len()];
    for (xi, mat) in scene.materials.iter().enumerate() {
        for (texref, _) in material_texture_slots(mat) {
            if let Some(slot) = first_ref.get_mut(texref.texture.0 as usize) {
                if slot.is_none() {
                    *slot = Some((xi, texref));
                }
            }
        }
    }
    for (tex_idx, tex) in scene.textures.iter().enumerate() {
        let (uv_label, texref) = match first_ref[tex_idx] {
            Some((xi, texref)) => (uv_set_label(scene, mesh_of_material[xi], &texref), texref),
            None => (
                None,
                oxideav_mesh3d::TextureRef::new(oxideav_mesh3d::TextureId(tex_idx as u32)),
            ),
        };
        let raw_records = scene
            .extras
            .get("fbx:texture_records")
            .and_then(|v| v.get(tex_idx.to_string()));
        let (tex_node, video_node) = build_texture(
            tex,
            texture_ids[tex_idx],
            video_ids[tex_idx],
            &texref,
            uv_label.as_deref(),
            raw_records,
        );
        objects.children.push(tex_node);
        if let Some(vnode) = video_node {
            objects.children.push(vnode);
            // Video -> Texture OO (backing media).
            connections
                .children
                .push(connection_oo(video_ids[tex_idx], texture_ids[tex_idx]));
        }
    }
    for (xi, mat) in scene.materials.iter().enumerate() {
        for (texref, prop_name) in material_texture_slots(mat) {
            let tex_idx = texref.texture.0 as usize;
            if tex_idx >= scene.textures.len() {
                continue;
            }
            // Texture -> Material(prop_name) OP connection.
            connections.children.push(connection_op(
                texture_ids[tex_idx],
                material_ids[xi],
                prop_name,
            ));
        }
    }

    // -- Model records (one per node) -----------------------------------
    for (ni, node) in scene.nodes.iter().enumerate() {
        let model = build_model(node, node_ids[ni]);
        objects.children.push(model);
        // Light / Camera NodeAttribute (round 384) — one attribute
        // element per bound node, OO-connected to the owning Model
        // (the wiring the decode side's lights_cameras walk reads).
        if let Some(light) = node.light.and_then(|l| scene.lights.get(l.0 as usize)) {
            let attr_id = alloc.next();
            objects
                .children
                .push(build_light_attribute(light, node, attr_id));
            connections
                .children
                .push(connection_oo(attr_id, node_ids[ni]));
        }
        if let Some(camera) = node.camera.and_then(|c| scene.cameras.get(c.0 as usize)) {
            let attr_id = alloc.next();
            objects
                .children
                .push(build_camera_attribute(camera, node, attr_id));
            connections
                .children
                .push(connection_oo(attr_id, node_ids[ni]));
        }
        // LimbNode / Null kind markers (round 384) — the decode side
        // records the §6 NodeAttribute discriminator on
        // `extras["fbx:node_attribute_kind"]`; re-emit the attribute
        // element so a bone / locator marker survives re-encode.
        if let Some(kind) = node
            .extras
            .get("fbx:node_attribute_kind")
            .and_then(|v| v.as_str())
        {
            if kind == "LimbNode" || kind == "Null" {
                let attr_id = alloc.next();
                objects.children.push(node_attribute(
                    attr_id,
                    node,
                    kind,
                    attribute_raw_records(node),
                ));
                connections
                    .children
                    .push(connection_oo(attr_id, node_ids[ni]));
            }
        }
        // Geometry → Model attribute attachment.
        if let Some(mid) = node.mesh {
            let gid = mesh_ids[mid.0 as usize];
            connections.children.push(connection_oo(gid, node_ids[ni]));
        }
        // Material → Model surface assignment. Slot order matters:
        // the decode side rebuilds `fbx:material_slots` from the
        // `Material -> Model` OO connections in document order, and
        // the `LayerElementMaterial` per-polygon indices key into
        // that same slot vector. Multi-material primitives carry the
        // full slot table on `extras["fbx:material_slots"]`
        // (round-tripped from a decoded mesh); single-binding
        // primitives contribute their lone `Primitive::material`.
        if let Some(mid) = node.mesh {
            if let Some(prim) = scene
                .meshes
                .get(mid.0 as usize)
                .and_then(|m| m.primitives.first())
            {
                for slot in material_slot_table(prim, scene.materials.len()) {
                    connections
                        .children
                        .push(connection_oo(material_ids[slot], node_ids[ni]));
                }
            }
        }
    }

    // -- Scene-graph parent / child + root edges ------------------------
    // A node that is a child of another node connects to the parent;
    // a root connects to the document root (id 0).
    let mut is_child = vec![false; scene.nodes.len()];
    for (ni, node) in scene.nodes.iter().enumerate() {
        for child in &node.children {
            let cidx = child.0 as usize;
            if cidx < scene.nodes.len() {
                is_child[cidx] = true;
                connections
                    .children
                    .push(connection_oo(node_ids[cidx], node_ids[ni]));
            }
        }
    }
    // Every node nobody parents gets a `Model -> 0` document-root edge,
    // whether it is an explicit `Scene3D::roots` entry or an orphan
    // (the decode side's `build_scene` also treats both as roots — its
    // implicit-root recovery promotes any un-parented Model).
    for (ni, child) in is_child.iter().enumerate() {
        if !*child {
            connections.children.push(connection_oo(node_ids[ni], 0));
        }
    }

    // -- Bind pose (round 433) ------------------------------------------
    // One `Pose : "BindPose"` element re-emitted from the
    // `fbx:bind_pose` node extras the decode side surfaces
    // ([`crate::pose`]): each posed node contributes a
    // `PoseNode { Node : i64, Matrix : d[16] }` pair carrying the
    // world-space bind matrix (row-major — the exact record shape the
    // pose module reads). The derived `fbx:bind_pose_parent_local`
    // extras are NOT emitted: the decode side recomputes them from
    // the world matrices + the scene-graph parent map.
    let pose_entries: Vec<(i64, Vec<f64>)> = scene
        .nodes
        .iter()
        .enumerate()
        .filter_map(|(ni, node)| {
            let arr = node.extras.get("fbx:bind_pose")?.as_array()?;
            if arr.len() != 16 {
                return None;
            }
            let mut m = Vec::with_capacity(16);
            for v in arr {
                m.push(v.as_f64()?);
            }
            Some((node_ids[ni], m))
        })
        .collect();
    if !pose_entries.is_empty() {
        let pose_id = alloc.next();
        objects
            .children
            .push(build_bind_pose(pose_entries, pose_id));
    }

    // -- Deformers (round 384) -------------------------------------------
    // Skin / Cluster trees for every skinned node + BlendShape /
    // BlendShapeChannel / Geometry{Shape} trees for every primitive
    // carrying morph targets. Runs before the animation pass so
    // MorphWeights channels can target the emitted BlendShapeChannel
    // element ids.
    let deformer_emit = crate::deformer_writer::build_deformer_objects(
        scene,
        |mi| mesh_ids.get(mi).copied(),
        |ni| node_ids.get(ni).copied(),
        || alloc.next(),
    );
    let morph_channels = deformer_emit.morph_channels;
    objects.children.extend(deformer_emit.objects);
    connections.children.extend(deformer_emit.connections);

    // -- Animation graph (round 377) ------------------------------------
    // One AnimationStack / AnimationLayer per Scene3D::Animation, plus
    // the AnimationCurveNode / AnimationCurve records + OO/OP chain the
    // decode path's extract_animations walks. Channels target the Model
    // record for the scene NodeId via the node-id → fbx-id map below;
    // MorphWeights channels target the node's BlendShapeChannels (one
    // DeformPercent OP connection per morph-target slot).
    if !scene.animations.is_empty() {
        let node_to_fbx =
            |nid: oxideav_mesh3d::NodeId| -> Option<i64> { node_ids.get(nid.0 as usize).copied() };
        let morph_channels_for = |nid: oxideav_mesh3d::NodeId| -> Option<Vec<i64>> {
            morph_channels
                .iter()
                .find(|(n, _)| *n == nid)
                .map(|(_, ids)| ids.clone())
        };
        // A chain-bearing node's channels must be de-composed back to
        // authored Lcl curves (the Model re-gains its pivot records,
        // so emitting the composed values would double-apply the
        // chain on the next decode).
        let chain_for = |nid: oxideav_mesh3d::NodeId| {
            scene
                .nodes
                .get(nid.0 as usize)
                .and_then(crate::node_transform::chain_from_extras)
        };
        let anim_emit = crate::anim_writer::build_animation_objects(
            &scene.animations,
            node_to_fbx,
            morph_channels_for,
            chain_for,
            || alloc.next(),
        );
        objects.children.extend(anim_emit.objects);
        connections.children.extend(anim_emit.connections);
    }

    // -- Constraints (round 439) ----------------------------------------
    // `Constraint` elements + their target OP edges rebuilt from the
    // `fbx:constraints` extras the decode side surfaces
    // (`docs/3d/fbx/fbx-constraint-grammar.md` §2–§3); the per-kind
    // Definitions templates re-emit inside `build_definitions`.
    let (constraint_objects, constraint_connections) = crate::constraint::build_constraint_objects(
        scene,
        |ni| node_ids.get(ni).copied(),
        || alloc.next(),
    );
    objects.children.extend(constraint_objects);
    connections.children.extend(constraint_connections);

    // -- Top-level sections ---------------------------------------------
    let mut root = FbxNode {
        name: String::new(),
        properties: Vec::new(),
        children: Vec::new(),
    };
    root.children
        .push(build_header_extension(scene, opts.version));
    // Top-level provenance siblings — re-emitted from the
    // `fbx:file_id` / `fbx:file_creation_time` / `fbx:file_creator`
    // extras in the fixture-observed order (`FBXHeaderExtension,
    // FileId, CreationTime, Creator, GlobalSettings, ...`). Only
    // present when the source document carried them (see
    // `crate::header_info::extract_top_level_provenance`).
    if opts.binary_provenance {
        if let Some(bytes) = scene
            .extras
            .get("fbx:file_id")
            .and_then(|v| v.as_str())
            .and_then(hex_to_bytes)
        {
            root.children.push(FbxNode {
                name: "FileId".to_string(),
                properties: vec![FbxProperty::Raw(bytes)],
                children: Vec::new(),
            });
        }
        if let Some(t) = scene
            .extras
            .get("fbx:file_creation_time")
            .and_then(|v| v.as_str())
        {
            root.children.push(leaf_string("CreationTime", t));
        }
        if let Some(c) = scene
            .extras
            .get("fbx:file_creator")
            .and_then(|v| v.as_str())
        {
            root.children.push(leaf_string("Creator", c));
        }
    }
    root.children.push(build_global_settings(scene));
    // `Documents` + `References` — the §7 sections sitting between
    // `GlobalSettings` and `Definitions` (round 413; fixture order).
    // The document catalogue re-renders from the round-tripped
    // `fbx:documents` / `fbx:active_anim_stack` extras when present;
    // otherwise a single default `"Scene"` document is synthesised
    // (the SDK-written sample always carries one). `References` was
    // observed empty — the empty section is still emitted so the §7
    // section set survives a round trip.
    root.children.push(build_documents(scene, &mut alloc));
    root.children.push(FbxNode {
        name: "References".to_string(),
        properties: Vec::new(),
        children: Vec::new(),
    });
    root.children.push(build_definitions(&objects, scene));
    root.children.push(objects);
    root.children.push(connections);
    // `Takes` — the last §7 ordered section, re-rendered from the
    // round-tripped `fbx:takes` / `fbx:current_take` extras (round
    // 384). Omitted entirely when the scene carries neither.
    if let Some(takes) = build_takes(scene) {
        root.children.push(takes);
    }

    FbxDocument {
        version: opts.version,
        root,
    }
}

/// Merge typed records into a round-tripped raw record list by
/// name: the raw list's order is kept; a raw record whose name a
/// typed record shares is kept verbatim when the two carry the same
/// value (the wire form — label / flag strings, int vs double — is
/// the producer's to keep) and replaced by the typed one only when
/// the value differs (the typed field was edited); typed records
/// with no raw counterpart are appended. Works for `P` records
/// (name = first string property, payload = the values after the
/// four leading strings) and for body leaves (name = node name,
/// payload = every property) alike.
fn merge_by_name(
    raw: Vec<FbxNode>,
    typed: Vec<FbxNode>,
    name_of: fn(&FbxNode) -> String,
) -> Vec<FbxNode> {
    let mut pool: Vec<Option<FbxNode>> = typed.into_iter().map(Some).collect();
    let mut out = Vec::with_capacity(raw.len() + pool.len());
    for r in raw {
        let rn = name_of(&r);
        match pool
            .iter_mut()
            .find(|t| t.as_ref().is_some_and(|t| name_of(t) == rn))
        {
            Some(slot) => {
                let t = slot.take().unwrap();
                if payload_equal(&r, &t) {
                    out.push(r);
                } else {
                    out.push(t);
                }
            }
            None => out.push(r),
        }
    }
    out.extend(pool.into_iter().flatten());
    out
}

/// Value-level equality of two records of the same name: numeric
/// payloads to 1e-9 relative, strings exactly; the leading four
/// strings of a `P` record (name / typeName / label / flags) are
/// not part of the payload.
fn payload_equal(a: &FbxNode, b: &FbxNode) -> bool {
    enum V {
        N(f64),
        S(Vec<u8>),
    }
    fn payload(n: &FbxNode) -> Vec<V> {
        let skip = if n.name == "P" { 4 } else { 0 };
        n.properties
            .iter()
            .skip(skip)
            .map(|p| match p {
                FbxProperty::Bool(b) => V::N(if *b { 1.0 } else { 0.0 }),
                FbxProperty::I16(n) => V::N(f64::from(*n)),
                FbxProperty::I32(n) => V::N(f64::from(*n)),
                FbxProperty::I64(n) => V::N(*n as f64),
                FbxProperty::F32(x) => V::N(f64::from(*x)),
                FbxProperty::F64(x) => V::N(*x),
                FbxProperty::String(s) => V::S(s.clone()),
                other => V::S(format!("{other:?}").into_bytes()),
            })
            .collect()
    }
    let (pa, pb) = (payload(a), payload(b));
    pa.len() == pb.len()
        && pa.iter().zip(&pb).all(|(x, y)| match (x, y) {
            (V::N(x), V::N(y)) => (x - y).abs() <= 1.0e-9 * x.abs().max(y.abs()).max(1.0),
            (V::S(x), V::S(y)) => x == y,
            _ => false,
        })
}

fn p_record_name(p: &FbxNode) -> String {
    crate::properties70::p_name(p).unwrap_or("").to_owned()
}

fn leaf_name(n: &FbxNode) -> String {
    n.name.clone()
}

fn raw_records_from_extras(scene: &Scene3D, key: &str) -> Vec<FbxNode> {
    scene
        .extras
        .get(key)
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_p_record)
        .collect()
}

fn raw_leaves_from_extras(scene: &Scene3D, key: &str) -> Vec<FbxNode> {
    scene
        .extras
        .get(key)
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_leaf)
        .collect()
}

/// Monotonic FBX-id source.
struct IdAllocator {
    next: i64,
}

impl IdAllocator {
    fn new() -> Self {
        Self { next: ID_BASE }
    }
    fn next(&mut self) -> i64 {
        let id = self.next;
        self.next += 1;
        id
    }
}

/// `FBXHeaderExtension { FBXHeaderVersion; FBXVersion;
/// CreationTimeStamp; Creator; SceneInfo }` — the §7a authoring
/// section. The minimal form (bare `FBXVersion`) is always emitted;
/// the metadata leaves are re-rendered from the round-tripped
/// `fbx:header_version` / `fbx:creator` / `fbx:creation_time` /
/// `fbx:meta_*` / `fbx:application_*` / `fbx:document_url` extras the
/// decode side surfaces, so authoring provenance survives a
/// decode → encode → decode cycle.
fn build_header_extension(scene: &Scene3D, version: u32) -> FbxNode {
    let mut children = Vec::new();

    if let Some(hv) = scene
        .extras
        .get("fbx:header_version")
        .and_then(|v| v.as_i64())
    {
        children.push(leaf_i32("FBXHeaderVersion", hv as i32));
    }
    children.push(FbxNode {
        name: "FBXVersion".to_string(),
        properties: vec![FbxProperty::I32(version as i32)],
        children: Vec::new(),
    });
    if let Some(ts) = scene
        .extras
        .get("fbx:creation_time")
        .and_then(|v| v.as_str())
        .and_then(creation_timestamp_node)
    {
        children.push(ts);
    }
    if let Some(creator) = scene.extras.get("fbx:creator").and_then(|v| v.as_str()) {
        children.push(leaf_string("Creator", creator));
    }
    if let Some(scene_info) = build_scene_info(scene) {
        children.push(scene_info);
    }

    FbxNode {
        name: "FBXHeaderExtension".to_string(),
        properties: Vec::new(),
        children,
    }
}

/// Parse the decode side's composed `YYYY-MM-DDThh:mm:ss.mmm` stamp
/// back into the §7a `CreationTimeStamp` integer sub-leaves. Returns
/// `None` for a string that doesn't match the composed shape (the
/// stamp is then simply not re-emitted — no guessing).
fn creation_timestamp_node(stamp: &str) -> Option<FbxNode> {
    let parts: Vec<i64> = stamp
        .split(['-', 'T', ':', '.'])
        .map(str::parse)
        .collect::<std::result::Result<_, _>>()
        .ok()?;
    if parts.len() != 7 {
        return None;
    }
    let names = [
        "Year",
        "Month",
        "Day",
        "Hour",
        "Minute",
        "Second",
        "Millisecond",
    ];
    let mut children = vec![leaf_i32("Version", 1000)];
    for (name, value) in names.iter().zip(&parts) {
        children.push(leaf_i32(name, *value as i32));
    }
    Some(FbxNode {
        name: "CreationTimeStamp".to_string(),
        properties: Vec::new(),
        children,
    })
}

/// Build the §7a/§7c `SceneInfo` object (document `MetaData` block +
/// `Original|*` application-provenance `Properties70`) from the
/// round-tripped extras. Returns `None` when the scene carries no
/// metadata / provenance keys at all.
fn build_scene_info(scene: &Scene3D) -> Option<FbxNode> {
    let mut meta_typed = Vec::new();
    for field in [
        "Title", "Subject", "Author", "Keywords", "Revision", "Comment",
    ] {
        let key = format!("fbx:meta_{}", field.to_ascii_lowercase());
        if let Some(val) = scene.extras.get(&key).and_then(|v| v.as_str()) {
            meta_typed.push(leaf_string(field, val));
        }
    }

    let mut ps_typed = Vec::new();
    for (p_name, key) in [
        ("Original|ApplicationVendor", "fbx:application_vendor"),
        ("Original|ApplicationName", "fbx:application_name"),
        ("Original|ApplicationVersion", "fbx:application_version"),
        ("DocumentUrl", "fbx:document_url"),
    ] {
        if let Some(val) = scene.extras.get(key).and_then(|v| v.as_str()) {
            ps_typed.push(p_kstring(p_name, val));
        }
    }

    // Round-tripped raw sets (see `header_info::extract_scene_info_raw`)
    // with the typed values merged in by name.
    let raw_meta = raw_leaves_from_extras(scene, "fbx:meta_data_leaves");
    let raw_ps = raw_records_from_extras(scene, "fbx:scene_info_records");
    let raw_leaves = raw_leaves_from_extras(scene, "fbx:scene_info_leaves");
    let have_raw = !raw_meta.is_empty() || !raw_ps.is_empty() || !raw_leaves.is_empty();
    if meta_typed.is_empty() && ps_typed.is_empty() && !have_raw {
        return None;
    }
    let meta_children = if raw_meta.is_empty() {
        if meta_typed.is_empty() {
            Vec::new()
        } else {
            let mut m = vec![leaf_i32("Version", 100)];
            m.extend(meta_typed);
            m
        }
    } else {
        merge_by_name(raw_meta, meta_typed, leaf_name)
    };
    let ps = merge_by_name(raw_ps, ps_typed, p_record_name);

    // Body order as observed: `Type`, `Version`, `MetaData`,
    // `Properties70`.
    let mut children = raw_leaves;
    if !meta_children.is_empty() {
        children.push(FbxNode {
            name: "MetaData".to_string(),
            properties: Vec::new(),
            children: meta_children,
        });
    }
    if !ps.is_empty() {
        children.push(FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: ps,
        });
    }

    let header: Vec<FbxProperty> = scene
        .extras
        .get("fbx:scene_info_header")
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str())
                .map(|s| FbxProperty::String(s.as_bytes().to_vec()))
                .collect()
        })
        .filter(|h: &Vec<FbxProperty>| !h.is_empty())
        .unwrap_or_else(|| {
            vec![
                FbxProperty::String(b"SceneInfo::GlobalInfo".to_vec()),
                FbxProperty::String(b"UserData".to_vec()),
            ]
        });
    Some(FbxNode {
        name: "SceneInfo".to_string(),
        properties: header,
        children,
    })
}

/// `Takes { Current: "<name>"; Take: "<name>" { FileName; LocalTime;
/// ReferenceTime } }` per `docs/3d/fbx/fbx-ascii-grammar.md` §7e —
/// re-rendered from the `fbx:takes` / `fbx:current_take` extras the
/// decode side surfaces (KTime pairs re-emitted as two `L` scalars,
/// the shape the decode-side pair reader requires).
fn build_takes(scene: &Scene3D) -> Option<FbxNode> {
    let current = scene
        .extras
        .get("fbx:current_take")
        .and_then(|v| v.as_str());
    let takes = scene.extras.get("fbx:takes").and_then(|v| v.as_array());
    if current.is_none() && takes.is_none() {
        return None;
    }

    let mut children = Vec::new();
    if let Some(name) = current {
        children.push(leaf_string("Current", name));
    }
    for take in takes.into_iter().flatten() {
        let Some(name) = take.get("name").and_then(|v| v.as_str()) else {
            continue;
        };
        let mut take_children = Vec::new();
        if let Some(fname) = take.get("file_name").and_then(|v| v.as_str()) {
            take_children.push(leaf_string("FileName", fname));
        }
        for (leaf, key) in [
            ("LocalTime", "local_time"),
            ("ReferenceTime", "reference_time"),
        ] {
            if let Some(pair) = take.get(key).and_then(|v| v.as_array()) {
                if let (Some(start), Some(stop)) = (
                    pair.first().and_then(|v| v.as_i64()),
                    pair.get(1).and_then(|v| v.as_i64()),
                ) {
                    take_children.push(FbxNode {
                        name: leaf.to_string(),
                        properties: vec![FbxProperty::I64(start), FbxProperty::I64(stop)],
                        children: Vec::new(),
                    });
                }
            }
        }
        children.push(FbxNode {
            name: "Take".to_string(),
            properties: vec![FbxProperty::String(name.as_bytes().to_vec())],
            children: take_children,
        });
    }

    Some(FbxNode {
        name: "Takes".to_string(),
        properties: Vec::new(),
        children,
    })
}

/// `GlobalSettings { Version; Properties70 { UpAxis...; UnitScaleFactor } }`
/// per `docs/3d/fbx/fbx-binary-properties70.md` §4 + the
/// cubes-ascii-v7500.fbx fixture.
///
/// Emits the `UnitScaleFactor` `double` P-record derived from
/// [`oxideav_mesh3d::Scene3D::unit`] (the decode path's
/// `unit_from_scale_factor` maps `100.0 → Centimetres` / `1.0 → Metres`;
/// other units write the factor as `centimetres-per-unit` so the raw
/// value survives on `extras["fbx:unit_scale_factor"]`). Axis
/// convention: round-tripped `extras["fbx:up_axis"]` /
/// `["fbx:front_axis"]` / `["fbx:coord_axis"]` ints are re-emitted
/// verbatim; a scene without them synthesises the six `int` records
/// from the typed `Scene3D::up_axis` / `front_axis` fields via the
/// `docs/3d/fbx/fbx-node-transform-chain.md` §4a table, so a fresh
/// scene's axis convention reaches the wire too.
fn build_global_settings(scene: &Scene3D) -> FbxNode {
    let mut ps: Vec<FbxNode> = Vec::new();

    // Axis ints. Round-tripped `fbx:*_axis*` extras win (they carry
    // the source file's literal values, `OriginalUpAxis` included);
    // a scene without them synthesises the six records from the typed
    // `Scene3D::up_axis` / `front_axis` fields via the
    // `docs/3d/fbx/fbx-node-transform-chain.md` §4a integer table
    // (`0 = X`, `1 = Y`, `2 = Z`; signs as separate `±1` ints):
    // `CoordAxis` is the remaining third index with the `+1` sign
    // every staged fixture carries, and `OriginalUpAxis` is `−1` —
    // the §4a *"exporter did not record one"* sentinel. Synthesis is
    // skipped when up and front share an axis index (degenerate input
    // the table can't represent).
    let axis_extras_present = [
        "fbx:up_axis",
        "fbx:front_axis",
        "fbx:coord_axis",
        "fbx:up_axis_sign",
        "fbx:front_axis_sign",
        "fbx:coord_axis_sign",
    ]
    .iter()
    .any(|k| scene.extras.contains_key(*k));
    if !axis_extras_present {
        let (up, up_sign) = crate::globals::axis_to_ints(scene.up_axis);
        let (front, front_sign) = crate::globals::axis_to_ints(scene.front_axis);
        if up != front {
            let coord = 3 - up - front;
            for (name, v) in [
                ("UpAxis", up),
                ("UpAxisSign", up_sign),
                ("FrontAxis", front),
                ("FrontAxisSign", front_sign),
                ("CoordAxis", coord),
                ("CoordAxisSign", 1),
                ("OriginalUpAxis", -1),
                ("OriginalUpAxisSign", 1),
            ] {
                ps.push(p_int(name, v));
            }
        }
    }
    for (key, name) in [
        ("fbx:up_axis", "UpAxis"),
        ("fbx:up_axis_sign", "UpAxisSign"),
        ("fbx:front_axis", "FrontAxis"),
        ("fbx:front_axis_sign", "FrontAxisSign"),
        ("fbx:coord_axis", "CoordAxis"),
        ("fbx:coord_axis_sign", "CoordAxisSign"),
        ("fbx:original_up_axis", "OriginalUpAxis"),
        ("fbx:original_up_axis_sign", "OriginalUpAxisSign"),
        ("fbx:current_time_marker", "CurrentTimeMarker"),
    ] {
        if let Some(i) = scene.extras.get(key).and_then(|v| v.as_i64()) {
            ps.push(p_int(name, i as i32));
        }
    }
    // Enum-typed time-mode ints (the fixture's `"enum"` typeName; the
    // decode side's generic `as_i32` reads either, but the typed
    // `as_enum` accessor only fires on the correct typeName).
    for (key, name) in [
        ("fbx:time_mode", "TimeMode"),
        ("fbx:time_protocol", "TimeProtocol"),
        ("fbx:snap_on_frame_mode", "SnapOnFrameMode"),
    ] {
        if let Some(i) = scene.extras.get(key).and_then(|v| v.as_i64()) {
            ps.push(p_enum(name, i as i32));
        }
    }
    // KTime spans — i64-exact `L`-wire records.
    for (key, name) in [
        ("fbx:time_span_start", "TimeSpanStart"),
        ("fbx:time_span_stop", "TimeSpanStop"),
    ] {
        if let Some(t) = scene.extras.get(key).and_then(|v| v.as_i64()) {
            ps.push(p_ktime(name, t));
        }
    }
    // Remaining doubles / string / colour from the decode-side
    // recognised-name set.
    for (key, name) in [
        ("fbx:original_unit_scale_factor", "OriginalUnitScaleFactor"),
        ("fbx:custom_frame_rate", "CustomFrameRate"),
    ] {
        if let Some(v) = scene.extras.get(key).and_then(|v| v.as_f64()) {
            ps.push(p_double(name, v));
        }
    }
    if let Some(s) = scene
        .extras
        .get("fbx:default_camera")
        .and_then(|v| v.as_str())
    {
        ps.push(p_kstring("DefaultCamera", s));
    }
    if let Some(rgb) = scene
        .extras
        .get("fbx:ambient_color")
        .and_then(|v| v.as_array())
        .and_then(|a| {
            Some([
                a.first().and_then(|v| v.as_f64())?,
                a.get(1).and_then(|v| v.as_f64())?,
                a.get(2).and_then(|v| v.as_f64())?,
            ])
        })
    {
        ps.push(p_color("AmbientColor", rgb));
    }

    // UnitScaleFactor — centimetres-per-unit. The decode side's
    // `unit_from_scale_factor` recovers Centimetres (100) / Metres (1);
    // a round-tripped *non-canonical* factor (the decode side left
    // `scene.unit` at its default and stashed the raw value on
    // `extras["fbx:unit_scale_factor"]`) is preferred so the literal
    // exporter-side factor survives re-encode. Other typed units write
    // their `cm per unit` equivalent.
    let extras_factor = scene
        .extras
        .get("fbx:unit_scale_factor")
        .and_then(|v| v.as_f64())
        .filter(|&f| crate::globals::unit_from_scale_factor(f).is_none());
    let scale_factor = extras_factor.unwrap_or(match scene.unit {
        oxideav_mesh3d::Unit::Centimetres => 100.0,
        oxideav_mesh3d::Unit::Metres => 1.0,
        // metres-per-unit → centimetres-per-unit.
        other => other.to_metres() as f64 * 100.0,
    });
    ps.push(p_double("UnitScaleFactor", scale_factor));

    // Round-tripped raw set (`fbx:global_settings_records`) with the
    // typed records merged in by name — keeps the producer's order and
    // the records outside the recognised set (`TimeMarker`, …).
    let ps = merge_by_name(
        raw_records_from_extras(scene, "fbx:global_settings_records"),
        ps,
        p_record_name,
    );

    FbxNode {
        name: "GlobalSettings".to_string(),
        properties: Vec::new(),
        children: vec![
            leaf_i32("Version", 1000),
            FbxNode {
                name: "Properties70".to_string(),
                properties: Vec::new(),
                children: ps,
            },
        ],
    }
}

/// `Documents { Count; Document: <uid>, "<name>", "<subtype>" {
/// Properties70 { SourceObject; ActiveAnimStackName }; RootNode: 0 } }`
/// — the document catalogue per the §7 top-level section list + the
/// staged cubes-ascii-v7500.fbx fixture body (see [`crate::documents`]
/// for the decode side).
///
/// Re-rendered from the round-tripped `fbx:documents` extras when
/// present (each entry keeps only its own recorded
/// `active_anim_stack`); a scene without the catalogue gets the single
/// default `"Scene"` document the SDK-written sample always carries,
/// whose `ActiveAnimStackName` resolves from `fbx:active_anim_stack`,
/// then `fbx:current_take`, then the first animation's name — so a
/// freshly-authored animated scene opens on its animation. `RootNode`
/// is always the `0` implicit-root sentinel (the same convention the
/// `C:` root attachments use); source-file UIDs are not round-tripped
/// (the decode side deliberately doesn't surface them).
fn build_documents(scene: &Scene3D, alloc: &mut IdAllocator) -> FbxNode {
    // One (name, subtype, stack) entry per document.
    let mut entries: Vec<(String, String, Option<String>)> = Vec::new();
    if let Some(docs) = scene.extras.get("fbx:documents").and_then(|v| v.as_array()) {
        for d in docs {
            entries.push((
                d.get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_owned(),
                d.get("subtype")
                    .and_then(|v| v.as_str())
                    .unwrap_or("Scene")
                    .to_owned(),
                d.get("active_anim_stack")
                    .and_then(|v| v.as_str())
                    .map(str::to_owned),
            ));
        }
    }
    if entries.is_empty() {
        // Default document: the stack-name fallback chain.
        let stack = scene
            .extras
            .get("fbx:active_anim_stack")
            .and_then(|v| v.as_str())
            .map(str::to_owned)
            .or_else(|| {
                scene
                    .extras
                    .get("fbx:current_take")
                    .and_then(|v| v.as_str())
                    .map(str::to_owned)
            })
            .or_else(|| scene.animations.first().and_then(|a| a.name.clone()));
        entries.push((String::new(), "Scene".to_owned(), stack));
    }

    let mut children = vec![leaf_i32("Count", entries.len() as i32)];
    for (name, subtype, stack) in entries {
        // `ActiveAnimStackName` is always written (an empty string
        // when the document names no stack — the shape every
        // staged fixture without animation carries).
        let ps = vec![
            p_object_ref("SourceObject"),
            p_kstring("ActiveAnimStackName", stack.as_deref().unwrap_or("")),
        ];
        children.push(FbxNode {
            name: "Document".to_string(),
            properties: vec![
                FbxProperty::I64(alloc.next()),
                // The fixture's Document line carries a plain name
                // string (`""` — no ClassTag join, unlike the §7c
                // Objects records).
                FbxProperty::String(name.into_bytes()),
                FbxProperty::String(subtype.into_bytes()),
            ],
            children: vec![
                FbxNode {
                    name: "Properties70".to_string(),
                    properties: Vec::new(),
                    children: ps,
                },
                FbxNode {
                    name: "RootNode".to_string(),
                    properties: vec![FbxProperty::I64(0)],
                    children: Vec::new(),
                },
            ],
        });
    }
    FbxNode {
        name: "Documents".to_string(),
        properties: Vec::new(),
        children,
    }
}

/// `Definitions { Version; Count; ObjectType: "<class>" { Count } }`
/// per `docs/3d/fbx/fbx-ascii-grammar.md` §7b: *"`Count` at the top is
/// the total object count; each `ObjectType:` block names a class"*
/// and *"its instance `Count`"*.
///
/// The per-class counts are derived from the **actually emitted**
/// `Objects` children (round 413) — the earlier scene-derived
/// tally missed every class beyond Geometry / Model / Material
/// (Texture, Video, NodeAttribute, Deformer, AnimationStack /
/// AnimationLayer / AnimationCurveNode / AnimationCurve), so the §7b
/// total drifted from the real object population. The fixture shows
/// `GlobalSettings` participating in the census too (its
/// `ObjectType: "GlobalSettings" { Count: 1 }` block is counted in
/// the top-level `Count: 13`), so the census is `1 + Objects
/// children`, with the GlobalSettings block emitted first as in the
/// sample and the remaining classes in first-appearance order.
fn build_definitions(objects: &FbxNode, scene: &Scene3D) -> FbxNode {
    let mut children = vec![FbxNode {
        name: "Version".to_string(),
        properties: vec![FbxProperty::I32(100)],
        children: Vec::new(),
    }];
    // Total census: the always-present GlobalSettings section + every
    // emitted object record.
    let total = 1 + objects.children.len();
    children.push(FbxNode {
        name: "Count".to_string(),
        properties: vec![FbxProperty::I32(total as i32)],
        children: Vec::new(),
    });

    // Per-class instance counts in first-appearance order, then
    // re-ordered to the source file's own `Definitions` block order
    // when the scene round-trips one (`fbx:property_templates` keeps
    // it), so the section re-emits in the producer's sequence;
    // classes the source did not list follow in appearance order.
    let mut classes: Vec<(&str, usize)> = Vec::new();
    for child in &objects.children {
        match classes.iter_mut().find(|(name, _)| *name == child.name) {
            Some((_, count)) => *count += 1,
            None => classes.push((child.name.as_str(), 1)),
        }
    }
    let source_order: Vec<String> = scene
        .extras
        .get(crate::definitions::PROPERTY_TEMPLATES_KEY)
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(|e| e.get("object_type").and_then(|v| v.as_str()))
        .map(str::to_owned)
        .collect();
    classes.sort_by_key(|(class, _)| {
        source_order
            .iter()
            .position(|c| c == class)
            .unwrap_or(usize::MAX)
    });

    // `NodeAttribute` follows `fbx-property-templates.md` §2 rule 2:
    // the template is named for the *concrete* attribute class, and a
    // file whose attributes are a mixture of kinds gets **no**
    // template rather than choosing one. The concrete body staged in
    // the docs is `FbxCamera` (§3.5), so it is emitted exactly when
    // every emitted `NodeAttribute` is a `"Camera"`.
    let mut node_attr_subtypes = objects
        .children
        .iter()
        .filter(|c| c.name == "NodeAttribute")
        .map(|c| c.properties.get(2).and_then(FbxProperty::as_str));
    let node_attr_all_camera = objects.children.iter().any(|c| c.name == "NodeAttribute")
        && node_attr_subtypes.all(|s| s == Some("Camera"));

    let mut push_class = |class: &str, count: usize| {
        let mut ot_children = vec![FbxNode {
            name: "Count".to_string(),
            properties: vec![FbxProperty::I32(count as i32)],
            children: Vec::new(),
        }];
        // §7b: "each ObjectType: block names a class, its instance
        // Count, and a PropertyTemplate holding the default
        // Properties70 for that class". The fixture stages full
        // template bodies for five classes; the rest stay count-only
        // (a template-less block is also observed — GlobalSettings).
        // `Constraint` is the documented multi-template class
        // (`fbx-constraint-grammar.md` §1 — one PropertyTemplate per
        // kind), re-emitted from the round-tripped
        // `fbx:constraint_templates` extras.
        if class == "Constraint" {
            ot_children.extend(crate::constraint::constraint_template_nodes(scene));
        } else if let Some(nodes) = crate::definitions::template_nodes_from_extras(scene, class) {
            // The source file's own template bodies for this class
            // (`fbx:property_templates`), verbatim — the doc §5 rule
            // that bodies are per-producer renditions, so a round
            // trip must not swap them for this crate's built-ins.
            ot_children.extend(nodes);
        } else if class == "NodeAttribute" {
            if node_attr_all_camera {
                ot_children.push(template_node("FbxCamera", FBX_CAMERA_TEMPLATE));
            }
        } else if class == "Material" {
            // Concrete-class rule again: `FbxSurfaceLambert` when
            // every emitted material declares a lambert shading
            // model, else the `FbxSurfacePhong` body (whose specular
            // records the typed `roughness` / `metallic` map onto).
            let all_lambert = !scene.materials.is_empty()
                && scene.materials.iter().all(|m| {
                    m.extras
                        .get("fbx:shading_model")
                        .and_then(|v| v.as_str())
                        .is_some_and(|s| s.eq_ignore_ascii_case("lambert"))
                });
            ot_children.push(if all_lambert {
                template_node("FbxSurfaceLambert", FBX_SURFACE_LAMBERT_TEMPLATE)
            } else {
                template_node("FbxSurfacePhong", FBX_SURFACE_PHONG_TEMPLATE)
            });
        } else if let Some(template) = class_property_template(class) {
            ot_children.push(template);
        }
        children.push(FbxNode {
            name: "ObjectType".to_string(),
            properties: vec![FbxProperty::String(class.as_bytes().to_vec())],
            children: ot_children,
        });
    };
    push_class("GlobalSettings", 1);
    for (class, count) in classes {
        push_class(class, count);
    }

    FbxNode {
        name: "Definitions".to_string(),
        properties: Vec::new(),
        children,
    }
}

/// Default-value wire shape for one template `P` record. The wire
/// variant per typeName follows the `docs/3d/fbx/fbx-ascii-grammar.md`
/// §8 value-count rules + the `fbx-binary-properties70.md` §4 wire
/// notes (ints for `int` / `enum` / `bool`, one number for `double` /
/// `Number` / `Visibility*`, an `L` for `KTime` / `ULongLong`, three
/// numbers for the triple types, a string for `KString`, and no value
/// at all for `object`).
#[derive(Clone, Copy)]
enum TDef {
    /// `int` / `enum` / `bool` — one integer.
    I(i32),
    /// `KTime` / `ULongLong` — one 64-bit integer.
    L(i64),
    /// `double` / `Number` / `Visibility` — one number.
    D(f64),
    /// Triple types (`ColorRGB` / `Color` / `Vector3D` / `Lcl *`).
    V(f64, f64, f64),
    /// `KString` — one string.
    S(&'static str),
    /// `object` — value-less.
    None,
}

/// One template record: `(name, typeName, label, flags, default)`.
type TRecord = (&'static str, &'static str, &'static str, &'static str, TDef);

/// `PropertyTemplate: "<template>" { Properties70 { ... } }` — the
/// §7b class-default property set for the classes whose template
/// bodies the staged fixtures carry verbatim: `FbxAnimStack` /
/// `FbxAnimLayer` / `FbxMesh` / `FbxSurfaceLambert` / `FbxNode` from
/// `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`, plus — per
/// `docs/3d/fbx/fbx-property-templates.md` §3 — `FbxFileTexture`
/// (§3.1) / `FbxVideo` (§3.2) / `FbxAnimCurveNode` (§3.3) from
/// `texture-video-ascii-v7500.fbx` and `FbxCamera` (§3.5) from
/// `camera-attr-binary-v7400.fbx`. `Deformer` / `Pose` /
/// `AnimationCurve` stay count-only **by rule**, not by gap: the
/// doc's §2 rule 1 establishes those classes declare no FBX
/// properties, so no producer ever writes a template for them.
/// `NodeAttribute` follows the §2 rule 2 concrete-class behaviour —
/// see [`build_definitions`].
fn class_property_template(class: &str) -> Option<FbxNode> {
    let (template_name, records) = match class {
        "AnimationStack" => ("FbxAnimStack", FBX_ANIM_STACK_TEMPLATE),
        "AnimationLayer" => ("FbxAnimLayer", FBX_ANIM_LAYER_TEMPLATE),
        "AnimationCurveNode" => ("FbxAnimCurveNode", FBX_ANIM_CURVE_NODE_TEMPLATE),
        "Geometry" => ("FbxMesh", FBX_MESH_TEMPLATE),
        "Model" => ("FbxNode", FBX_NODE_TEMPLATE),
        "Texture" => ("FbxFileTexture", FBX_FILE_TEXTURE_TEMPLATE),
        "Video" => ("FbxVideo", FBX_VIDEO_TEMPLATE),
        _ => return None,
    };
    Some(template_node(template_name, records))
}

/// Materialise one `PropertyTemplate` node from a static
/// [`TRecord`] table.
fn template_node(template_name: &str, records: &[TRecord]) -> FbxNode {
    let ps: Vec<FbxNode> = records
        .iter()
        .map(|&(name, type_name, label, flags, def)| {
            let mut properties = vec![
                FbxProperty::String(name.as_bytes().to_vec()),
                FbxProperty::String(type_name.as_bytes().to_vec()),
                FbxProperty::String(label.as_bytes().to_vec()),
                FbxProperty::String(flags.as_bytes().to_vec()),
            ];
            match def {
                TDef::I(v) => properties.push(FbxProperty::I32(v)),
                TDef::L(v) => properties.push(FbxProperty::I64(v)),
                TDef::D(v) => properties.push(FbxProperty::F64(v)),
                TDef::V(x, y, z) => properties.extend([
                    FbxProperty::F64(x),
                    FbxProperty::F64(y),
                    FbxProperty::F64(z),
                ]),
                TDef::S(v) => properties.push(FbxProperty::String(v.as_bytes().to_vec())),
                TDef::None => {}
            }
            FbxNode {
                name: "P".to_string(),
                properties,
                children: Vec::new(),
            }
        })
        .collect();
    FbxNode {
        name: "PropertyTemplate".to_string(),
        properties: vec![FbxProperty::String(template_name.as_bytes().to_vec())],
        children: vec![FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: ps,
        }],
    }
}

/// `ObjectType: "AnimationStack" { PropertyTemplate: "FbxAnimStack" }`
/// default set, transcribed from the staged fixture's Definitions.
const FBX_ANIM_STACK_TEMPLATE: &[TRecord] = &[
    ("Description", "KString", "", "", TDef::S("")),
    ("LocalStart", "KTime", "Time", "", TDef::L(0)),
    ("LocalStop", "KTime", "Time", "", TDef::L(0)),
    ("ReferenceStart", "KTime", "Time", "", TDef::L(0)),
    ("ReferenceStop", "KTime", "Time", "", TDef::L(0)),
];

/// `ObjectType: "AnimationLayer" { PropertyTemplate: "FbxAnimLayer" }`
/// default set, transcribed from the staged fixture's Definitions.
const FBX_ANIM_LAYER_TEMPLATE: &[TRecord] = &[
    ("Weight", "Number", "", "A", TDef::D(100.0)),
    ("Mute", "bool", "", "", TDef::I(0)),
    ("Solo", "bool", "", "", TDef::I(0)),
    ("Lock", "bool", "", "", TDef::I(0)),
    ("Color", "ColorRGB", "Color", "", TDef::V(0.8, 0.8, 0.8)),
    ("BlendMode", "enum", "", "", TDef::I(0)),
    ("RotationAccumulationMode", "enum", "", "", TDef::I(0)),
    ("ScaleAccumulationMode", "enum", "", "", TDef::I(0)),
    ("BlendModeBypass", "ULongLong", "", "", TDef::L(0)),
];

/// `ObjectType: "AnimationCurveNode" { PropertyTemplate:
/// "FbxAnimCurveNode" }` default set —
/// `docs/3d/fbx/fbx-property-templates.md` §3.3, identical in the
/// two staged producers' fixtures: the whole template is the single
/// value-less compound property `d` (the real channels are authored
/// on the object as `d|X` / `d|Y` / `d|Z` / `d|DeformPercent`
/// children of it, so there is nothing to default).
const FBX_ANIM_CURVE_NODE_TEMPLATE: &[TRecord] = &[("d", "Compound", "", "", TDef::None)];

/// `ObjectType: "Texture" { PropertyTemplate: "FbxFileTexture" }`
/// default set — `docs/3d/fbx/fbx-property-templates.md` §3.1
/// (16 records, from the staged `texture-video-ascii-v7500.fbx`
/// producer rendition; note `"Texture alpha"`, a property name
/// containing a space — names are free-form strings). Per the doc §5
/// caveat, template bodies are producer renditions rather than a
/// normative table; this is the staged one.
const FBX_FILE_TEXTURE_TEMPLATE: &[TRecord] = &[
    ("TextureTypeUse", "enum", "", "", TDef::I(0)),
    ("Texture alpha", "Number", "", "A", TDef::D(1.0)),
    ("CurrentMappingType", "enum", "", "", TDef::I(0)),
    ("WrapModeU", "enum", "", "", TDef::I(0)),
    ("WrapModeV", "enum", "", "", TDef::I(0)),
    ("UVSwap", "bool", "", "", TDef::I(0)),
    ("PremultiplyAlpha", "bool", "", "", TDef::I(1)),
    ("Translation", "Vector", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("Rotation", "Vector", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("Scaling", "Vector", "", "A", TDef::V(1.0, 1.0, 1.0)),
    (
        "TextureRotationPivot",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "TextureScalingPivot",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("CurrentTextureBlendMode", "enum", "", "", TDef::I(1)),
    ("UVSet", "KString", "", "", TDef::S("default")),
    ("UseMaterial", "bool", "", "", TDef::I(0)),
    ("UseMipMap", "bool", "", "", TDef::I(0)),
];

/// `ObjectType: "Video" { PropertyTemplate: "FbxVideo" }` default
/// set — `docs/3d/fbx/fbx-property-templates.md` §3.2 (20 records,
/// staged `texture-video-ascii-v7500.fbx` producer rendition).
const FBX_VIDEO_TEMPLATE: &[TRecord] = &[
    ("Path", "KString", "XRefUrl", "", TDef::S("")),
    ("RelPath", "KString", "XRefUrl", "", TDef::S("")),
    ("Color", "ColorRGB", "Color", "", TDef::V(0.8, 0.8, 0.8)),
    ("ClipIn", "KTime", "Time", "", TDef::L(0)),
    ("ClipOut", "KTime", "Time", "", TDef::L(0)),
    ("Offset", "KTime", "Time", "", TDef::L(0)),
    ("PlaySpeed", "double", "Number", "", TDef::D(0.0)),
    ("FreeRunning", "bool", "", "", TDef::I(0)),
    ("Loop", "bool", "", "", TDef::I(0)),
    ("Mute", "bool", "", "", TDef::I(0)),
    ("AccessMode", "enum", "", "", TDef::I(0)),
    ("ImageSequence", "bool", "", "", TDef::I(0)),
    ("ImageSequenceOffset", "int", "Integer", "", TDef::I(0)),
    ("FrameRate", "double", "Number", "", TDef::D(0.0)),
    ("LastFrame", "int", "Integer", "", TDef::I(0)),
    ("Width", "int", "Integer", "", TDef::I(0)),
    ("Height", "int", "Integer", "", TDef::I(0)),
    ("StartFrame", "int", "Integer", "", TDef::I(0)),
    ("StopFrame", "int", "Integer", "", TDef::I(0)),
    ("InterlaceMode", "enum", "", "", TDef::I(0)),
];

/// `ObjectType: "NodeAttribute" { PropertyTemplate: "FbxCamera" }`
/// default set — `docs/3d/fbx/fbx-property-templates.md` §3.5 (106
/// records, staged `camera-attr-binary-v7400.fbx` producer
/// rendition). Emitted only when every `NodeAttribute` in the
/// document is a `"Camera"` — the §2 rule 2 concrete-class /
/// no-template-on-mixture behaviour. `"Background Texture"` /
/// `"Foreground Texture"` are value-less `object` slots (filled by
/// `OP` connections, the same mechanism constraint targets use).
const FBX_CAMERA_TEMPLATE: &[TRecord] = &[
    ("Color", "ColorRGB", "Color", "", TDef::V(0.8, 0.8, 0.8)),
    ("Position", "Vector", "", "A", TDef::V(0.0, 0.0, -50.0)),
    ("UpVector", "Vector", "", "A", TDef::V(0.0, 1.0, 0.0)),
    (
        "InterestPosition",
        "Vector",
        "",
        "A",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("Roll", "Roll", "", "A", TDef::D(0.0)),
    ("OpticalCenterX", "OpticalCenterX", "", "A", TDef::D(0.0)),
    ("OpticalCenterY", "OpticalCenterY", "", "A", TDef::D(0.0)),
    (
        "BackgroundColor",
        "Color",
        "",
        "A",
        TDef::V(0.63, 0.63, 0.63),
    ),
    ("TurnTable", "Number", "", "A", TDef::D(0.0)),
    ("DisplayTurnTableIcon", "bool", "", "", TDef::I(0)),
    ("UseMotionBlur", "bool", "", "", TDef::I(0)),
    ("UseRealTimeMotionBlur", "bool", "", "", TDef::I(1)),
    ("Motion Blur Intensity", "Number", "", "A", TDef::D(1.0)),
    ("AspectRatioMode", "enum", "", "", TDef::I(0)),
    ("AspectWidth", "double", "Number", "", TDef::D(320.0)),
    ("AspectHeight", "double", "Number", "", TDef::D(200.0)),
    ("PixelAspectRatio", "double", "Number", "", TDef::D(1.0)),
    ("FilmOffsetX", "Number", "", "A", TDef::D(0.0)),
    ("FilmOffsetY", "Number", "", "A", TDef::D(0.0)),
    ("FilmWidth", "double", "Number", "", TDef::D(0.816)),
    ("FilmHeight", "double", "Number", "", TDef::D(0.612)),
    (
        "FilmAspectRatio",
        "double",
        "Number",
        "",
        TDef::D(1.3333333333333333),
    ),
    ("FilmSqueezeRatio", "double", "Number", "", TDef::D(1.0)),
    ("FilmFormatIndex", "enum", "", "", TDef::I(0)),
    ("PreScale", "Number", "", "A", TDef::D(1.0)),
    ("FilmTranslateX", "Number", "", "A", TDef::D(0.0)),
    ("FilmTranslateY", "Number", "", "A", TDef::D(0.0)),
    ("FilmRollPivotX", "Number", "", "A", TDef::D(0.0)),
    ("FilmRollPivotY", "Number", "", "A", TDef::D(0.0)),
    ("FilmRollValue", "Number", "", "A", TDef::D(0.0)),
    ("FilmRollOrder", "enum", "", "", TDef::I(0)),
    ("ApertureMode", "enum", "", "", TDef::I(2)),
    ("GateFit", "enum", "", "", TDef::I(0)),
    (
        "FieldOfView",
        "FieldOfView",
        "",
        "A",
        TDef::D(25.114999771118164),
    ),
    ("FieldOfViewX", "FieldOfViewX", "", "A", TDef::D(40.0)),
    ("FieldOfViewY", "FieldOfViewY", "", "A", TDef::D(40.0)),
    ("FocalLength", "Number", "", "A", TDef::D(34.89327621672628)),
    ("CameraFormat", "enum", "", "", TDef::I(0)),
    ("UseFrameColor", "bool", "", "", TDef::I(0)),
    (
        "FrameColor",
        "ColorRGB",
        "Color",
        "",
        TDef::V(0.3, 0.3, 0.3),
    ),
    ("ShowName", "bool", "", "", TDef::I(1)),
    ("ShowInfoOnMoving", "bool", "", "", TDef::I(1)),
    ("ShowGrid", "bool", "", "", TDef::I(1)),
    ("ShowOpticalCenter", "bool", "", "", TDef::I(0)),
    ("ShowAzimut", "bool", "", "", TDef::I(1)),
    ("ShowTimeCode", "bool", "", "", TDef::I(0)),
    ("ShowAudio", "bool", "", "", TDef::I(0)),
    (
        "AudioColor",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 1.0, 0.0),
    ),
    ("NearPlane", "double", "Number", "", TDef::D(10.0)),
    ("FarPlane", "double", "Number", "", TDef::D(4000.0)),
    ("AutoComputeClipPanes", "bool", "", "", TDef::I(0)),
    ("ViewCameraToLookAt", "bool", "", "", TDef::I(1)),
    ("ViewFrustumNearFarPlane", "bool", "", "", TDef::I(0)),
    ("ViewFrustumBackPlaneMode", "enum", "", "", TDef::I(2)),
    ("BackPlaneDistance", "Number", "", "A", TDef::D(4000.0)),
    ("BackPlaneDistanceMode", "enum", "", "", TDef::I(1)),
    ("ViewFrustumFrontPlaneMode", "enum", "", "", TDef::I(2)),
    ("FrontPlaneDistance", "Number", "", "A", TDef::D(10.0)),
    ("FrontPlaneDistanceMode", "enum", "", "", TDef::I(1)),
    ("LockMode", "bool", "", "", TDef::I(0)),
    ("LockInterestNavigation", "bool", "", "", TDef::I(0)),
    ("FitImage", "bool", "", "", TDef::I(0)),
    ("Crop", "bool", "", "", TDef::I(0)),
    ("Center", "bool", "", "", TDef::I(1)),
    ("KeepRatio", "bool", "", "", TDef::I(1)),
    (
        "BackgroundAlphaTreshold",
        "double",
        "Number",
        "",
        TDef::D(0.5),
    ),
    ("ShowBackplate", "bool", "", "", TDef::I(1)),
    ("BackPlaneOffsetX", "Number", "", "A", TDef::D(0.0)),
    ("BackPlaneOffsetY", "Number", "", "A", TDef::D(0.0)),
    ("BackPlaneRotation", "Number", "", "A", TDef::D(0.0)),
    ("BackPlaneScaleX", "Number", "", "A", TDef::D(1.0)),
    ("BackPlaneScaleY", "Number", "", "A", TDef::D(1.0)),
    ("Background Texture", "object", "", "", TDef::None),
    ("FrontPlateFitImage", "bool", "", "", TDef::I(1)),
    ("FrontPlateCrop", "bool", "", "", TDef::I(0)),
    ("FrontPlateCenter", "bool", "", "", TDef::I(1)),
    ("FrontPlateKeepRatio", "bool", "", "", TDef::I(1)),
    ("Foreground Opacity", "double", "Number", "", TDef::D(1.0)),
    ("ShowFrontplate", "bool", "", "", TDef::I(1)),
    ("FrontPlaneOffsetX", "Number", "", "A", TDef::D(0.0)),
    ("FrontPlaneOffsetY", "Number", "", "A", TDef::D(0.0)),
    ("FrontPlaneRotation", "Number", "", "A", TDef::D(0.0)),
    ("FrontPlaneScaleX", "Number", "", "A", TDef::D(1.0)),
    ("FrontPlaneScaleY", "Number", "", "A", TDef::D(1.0)),
    ("Foreground Texture", "object", "", "", TDef::None),
    ("DisplaySafeArea", "bool", "", "", TDef::I(0)),
    ("DisplaySafeAreaOnRender", "bool", "", "", TDef::I(0)),
    ("SafeAreaDisplayStyle", "enum", "", "", TDef::I(1)),
    (
        "SafeAreaAspectRatio",
        "double",
        "Number",
        "",
        TDef::D(1.3333333333333333),
    ),
    ("Use2DMagnifierZoom", "bool", "", "", TDef::I(0)),
    ("2D Magnifier Zoom", "Number", "", "A", TDef::D(100.0)),
    ("2D Magnifier X", "Number", "", "A", TDef::D(50.0)),
    ("2D Magnifier Y", "Number", "", "A", TDef::D(50.0)),
    ("CameraProjectionType", "enum", "", "", TDef::I(0)),
    ("OrthoZoom", "double", "Number", "", TDef::D(1.0)),
    ("UseRealTimeDOFAndAA", "bool", "", "", TDef::I(0)),
    ("UseDepthOfField", "bool", "", "", TDef::I(0)),
    ("FocusSource", "enum", "", "", TDef::I(0)),
    ("FocusAngle", "double", "Number", "", TDef::D(3.5)),
    ("FocusDistance", "double", "Number", "", TDef::D(200.0)),
    ("UseAntialiasing", "bool", "", "", TDef::I(0)),
    (
        "AntialiasingIntensity",
        "double",
        "Number",
        "",
        TDef::D(0.77777),
    ),
    ("AntialiasingMethod", "enum", "", "", TDef::I(0)),
    ("UseAccumulationBuffer", "bool", "", "", TDef::I(0)),
    ("FrameSamplingCount", "int", "Integer", "", TDef::I(7)),
    ("FrameSamplingType", "enum", "", "", TDef::I(1)),
];

/// `ObjectType: "Geometry" { PropertyTemplate: "FbxMesh" }` default
/// set, transcribed from the staged fixture's Definitions.
const FBX_MESH_TEMPLATE: &[TRecord] = &[
    ("Color", "ColorRGB", "Color", "", TDef::V(0.8, 0.8, 0.8)),
    ("BBoxMin", "Vector3D", "Vector", "", TDef::V(0.0, 0.0, 0.0)),
    ("BBoxMax", "Vector3D", "Vector", "", TDef::V(0.0, 0.0, 0.0)),
    ("Primary Visibility", "bool", "", "", TDef::I(1)),
    ("Casts Shadows", "bool", "", "", TDef::I(1)),
    ("Receive Shadows", "bool", "", "", TDef::I(1)),
];

/// `ObjectType: "Material" { PropertyTemplate: "FbxSurfaceLambert" }`
/// default set, transcribed from the staged fixture's Definitions.
/// Note the fixture's mixed `"Color"` vs `"ColorRGB"` typeNames —
/// both accepted by the decode side's `as_color_rgb`.
/// `FbxSurfacePhong` — the 22-record body the SDK-authored
/// `docs/3d/fbx/fixtures/texture-video-ascii-v7500.fbx` carries
/// (`fbx-property-templates.md` §6 lists it among that fixture's
/// bodies): the Lambert set plus the specular / reflection records
/// the typed `roughness` / `metallic` fields map onto.
const FBX_SURFACE_PHONG_TEMPLATE: &[TRecord] = &[
    ("ShadingModel", "KString", "", "", TDef::S("Phong")),
    ("MultiLayer", "bool", "", "", TDef::I(0)),
    ("EmissiveColor", "Color", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("EmissiveFactor", "Number", "", "A", TDef::D(1.0)),
    ("AmbientColor", "Color", "", "A", TDef::V(0.2, 0.2, 0.2)),
    ("AmbientFactor", "Number", "", "A", TDef::D(1.0)),
    ("DiffuseColor", "Color", "", "A", TDef::V(0.8, 0.8, 0.8)),
    ("DiffuseFactor", "Number", "", "A", TDef::D(1.0)),
    ("Bump", "Vector3D", "Vector", "", TDef::V(0.0, 0.0, 0.0)),
    (
        "NormalMap",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("BumpFactor", "double", "Number", "", TDef::D(1.0)),
    ("TransparentColor", "Color", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("TransparencyFactor", "Number", "", "A", TDef::D(0.0)),
    (
        "DisplacementColor",
        "ColorRGB",
        "Color",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("DisplacementFactor", "double", "Number", "", TDef::D(1.0)),
    (
        "VectorDisplacementColor",
        "ColorRGB",
        "Color",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "VectorDisplacementFactor",
        "double",
        "Number",
        "",
        TDef::D(1.0),
    ),
    ("SpecularColor", "Color", "", "A", TDef::V(0.2, 0.2, 0.2)),
    ("SpecularFactor", "Number", "", "A", TDef::D(1.0)),
    ("ShininessExponent", "Number", "", "A", TDef::D(20.0)),
    ("ReflectionColor", "Color", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("ReflectionFactor", "Number", "", "A", TDef::D(1.0)),
];

const FBX_SURFACE_LAMBERT_TEMPLATE: &[TRecord] = &[
    ("ShadingModel", "KString", "", "", TDef::S("Lambert")),
    ("MultiLayer", "bool", "", "", TDef::I(0)),
    ("EmissiveColor", "Color", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("EmissiveFactor", "Number", "", "A", TDef::D(1.0)),
    ("AmbientColor", "Color", "", "A", TDef::V(0.2, 0.2, 0.2)),
    ("AmbientFactor", "Number", "", "A", TDef::D(1.0)),
    ("DiffuseColor", "Color", "", "A", TDef::V(0.8, 0.8, 0.8)),
    ("DiffuseFactor", "Number", "", "A", TDef::D(1.0)),
    ("Bump", "Vector3D", "Vector", "", TDef::V(0.0, 0.0, 0.0)),
    (
        "NormalMap",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("BumpFactor", "double", "Number", "", TDef::D(1.0)),
    ("TransparentColor", "Color", "", "A", TDef::V(0.0, 0.0, 0.0)),
    ("TransparencyFactor", "Number", "", "A", TDef::D(0.0)),
    (
        "DisplacementColor",
        "ColorRGB",
        "Color",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("DisplacementFactor", "double", "Number", "", TDef::D(1.0)),
    (
        "VectorDisplacementColor",
        "ColorRGB",
        "Color",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "VectorDisplacementFactor",
        "double",
        "Number",
        "",
        TDef::D(1.0),
    ),
];

/// `ObjectType: "Model" { PropertyTemplate: "FbxNode" }` default set,
/// transcribed from the staged fixture's Definitions. All pivot /
/// offset / pre-post-rotation defaults are zero and `RotationOrder`
/// is `0` (XYZ), so a decode of these defaults stays on the reduced
/// `T * R(XYZ) * S` path [`crate::node_transform`] resolves.
const FBX_NODE_TEMPLATE: &[TRecord] = &[
    ("QuaternionInterpolate", "enum", "", "", TDef::I(0)),
    (
        "RotationOffset",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "RotationPivot",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "ScalingOffset",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "ScalingPivot",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("TranslationActive", "bool", "", "", TDef::I(0)),
    (
        "TranslationMin",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "TranslationMax",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("TranslationMinX", "bool", "", "", TDef::I(0)),
    ("TranslationMinY", "bool", "", "", TDef::I(0)),
    ("TranslationMinZ", "bool", "", "", TDef::I(0)),
    ("TranslationMaxX", "bool", "", "", TDef::I(0)),
    ("TranslationMaxY", "bool", "", "", TDef::I(0)),
    ("TranslationMaxZ", "bool", "", "", TDef::I(0)),
    ("RotationOrder", "enum", "", "", TDef::I(0)),
    ("RotationSpaceForLimitOnly", "bool", "", "", TDef::I(0)),
    ("RotationStiffnessX", "double", "Number", "", TDef::D(0.0)),
    ("RotationStiffnessY", "double", "Number", "", TDef::D(0.0)),
    ("RotationStiffnessZ", "double", "Number", "", TDef::D(0.0)),
    ("AxisLen", "double", "Number", "", TDef::D(10.0)),
    (
        "PreRotation",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "PostRotation",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("RotationActive", "bool", "", "", TDef::I(0)),
    (
        "RotationMin",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "RotationMax",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    ("RotationMinX", "bool", "", "", TDef::I(0)),
    ("RotationMinY", "bool", "", "", TDef::I(0)),
    ("RotationMinZ", "bool", "", "", TDef::I(0)),
    ("RotationMaxX", "bool", "", "", TDef::I(0)),
    ("RotationMaxY", "bool", "", "", TDef::I(0)),
    ("RotationMaxZ", "bool", "", "", TDef::I(0)),
    ("InheritType", "enum", "", "", TDef::I(0)),
    ("ScalingActive", "bool", "", "", TDef::I(0)),
    (
        "ScalingMin",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "ScalingMax",
        "Vector3D",
        "Vector",
        "",
        TDef::V(1.0, 1.0, 1.0),
    ),
    ("ScalingMinX", "bool", "", "", TDef::I(0)),
    ("ScalingMinY", "bool", "", "", TDef::I(0)),
    ("ScalingMinZ", "bool", "", "", TDef::I(0)),
    ("ScalingMaxX", "bool", "", "", TDef::I(0)),
    ("ScalingMaxY", "bool", "", "", TDef::I(0)),
    ("ScalingMaxZ", "bool", "", "", TDef::I(0)),
    (
        "GeometricTranslation",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "GeometricRotation",
        "Vector3D",
        "Vector",
        "",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "GeometricScaling",
        "Vector3D",
        "Vector",
        "",
        TDef::V(1.0, 1.0, 1.0),
    ),
    ("MinDampRangeX", "double", "Number", "", TDef::D(0.0)),
    ("MinDampRangeY", "double", "Number", "", TDef::D(0.0)),
    ("MinDampRangeZ", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampRangeX", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampRangeY", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampRangeZ", "double", "Number", "", TDef::D(0.0)),
    ("MinDampStrengthX", "double", "Number", "", TDef::D(0.0)),
    ("MinDampStrengthY", "double", "Number", "", TDef::D(0.0)),
    ("MinDampStrengthZ", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampStrengthX", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampStrengthY", "double", "Number", "", TDef::D(0.0)),
    ("MaxDampStrengthZ", "double", "Number", "", TDef::D(0.0)),
    ("PreferedAngleX", "double", "Number", "", TDef::D(0.0)),
    ("PreferedAngleY", "double", "Number", "", TDef::D(0.0)),
    ("PreferedAngleZ", "double", "Number", "", TDef::D(0.0)),
    ("LookAtProperty", "object", "", "", TDef::None),
    ("UpVectorProperty", "object", "", "", TDef::None),
    ("Show", "bool", "", "", TDef::I(1)),
    ("NegativePercentShapeSupport", "bool", "", "", TDef::I(1)),
    ("DefaultAttributeIndex", "int", "Integer", "", TDef::I(-1)),
    ("Freeze", "bool", "", "", TDef::I(0)),
    ("LODBox", "bool", "", "", TDef::I(0)),
    (
        "Lcl Translation",
        "Lcl Translation",
        "",
        "A",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "Lcl Rotation",
        "Lcl Rotation",
        "",
        "A",
        TDef::V(0.0, 0.0, 0.0),
    ),
    (
        "Lcl Scaling",
        "Lcl Scaling",
        "",
        "A",
        TDef::V(1.0, 1.0, 1.0),
    ),
    ("Visibility", "Visibility", "", "A", TDef::D(1.0)),
    (
        "Visibility Inheritance",
        "Visibility Inheritance",
        "",
        "",
        TDef::D(1.0),
    ),
];

/// Decode an even-length lowercase/uppercase hex string into bytes
/// (the inverse of the `fbx:file_id` extras rendering). `None` on any
/// length / digit deviation.
fn hex_to_bytes(s: &str) -> Option<Vec<u8>> {
    let b = s.as_bytes();
    if b.len() % 2 != 0 {
        return None;
    }
    let mut out = Vec::with_capacity(b.len() / 2);
    for chunk in b.chunks_exact(2) {
        let hi = (chunk[0] as char).to_digit(16)?;
        let lo = (chunk[1] as char).to_digit(16)?;
        out.push(((hi << 4) | lo) as u8);
    }
    Some(out)
}

/// FBX joins `Name` + `ClassTag` with `\x00\x01` in the binary
/// encoding (the decode path's `element_name` splits on the `\x00`).
fn name_class(name: &str, class: &str) -> Vec<u8> {
    let mut v = name.as_bytes().to_vec();
    v.push(0x00);
    v.push(0x01);
    v.extend_from_slice(class.as_bytes());
    v
}

/// Build the `Pose : "BindPose"` element from the collected
/// `(Model id, world matrix)` pairs — the inverse of
/// [`crate::pose::extract_poses`]'s read side. The record shape is the
/// one the pose module documents from the FBX 7.x element convention:
/// object-record triple `(uid, "BindPose\x00\x01Pose", "BindPose")`
/// with one `PoseNode { Node : i64, Matrix : d[16] }` child per posed
/// bone (`Matrix` is a direct d-array sub-record, row-major, world
/// space).
fn build_bind_pose(entries: Vec<(i64, Vec<f64>)>, id: i64) -> FbxNode {
    let children = entries
        .into_iter()
        .map(|(node_id, matrix)| FbxNode {
            name: "PoseNode".to_string(),
            properties: Vec::new(),
            children: vec![
                FbxNode {
                    name: "Node".to_string(),
                    properties: vec![FbxProperty::I64(node_id)],
                    children: Vec::new(),
                },
                FbxNode {
                    name: "Matrix".to_string(),
                    properties: vec![FbxProperty::F64Array(matrix)],
                    children: Vec::new(),
                },
            ],
        })
        .collect();
    FbxNode {
        name: "Pose".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class("BindPose", "Pose")),
            FbxProperty::String(b"BindPose".to_vec()),
        ],
        children,
    }
}

/// Build a `Geometry` element record from a [`Mesh`].
///
/// Concatenates every primitive's per-corner positions into one
/// `Vertices` array and emits sequential triangle indices into
/// `PolygonVertexIndex`. Only `Topology::Triangles` primitives are
/// encoded geometrically; other topologies are skipped for the vertex
/// table (their positions still appear so nothing is silently lost —
/// they re-triangulate as triangle soup on decode).
fn build_geometry(mesh: &Mesh, id: i64, opts: &SceneEncodeOptions) -> FbxNode {
    let name = mesh.name.clone().unwrap_or_default();
    let mut vertices: Vec<f64> = Vec::new();
    let mut pvi: Vec<i32> = Vec::new();
    let mut normals: Vec<f64> = Vec::new();
    let mut have_normals = true;

    // Per-set attribute accumulators. The decode side surfaces every
    // `LayerElementUV` / `LayerElementColor` in document order as one
    // `Primitive::uvs` / `Primitive::colors` entry each, so the
    // encoder emits one layer record per set. A multi-primitive mesh
    // concatenates per set index; only the set count common to every
    // primitive is emitted (a ragged per-primitive set count has no
    // representation in the one-Geometry-per-mesh layout this writer
    // uses).
    let n_uv_sets = mesh
        .primitives
        .iter()
        .map(|p| p.uvs.len())
        .min()
        .unwrap_or(0);
    let mut uv_sets: Vec<Vec<f64>> = vec![Vec::new(); n_uv_sets];
    let mut uv_valid: Vec<bool> = vec![true; n_uv_sets];
    let n_color_sets = mesh
        .primitives
        .iter()
        .map(|p| p.colors.len())
        .min()
        .unwrap_or(0);
    let mut color_sets: Vec<Vec<f64>> = vec![Vec::new(); n_color_sets];
    let mut color_valid: Vec<bool> = vec![true; n_color_sets];
    // Canonical tangent slot — FBX splits the glTF-style `[x,y,z,w]`
    // into an xyz `Tangents` triple array + a per-corner `TangentsW`
    // handedness-sign array (the shape the decode side recombines).
    let mut tangents_xyz: Vec<f64> = Vec::new();
    let mut tangents_w: Vec<f64> = Vec::new();
    let mut have_tangents = true;
    // Per-triangle material slot indices (`LayerElementMaterial`
    // `ByPolygon` payload — every emitted polygon is a triangle).
    // Only emitted when at least one primitive carries the
    // extras-borne `fbx:face_material_slots` table (round-tripped
    // from a decoded multi-material mesh); primitives without one
    // contribute slot 0.
    let mut face_slots: Vec<i32> = Vec::new();
    let mut have_face_slots = false;

    let mut corner: i32 = 0;
    for prim in &mesh.primitives {
        // Expand the primitive into a flat per-corner position stream.
        let corners = primitive_corner_positions(prim);
        let n_corners = corners.len();
        for [x, y, z] in &corners {
            vertices.push(*x as f64);
            vertices.push(*y as f64);
            vertices.push(*z as f64);
        }
        // PolygonVertexIndex: sequential triangles, last corner of each
        // triangle bit-NOT'd to mark the polygon end (§6 convention).
        let tri_count = n_corners / 3;
        for t in 0..tri_count {
            let base = corner + (t as i32) * 3;
            pvi.push(base);
            pvi.push(base + 1);
            pvi.push(!(base + 2));
        }
        corner += (tri_count as i32) * 3;

        // Normals — only emit when *every* triangulated primitive has a
        // matching per-corner buffer (so the flattened layer length
        // equals the corner count).
        match prim_corner_vec3(prim, prim.normals.as_ref()) {
            Some(buf) if buf.len() == n_corners => {
                for [x, y, z] in &buf {
                    normals.push(*x as f64);
                    normals.push(*y as f64);
                    normals.push(*z as f64);
                }
            }
            _ => have_normals = false,
        }
        // UV sets — every channel present on all primitives.
        for k in 0..n_uv_sets {
            let set = &prim.uvs[k];
            if set.len() != prim.positions.len() {
                uv_valid[k] = false;
                continue;
            }
            let buf = expand_uv(prim, set);
            if buf.len() != n_corners {
                uv_valid[k] = false;
                continue;
            }
            for [u, v] in &buf {
                uv_sets[k].push(*u as f64);
                uv_sets[k].push(*v as f64);
            }
        }
        // Vertex-colour sets — RGBA quadruples per corner.
        for k in 0..n_color_sets {
            let set = &prim.colors[k];
            if set.len() != prim.positions.len() {
                color_valid[k] = false;
                continue;
            }
            let buf = expand_vec4(prim, set);
            if buf.len() != n_corners {
                color_valid[k] = false;
                continue;
            }
            for rgba in &buf {
                for comp in rgba {
                    color_sets[k].push(*comp as f64);
                }
            }
        }
        // Tangents — canonical glTF-style slot only (extras-borne
        // extra layers / binormals are re-emitted separately).
        match &prim.tangents {
            Some(t) if t.len() == prim.positions.len() => {
                let buf = expand_vec4(prim, t);
                if buf.len() == n_corners {
                    for [x, y, z, w] in &buf {
                        tangents_xyz.push(*x as f64);
                        tangents_xyz.push(*y as f64);
                        tangents_xyz.push(*z as f64);
                        tangents_w.push(*w as f64);
                    }
                } else {
                    have_tangents = false;
                }
            }
            _ => have_tangents = false,
        }
        // Per-face material slots — one entry per triangle, pulled
        // from the per-corner extras table (corner 3t speaks for the
        // whole triangle; the decode side broadcast it per corner).
        match prim
            .extras
            .get("fbx:face_material_slots")
            .and_then(|v| v.as_array())
        {
            Some(arr) if arr.len() == n_corners => {
                have_face_slots = true;
                for t in 0..tri_count {
                    let s = arr
                        .get(t * 3)
                        .and_then(|v| v.as_i64())
                        .unwrap_or(0)
                        .clamp(0, i32::MAX as i64) as i32;
                    face_slots.push(s);
                }
            }
            _ => {
                face_slots.resize(face_slots.len() + tri_count, 0);
            }
        }
    }

    let mut children = vec![
        FbxNode {
            name: "Vertices".to_string(),
            properties: vec![FbxProperty::F64Array(vertices)],
            children: Vec::new(),
        },
        FbxNode {
            name: "PolygonVertexIndex".to_string(),
            properties: vec![FbxProperty::I32Array(pvi)],
            children: Vec::new(),
        },
    ];

    if opts.emit_normals && have_normals && !normals.is_empty() {
        children.push(layer_element_vec3("LayerElementNormal", "Normals", normals));
    }
    if opts.emit_uvs {
        // Authored channel labels round-trip via
        // `Primitive::extras["fbx:uv_set_names"]` (one entry per UV
        // set, recorded by the decode side from each `LayerElementUV`
        // `Name` leaf); unnamed channels get the synthesized
        // `map{k+1}` fallback inside `layer_element_uv`.
        let uv_names = mesh
            .primitives
            .first()
            .and_then(|p| p.extras.get("fbx:uv_set_names"))
            .and_then(|v| v.as_array());
        for (k, data) in uv_sets.into_iter().enumerate() {
            if uv_valid[k] && !data.is_empty() {
                let name = uv_names
                    .and_then(|names| names.get(k))
                    .and_then(|v| v.as_str());
                children.push(layer_element_uv(k, name, data));
            }
        }
    }
    if opts.emit_colors {
        for (k, data) in color_sets.into_iter().enumerate() {
            if color_valid[k] && !data.is_empty() {
                children.push(layer_element_color(k, data));
            }
        }
    }
    if opts.emit_tangents && have_tangents && !tangents_xyz.is_empty() {
        children.push(layer_element_tangent(tangents_xyz, tangents_w));
    }
    if have_face_slots && !face_slots.is_empty() {
        children.push(layer_element_material(face_slots));
    }
    // Extras-borne extra layers (round 384) — additional normal /
    // tangent layers + explicitly-authored binormals the decode side
    // flattened onto `Primitive::extras`. Only re-emitted for a
    // single-primitive mesh (the flattened extras are per-primitive,
    // and concatenating them across primitives would be ambiguous —
    // the decode side itself only ever produces one primitive per
    // Geometry).
    if mesh.primitives.len() == 1 {
        let prim = &mesh.primitives[0];
        let n_corners = corner as usize;
        emit_extra_layers(prim, n_corners, &mut children);
        emit_edges_and_smoothing(prim, n_corners, &mut children);
    }

    FbxNode {
        name: "Geometry".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(&name, "Geometry")),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children,
    }
}

/// Re-emit the extras-borne extra layers the decode side flattened:
///
/// - `fbx:extra_normals` (per-layer flat `[x,y,z,…]`, 3 components
///   per corner) → additional `LayerElementNormal` records.
/// - `fbx:extra_tangents` (per-layer flat `[x,y,z,w,…]`) → additional
///   `LayerElementTangent` records (`Tangents` xyz + `TangentsW` w).
/// - `fbx:binormals` (per-layer flat `[x,y,z,w,…]`) →
///   `LayerElementBinormal` records (`Binormals` xyz + `BinormalsW`).
///
/// The flattened buffers are already per-corner, so every re-emitted
/// layer uses the `ByPolygonVertex` / `Direct` mapping regardless of
/// the source file's original mode (the companion `*_mapping` extras
/// record what it was). Layers whose length doesn't match the corner
/// count are skipped. The layer's `TypedIndex` integer is recovered
/// from the `*_typed_index` companion when present, else `i + 1`
/// (slot 0 is the canonical layer).
fn emit_extra_layers(prim: &Primitive, n_corners: usize, children: &mut Vec<FbxNode>) {
    let flat_layers = |key: &str| -> Vec<Vec<f64>> {
        prim.extras
            .get(key)
            .and_then(|v| v.as_array())
            .map(|layers| {
                layers
                    .iter()
                    .filter_map(|l| l.as_array())
                    .map(|a| a.iter().filter_map(|x| x.as_f64()).collect())
                    .collect()
            })
            .unwrap_or_default()
    };
    let typed_index = |key: &str, i: usize| -> i32 {
        prim.extras
            .get(key)
            .and_then(|v| v.as_array())
            .and_then(|a| a.get(i))
            .and_then(|v| v.as_i64())
            .unwrap_or((i + 1) as i64) as i32
    };
    // Split a flat `[x,y,z,w,…]` buffer into the xyz triple array +
    // the per-corner w sign array FBX stores separately.
    let split_xyzw = |flat: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let mut xyz = Vec::with_capacity(flat.len() / 4 * 3);
        let mut w = Vec::with_capacity(flat.len() / 4);
        for chunk in flat.chunks_exact(4) {
            xyz.extend_from_slice(&chunk[..3]);
            w.push(chunk[3]);
        }
        (xyz, w)
    };

    for (i, flat) in flat_layers("fbx:extra_normals").into_iter().enumerate() {
        if flat.len() != n_corners * 3 {
            continue;
        }
        let mut layer = layer_element_vec3("LayerElementNormal", "Normals", flat);
        layer.properties = vec![FbxProperty::I32(typed_index(
            "fbx:extra_normals_typed_index",
            i,
        ))];
        children.push(layer);
    }
    for (i, flat) in flat_layers("fbx:extra_tangents").into_iter().enumerate() {
        if flat.len() != n_corners * 4 {
            continue;
        }
        let (xyz, w) = split_xyzw(&flat);
        let mut layer = layer_element_tangent(xyz, w);
        layer.properties = vec![FbxProperty::I32(typed_index(
            "fbx:extra_tangents_typed_index",
            i,
        ))];
        children.push(layer);
    }
    for (i, flat) in flat_layers("fbx:binormals").into_iter().enumerate() {
        if flat.len() != n_corners * 4 {
            continue;
        }
        let (xyz, w) = split_xyzw(&flat);
        children.push(FbxNode {
            name: "LayerElementBinormal".to_string(),
            properties: vec![FbxProperty::I32(i as i32)],
            children: vec![
                leaf_i32("Version", 101),
                leaf_string("Name", ""),
                leaf_string("MappingInformationType", "ByPolygonVertex"),
                leaf_string("ReferenceInformationType", "Direct"),
                FbxNode {
                    name: "Binormals".to_string(),
                    properties: vec![FbxProperty::F64Array(xyz)],
                    children: Vec::new(),
                },
                FbxNode {
                    name: "BinormalsW".to_string(),
                    properties: vec![FbxProperty::F64Array(w)],
                    children: Vec::new(),
                },
            ],
        });
    }
}

/// Re-emit the `Edges` array + `LayerElementSmoothing` layer the
/// decode side surfaced on `Primitive::extras` (per
/// `docs/3d/fbx/fbx-edges-smoothing-layer.md`).
///
/// The emitted geometry is a disconnected triangle list — every
/// corner owns its own `Vertices` entry — so the mesh's unique-edge
/// set (§1: what `Edges` enumerates, one entry per undirected edge,
/// each value the edge's start corner in `PolygonVertexIndex`) is
/// exactly one edge per corner slot: edge `i` starts at corner `i`
/// and runs to the next corner in its triangle, wrapping at the
/// closing corner. `Edges` is therefore the identity enumeration
/// `0..corner_count`, and a `ByEdge` `Smoothing` array is the
/// per-corner buffer verbatim — which is what makes the
/// decode→encode→decode round trip preserve `fbx:smoothing` exactly
/// (the source file's edge *count* is not preserved, because the
/// per-corner layout un-shares the edges two source polygons shared).
///
/// - `fbx:smoothing` + `fbx:smoothing_mapping == "ByEdge"` →
///   `Edges: 0..N` + a `ByEdge`/`Direct` `LayerElementSmoothing`
///   whose per-edge array is the per-corner flags verbatim.
/// - `fbx:smoothing_mapping == "ByPolygon"` → one smoothing-group
///   bitmask per emitted triangle-polygon (corner `3t` speaks for
///   the whole triangle, the `fbx:face_material_slots` convention —
///   the decode side broadcast the polygon value to every corner).
/// - `fbx:edges` present without a usable smoothing layer → the
///   `Edges` enumeration alone (the decoded pairs index the source
///   file's shared-vertex table, which the per-corner layout does
///   not preserve; the emitted mesh's own edge set is the full
///   corner enumeration).
///
/// The `Edges` node is inserted right after `Vertices` /
/// `PolygonVertexIndex`, matching observed exporter layout (cosmetic
/// — the reader looks Geometry children up by name).
fn emit_edges_and_smoothing(prim: &Primitive, n_corners: usize, children: &mut Vec<FbxNode>) {
    let smoothing: Option<Vec<i64>> = prim
        .extras
        .get("fbx:smoothing")
        .and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|x| x.as_i64()).collect());
    let mapping = prim
        .extras
        .get("fbx:smoothing_mapping")
        .and_then(|v| v.as_str());
    let usable = smoothing.as_ref().is_some_and(|s| s.len() == n_corners);
    let by_edge = usable && mapping == Some("ByEdge");
    if by_edge || prim.extras.contains_key("fbx:edges") {
        let edges = FbxNode {
            name: "Edges".to_string(),
            properties: vec![FbxProperty::I32Array((0..n_corners as i32).collect())],
            children: Vec::new(),
        };
        children.insert(2.min(children.len()), edges);
    }
    if !usable {
        return;
    }
    let s = smoothing.unwrap_or_default();
    match mapping {
        Some("ByEdge") => children.push(layer_element_smoothing("ByEdge", &s)),
        Some("ByPolygon") => {
            let per_tri: Vec<i64> = (0..n_corners / 3).map(|t| s[t * 3]).collect();
            children.push(layer_element_smoothing("ByPolygon", &per_tri));
        }
        _ => {}
    }
}

/// `LayerElementSmoothing` — `Smoothing` `i`-array under the given
/// mapping mode, `Direct`-referenced (§4a of
/// `docs/3d/fbx/fbx-edges-smoothing-layer.md`). `Version: 102` is the
/// value observed on the staged fixture's smoothing layers.
fn layer_element_smoothing(mapping: &str, values: &[i64]) -> FbxNode {
    FbxNode {
        name: "LayerElementSmoothing".to_string(),
        properties: vec![FbxProperty::I32(0)],
        children: vec![
            leaf_i32("Version", 102),
            leaf_string("Name", ""),
            leaf_string("MappingInformationType", mapping),
            leaf_string("ReferenceInformationType", "Direct"),
            FbxNode {
                name: "Smoothing".to_string(),
                properties: vec![FbxProperty::I32Array(
                    values.iter().map(|&v| v as i32).collect(),
                )],
                children: Vec::new(),
            },
        ],
    }
}

/// Flatten a primitive into per-corner triangle positions. Triangle
/// topologies stay as-is; indexed primitives expand through the index
/// buffer; non-triangle topologies fall back to their raw positions.
fn primitive_corner_positions(prim: &Primitive) -> Vec<[f32; 3]> {
    match &prim.indices {
        Some(indices) => expand_indexed(&prim.positions, indices),
        None => prim.positions.clone(),
    }
}

/// Expand an indexed position buffer into a flat per-corner stream.
fn expand_indexed(positions: &[[f32; 3]], indices: &Indices) -> Vec<[f32; 3]> {
    let idx_iter: Vec<usize> = match indices {
        Indices::U16(v) => v.iter().map(|&i| i as usize).collect(),
        Indices::U32(v) => v.iter().map(|&i| i as usize).collect(),
    };
    idx_iter
        .into_iter()
        .filter_map(|i| positions.get(i).copied())
        .collect()
}

/// Expand a per-vertex vec3 attribute (normals) into a per-corner
/// stream matching [`primitive_corner_positions`].
fn prim_corner_vec3(prim: &Primitive, attr: Option<&Vec<[f32; 3]>>) -> Option<Vec<[f32; 3]>> {
    let attr = attr?;
    if attr.len() != prim.positions.len() {
        return None;
    }
    Some(match &prim.indices {
        Some(Indices::U16(v)) => v
            .iter()
            .filter_map(|&i| attr.get(i as usize).copied())
            .collect(),
        Some(Indices::U32(v)) => v
            .iter()
            .filter_map(|&i| attr.get(i as usize).copied())
            .collect(),
        None => attr.clone(),
    })
}

/// Expand a per-vertex UV set into a per-corner stream.
fn expand_uv(prim: &Primitive, set: &[[f32; 2]]) -> Vec<[f32; 2]> {
    match &prim.indices {
        Some(Indices::U16(v)) => v
            .iter()
            .filter_map(|&i| set.get(i as usize).copied())
            .collect(),
        Some(Indices::U32(v)) => v
            .iter()
            .filter_map(|&i| set.get(i as usize).copied())
            .collect(),
        None => set.to_vec(),
    }
}

/// Expand a per-vertex 4-component attribute (vertex colours RGBA /
/// tangents xyzw) into a per-corner stream.
fn expand_vec4(prim: &Primitive, set: &[[f32; 4]]) -> Vec<[f32; 4]> {
    match &prim.indices {
        Some(Indices::U16(v)) => v
            .iter()
            .filter_map(|&i| set.get(i as usize).copied())
            .collect(),
        Some(Indices::U32(v)) => v
            .iter()
            .filter_map(|&i| set.get(i as usize).copied())
            .collect(),
        None => set.to_vec(),
    }
}

/// `LayerElement{Normal}` (or similar vec3 layer) with the
/// `ByPolygonVertex` / `Direct` mapping the geometry puller flattens
/// 1:1. The `d`-array data name matches what the puller looks up
/// (`Normals`).
fn layer_element_vec3(layer_name: &str, data_name: &str, data: Vec<f64>) -> FbxNode {
    FbxNode {
        name: layer_name.to_string(),
        properties: vec![FbxProperty::I32(0)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", ""),
            leaf_string("MappingInformationType", "ByPolygonVertex"),
            leaf_string("ReferenceInformationType", "Direct"),
            FbxNode {
                name: data_name.to_string(),
                properties: vec![FbxProperty::F64Array(data)],
                children: Vec::new(),
            },
        ],
    }
}

/// `LayerElementUV` — same mapping shape as the vec3 layer but the
/// data record is named `UV`. `index` is the layer's `TypedIndex`
/// integer (the §6-point-4 sub-discriminator distinguishing multiple
/// UV channels on one `Geometry`). `name` is the channel's authored
/// label (round-tripped from `Primitive::extras["fbx:uv_set_names"]`);
/// `None` / empty falls back to the synthesized `map{index+1}` so the
/// `Texture` element's `UVSet` join always has a label to match.
fn layer_element_uv(index: usize, name: Option<&str>, data: Vec<f64>) -> FbxNode {
    let label = match name {
        Some(n) if !n.is_empty() => n.to_owned(),
        _ => format!("map{}", index + 1),
    };
    FbxNode {
        name: "LayerElementUV".to_string(),
        properties: vec![FbxProperty::I32(index as i32)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", &label),
            leaf_string("MappingInformationType", "ByPolygonVertex"),
            leaf_string("ReferenceInformationType", "Direct"),
            FbxNode {
                name: "UV".to_string(),
                properties: vec![FbxProperty::F64Array(data)],
                children: Vec::new(),
            },
        ],
    }
}

/// `LayerElementColor` — RGBA vertex-colour layer. The `Colors`
/// `d`-array carries 4-component quadruples (the decode side's
/// `pull_layer_vec4` shape); mapping is the same `ByPolygonVertex` /
/// `Direct` form the other layers use.
fn layer_element_color(index: usize, data: Vec<f64>) -> FbxNode {
    FbxNode {
        name: "LayerElementColor".to_string(),
        properties: vec![FbxProperty::I32(index as i32)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", &format!("colorSet{}", index + 1)),
            leaf_string("MappingInformationType", "ByPolygonVertex"),
            leaf_string("ReferenceInformationType", "Direct"),
            FbxNode {
                name: "Colors".to_string(),
                properties: vec![FbxProperty::F64Array(data)],
                children: Vec::new(),
            },
        ],
    }
}

/// `LayerElementTangent` — xyz `Tangents` triple array + companion
/// per-corner `TangentsW` handedness-sign array (the split the decode
/// side recombines into the glTF-style `[x,y,z,w]` slot).
fn layer_element_tangent(xyz: Vec<f64>, w: Vec<f64>) -> FbxNode {
    FbxNode {
        name: "LayerElementTangent".to_string(),
        properties: vec![FbxProperty::I32(0)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", ""),
            leaf_string("MappingInformationType", "ByPolygonVertex"),
            leaf_string("ReferenceInformationType", "Direct"),
            FbxNode {
                name: "Tangents".to_string(),
                properties: vec![FbxProperty::F64Array(xyz)],
                children: Vec::new(),
            },
            FbxNode {
                name: "TangentsW".to_string(),
                properties: vec![FbxProperty::F64Array(w)],
                children: Vec::new(),
            },
        ],
    }
}

/// `LayerElementMaterial` — per-polygon material slot indices
/// (`ByPolygon` / `IndexToDirect`, the form the decode side's
/// material-slot puller reads; slot indices key the
/// `Material -> Model` OO connections in document order).
fn layer_element_material(per_polygon_slots: Vec<i32>) -> FbxNode {
    FbxNode {
        name: "LayerElementMaterial".to_string(),
        properties: vec![FbxProperty::I32(0)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", ""),
            leaf_string("MappingInformationType", "ByPolygon"),
            leaf_string("ReferenceInformationType", "IndexToDirect"),
            FbxNode {
                name: "Materials".to_string(),
                properties: vec![FbxProperty::I32Array(per_polygon_slots)],
                children: Vec::new(),
            },
        ],
    }
}

/// Build a `Model` element record from a scene-graph [`Node`].
fn build_model(node: &Node, id: i64) -> FbxNode {
    let name = node.name.clone().unwrap_or_default();
    let mut children = Vec::new();

    // Body leaves round-tripped verbatim (`fbx:model_leaves`:
    // `Version` / `MultiLayer` / `MultiTake` …). `Version` leads the
    // body in every staged fixture; the rest follow `Properties70`.
    // A fresh scene gets the fixture-observed `Version: 232`.
    let leaves: Vec<FbxNode> = node
        .extras
        .get("fbx:model_leaves")
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_leaf)
        .collect();
    let (version_leaf, other_leaves): (Vec<FbxNode>, Vec<FbxNode>) =
        leaves.into_iter().partition(|l| l.name == "Version");
    match version_leaf.into_iter().next() {
        Some(v) => children.push(v),
        None => children.push(leaf_i32("Version", 232)),
    }

    // `Properties70`: the round-tripped raw record set verbatim
    // (`fbx:model_records`) as long as every chain-mapped record in
    // it still carries the value the typed emission would write —
    // i.e. nobody edited the typed transform / chain extras — else
    // the typed chain records win and every other raw record still
    // rides along.
    let typed = build_node_transform_props(node);
    let raw: Vec<FbxNode> = node
        .extras
        .get("fbx:model_records")
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_p_record)
        .collect();
    let props70 = if !raw.is_empty() && raw_records_agree(&raw, &typed.children, MODEL_DEFAULTS) {
        FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: raw,
        }
    } else {
        let mut ps = typed.children;
        for r in raw {
            let keep = crate::properties70::p_name(&r)
                .is_some_and(|n| !MODEL_DEFAULTS.iter().any(|(t, _)| *t == n));
            if keep {
                ps.push(r);
            }
        }
        FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: ps,
        }
    };
    if !props70.children.is_empty() {
        children.push(props70);
    }
    children.extend(other_leaves);

    // Trailing Model-body leaves (`docs/3d/fbx/fbx-ascii-grammar.md`
    // §7c: `Shading: T` / `Culling: "CullingOff"`) — re-emitted from
    // the decode-side `fbx:shading` / `fbx:culling` extras so they
    // survive the Scene3D round trip.
    if let Some(shading) = node.extras.get("fbx:shading").and_then(|v| v.as_bool()) {
        children.push(FbxNode {
            name: "Shading".to_string(),
            properties: vec![FbxProperty::Bool(shading)],
            children: Vec::new(),
        });
    }
    if let Some(culling) = node.extras.get("fbx:culling").and_then(|v| v.as_str()) {
        children.push(leaf_string("Culling", culling));
    }
    // prop2 subtype discriminator (`fbx-binary-properties70.md` §6) —
    // `"Mesh"` unless the decode side surfaced a different Model
    // subtype (`"LimbNode"` / `"Null"` / ... ) on
    // `extras["fbx:model_subtype"]`.
    let subtype = node
        .extras
        .get("fbx:model_subtype")
        .and_then(|v| v.as_str())
        .unwrap_or("Mesh");
    FbxNode {
        name: "Model".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(&name, "Model")),
            FbxProperty::String(subtype.as_bytes().to_vec()),
        ],
        children,
    }
}

/// The transform-chain record names the typed `Node` owns, with the
/// value each takes when absent (the `FbxNode` template defaults —
/// identity chain, XYZ order, `RrSs` inheritance).
const MODEL_DEFAULTS: &[(&str, &[f64])] = &[
    ("Lcl Translation", &[0.0, 0.0, 0.0]),
    ("Lcl Rotation", &[0.0, 0.0, 0.0]),
    ("Lcl Scaling", &[1.0, 1.0, 1.0]),
    ("RotationOffset", &[0.0, 0.0, 0.0]),
    ("RotationPivot", &[0.0, 0.0, 0.0]),
    ("PreRotation", &[0.0, 0.0, 0.0]),
    ("PostRotation", &[0.0, 0.0, 0.0]),
    ("ScalingOffset", &[0.0, 0.0, 0.0]),
    ("ScalingPivot", &[0.0, 0.0, 0.0]),
    ("GeometricTranslation", &[0.0, 0.0, 0.0]),
    ("GeometricRotation", &[0.0, 0.0, 0.0]),
    ("GeometricScaling", &[1.0, 1.0, 1.0]),
    ("RotationOrder", &[0.0]),
    ("InheritType", &[0.0]),
];

/// Does a round-tripped raw `P` set still agree with the typed
/// emission on every typed-mapped name? A name absent on one side
/// takes its documented default. Compared numerically to 1e-6 so
/// the raw records' wire form (`Lcl Rotation` vs `Vector3D`, ints
/// vs doubles) is not what decides.
fn raw_records_agree(raw: &[FbxNode], typed: &[FbxNode], defaults: &[(&str, &[f64])]) -> bool {
    let find = |set: &[FbxNode], name: &str| -> Option<Vec<f64>> {
        set.iter()
            .find(|p| crate::properties70::p_name(p) == Some(name))
            .map(crate::properties70::p_numeric_values)
    };
    // `Lcl Rotation` is compared as a rotation, not as a triple: the
    // typed emission decomposes `Node::transform` into *an* Euler
    // triple for the record's `RotationOrder`, which may be a
    // different branch of the same rotation than the producer wrote.
    let order = find(raw, "RotationOrder")
        .and_then(|v| v.first().copied())
        .and_then(|v| crate::node_transform::RotationOrder::from_enum_int(v as i64))
        .unwrap_or(crate::node_transform::RotationOrder::Xyz);
    defaults.iter().all(|(name, default)| {
        let a = find(raw, name).unwrap_or_else(|| default.to_vec());
        let b = find(typed, name).unwrap_or_else(|| default.to_vec());
        if a.len() != b.len() {
            return false;
        }
        if *name == "Lcl Rotation" && a.len() == 3 {
            let qa = crate::node_transform::euler_to_quat([a[0], a[1], a[2]], order);
            let qb = crate::node_transform::euler_to_quat([b[0], b[1], b[2]], order);
            let dot: f64 = qa.iter().zip(&qb).map(|(x, y)| x * y).sum();
            return (dot.abs() - 1.0).abs() <= 1.0e-6;
        }
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() <= 1.0e-6)
    })
}

/// Build the `Properties70` block carrying the node-transform chain.
///
/// Two source forms (mirroring `crate::node_transform`'s decode):
///
/// - **Authored-chain form** — the node carries `fbx:lcl_*` extras
///   (the decode side surfaces them whenever any pivot / offset /
///   Pre-/PostRotation / non-XYZ `RotationOrder` is non-trivial;
///   `Node::transform` then holds the *composed* reduction). The
///   authored `Lcl` triple is re-emitted from the extras — never from
///   `Node::transform`, which would double-apply the pivot terms —
///   alongside the chain-extension records below.
/// - **Plain form** — no chain extras: `Node::transform` decomposes
///   to the `Lcl` triple directly (`T * R(XYZ) * S`, the exact
///   inverse of the decode-side composition for a trivial chain).
///   Only non-default components are emitted (the decode path
///   resolves omissions against the template / identity default), so
///   an identity transform produces no records.
///
/// In both forms every surfaced chain extra is re-emitted with its
/// documented record shape: `Vector3D` triples for pivots / offsets /
/// Pre-/PostRotation and the doc-§2 geometric TRS, `enum` ints for
/// `RotationOrder` / `InheritType`.
fn build_node_transform_props(node: &Node) -> FbxNode {
    let mut ps: Vec<FbxNode> = Vec::new();

    let authored_chain = node.extras.contains_key("fbx:lcl_translation")
        || node.extras.contains_key("fbx:lcl_rotation")
        || node.extras.contains_key("fbx:lcl_scaling");
    let (translation, rotation_deg, scale) = if authored_chain {
        (
            crate::node_transform::extras_vec3(node, "fbx:lcl_translation").unwrap_or([0.0; 3]),
            crate::node_transform::extras_vec3(node, "fbx:lcl_rotation").unwrap_or([0.0; 3]),
            crate::node_transform::extras_vec3(node, "fbx:lcl_scaling").unwrap_or([1.0, 1.0, 1.0]),
        )
    } else {
        decompose_trs(node.transform)
    };
    // The `Lcl` triple is always written (every staged producer
    // writes all three even at their identity defaults).
    ps.push(p_lcl("Lcl Translation", translation));
    ps.push(p_lcl("Lcl Rotation", rotation_deg));
    ps.push(p_lcl("Lcl Scaling", scale));

    // Chain-extension + geometric-TRS records (doc §1 / §2 names).
    for (key, name) in [
        ("fbx:rotation_offset", "RotationOffset"),
        ("fbx:rotation_pivot", "RotationPivot"),
        ("fbx:pre_rotation", "PreRotation"),
        ("fbx:post_rotation", "PostRotation"),
        ("fbx:scaling_offset", "ScalingOffset"),
        ("fbx:scaling_pivot", "ScalingPivot"),
        ("fbx:geometric_translation", "GeometricTranslation"),
        ("fbx:geometric_rotation", "GeometricRotation"),
        ("fbx:geometric_scaling", "GeometricScaling"),
    ] {
        if let Some(v) = crate::node_transform::extras_vec3(node, key) {
            ps.push(p_vector3d(name, v));
        }
    }
    for (key, name) in [
        ("fbx:rotation_order", "RotationOrder"),
        ("fbx:inherit_type", "InheritType"),
    ] {
        if let Some(v) = node.extras.get(key).and_then(|v| v.as_i64()) {
            ps.push(p_enum(name, v as i32));
        }
    }

    FbxNode {
        name: "Properties70".to_string(),
        properties: Vec::new(),
        children: ps,
    }
}

/// `P: "<name>", "Vector3D", "Vector", "", x, y, z` — the `Vector3D`
/// triple shape the fixture's `FbxNode` template uses for the
/// transform-chain records (`RotationPivot` / `PreRotation` / ...).
fn p_vector3d(name: &str, v: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"Vector3D".to_vec()),
            FbxProperty::String(b"Vector".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::F64(v[0]),
            FbxProperty::F64(v[1]),
            FbxProperty::F64(v[2]),
        ],
        children: Vec::new(),
    }
}

/// Decompose a [`Transform`] into FBX `(translation, rotation_degXYZ,
/// scale)`. The rotation is recovered as XYZ-Euler degrees — the
/// convention [`crate::node_transform`] decodes via
/// `euler_xyz_to_quat`.
fn decompose_trs(t: Transform) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let (translation, rotation_quat, scale) = match t {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => (translation, rotation, scale),
        Transform::Matrix(m) => match Transform::from_matrix(m) {
            Transform::Trs {
                translation,
                rotation,
                scale,
            } => (translation, rotation, scale),
            // from_matrix always returns Trs; unreachable in practice.
            Transform::Matrix(_) => ([0.0; 3], [0.0, 0.0, 0.0, 1.0], [1.0; 3]),
        },
    };
    let euler = quat_to_euler_xyz_deg(rotation_quat);
    (
        [
            translation[0] as f64,
            translation[1] as f64,
            translation[2] as f64,
        ],
        [euler[0] as f64, euler[1] as f64, euler[2] as f64],
        [scale[0] as f64, scale[1] as f64, scale[2] as f64],
    )
}

/// Crate-internal re-export of [`quat_to_euler_xyz_deg`] for the
/// [`crate::anim_writer`] rotation-curve emitter.
pub(crate) fn quat_to_euler_xyz_deg_pub(q: [f32; 4]) -> [f32; 3] {
    quat_to_euler_xyz_deg(q)
}

/// Inverse of [`crate::animation::euler_xyz_to_quat`] — recover XYZ
/// intrinsic Euler angles (degrees) from an xyzw quaternion.
///
/// The forward map composes `q = qz * qy * qx` (apply Rx, then Ry,
/// then Rz). This recovers the angles assuming that order; it is exact
/// for axis-aligned rotations and stable away from the ±90° pitch
/// gimbal singularity.
fn quat_to_euler_xyz_deg(q: [f32; 4]) -> [f32; 3] {
    let [x, y, z, w] = q;
    let to_deg = 180.0 / std::f32::consts::PI;
    // ZYX-style extraction for the q = qz*qy*qx composition.
    // roll (x-axis)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);
    // pitch (y-axis)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        (std::f32::consts::FRAC_PI_2).copysign(sinp)
    } else {
        sinp.asin()
    };
    // yaw (z-axis)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);
    [roll * to_deg, pitch * to_deg, yaw * to_deg]
}

/// `P: "<name>", "<name>", "", "A", v0, v1, v2` — an animatable triple
/// P-record (the `Lcl …` transform shape the cubes fixture carries).
fn p_lcl(name: &str, v: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(v[0]),
            FbxProperty::F64(v[1]),
            FbxProperty::F64(v[2]),
        ],
        children: Vec::new(),
    }
}

/// Build a `NodeAttribute : "Light"` element — the inverse of the
/// decode side's light decoder. The `LightType` enum int (0=Point,
/// 1=Directional, 2=Spot, 3=Area, 4=Volume) is recovered from the
/// typed [`oxideav_mesh3d::Light`] variant, with the lossy
/// `Area` / `Volume` → `Point` collapse undone via the owning node's
/// `extras["fbx:light_type"]` tag. `Intensity` re-applies the DCC
/// percentage scale (mesh3d intensity × 100); a `range` becomes
/// `DecayType != 0` + `DecayStart` (the decode-side promotion rule).
fn build_light_attribute(light: &oxideav_mesh3d::Light, node: &Node, id: i64) -> FbxNode {
    use oxideav_mesh3d::Light;

    let kind_tag = node
        .extras
        .get("fbx:light_type")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    let (light_type, color, intensity, range) = match light {
        Light::Directional { color, intensity } => (1, *color, *intensity, None),
        Light::Spot {
            color,
            intensity,
            range,
            ..
        } => (2, *color, *intensity, *range),
        Light::Point {
            color,
            intensity,
            range,
        } => {
            let lt = match kind_tag {
                "Area" => 3,
                "Volume" => 4,
                _ => 0,
            };
            (lt, *color, *intensity, *range)
        }
    };

    let mut ps: Vec<FbxNode> = vec![
        p_int("LightType", light_type),
        p_color("Color", [color[0] as f64, color[1] as f64, color[2] as f64]),
        p_number("Intensity", intensity as f64 * 100.0),
    ];
    // DecayType: keep the round-tripped enum value when the node
    // carries it; otherwise 1 (linear) when a range cutoff exists and
    // 0 (none) when it doesn't — the decode side only promotes
    // DecayStart to `range` when DecayType != 0.
    let decay_type = node
        .extras
        .get("fbx:decay_type")
        .and_then(|v| v.as_i64())
        .map(|v| v as i32)
        .unwrap_or(if range.is_some() { 1 } else { 0 });
    ps.push(p_int("DecayType", decay_type));
    if let Some(r) = range {
        ps.push(p_double("DecayStart", r as f64));
    }
    if let Light::Spot {
        inner_cone_angle,
        outer_cone_angle,
        ..
    } = light
    {
        // mesh3d half-cone radians → FBX full-cone degrees.
        let to_full_deg = |half_rad: f32| (half_rad as f64) * 2.0 * 180.0 / std::f64::consts::PI;
        ps.push(p_double("InnerAngle", to_full_deg(*inner_cone_angle)));
        ps.push(p_double("OuterAngle", to_full_deg(*outer_cone_angle)));
    }
    if let Some(b) = node
        .extras
        .get("fbx:cast_shadows")
        .and_then(|v| v.as_bool())
    {
        ps.push(p_bool("CastShadows", b));
    }

    // Verbatim path: the round-tripped raw record set re-decodes to
    // this very light (typed fields + kind tag + extras), so it is
    // emitted untouched; else the typed records above win and the
    // raw records outside the typed light mapping ride along.
    let raw = attribute_raw_records(node);
    let agrees = !raw.is_empty() && {
        let (l2, tag2, extras2) = crate::lights_cameras::light_from_records(raw.clone());
        light_eq(light, &l2)
            && tag2.as_deref() == node.extras.get("fbx:light_type").and_then(|v| v.as_str())
            && extras2.iter().all(|(k, v)| node.extras.get(k) == Some(v))
    };
    let ps = merge_typed_and_raw(ps, raw, agrees, LIGHT_TYPED_NAMES);
    node_attribute(id, node, "Light", ps)
}

/// The `P` names the typed [`oxideav_mesh3d::Light`] decode consumes.
const LIGHT_TYPED_NAMES: &[&str] = &[
    "LightType",
    "Color",
    "Intensity",
    "DecayType",
    "DecayStart",
    "InnerAngle",
    "OuterAngle",
    "CastShadows",
];

/// The `P` names the typed [`oxideav_mesh3d::Camera`] decode consumes.
const CAMERA_TYPED_NAMES: &[&str] = &[
    "CameraProjectionType",
    "FieldOfView",
    "FieldOfViewX",
    "FieldOfViewY",
    "NearPlane",
    "FarPlane",
    "AspectWidth",
    "AspectHeight",
    "OrthoZoom",
];

fn attribute_raw_records(node: &Node) -> Vec<FbxNode> {
    node.extras
        .get("fbx:node_attribute_records")
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_p_record)
        .collect()
}

/// Either the raw set verbatim (`agrees`) or the typed records plus
/// every raw record whose name the typed mapping does not own.
fn merge_typed_and_raw(
    typed: Vec<FbxNode>,
    raw: Vec<FbxNode>,
    agrees: bool,
    typed_names: &[&str],
) -> Vec<FbxNode> {
    if agrees {
        return raw;
    }
    let mut ps = typed;
    for r in raw {
        if crate::properties70::p_name(&r).is_some_and(|n| !typed_names.contains(&n)) {
            ps.push(r);
        }
    }
    ps
}

fn close(a: f32, b: f32) -> bool {
    (a - b).abs() <= 1.0e-5 * a.abs().max(b.abs()).max(1.0)
}

fn light_eq(a: &oxideav_mesh3d::Light, b: &oxideav_mesh3d::Light) -> bool {
    use oxideav_mesh3d::Light;
    let opt = |x: Option<f32>, y: Option<f32>| match (x, y) {
        (None, None) => true,
        (Some(x), Some(y)) => close(x, y),
        _ => false,
    };
    let rgb = |x: [f32; 3], y: [f32; 3]| x.iter().zip(&y).all(|(p, q)| close(*p, *q));
    match (a, b) {
        (
            Light::Directional { color, intensity },
            Light::Directional {
                color: c2,
                intensity: i2,
            },
        ) => rgb(*color, *c2) && close(*intensity, *i2),
        (
            Light::Point {
                color,
                intensity,
                range,
            },
            Light::Point {
                color: c2,
                intensity: i2,
                range: r2,
            },
        ) => rgb(*color, *c2) && close(*intensity, *i2) && opt(*range, *r2),
        (
            Light::Spot {
                color,
                intensity,
                range,
                inner_cone_angle,
                outer_cone_angle,
            },
            Light::Spot {
                color: c2,
                intensity: i2,
                range: r2,
                inner_cone_angle: in2,
                outer_cone_angle: out2,
            },
        ) => {
            rgb(*color, *c2)
                && close(*intensity, *i2)
                && opt(*range, *r2)
                && close(*inner_cone_angle, *in2)
                && close(*outer_cone_angle, *out2)
        }
        _ => false,
    }
}

fn camera_eq(a: &oxideav_mesh3d::Camera, b: &oxideav_mesh3d::Camera) -> bool {
    use oxideav_mesh3d::Camera;
    let opt = |x: Option<f32>, y: Option<f32>| match (x, y) {
        (None, None) => true,
        (Some(x), Some(y)) => close(x, y),
        _ => false,
    };
    match (a, b) {
        (
            Camera::Perspective {
                aspect_ratio,
                yfov,
                znear,
                zfar,
            },
            Camera::Perspective {
                aspect_ratio: a2,
                yfov: y2,
                znear: n2,
                zfar: f2,
            },
        ) => opt(*aspect_ratio, *a2) && close(*yfov, *y2) && close(*znear, *n2) && opt(*zfar, *f2),
        (
            Camera::Orthographic {
                xmag,
                ymag,
                znear,
                zfar,
            },
            Camera::Orthographic {
                xmag: x2,
                ymag: y2,
                znear: n2,
                zfar: f2,
            },
        ) => close(*xmag, *x2) && close(*ymag, *y2) && close(*znear, *n2) && close(*zfar, *f2),
        _ => false,
    }
}

/// Build a `NodeAttribute : "Camera"` element — the inverse of the
/// decode side's camera decoder. Perspective cameras emit
/// `FieldOfViewY` (the decode side's highest-priority source, a 1:1
/// `yfov` mapping); orthographic cameras emit `OrthoZoom` (the
/// vertical half-extent, `ymag`). `AspectWidth` / `AspectHeight`
/// reproduce the authored resolution pair from
/// `extras["fbx:camera_resolution"]` when present, else encode the
/// bare ratio as `w = ratio, h = 1`.
fn build_camera_attribute(camera: &oxideav_mesh3d::Camera, node: &Node, id: i64) -> FbxNode {
    use oxideav_mesh3d::Camera;

    let resolution = node
        .extras
        .get("fbx:camera_resolution")
        .and_then(|v| v.as_array())
        .and_then(|a| {
            let w = a.first().and_then(|v| v.as_f64())?;
            let h = a.get(1).and_then(|v| v.as_f64())?;
            Some((w, h))
        });

    let mut ps: Vec<FbxNode> = Vec::new();
    match camera {
        Camera::Perspective {
            aspect_ratio,
            yfov,
            znear,
            zfar,
        } => {
            ps.push(p_int("CameraProjectionType", 0));
            ps.push(p_double(
                "FieldOfViewY",
                (*yfov as f64) * 180.0 / std::f64::consts::PI,
            ));
            ps.push(p_double("NearPlane", *znear as f64));
            if let Some(far) = zfar {
                ps.push(p_double("FarPlane", *far as f64));
            }
            let (w, h) = resolution
                .or(aspect_ratio.map(|ar| (ar as f64, 1.0)))
                .unwrap_or((16.0, 9.0));
            ps.push(p_double("AspectWidth", w));
            ps.push(p_double("AspectHeight", h));
        }
        Camera::Orthographic {
            xmag,
            ymag,
            znear,
            zfar,
        } => {
            ps.push(p_int("CameraProjectionType", 1));
            // OrthoZoom is the vertical half-extent; the horizontal
            // extent reconstructs via the aspect ratio.
            ps.push(p_double("OrthoZoom", *ymag as f64));
            let (w, h) = resolution.unwrap_or((*xmag as f64, *ymag as f64));
            ps.push(p_double("AspectWidth", w));
            ps.push(p_double("AspectHeight", h));
            ps.push(p_double("NearPlane", *znear as f64));
            ps.push(p_double("FarPlane", *zfar as f64));
        }
    }

    // Verbatim path (see `build_light_attribute`).
    let raw = attribute_raw_records(node);
    let agrees = !raw.is_empty() && {
        let (c2, extras2) = crate::lights_cameras::camera_from_records(raw.clone());
        camera_eq(camera, &c2) && extras2.iter().all(|(k, v)| node.extras.get(k) == Some(v))
    };
    let ps = merge_typed_and_raw(ps, raw, agrees, CAMERA_TYPED_NAMES);
    node_attribute(id, node, "Camera", ps)
}

/// Build a `NodeAttribute` element with the given §6 subtype
/// discriminator and `Properties70` P-records, its round-tripped
/// display name (`fbx:node_attribute_name`) and scalar body leaves
/// (`fbx:node_attribute_leaves` — `TypeFlags`, `GeometryVersion`,
/// the camera `Position` / `Up` / `LookAt` triples, …) after the
/// property block, in document order.
fn node_attribute(id: i64, node: &Node, subtype: &str, ps: Vec<FbxNode>) -> FbxNode {
    let name = node
        .extras
        .get("fbx:node_attribute_name")
        .and_then(|v| v.as_str())
        .unwrap_or("");
    let mut children = Vec::new();
    if !ps.is_empty() {
        children.push(FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: ps,
        });
    }
    children.extend(
        node.extras
            .get("fbx:node_attribute_leaves")
            .and_then(|v| v.as_array())
            .into_iter()
            .flatten()
            .filter_map(crate::properties70::json_to_leaf),
    );
    FbxNode {
        name: "NodeAttribute".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(name, "NodeAttribute")),
            FbxProperty::String(subtype.as_bytes().to_vec()),
        ],
        children,
    }
}

/// Build a `Material` element record from a [`Material`].
fn build_material(mat: &Material, id: i64) -> FbxNode {
    let name = mat.name.clone().unwrap_or_default();

    // Verbatim path: the decode side stashed the element's own
    // `P` records on `fbx:material_records`. They are re-emitted
    // untouched as long as the typed PBR fields still decode to the
    // same values from them (i.e. nobody edited the typed
    // material); otherwise the typed fields win for the names this
    // crate maps and every *other* raw record still rides along.
    let raw: Vec<FbxNode> = mat
        .extras
        .get("fbx:material_records")
        .and_then(|v| v.as_array())
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_p_record)
        .collect();
    let raw_agrees = crate::material::material_from_own_records(raw.clone())
        .map(|decoded| typed_material_eq(&decoded, mat))
        .unwrap_or(false);

    let ps: Vec<FbxNode> = if raw_agrees {
        raw
    } else {
        let mut ps: Vec<FbxNode> = Vec::new();
        // DiffuseColor (the rgb of base_color; the decode path
        // multiplies DiffuseColor × DiffuseFactor, so we emit
        // DiffuseFactor 1.0).
        ps.push(p_color(
            "DiffuseColor",
            [
                mat.base_color[0] as f64,
                mat.base_color[1] as f64,
                mat.base_color[2] as f64,
            ],
        ));
        ps.push(p_number("DiffuseFactor", 1.0));
        // Opacity (base_color alpha).
        if matches!(mat.alpha_mode, AlphaMode::Blend) || mat.base_color[3] < 1.0 {
            ps.push(p_double("Opacity", mat.base_color[3] as f64));
        }
        // EmissiveColor × EmissiveFactor.
        if mat.emissive_factor != [0.0, 0.0, 0.0] {
            ps.push(p_color(
                "EmissiveColor",
                [
                    mat.emissive_factor[0] as f64,
                    mat.emissive_factor[1] as f64,
                    mat.emissive_factor[2] as f64,
                ],
            ));
            ps.push(p_number("EmissiveFactor", 1.0));
        }
        // Shininess ← roughness: the exact inverse of the decode
        // side's `roughness = sqrt(2 / (n + 2))`, always written so
        // the value is independent of whichever class template the
        // file carries (a Phong template defaults ShininessExponent
        // to 20, which would otherwise re-decode as roughness 0.30).
        ps.push(p_double(
            "Shininess",
            shininess_from_roughness(mat.roughness),
        ));
        // ReflectionFactor ← metallic (same template-independence
        // argument).
        ps.push(p_number("ReflectionFactor", mat.metallic as f64));
        // Raw records outside the typed mapping ride along verbatim.
        for r in raw {
            let keep = match r.properties.first() {
                Some(FbxProperty::String(n)) => !TYPED_MATERIAL_NAMES
                    .iter()
                    .any(|t| t.as_bytes() == n.as_slice()),
                _ => false,
            };
            if keep {
                ps.push(r);
            }
        }
        ps
    };

    // Body leaves in fixture order: `Version`, `ShadingModel`,
    // `MultiLayer`, then `Properties70`. The shading-model string is
    // re-emitted with its authored spelling (`"phong"` / `"Phong"` /
    // `"lambert"` all occur in the staged corpus); a fresh scene
    // defaults to `"Phong"`, the classic material whose template
    // carries the specular records the typed PBR fields map onto.
    let version = mat
        .extras
        .get("fbx:material_version")
        .and_then(|v| v.as_i64())
        .unwrap_or(102);
    let shading = mat
        .extras
        .get("fbx:shading_model")
        .and_then(|v| v.as_str())
        .unwrap_or("Phong");
    let multi_layer = mat
        .extras
        .get("fbx:multi_layer")
        .and_then(|v| v.as_i64())
        .unwrap_or(0);
    let children = vec![
        leaf_i32("Version", version as i32),
        leaf_string("ShadingModel", shading),
        leaf_i32("MultiLayer", multi_layer as i32),
        FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: ps,
        },
    ];

    FbxNode {
        name: "Material".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(&name, "Material")),
            FbxProperty::String(Vec::new()),
        ],
        children,
    }
}

/// The `P`-record names whose values the typed [`Material`] fields
/// own (see `crate::material::apply_properties70`); every other name
/// is passthrough-only.
const TYPED_MATERIAL_NAMES: &[&str] = &[
    "DiffuseColor",
    "Diffuse",
    "DiffuseFactor",
    "Opacity",
    "EmissiveColor",
    "EmissiveFactor",
    "Shininess",
    "ShininessExponent",
    "ReflectionFactor",
    "ShadingModel",
];

/// Inverse of the decode-side `roughness = sqrt(2 / (n + 2))`.
fn shininess_from_roughness(roughness: f32) -> f64 {
    let r = f64::from(roughness.clamp(1.0e-3, 1.0));
    (2.0 / (r * r) - 2.0).max(0.0)
}

/// Do two materials agree on every field the FBX classic-material
/// mapping produces? (Name / textures / extras are not compared —
/// only the values the `P` records decide.)
fn typed_material_eq(a: &Material, b: &Material) -> bool {
    let close = |x: f32, y: f32| (x - y).abs() <= 1.0e-5;
    a.base_color
        .iter()
        .zip(&b.base_color)
        .all(|(x, y)| close(*x, *y))
        && a.emissive_factor
            .iter()
            .zip(&b.emissive_factor)
            .all(|(x, y)| close(*x, *y))
        && close(a.roughness, b.roughness)
        && close(a.metallic, b.metallic)
        && a.alpha_mode == b.alpha_mode
}

/// A primitive's material slot table in FBX OO-connection order —
/// the extras-borne multi-material table
/// (`extras["fbx:material_slots"]`, a JSON array of `MaterialId.0`
/// indices the decode side stashed from the N `Material -> Model`
/// connections) when present, else the single bound
/// [`Primitive::material`]. Out-of-range indices are dropped.
fn material_slot_table(prim: &Primitive, n_materials: usize) -> Vec<usize> {
    if let Some(arr) = prim
        .extras
        .get("fbx:material_slots")
        .and_then(|v| v.as_array())
    {
        let slots: Vec<usize> = arr
            .iter()
            .filter_map(|v| v.as_u64())
            .map(|v| v as usize)
            .filter(|&i| i < n_materials)
            .collect();
        if !slots.is_empty() {
            return slots;
        }
    }
    prim.material
        .map(|m| m.0 as usize)
        .into_iter()
        .filter(|&i| i < n_materials)
        .collect()
}

/// Enumerate a material's bound texture slots as
/// `(TextureId, OP-prop-name)` pairs. The prop names are the canonical
/// FBX material channel names the decode path's [`crate::material`] OP walk
/// maps back into the typed PBR slots (`DiffuseColor` → base colour,
/// `NormalMap` → normal, `EmissiveColor` → emission,
/// `Maya|TEX_metallic_map` → metallic-roughness, `AmbientOcclusion` →
/// occlusion).
fn material_texture_slots(mat: &Material) -> Vec<(oxideav_mesh3d::TextureRef, &'static str)> {
    let mut slots = Vec::new();
    if let Some(t) = mat.base_color_texture {
        slots.push((t, "DiffuseColor"));
    }
    if let Some(t) = mat.normal_texture {
        slots.push((t, "NormalMap"));
    }
    if let Some(t) = mat.emissive_texture {
        slots.push((t, "EmissiveColor"));
    }
    if let Some(t) = mat.metallic_roughness_texture {
        slots.push((t, "Maya|TEX_metallic_map"));
    }
    if let Some(t) = mat.occlusion_texture {
        slots.push((t, "AmbientOcclusion"));
    }
    slots
}

/// Build a `Texture` element (and, for an embedded-blob texture, a
/// backing `Video` element with the bytes on `Video.Content`).
///
/// Returns `(texture_node, Option<video_node>)`. An
/// [`oxideav_mesh3d::ImageData::External`] texture writes its URI to
/// `RelativeFilename` + `FileName`; a `Source` blob whose bytes resolve
/// synchronously is emitted as a `Video.Content` R-blob (the
/// self-contained-FBX shape the decode path prefers). Embedded
/// already-decoded pixel buffers (no encoded bytes) fall back to an
/// empty `RelativeFilename` so the texture element still round-trips.
/// The `UVSet` KString label to emit for `texref`'s effective UV set:
/// the bound mesh's authored channel label
/// (`Primitive::extras["fbx:uv_set_names"]`) when present and
/// non-empty, else the same synthesized `map{k+1}` fallback
/// [`layer_element_uv`] names the channel with, so the decode-side
/// join resolves either way. `None` (emit no record) for the default
/// channel 0 on a mesh without authored labels — the decode side
/// defaults to channel 0.
fn uv_set_label(
    scene: &Scene3D,
    mesh_idx: Option<usize>,
    texref: &oxideav_mesh3d::TextureRef,
) -> Option<String> {
    let k = texref.effective_uv_set() as usize;
    let names = mesh_idx
        .and_then(|mi| scene.meshes.get(mi))
        .and_then(|m| m.primitives.first())
        .and_then(|p| p.extras.get("fbx:uv_set_names"))
        .and_then(|v| v.as_array());
    if let Some(names) = names {
        if let Some(n) = names.get(k).and_then(|v| v.as_str()) {
            if !n.is_empty() {
                return Some(n.to_owned());
            }
        }
        return Some(format!("map{}", k + 1));
    }
    if k != 0 {
        Some(format!("map{}", k + 1))
    } else {
        None
    }
}

/// `P: "<name>", "Vector", "", "A", x, y, z` — the animatable
/// `Vector` triple shape the staged `FbxFileTexture` template
/// (`docs/3d/fbx/fbx-property-templates.md` §3.1) uses for the
/// texture placement records (`Translation` / `Rotation` / `Scaling`).
fn p_vector(name: &str, v: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"Vector".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(v[0]),
            FbxProperty::F64(v[1]),
            FbxProperty::F64(v[2]),
        ],
        children: Vec::new(),
    }
}

/// Read a `[x, y, z]` JSON array out of one `fbx:texture_records`
/// raw-record entry.
fn raw_vec3(raw: &serde_json::Value, key: &str) -> Option<[f64; 3]> {
    let arr = raw.get(key)?.as_array()?;
    Some([
        arr.first()?.as_f64()?,
        arr.get(1)?.as_f64()?,
        arr.get(2)?.as_f64()?,
    ])
}

fn build_texture(
    tex: &oxideav_mesh3d::Texture,
    tex_id: i64,
    video_id: i64,
    texref: &oxideav_mesh3d::TextureRef,
    uv_label: Option<&str>,
    raw_records: Option<&serde_json::Value>,
) -> (FbxNode, Option<FbxNode>) {
    let name = tex.name.clone().unwrap_or_default();
    let mut tex_children: Vec<FbxNode> = vec![leaf_i32("Version", 202)];

    // Properties70 — the `FbxFileTexture` records this crate types
    // (`docs/3d/fbx/fbx-property-templates.md` §3.1): the `UVSet`
    // channel label, the typed placement transform (offset → degrees
    // → `Translation`/`Rotation`/`Scaling` `Vector` records, the
    // inverse of the decode-side literal reading), and the raw
    // untypable records round-tripped verbatim from
    // `Scene3D::extras["fbx:texture_records"]`.
    let mut p_records: Vec<FbxNode> = Vec::new();
    if let Some(label) = uv_label {
        p_records.push(p_kstring("UVSet", label));
    }
    if let Some(t) = &texref.transform {
        p_records.push(p_vector(
            "Translation",
            [f64::from(t.offset[0]), f64::from(t.offset[1]), 0.0],
        ));
        p_records.push(p_vector(
            "Rotation",
            [0.0, 0.0, f64::from(t.rotation).to_degrees()],
        ));
        p_records.push(p_vector(
            "Scaling",
            [f64::from(t.scale[0]), f64::from(t.scale[1]), 1.0],
        ));
    }
    if let Some(raw) = raw_records {
        if let Some(v) = raw.get("wrap_mode_u").and_then(|v| v.as_i64()) {
            p_records.push(p_enum("WrapModeU", v as i32));
        }
        if let Some(v) = raw.get("wrap_mode_v").and_then(|v| v.as_i64()) {
            p_records.push(p_enum("WrapModeV", v as i32));
        }
        if let Some(v) = raw.get("uv_swap").and_then(|v| v.as_bool()) {
            p_records.push(p_bool("UVSwap", v));
        }
        if let Some(v) = raw.get("use_mip_map").and_then(|v| v.as_bool()) {
            p_records.push(p_bool("UseMipMap", v));
        }
        for (key, name) in [
            ("texture_type_use", "TextureTypeUse"),
            ("current_mapping_type", "CurrentMappingType"),
            ("current_texture_blend_mode", "CurrentTextureBlendMode"),
        ] {
            if let Some(v) = raw.get(key).and_then(|v| v.as_i64()) {
                p_records.push(p_enum(name, v as i32));
            }
        }
        for (key, name) in [
            ("premultiply_alpha", "PremultiplyAlpha"),
            ("use_material", "UseMaterial"),
        ] {
            if let Some(v) = raw.get(key).and_then(|v| v.as_bool()) {
                p_records.push(p_bool(name, v));
            }
        }
        if let Some(v) = raw.get("texture_alpha").and_then(|v| v.as_f64()) {
            p_records.push(p_number("Texture alpha", v));
        }
        if let Some(v) = raw_vec3(raw, "translation") {
            p_records.push(p_vector("Translation", v));
        }
        if let Some(v) = raw_vec3(raw, "rotation") {
            p_records.push(p_vector("Rotation", v));
        }
        if let Some(v) = raw_vec3(raw, "scaling") {
            p_records.push(p_vector("Scaling", v));
        }
        if let Some(v) = raw_vec3(raw, "rotation_pivot") {
            p_records.push(p_vector3d("TextureRotationPivot", v));
        }
        if let Some(v) = raw_vec3(raw, "scaling_pivot") {
            p_records.push(p_vector3d("TextureScalingPivot", v));
        }
    }
    if !p_records.is_empty() {
        tex_children.push(FbxNode {
            name: "Properties70".to_string(),
            properties: Vec::new(),
            children: p_records,
        });
    }

    let (uri, embedded): (String, Option<Vec<u8>>) = match &tex.image {
        oxideav_mesh3d::ImageData::External { uri, .. } => (uri.clone(), None),
        oxideav_mesh3d::ImageData::Source(src) => {
            // Pull the raw encoded bytes if the source exposes them
            // synchronously (in-memory asset). Streaming-only sources
            // fall back to the URI-less embedded-empty case.
            let bytes = read_source_bytes(src.as_ref());
            (String::new(), bytes)
        }
        #[cfg(feature = "registry")]
        oxideav_mesh3d::ImageData::Embedded(_) => (String::new(), None),
    };

    // Path leaves: the round-tripped producer strings when the decode
    // side stashed them (`fbx:texture_records[i].relative_filename`
    // / `file_name` / `filename`, plus the `video_*` trio for the
    // backing Video element), else the typed URI.
    let raw_str = |key: &str| -> Option<String> {
        raw_records
            .and_then(|r| r.get(key))
            .and_then(|v| v.as_str())
            .map(str::to_owned)
    };
    tex_children.push(leaf_string(
        "RelativeFilename",
        &raw_str("relative_filename").unwrap_or_else(|| uri.clone()),
    ));
    tex_children.push(leaf_string(
        "FileName",
        &raw_str("file_name").unwrap_or_else(|| uri.clone()),
    ));
    if let Some(v) = raw_str("filename") {
        tex_children.push(leaf_string("Filename", &v));
    }

    let tex_node = FbxNode {
        name: "Texture".to_string(),
        properties: vec![
            FbxProperty::I64(tex_id),
            FbxProperty::String(name_class(&name, "Texture")),
            FbxProperty::String(Vec::new()),
        ],
        children: tex_children,
    };

    // A backing `Video` element is emitted for embedded bytes (the
    // self-contained form) and also for an external texture whose
    // source file carried one (`video_*` path leaves round-tripped
    // on the raw records) — the staged texture-video fixture backs
    // its external texture with a Content-less Video element.
    let has_video_leaves = [
        "video_filename",
        "video_file_name",
        "video_relative_filename",
    ]
    .iter()
    .any(|k| raw_str(k).is_some());
    let video_node = (embedded.is_some() || has_video_leaves).then(|| {
        let mut children = Vec::new();
        if let Some(f) = raw_str("video_filename") {
            children.push(leaf_string("Filename", &f));
        }
        if let Some(f) = raw_str("video_file_name") {
            children.push(leaf_string("FileName", &f));
        }
        children.push(leaf_string(
            "RelativeFilename",
            &raw_str("video_relative_filename").unwrap_or_else(|| uri.clone()),
        ));
        if let Some(bytes) = embedded {
            children.push(FbxNode {
                name: "Content".to_string(),
                properties: vec![FbxProperty::Raw(bytes)],
                children: Vec::new(),
            });
        }
        FbxNode {
            name: "Video".to_string(),
            properties: vec![
                FbxProperty::I64(video_id),
                FbxProperty::String(name_class(&name, "Video")),
                FbxProperty::String(b"Clip".to_vec()),
            ],
            children,
        }
    });

    (tex_node, video_node)
}

/// Best-effort synchronous read of an [`oxideav_mesh3d::AssetSource`]'s
/// bytes — used to embed a texture blob in a `Video.Content` record.
/// Returns `None` when the source can't be opened or read.
fn read_source_bytes(src: &dyn oxideav_mesh3d::AssetSource) -> Option<Vec<u8>> {
    use std::io::Read;
    // `raw_storage()` hands back the stored payload slice for sources
    // that expose a scheme-matched passthrough (ZIP / USDZ / GLB); for
    // an in-memory asset it's `None`, so fall back to the streaming
    // `open()` reader (synchronous Cursor for the InMemoryAsset case).
    if let Some(rs) = src.raw_storage() {
        return Some(rs.bytes.to_vec());
    }
    let mut reader = src.open().ok()?;
    let mut buf = Vec::new();
    reader.read_to_end(&mut buf).ok()?;
    Some(buf)
}

/// `C: "OP", child_id, parent_id, prop_name` connection record (the
/// object→property binding the decode path's texture walk reads).
fn connection_op(child_id: i64, parent_id: i64, prop_name: &str) -> FbxNode {
    FbxNode {
        name: "C".to_string(),
        properties: vec![
            FbxProperty::String(b"OP".to_vec()),
            FbxProperty::I64(child_id),
            FbxProperty::I64(parent_id),
            FbxProperty::String(prop_name.as_bytes().to_vec()),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "Color", "", "A", r, g, b` — the material colour
/// P-record shape (`as_color_rgb` accepts `"Color"` / `"ColorRGB"`).
fn p_color(name: &str, rgb: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"Color".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(rgb[0]),
            FbxProperty::F64(rgb[1]),
            FbxProperty::F64(rgb[2]),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "int", "Integer", "", v` — the `int`-typed scalar
/// shape (`UpAxis` / `FrontAxis` / `CoordAxis` GlobalSettings records).
fn p_int(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"int".to_vec()),
            FbxProperty::String(b"Integer".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::I32(v),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "Number", "", "A", v` — the `Number`-typed scalar
/// shape (`DiffuseFactor` / `EmissiveFactor` / `ReflectionFactor`).
fn p_number(name: &str, v: f64) -> FbxNode {
    p_scalar(name, "Number", v)
}

/// `P: "<name>", "double", "", "", v` — the `double`-typed scalar
/// shape (`Opacity`).
fn p_double(name: &str, v: f64) -> FbxNode {
    p_scalar(name, "double", v)
}

/// `P: "<name>", "enum", "", "", v` — the `enum`-typed scalar shape
/// (`TimeMode` / `TimeProtocol` / `SnapOnFrameMode`).
fn p_enum(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"enum".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(Vec::new()),
            FbxProperty::I32(v),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "KTime", "Time", "", v` — the `KTime`-typed int64
/// shape (`TimeSpanStart` / `TimeSpanStop`), i64-exact `L` wire.
fn p_ktime(name: &str, v: i64) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"KTime".to_vec()),
            FbxProperty::String(b"Time".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::I64(v),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "KString", "", "", "<v>"` — the `KString`-typed
/// string shape (`Original|Application*` / `DocumentUrl`).
fn p_kstring(name: &str, v: &str) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"KString".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(v.as_bytes().to_vec()),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "object", "", ""` — the empty object-reference shape
/// (§8 `"object"` typeName with no trailing value; the fixture
/// Document's `SourceObject` record). The decode side's
/// `as_object_ref` surfaces the empty-body case as `""`.
fn p_object_ref(name: &str) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"object".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(Vec::new()),
        ],
        children: Vec::new(),
    }
}

/// `P: "<name>", "bool", "", "", v` — the `bool`-typed scalar shape
/// (`CastShadows`).
fn p_bool(name: &str, v: bool) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"bool".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(Vec::new()),
            FbxProperty::Bool(v),
        ],
        children: Vec::new(),
    }
}

fn p_scalar(name: &str, type_name: &str, v: f64) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(type_name.as_bytes().to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(v),
        ],
        children: Vec::new(),
    }
}

/// `C: "OO", child_id, parent_id` connection record.
fn connection_oo(child_id: i64, parent_id: i64) -> FbxNode {
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

fn leaf_i32(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: name.to_string(),
        properties: vec![FbxProperty::I32(v)],
        children: Vec::new(),
    }
}

fn leaf_string(name: &str, v: &str) -> FbxNode {
    FbxNode {
        name: name.to_string(),
        properties: vec![FbxProperty::String(v.as_bytes().to_vec())],
        children: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_mesh3d::Topology;

    use crate::binary;
    use crate::scene::build_scene;
    use crate::writer::write_document;

    fn triangle_mesh(name: &str) -> Mesh {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
        let mut mesh = Mesh::new(Some(name.to_string()));
        mesh.primitives.push(prim);
        mesh
    }

    #[test]
    fn single_triangle_round_trips_positions() {
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let nid = scene.add_node(Node::new().with_name("TriNode").with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).expect("write");
        let reparsed = binary::parse(&bytes).expect("parse");
        let scene2 = build_scene(&reparsed).expect("build_scene");

        assert_eq!(scene2.meshes.len(), 1);
        let prim = &scene2.meshes[0].primitives[0];
        assert_eq!(prim.topology, Topology::Triangles);
        assert_eq!(prim.positions.len(), 3);
        assert_eq!(prim.positions[0], [0.0, 0.0, 0.0]);
        assert_eq!(prim.positions[1], [1.0, 0.0, 0.0]);
        assert_eq!(prim.positions[2], [1.0, 1.0, 0.0]);
        assert_eq!(scene2.meshes[0].name.as_deref(), Some("Tri"));
    }

    #[test]
    fn emits_documents_and_references_in_section_order() {
        // Round 413 — the §7 top-level section order places Documents
        // and References between GlobalSettings and Definitions.
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let names: Vec<&str> = doc.root.children.iter().map(|c| c.name.as_str()).collect();
        let pos = |n: &str| {
            names
                .iter()
                .position(|s| *s == n)
                .unwrap_or_else(|| panic!("missing section {n}"))
        };
        assert!(pos("GlobalSettings") < pos("Documents"));
        assert!(pos("Documents") < pos("References"));
        assert!(pos("References") < pos("Definitions"));
        assert!(pos("Definitions") < pos("Objects"));

        // Default document shape: Count: 1 + one "Scene" Document with
        // the SourceObject record and the RootNode 0 sentinel.
        let documents = doc.root.child("Documents").unwrap();
        assert_eq!(
            documents.child("Count").unwrap().properties[0].as_i64(),
            Some(1)
        );
        let d = documents.children_named("Document").next().unwrap();
        assert_eq!(d.properties.get(2).and_then(|p| p.as_str()), Some("Scene"));
        assert_eq!(d.child("RootNode").unwrap().properties[0].as_i64(), Some(0));
        let p70 = d.child("Properties70").unwrap();
        assert!(p70
            .children_named("P")
            .any(|p| p.properties.first().and_then(|v| v.as_str()) == Some("SourceObject")));
        // No animations, no take extras — ActiveAnimStackName is
        // still written, as the empty string (the shape every staged
        // fixture without animation carries).
        let stack = p70
            .children_named("P")
            .find(|p| p.properties.first().and_then(|v| v.as_str()) == Some("ActiveAnimStackName"))
            .expect("ActiveAnimStackName always present");
        assert_eq!(stack.properties.get(4).and_then(|v| v.as_str()), Some(""));

        // References is the observed-empty section.
        let refs = doc.root.child("References").unwrap();
        assert!(refs.children.is_empty() && refs.properties.is_empty());
    }

    #[test]
    fn default_document_stack_name_falls_back_to_first_animation() {
        use oxideav_mesh3d::{
            Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationTarget,
            AnimationValues, Interpolation,
        };
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);
        let mut anim = Animation::new(Some("Walk".to_string()));
        anim.channels.push(AnimationChannel {
            target: AnimationTarget {
                node: nid,
                property: AnimationProperty::Translation,
            },
            sampler: AnimationSampler {
                keyframes: vec![0.0, 1.0],
                values: AnimationValues::Vec3(vec![[0.0; 3], [1.0, 0.0, 0.0]]),
                interpolation: Interpolation::Linear,
            },
        });
        scene.add_animation(anim);

        let doc = encode_scene(&scene);
        let documents = doc.root.child("Documents").unwrap();
        let d = documents.children_named("Document").next().unwrap();
        let p70 = d.child("Properties70").unwrap();
        let stack = p70
            .children_named("P")
            .find(|p| p.properties.first().and_then(|v| v.as_str()) == Some("ActiveAnimStackName"))
            .expect("ActiveAnimStackName emitted for an animated scene");
        assert_eq!(
            stack.properties.get(4).and_then(|p| p.as_str()),
            Some("Walk")
        );
    }

    #[test]
    fn documents_extras_catalogue_round_trips() {
        // A decoded catalogue (fbx:documents + fbx:active_anim_stack)
        // re-renders per entry and survives a decode → encode → decode
        // cycle.
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);
        scene.extras.insert(
            "fbx:documents".to_owned(),
            serde_json::json!([
                { "name": "", "subtype": "Scene", "active_anim_stack": "Take 001" }
            ]),
        );
        scene.extras.insert(
            "fbx:active_anim_stack".to_owned(),
            serde_json::Value::String("Take 001".to_owned()),
        );

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        assert_eq!(
            crate::documents::active_anim_stack_from_extras(&scene2),
            Some("Take 001")
        );
        let docs = crate::documents::documents_from_extras(&scene2).unwrap();
        assert_eq!(docs.len(), 1);
        assert_eq!(docs[0]["subtype"].as_str(), Some("Scene"));
        assert_eq!(docs[0]["active_anim_stack"].as_str(), Some("Take 001"));
    }

    #[test]
    fn definitions_census_matches_emitted_objects() {
        // Round 413 — §7b: "Count at the top is the total object
        // count; each ObjectType block names a class [and] its
        // instance Count". The census must cover EVERY emitted object
        // class (the fixture shows GlobalSettings participating too:
        // its ObjectType block's Count: 1 is part of the total 13).
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let mat = scene.add_material(Material::new());
        scene.meshes[0].primitives[0].material = Some(mat);
        let light = scene.add_light(oxideav_mesh3d::Light::Point {
            color: [1.0, 1.0, 1.0],
            intensity: 1.0,
            range: None,
        });
        let mut node = Node::new().with_mesh(mid);
        node.light = Some(light);
        let nid = scene.add_node(node);
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let objects = doc.root.child("Objects").unwrap();
        let defs = doc.root.child("Definitions").unwrap();

        // Total census = 1 (GlobalSettings) + every Objects child.
        assert_eq!(
            defs.child("Count").unwrap().properties[0].as_i64(),
            Some(1 + objects.children.len() as i64)
        );

        // Per-class blocks: manual tally of the emitted Objects tree
        // (Geometry / Material / Model / NodeAttribute here) plus the
        // GlobalSettings block, each with the right instance count.
        let mut expected: Vec<(String, i64)> = vec![("GlobalSettings".to_string(), 1)];
        for child in &objects.children {
            match expected.iter_mut().find(|(n, _)| *n == child.name) {
                Some((_, c)) => *c += 1,
                None => expected.push((child.name.clone(), 1)),
            }
        }
        let mut emitted: Vec<(String, i64)> = defs
            .children_named("ObjectType")
            .map(|ot| {
                (
                    ot.properties[0].as_str().unwrap().to_string(),
                    ot.child("Count").unwrap().properties[0].as_i64().unwrap(),
                )
            })
            .collect();
        let mut expected_sorted = expected.clone();
        expected_sorted.sort();
        emitted.sort();
        assert_eq!(emitted, expected_sorted);

        // The light produced a NodeAttribute record — the class the
        // pre-413 scene-derived tally missed entirely.
        assert!(
            emitted.iter().any(|(n, c)| n == "NodeAttribute" && *c == 1),
            "NodeAttribute counted: {emitted:?}"
        );

        // Per-class sum equals the total census.
        let sum: i64 = emitted.iter().map(|(_, c)| c).sum();
        assert_eq!(sum, 1 + objects.children.len() as i64);
    }

    #[test]
    fn definitions_carry_fixture_staged_property_templates() {
        // Round 413 — §7b: each ObjectType block carries "a
        // PropertyTemplate holding the default Properties70 for that
        // class". The five fixture-staged template bodies are
        // re-emitted; unstaged classes stay count-only.
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let mat = scene.add_material(Material::new());
        scene.meshes[0].primitives[0].material = Some(mat);
        let light = scene.add_light(oxideav_mesh3d::Light::Point {
            color: [1.0, 1.0, 1.0],
            intensity: 1.0,
            range: None,
        });
        let mut node = Node::new().with_mesh(mid);
        node.light = Some(light);
        let nid = scene.add_node(node);
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let defs = crate::definitions::Definitions::from_document(&doc);

        // Material → FbxSurfacePhong for a fresh (typed-PBR)
        // material, with the fixture defaults incl. the specular /
        // reflection records the typed fields map onto.
        let mat_def = defs.get("Material").expect("Material ObjectType");
        assert_eq!(mat_def.template_name.as_deref(), Some("FbxSurfacePhong"));
        let tpl = defs.template_for("Material").expect("template body");
        assert_eq!(tpl.as_kstring("ShadingModel"), Some("Phong"));
        assert_eq!(tpl.as_number("DiffuseFactor"), Some(1.0));
        assert_eq!(tpl.as_color_rgb("DiffuseColor"), Some([0.8, 0.8, 0.8]));
        assert_eq!(tpl.as_number("ShininessExponent"), Some(20.0));
        assert_eq!(tpl.as_number("ReflectionFactor"), Some(1.0));

        // A scene whose every material declares a lambert shading
        // model gets the FbxSurfaceLambert body instead.
        let mut lambert = scene.clone();
        lambert.materials[0].extras.insert(
            "fbx:shading_model".into(),
            serde_json::Value::String("lambert".into()),
        );
        let ldefs = crate::definitions::Definitions::from_document(&encode_scene(&lambert));
        assert_eq!(
            ldefs.get("Material").unwrap().template_name.as_deref(),
            Some("FbxSurfaceLambert")
        );
        assert!(ldefs
            .template_for("Material")
            .unwrap()
            .as_number("ShininessExponent")
            .is_none());

        // Model → FbxNode with zero pivots / identity Lcl defaults.
        let tpl = defs.template_for("Model").expect("Model template");
        assert_eq!(tpl.as_lcl_scaling("Lcl Scaling"), Some([1.0, 1.0, 1.0]));
        assert_eq!(tpl.as_vector3d("RotationPivot"), Some([0.0, 0.0, 0.0]));
        assert_eq!(tpl.as_enum("RotationOrder"), Some(0));
        assert_eq!(tpl.as_int_typed("DefaultAttributeIndex"), Some(-1));

        // Geometry → FbxMesh.
        let tpl = defs.template_for("Geometry").expect("Geometry template");
        assert_eq!(tpl.as_bool_typed("Primary Visibility"), Some(true));

        // NodeAttribute for a Light scene — count-only per the
        // fbx-property-templates.md §2 rule 2 (the staged concrete
        // body is FbxCamera; a non-camera attribute set gets none).
        let na = defs.get("NodeAttribute").expect("NodeAttribute counted");
        assert_eq!(na.template_name, None);
        assert!(defs.template_for("NodeAttribute").is_none());
    }

    /// The `fbx-property-templates.md` §3 bodies staged in round 439:
    /// Texture → FbxFileTexture (§3.1), Video → FbxVideo (§3.2),
    /// AnimationCurveNode → FbxAnimCurveNode (§3.3), and the §2
    /// rule-2 NodeAttribute behaviour — FbxCamera (§3.5) exactly when
    /// every attribute is a Camera, none on a mixture.
    #[test]
    fn staged_template_bodies_for_texture_video_and_camera() {
        // Embedded-texture scene → Texture + Video elements.
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let tex = scene.add_texture(oxideav_mesh3d::Texture::from_encoded(
            "image/png",
            vec![0x89, 0x50, 0x4e, 0x47],
        ));
        let mut mat = Material::new();
        mat.base_color_texture = Some(oxideav_mesh3d::TextureRef::new(tex));
        let matid = scene.add_material(mat);
        scene.meshes[0].primitives[0].material = Some(matid);
        let camera = scene.add_camera(oxideav_mesh3d::Camera::Perspective {
            yfov: 1.0,
            znear: 0.1,
            zfar: Some(100.0),
            aspect_ratio: Some(1.5),
        });
        let mut node = Node::new().with_mesh(mid);
        node.camera = Some(camera);
        let nid = scene.add_node(node);
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let defs = crate::definitions::Definitions::from_document(&doc);

        let tpl = defs.template_for("Texture").expect("FbxFileTexture body");
        assert_eq!(
            defs.get("Texture").unwrap().template_name.as_deref(),
            Some("FbxFileTexture")
        );
        assert_eq!(tpl.len(), 16);
        assert_eq!(tpl.as_number("Texture alpha"), Some(1.0)); // space-bearing name
        assert_eq!(tpl.as_kstring("UVSet"), Some("default"));
        assert_eq!(tpl.as_enum("CurrentTextureBlendMode"), Some(1));

        let tpl = defs.template_for("Video").expect("FbxVideo body");
        assert_eq!(
            defs.get("Video").unwrap().template_name.as_deref(),
            Some("FbxVideo")
        );
        assert_eq!(tpl.len(), 20);
        assert_eq!(tpl.as_ktime("ClipIn"), Some(0));
        assert_eq!(tpl.as_color_rgb("Color"), Some([0.8, 0.8, 0.8]));

        // All-camera NodeAttribute → the §3.5 FbxCamera body.
        let tpl = defs
            .template_for("NodeAttribute")
            .expect("FbxCamera body for an all-camera attribute set");
        assert_eq!(
            defs.get("NodeAttribute").unwrap().template_name.as_deref(),
            Some("FbxCamera")
        );
        assert_eq!(tpl.len(), 106); // §3.5: 106 properties
        assert_eq!(tpl.as_double("FilmWidth"), Some(0.816));
        assert_eq!(tpl.as_f64("FieldOfView"), Some(25.114999771118164));
        assert_eq!(tpl.as_object_ref("Background Texture"), Some(""));
        assert_eq!(tpl.as_enum("ApertureMode"), Some(2));
        assert_eq!(tpl.as_int_typed("FrameSamplingCount"), Some(7));

        // Mixture (camera + light) → no NodeAttribute template.
        let light = scene.add_light(oxideav_mesh3d::Light::Point {
            color: [1.0, 1.0, 1.0],
            intensity: 1.0,
            range: None,
        });
        let mut lnode = Node::new();
        lnode.light = Some(light);
        let lid = scene.add_node(lnode);
        scene.roots.push(lid);
        let doc = encode_scene(&scene);
        let defs = crate::definitions::Definitions::from_document(&doc);
        assert!(defs.get("NodeAttribute").is_some());
        assert!(defs.template_for("NodeAttribute").is_none());
    }

    #[test]
    fn anim_class_templates_emitted_for_animated_scenes() {
        use oxideav_mesh3d::{
            Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationTarget,
            AnimationValues, Interpolation,
        };
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);
        let mut anim = Animation::new(Some("Walk".to_string()));
        anim.channels.push(AnimationChannel {
            target: AnimationTarget {
                node: nid,
                property: AnimationProperty::Translation,
            },
            sampler: AnimationSampler {
                keyframes: vec![0.0, 1.0],
                values: AnimationValues::Vec3(vec![[0.0; 3], [1.0, 0.0, 0.0]]),
                interpolation: Interpolation::Linear,
            },
        });
        scene.add_animation(anim);

        let doc = encode_scene(&scene);
        let defs = crate::definitions::Definitions::from_document(&doc);
        assert_eq!(
            defs.get("AnimationStack").unwrap().template_name.as_deref(),
            Some("FbxAnimStack")
        );
        let tpl = defs.template_for("AnimationStack").unwrap();
        assert_eq!(tpl.as_ktime("LocalStart"), Some(0));
        assert_eq!(
            defs.get("AnimationLayer").unwrap().template_name.as_deref(),
            Some("FbxAnimLayer")
        );
        let tpl = defs.template_for("AnimationLayer").unwrap();
        assert_eq!(tpl.as_number("Weight"), Some(100.0));
        assert_eq!(tpl.as_ulonglong("BlendModeBypass"), Some(0));
    }

    #[test]
    fn emitted_node_template_keeps_identity_transforms_reducible() {
        // The FbxNode template's pivot / offset / pre-post-rotation
        // defaults are all zero and RotationOrder is 0 (XYZ), so the
        // decode side's node_transform reduction must still resolve a
        // plain TRS — no fbx:transform_incomplete marker may appear
        // from template defaults alone.
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("Tri"));
        let node = Node::new().with_mesh(mid).with_transform(Transform::Trs {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [2.0, 2.0, 2.0],
        });
        let nid = scene.add_node(node);
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        let n = &scene2.nodes[0];
        assert!(
            !n.extras.contains_key("fbx:transform_incomplete"),
            "template defaults must not trip the reduction check: {:?}",
            n.extras
        );
        match n.transform {
            Transform::Trs {
                translation, scale, ..
            } => {
                assert_eq!(translation, [1.0, 2.0, 3.0]);
                assert_eq!(scale, [2.0, 2.0, 2.0]);
            }
            ref other => panic!("expected TRS, got {other:?}"),
        }
    }

    #[test]
    fn node_name_and_mesh_binding_round_trips() {
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("M"));
        let nid = scene.add_node(Node::new().with_name("Hello").with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        assert_eq!(scene2.nodes.len(), 1);
        assert_eq!(scene2.nodes[0].name.as_deref(), Some("Hello"));
        assert_eq!(scene2.nodes[0].mesh.map(|m| m.0), Some(0));
        assert_eq!(scene2.roots.len(), 1);
    }

    #[test]
    fn translation_scale_round_trip() {
        let mut scene = Scene3D::new();
        let mid = scene.add_mesh(triangle_mesh("M"));
        let node = Node::new().with_mesh(mid).with_transform(Transform::Trs {
            translation: [3.0, -2.0, 5.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [2.0, 2.0, 2.0],
        });
        let nid = scene.add_node(node);
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        match scene2.nodes[0].transform {
            Transform::Trs {
                translation, scale, ..
            } => {
                assert!((translation[0] - 3.0).abs() < 1e-5);
                assert!((translation[1] + 2.0).abs() < 1e-5);
                assert!((translation[2] - 5.0).abs() < 1e-5);
                assert!((scale[0] - 2.0).abs() < 1e-5);
            }
            other => panic!("expected Trs, got {other:?}"),
        }
    }

    #[test]
    fn material_binding_round_trips() {
        let mut scene = Scene3D::new();
        let matid = scene.add_material(Material::new().with_base_color([0.8, 0.2, 0.1, 1.0]));
        let mut mesh = triangle_mesh("M");
        mesh.primitives[0].material = Some(matid);
        let mid = scene.add_mesh(mesh);
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        assert_eq!(scene2.materials.len(), 1);
        let m = &scene2.materials[0];
        assert!((m.base_color[0] - 0.8).abs() < 1e-3);
        assert!((m.base_color[1] - 0.2).abs() < 1e-3);
        // The mesh's primitive should bind the material.
        let prim = &scene2.meshes[0].primitives[0];
        assert_eq!(prim.material.map(|x| x.0), Some(0));
    }

    #[test]
    fn external_texture_uri_round_trips() {
        use oxideav_mesh3d::{Texture, TextureRef};
        let mut scene = Scene3D::new();
        let tid = scene.add_texture(Texture::from_uri("textures/diffuse.png"));
        let mut mat = Material::new();
        mat.base_color_texture = Some(TextureRef::new(tid));
        let matid = scene.add_material(mat);
        let mut mesh = triangle_mesh("M");
        mesh.primitives[0].material = Some(matid);
        let mid = scene.add_mesh(mesh);
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();

        assert_eq!(scene2.textures.len(), 1, "one texture round-tripped");
        // The material's base-colour slot binds the texture.
        let m = &scene2.materials[0];
        let bind = m
            .base_color_texture
            .as_ref()
            .expect("base_color_texture wired via OP");
        // The bound texture's URI survived.
        match &scene2.textures[bind.texture.0 as usize].image {
            oxideav_mesh3d::ImageData::External { uri, .. } => {
                assert_eq!(uri, "textures/diffuse.png");
            }
            other => panic!("expected External uri, got {other:?}"),
        }
    }

    #[test]
    fn embedded_texture_blob_round_trips() {
        use oxideav_mesh3d::{Texture, TextureRef};
        let mut scene = Scene3D::new();
        // A tiny PNG-ish blob (content is opaque to the encoder).
        let blob = vec![0x89, b'P', b'N', b'G', 1, 2, 3, 4, 5, 6];
        let tex = Texture::from_encoded("image/png", blob.clone());
        let tid = scene.add_texture(tex);
        let mut mat = Material::new();
        mat.normal_texture = Some(TextureRef::new(tid));
        let matid = scene.add_material(mat);
        let mut mesh = triangle_mesh("M");
        mesh.primitives[0].material = Some(matid);
        let mid = scene.add_mesh(mesh);
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        // The Objects block should carry both a Texture and a Video
        // element with the embedded Content blob.
        let objects = doc.root.child("Objects").unwrap();
        let video = objects
            .children
            .iter()
            .find(|c| c.name == "Video")
            .expect("Video element emitted for embedded blob");
        let content = video.child("Content").expect("Content R-blob");
        match &content.properties[0] {
            FbxProperty::Raw(b) => assert_eq!(b, &blob, "embedded bytes preserved"),
            other => panic!("expected Raw blob, got {other:?}"),
        }

        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();
        assert_eq!(scene2.textures.len(), 1);
        // Normal slot bound.
        assert!(scene2.materials[0].normal_texture.is_some());
    }

    #[test]
    fn unit_centimetres_round_trips() {
        let mut scene = Scene3D::new();
        scene.unit = oxideav_mesh3d::Unit::Centimetres;
        let mid = scene.add_mesh(triangle_mesh("M"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();
        assert_eq!(scene2.unit, oxideav_mesh3d::Unit::Centimetres);
    }

    #[test]
    fn unit_metres_round_trips() {
        let mut scene = Scene3D::new();
        scene.unit = oxideav_mesh3d::Unit::Metres;
        let mid = scene.add_mesh(triangle_mesh("M"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();
        assert_eq!(scene2.unit, oxideav_mesh3d::Unit::Metres);
    }

    #[test]
    fn axis_extras_round_trip() {
        let mut scene = Scene3D::new();
        scene
            .extras
            .insert("fbx:up_axis".to_string(), serde_json::json!(1));
        scene
            .extras
            .insert("fbx:front_axis".to_string(), serde_json::json!(2));
        let mid = scene.add_mesh(triangle_mesh("M"));
        let nid = scene.add_node(Node::new().with_mesh(mid));
        scene.roots.push(nid);

        let doc = encode_scene(&scene);
        let bytes = write_document(&doc).unwrap();
        let scene2 = build_scene(&binary::parse(&bytes).unwrap()).unwrap();
        assert_eq!(
            scene2.extras.get("fbx:up_axis").and_then(|v| v.as_i64()),
            Some(1)
        );
        assert_eq!(
            scene2.extras.get("fbx:front_axis").and_then(|v| v.as_i64()),
            Some(2)
        );
    }

    #[test]
    fn quat_euler_round_trip_identity() {
        let e = quat_to_euler_xyz_deg([0.0, 0.0, 0.0, 1.0]);
        assert_eq!(e, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn quat_euler_round_trip_axis_rotations() {
        use crate::animation::euler_xyz_to_quat;
        for deg in [
            [90.0_f32, 0.0, 0.0],
            [0.0, 45.0, 0.0],
            [0.0, 0.0, 30.0],
            [10.0, 20.0, 30.0],
        ] {
            let q = euler_xyz_to_quat(deg);
            let back = quat_to_euler_xyz_deg(q);
            let q2 = euler_xyz_to_quat(back);
            // Quaternions should match (up to sign) — compare the
            // rotation, not the raw Euler angles.
            let dot = q[0] * q2[0] + q[1] * q2[1] + q[2] * q2[2] + q[3] * q2[3];
            assert!(dot.abs() > 0.9999, "deg {deg:?}: dot {dot}");
        }
    }
}
