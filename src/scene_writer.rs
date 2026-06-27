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
    /// Emit a `LayerElementUV` record for primitives that carry at
    /// least one UV set (set 0 only — additional sets are a follow-up).
    /// Default `true`.
    pub emit_uvs: bool,
}

impl Default for SceneEncodeOptions {
    fn default() -> Self {
        Self {
            version: DEFAULT_ENCODE_VERSION,
            emit_normals: true,
            emit_uvs: true,
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
    let emit_texture =
        |tex_idx: usize, objs: &mut FbxNode, conns: &mut FbxNode, emitted: &mut [bool]| {
            if emitted[tex_idx] {
                return;
            }
            emitted[tex_idx] = true;
            let tex = &scene.textures[tex_idx];
            let (tex_node, video_node) =
                build_texture(tex, texture_ids[tex_idx], video_ids[tex_idx]);
            objs.children.push(tex_node);
            if let Some(vnode) = video_node {
                objs.children.push(vnode);
                // Video -> Texture OO (backing media).
                conns
                    .children
                    .push(connection_oo(video_ids[tex_idx], texture_ids[tex_idx]));
            }
        };
    let mut texture_emitted = vec![false; scene.textures.len()];
    for (xi, mat) in scene.materials.iter().enumerate() {
        for (slot, prop_name) in material_texture_slots(mat) {
            let tex_idx = slot.0 as usize;
            if tex_idx >= scene.textures.len() {
                continue;
            }
            emit_texture(
                tex_idx,
                &mut objects,
                &mut connections,
                &mut texture_emitted,
            );
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
        // Geometry → Model attribute attachment.
        if let Some(mid) = node.mesh {
            let gid = mesh_ids[mid.0 as usize];
            connections.children.push(connection_oo(gid, node_ids[ni]));
        }
        // Material → Model surface assignment. The first material on
        // the mesh's first primitive binds to the model.
        if let Some(mid) = node.mesh {
            if let Some(prim) = scene
                .meshes
                .get(mid.0 as usize)
                .and_then(|m| m.primitives.first())
            {
                if let Some(matid) = prim.material {
                    let xid = material_ids[matid.0 as usize];
                    connections.children.push(connection_oo(xid, node_ids[ni]));
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

    // -- Animation graph (round 377) ------------------------------------
    // One AnimationStack / AnimationLayer per Scene3D::Animation, plus
    // the AnimationCurveNode / AnimationCurve records + OO/OP chain the
    // decode path's extract_animations walks. Channels target the Model
    // record for the scene NodeId via the node-id → fbx-id map below.
    if !scene.animations.is_empty() {
        let node_to_fbx =
            |nid: oxideav_mesh3d::NodeId| -> Option<i64> { node_ids.get(nid.0 as usize).copied() };
        let anim_emit =
            crate::anim_writer::build_animation_objects(&scene.animations, node_to_fbx, || {
                alloc.next()
            });
        objects.children.extend(anim_emit.objects);
        connections.children.extend(anim_emit.connections);
    }

    // -- Top-level sections ---------------------------------------------
    let mut root = FbxNode {
        name: String::new(),
        properties: Vec::new(),
        children: Vec::new(),
    };
    root.children.push(build_header_extension(opts.version));
    root.children.push(build_global_settings(scene));
    root.children.push(build_definitions(scene));
    root.children.push(objects);
    root.children.push(connections);

    FbxDocument {
        version: opts.version,
        root,
    }
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

/// `FBXHeaderExtension { FBXVersion: <version> }` — the minimal §7a
/// section the ASCII reader's version sniff + the decode path tolerate.
fn build_header_extension(version: u32) -> FbxNode {
    FbxNode {
        name: "FBXHeaderExtension".to_string(),
        properties: Vec::new(),
        children: vec![FbxNode {
            name: "FBXVersion".to_string(),
            properties: vec![FbxProperty::I32(version as i32)],
            children: Vec::new(),
        }],
    }
}

/// `GlobalSettings { Version; Properties70 { UpAxis...; UnitScaleFactor } }`
/// per `docs/3d/fbx/fbx-binary-properties70.md` §4 + the
/// cubes-ascii-v7500.fbx fixture.
///
/// Emits the `UnitScaleFactor` `double` P-record derived from
/// [`oxideav_mesh3d::Scene3D::unit`] (the decode path's
/// `unit_from_scale_factor` maps `100.0 → Centimetres` / `1.0 → Metres`;
/// other units write the factor as `centimetres-per-unit` so the raw
/// value survives on `extras["fbx:unit_scale_factor"]`). When the scene
/// carries axis `extras["fbx:up_axis"]` / `["fbx:front_axis"]` /
/// `["fbx:coord_axis"]` ints (round-tripped from a decoded FBX), they
/// are re-emitted as `int` P-records so the axis convention survives a
/// decode→encode→decode cycle.
fn build_global_settings(scene: &Scene3D) -> FbxNode {
    let mut ps: Vec<FbxNode> = Vec::new();

    // Axis ints — re-emit only when the scene actually carries them
    // (round-tripped from a decoded file). The FBX-int → Axis variant
    // table is a docs gap, so we don't synthesise them from
    // `Scene3D::up_axis` / `front_axis` (which would require the table).
    for (key, name) in [
        ("fbx:up_axis", "UpAxis"),
        ("fbx:up_axis_sign", "UpAxisSign"),
        ("fbx:front_axis", "FrontAxis"),
        ("fbx:front_axis_sign", "FrontAxisSign"),
        ("fbx:coord_axis", "CoordAxis"),
        ("fbx:coord_axis_sign", "CoordAxisSign"),
    ] {
        if let Some(i) = scene.extras.get(key).and_then(|v| v.as_i64()) {
            ps.push(p_int(name, i as i32));
        }
    }

    // UnitScaleFactor — centimetres-per-unit. The decode side's
    // `unit_from_scale_factor` recovers Centimetres (100) / Metres (1);
    // for the other units we write the literal `cm per unit` so the raw
    // factor survives on extras even though no typed `Unit` is recovered.
    let scale_factor = match scene.unit {
        oxideav_mesh3d::Unit::Centimetres => 100.0,
        oxideav_mesh3d::Unit::Metres => 1.0,
        // metres-per-unit → centimetres-per-unit.
        other => other.to_metres() as f64 * 100.0,
    };
    ps.push(p_double("UnitScaleFactor", scale_factor));

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

/// `Definitions { Version; Count; ObjectType: "<class>" { Count } }`
/// per `docs/3d/fbx/fbx-ascii-grammar.md` §7b. We emit a count-only
/// block per populated class (no `PropertyTemplate`, which is optional
/// — the decode path resolves against an empty template just fine).
fn build_definitions(scene: &Scene3D) -> FbxNode {
    let mut children = vec![FbxNode {
        name: "Version".to_string(),
        properties: vec![FbxProperty::I32(100)],
        children: Vec::new(),
    }];
    let total = scene.meshes.len() + scene.nodes.len() + scene.materials.len();
    children.push(FbxNode {
        name: "Count".to_string(),
        properties: vec![FbxProperty::I32(total as i32)],
        children: Vec::new(),
    });
    let mut push_class = |class: &str, count: usize| {
        if count == 0 {
            return;
        }
        children.push(FbxNode {
            name: "ObjectType".to_string(),
            properties: vec![FbxProperty::String(class.as_bytes().to_vec())],
            children: vec![FbxNode {
                name: "Count".to_string(),
                properties: vec![FbxProperty::I32(count as i32)],
                children: Vec::new(),
            }],
        });
    };
    push_class("Geometry", scene.meshes.len());
    push_class("Model", scene.nodes.len());
    push_class("Material", scene.materials.len());
    FbxNode {
        name: "Definitions".to_string(),
        properties: Vec::new(),
        children,
    }
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
    let mut uvs: Vec<f64> = Vec::new();
    let mut have_uvs = true;

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
        // UVs — set 0.
        match prim.uvs.first() {
            Some(set) if set.len() == prim.positions.len() => {
                let buf = expand_uv(prim, set);
                if buf.len() == n_corners {
                    for [u, v] in &buf {
                        uvs.push(*u as f64);
                        uvs.push(*v as f64);
                    }
                } else {
                    have_uvs = false;
                }
            }
            _ => have_uvs = false,
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
    if opts.emit_uvs && have_uvs && !uvs.is_empty() {
        children.push(layer_element_uv(uvs));
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
/// data record is named `UV`.
fn layer_element_uv(data: Vec<f64>) -> FbxNode {
    FbxNode {
        name: "LayerElementUV".to_string(),
        properties: vec![FbxProperty::I32(0)],
        children: vec![
            leaf_i32("Version", 101),
            leaf_string("Name", "map1"),
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

/// Build a `Model` element record from a scene-graph [`Node`].
fn build_model(node: &Node, id: i64) -> FbxNode {
    let name = node.name.clone().unwrap_or_default();
    let mut children = Vec::new();
    let props70 = build_node_transform_props(node);
    if !props70.children.is_empty() {
        children.push(props70);
    }
    FbxNode {
        name: "Model".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(&name, "Model")),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children,
    }
}

/// Build the `Properties70` block carrying `Lcl Translation` /
/// `Lcl Rotation` / `Lcl Scaling`. Only non-default components are
/// emitted (the decode path resolves omissions against the template /
/// identity default), so an identity transform produces no records.
fn build_node_transform_props(node: &Node) -> FbxNode {
    let (translation, rotation_deg, scale) = decompose_trs(node.transform);
    let mut ps: Vec<FbxNode> = Vec::new();
    if translation != [0.0, 0.0, 0.0] {
        ps.push(p_lcl("Lcl Translation", translation));
    }
    if rotation_deg != [0.0, 0.0, 0.0] {
        ps.push(p_lcl("Lcl Rotation", rotation_deg));
    }
    if scale != [1.0, 1.0, 1.0] {
        ps.push(p_lcl("Lcl Scaling", scale));
    }
    FbxNode {
        name: "Properties70".to_string(),
        properties: Vec::new(),
        children: ps,
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

/// Build a `Material` element record from a [`Material`].
fn build_material(mat: &Material, id: i64) -> FbxNode {
    let name = mat.name.clone().unwrap_or_default();
    let mut ps: Vec<FbxNode> = Vec::new();
    // DiffuseColor (the rgb of base_color; the decode path multiplies
    // DiffuseColor × DiffuseFactor, so we emit DiffuseFactor 1.0).
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
    // ReflectionFactor ← metallic.
    if mat.metallic != 1.0 {
        ps.push(p_number("ReflectionFactor", mat.metallic as f64));
    }

    let children = vec![FbxNode {
        name: "Properties70".to_string(),
        properties: Vec::new(),
        children: ps,
    }];

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

/// Enumerate a material's bound texture slots as
/// `(TextureId, OP-prop-name)` pairs. The prop names are the canonical
/// FBX material channel names the decode path's [`crate::material`] OP walk
/// maps back into the typed PBR slots (`DiffuseColor` → base colour,
/// `NormalMap` → normal, `EmissiveColor` → emission,
/// `Maya|TEX_metallic_map` → metallic-roughness, `AmbientOcclusion` →
/// occlusion).
fn material_texture_slots(mat: &Material) -> Vec<(oxideav_mesh3d::TextureId, &'static str)> {
    let mut slots = Vec::new();
    if let Some(t) = &mat.base_color_texture {
        slots.push((t.texture, "DiffuseColor"));
    }
    if let Some(t) = &mat.normal_texture {
        slots.push((t.texture, "NormalMap"));
    }
    if let Some(t) = &mat.emissive_texture {
        slots.push((t.texture, "EmissiveColor"));
    }
    if let Some(t) = &mat.metallic_roughness_texture {
        slots.push((t.texture, "Maya|TEX_metallic_map"));
    }
    if let Some(t) = &mat.occlusion_texture {
        slots.push((t.texture, "AmbientOcclusion"));
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
fn build_texture(
    tex: &oxideav_mesh3d::Texture,
    tex_id: i64,
    video_id: i64,
) -> (FbxNode, Option<FbxNode>) {
    let name = tex.name.clone().unwrap_or_default();
    let mut tex_children: Vec<FbxNode> = vec![leaf_i32("Version", 202)];

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

    tex_children.push(leaf_string("RelativeFilename", &uri));
    tex_children.push(leaf_string("FileName", &uri));

    let tex_node = FbxNode {
        name: "Texture".to_string(),
        properties: vec![
            FbxProperty::I64(tex_id),
            FbxProperty::String(name_class(&name, "Texture")),
            FbxProperty::String(Vec::new()),
        ],
        children: tex_children,
    };

    let video_node = embedded.map(|bytes| FbxNode {
        name: "Video".to_string(),
        properties: vec![
            FbxProperty::I64(video_id),
            FbxProperty::String(name_class(&name, "Video")),
            FbxProperty::String(b"Clip".to_vec()),
        ],
        children: vec![
            leaf_string("RelativeFilename", &uri),
            FbxNode {
                name: "Content".to_string(),
                properties: vec![FbxProperty::Raw(bytes)],
                children: Vec::new(),
            },
        ],
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
