//! Object-graph walker — turn an [`FbxDocument`] into a [`Scene3D`].
//!
//! Reads the top-level `Objects` and `Connections` records (per
//! `docs/3d/fbx/ufbx/elements-overview.md` + `elements-meshes.md` +
//! `elements-nodes.md`):
//!
//! - **Objects** is a flat container whose direct children are the
//!   element records keyed by element-type tag (`Geometry`, `Model`,
//!   `Material`, `Texture`, `Video`, `AnimationStack`, ...). Every
//!   record has the property tuple `[id: i64, name_subtype: String,
//!   subtype: String]` (the FBX 7.x convention).
//! - **Connections** is a flat list of `C` records, each with the
//!   property tuple `[kind: String, child_id: i64, parent_id: i64
//!   (, prop_name: String)]`. `kind` is `OO` (object-object), `OP`
//!   (object-property), `PP` (property-property), or `PO`
//!   (property-object); the OP variant carries the additional
//!   `prop_name` string.
//!
//! For round 1 we wire the OO connections from each `Geometry` element
//! to its parent `Model` (typed `Mesh`); other connection kinds and
//! every other element type are deferred.

use std::collections::HashMap;

use oxideav_mesh3d::{Error, Node, Result, Scene3D};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::geometry::extract_geometry_mesh;

/// Decode the top-level `Objects` / `Connections` records into a
/// [`Scene3D`].
pub fn build_scene(doc: &FbxDocument) -> Result<Scene3D> {
    let mut scene = Scene3D::new();

    // Index every Geometry element by its FBX id. Materials,
    // animations, etc. are deferred — round 1 surfaces just enough
    // for downstream renderers to draw the mesh.
    let mut geometry_meshes: HashMap<i64, oxideav_mesh3d::MeshId> = HashMap::new();
    // Per-Model FBX id → the Node we created for it.
    let mut model_nodes: HashMap<i64, oxideav_mesh3d::NodeId> = HashMap::new();
    // Map the Model FBX id → its FBX subtype string (`"Mesh"`, etc.).
    // Only `Mesh`-subtype models receive Geometry attachments; other
    // subtypes (`LimbNode`, `Camera`, `Light`, `Null`, `Root`) are
    // surfaced as bare named nodes so the scene-graph hierarchy
    // round-trips even when their attribute payloads aren't decoded.
    let mut model_subtypes: HashMap<i64, String> = HashMap::new();

    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            match child.name.as_str() {
                "Geometry" => {
                    let id = read_element_id(child).ok_or_else(|| {
                        Error::invalid("FBX Geometry element missing id property")
                    })?;
                    if subtype(child).as_deref() == Some("Mesh") || subtype(child).is_none()
                    // ufbx elements-meshes.md: every binary FBX
                    // Geometry node we care about for round 1
                    // is the polygonal `Mesh` subtype. Nurbs /
                    // Patch / Boundary subtypes are not yet
                    // supported.
                    {
                        let mesh = extract_geometry_mesh(child, element_name(child))?;
                        let mid = scene.add_mesh(mesh);
                        geometry_meshes.insert(id, mid);
                    }
                }
                "Model" => {
                    let id = read_element_id(child)
                        .ok_or_else(|| Error::invalid("FBX Model element missing id property"))?;
                    let st = subtype(child).unwrap_or_default();
                    let name = element_name(child).unwrap_or_default();
                    let mut node = Node::new();
                    if !name.is_empty() {
                        node = node.with_name(name);
                    }
                    let nid = scene.add_node(node);
                    model_nodes.insert(id, nid);
                    model_subtypes.insert(id, st);
                }
                _ => {
                    // Other element types — Material, Texture, Video,
                    // AnimationStack, AnimationLayer, Pose, Skin,
                    // Cluster, Deformer ... — are not surfaced in
                    // round 1. They round-trip through the parsed
                    // FbxDocument; downstream callers can reach them
                    // via [`crate::FbxDecoder::last_document`].
                }
            }
        }
    }

    // Walk Connections to wire Geometry → Model and Model → root /
    // parent Model. Connection records use property tuple
    // (kind, child_id, parent_id [, prop_name]) per
    // ufbx/elements-overview.md §"Connections".
    let mut child_of_model: HashMap<i64, Vec<i64>> = HashMap::new();
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
            // Geometry → Model (attribute attachment).
            if let (Some(&mid), Some(&nid)) =
                (geometry_meshes.get(&child_id), model_nodes.get(&parent_id))
            {
                let node = &mut scene.nodes[nid.0 as usize];
                node.mesh = Some(mid);
                continue;
            }
            // Model → Model (scene-graph parent/child).
            if model_nodes.contains_key(&child_id) && model_nodes.contains_key(&parent_id) {
                child_of_model.entry(parent_id).or_default().push(child_id);
                continue;
            }
            // Model → 0 (root attachment) — FBX uses parent_id == 0
            // to denote the implicit document root.
            if parent_id == 0 && model_nodes.contains_key(&child_id) {
                child_of_model.entry(0).or_default().push(child_id);
                continue;
            }
        }
    }

    // Materialise child / root edges.
    for (parent_id, children_ids) in &child_of_model {
        let child_node_ids: Vec<oxideav_mesh3d::NodeId> = children_ids
            .iter()
            .filter_map(|cid| model_nodes.get(cid).copied())
            .collect();
        if *parent_id == 0 {
            scene.roots.extend(child_node_ids);
        } else if let Some(&pid) = model_nodes.get(parent_id) {
            scene.nodes[pid.0 as usize].children.extend(child_node_ids);
        }
    }
    // Models that received no parent edge become implicit roots —
    // exporters that omit the Connection to id 0 (rare, but observed)
    // would otherwise be lost.
    let parented: std::collections::HashSet<oxideav_mesh3d::NodeId> = scene
        .nodes
        .iter()
        .flat_map(|n| n.children.iter().copied())
        .chain(scene.roots.iter().copied())
        .collect();
    for &nid in model_nodes.values() {
        if !parented.contains(&nid) {
            scene.roots.push(nid);
        }
    }

    // If we surfaced Meshes but no Models referenced them, fabricate
    // one root Node per orphan Mesh so a downstream renderer can
    // still draw the geometry. This matches the "Geometry without a
    // Model" tolerance documented in ufbx/elements-meshes.md.
    let referenced_meshes: std::collections::HashSet<oxideav_mesh3d::MeshId> =
        scene.nodes.iter().filter_map(|n| n.mesh).collect();
    let orphan_mesh_ids: Vec<oxideav_mesh3d::MeshId> = geometry_meshes
        .values()
        .copied()
        .filter(|mid| !referenced_meshes.contains(mid))
        .collect();
    for mid in orphan_mesh_ids {
        let name = scene
            .meshes
            .get(mid.0 as usize)
            .and_then(|m| m.name.clone());
        let mut node = Node::new().with_mesh(mid);
        if let Some(n) = name {
            node = node.with_name(n);
        }
        let nid = scene.add_node(node);
        scene.roots.push(nid);
    }

    // Drop the unused-warning silencer once `model_subtypes` actually
    // gets read (e.g. when LimbNode → Skeleton wiring lands).
    let _ = model_subtypes;

    // If somehow no roots and no meshes ended up populated, surface
    // an empty scene rather than failing — this matches the
    // "FBX-with-no-Objects" tolerance other loaders apply.
    if scene.nodes.is_empty() && scene.meshes.is_empty() {
        return Ok(Mesh3DEmpty::scene());
    }
    Ok(scene)
}

/// Read the `id` property of an FBX element record. The convention
/// per Gessler's worked-example output is property[0] = id (i64),
/// property[1] = name+subtype string, property[2] = subtype string.
fn read_element_id(node: &FbxNode) -> Option<i64> {
    node.properties.first().and_then(FbxProperty::as_i64)
}

/// Read the user-facing element name from property[1]. The full
/// string is the FBX `Name::SubType` Pascal-case-and-double-colon
/// joined identifier; we strip the `\x00\x01` separator the binary
/// writer uses and return only the leading name.
fn element_name(node: &FbxNode) -> Option<String> {
    let raw = match node.properties.get(1)? {
        FbxProperty::String(b) => b,
        _ => return None,
    };
    // FBX joins Name + SubType with `\x00\x01` in the binary
    // encoding (vs `::` in the ASCII encoding). Both halves are
    // valid UTF-8 individually.
    if let Some(sep) = raw.iter().position(|&b| b == 0x00) {
        std::str::from_utf8(&raw[..sep]).ok().map(str::to_owned)
    } else {
        std::str::from_utf8(raw).ok().map(str::to_owned)
    }
}

/// Read the FBX subtype string from property[2].
fn subtype(node: &FbxNode) -> Option<String> {
    node.properties.get(2)?.as_str().map(str::to_owned)
}

/// Helper to pre-allocate an empty scene with the FBX coordinate
/// defaults (Y-up, -Z forward, centimetres — Maya-default; ufbx
/// elements/index.md §"Coordinate spaces" notes that 1 FBX unit ≈
/// 1 cm by default for files exported from Maya).
struct Mesh3DEmpty;
impl Mesh3DEmpty {
    fn scene() -> Scene3D {
        let mut s = Scene3D::new();
        s.unit = oxideav_mesh3d::Unit::Centimetres;
        s
    }
}
