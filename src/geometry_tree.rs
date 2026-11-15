//! Non-polygon `Geometry` objects — `NurbsSurface`, `Line`,
//! `TrimNurbsSurface` with its `Boundary` loops and edge curves, and
//! the older `Nurb` / `Patch` families — in the scene model.
//!
//! Grammar: `docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §3
//! (fixtures `fixtures/fbxsdk-nurbs/`). Each of these geometries
//! connects `OO` to a `Model` of the same sub-type; a trimmed surface
//! additionally owns other geometries through `Geometry → Geometry`
//! `OO` connections (§3.5):
//!
//! ```text
//! NurbsSurface (untrimmed)  →  TrimNurbsSurface
//! Boundary                  →  TrimNurbsSurface     (one per boundary)
//! NurbsCurve (edge)         →  Boundary             (one per edge, loop order)
//! ```
//!
//! `oxideav-mesh3d` has no curve or surface type, so such a geometry —
//! with everything connected under it — becomes one
//! [`oxideav_mesh3d::Mesh`] whose first primitive carries the whole
//! record tree on `extras[`[`GEOMETRY_TREE_KEY`]`]`:
//!
//! ```json
//! { "name": "trim01", "subtype": "TrimNurbsSurface",
//!   "properties70": 0,            // 0 absent, 1 present, 2 written twice
//!   "records": [P…], "body": [leaf…],
//!   "children": [ { …same shape… } ] }   // connection order
//! ```
//!
//! and whose primitives are the sampled geometry:
//!
//! | Sub-type | Primitives |
//! |---|---|
//! | `NurbsSurface` | one tessellated `Triangles` primitive |
//! | `TrimNurbsSurface` | the untrimmed surface's tessellation restricted to the trim regions |
//! | `Line` | one `LineStrip` per polyline |
//! | `Nurb`, `Patch`, anything else | one empty `Points` primitive (records only) |
//!
//! The encoder writes the tree back object for object, with the
//! `Geometry → Geometry` connections in the same order (the order of
//! the boundary connections is what defines the trim regions).
//! `"properties70": 2` reproduces the vendor exporter's habit of
//! writing an outer boundary's `Properties70` node twice (§3.5).

use std::collections::{HashMap, HashSet};

use oxideav_mesh3d::{Mesh, Primitive, Topology};
use serde_json::{Value, json};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::nurbs_geometry::{
    FbxBoundary, FbxLine, FbxNurbsCurve, FbxNurbsSurface, FbxTrimSurface, NurbsTessellation,
};

/// `Primitive::extras` key carrying a non-polygon geometry's record
/// tree (module docs).
pub const GEOMETRY_TREE_KEY: &str = "fbx:geometry_tree";

/// The `Geometry` objects of a document and the `Geometry → Geometry`
/// connections among them.
pub(crate) struct GeometryIndex<'a> {
    elements: HashMap<i64, &'a FbxNode>,
    children: HashMap<i64, Vec<i64>>,
    owned: HashSet<i64>,
}

impl<'a> GeometryIndex<'a> {
    pub(crate) fn new(doc: &'a FbxDocument) -> Self {
        let mut elements = HashMap::new();
        if let Some(objects) = doc.root.child("Objects") {
            for g in objects.children_named("Geometry") {
                if let Some(id) = g.properties.first().and_then(FbxProperty::as_i64) {
                    elements.insert(id, g);
                }
            }
        }
        let mut children: HashMap<i64, Vec<i64>> = HashMap::new();
        let mut owned = HashSet::new();
        if let Some(conns) = doc.root.child("Connections") {
            for c in conns.children_named("C") {
                if c.properties.first().and_then(FbxProperty::as_str) != Some("OO") {
                    continue;
                }
                let (Some(child), Some(parent)) = (
                    c.properties.get(1).and_then(FbxProperty::as_i64),
                    c.properties.get(2).and_then(FbxProperty::as_i64),
                ) else {
                    continue;
                };
                // A `Shape` hangs off a deformer, never off a geometry;
                // a self-edge or a repeated owner would loop.
                if child != parent
                    && elements.contains_key(&child)
                    && elements.contains_key(&parent)
                    && owned.insert(child)
                {
                    children.entry(parent).or_default().push(child);
                }
            }
        }
        Self {
            elements,
            children,
            owned,
        }
    }

    /// `true` when geometry `id` is connected under another geometry
    /// (an untrimmed surface, a boundary, an edge curve).
    pub(crate) fn is_owned(&self, id: i64) -> bool {
        self.owned.contains(&id)
    }

    /// The record tree rooted at geometry `id`.
    pub(crate) fn tree(&self, id: i64) -> Option<Value> {
        self.tree_at(id, 0)
    }

    fn tree_at(&self, id: i64, depth: usize) -> Option<Value> {
        // Trim → boundary → edge is three levels; a longer chain is a
        // malformed (possibly cyclic) graph.
        if depth > 8 {
            return None;
        }
        let element = self.elements.get(&id)?;
        let blocks: Vec<&FbxNode> = element.children_named("Properties70").collect();
        let children: Vec<Value> = self
            .children
            .get(&id)
            .into_iter()
            .flatten()
            .filter_map(|c| self.tree_at(*c, depth + 1))
            .collect();
        Some(json!({
            "name": display_name(element),
            "subtype": subtype(element),
            "properties70": blocks.len().min(2),
            "records": crate::properties70::own_records_json(element),
            "body": crate::properties70::body_json(element),
            "children": children,
        }))
    }
}

fn display_name(element: &FbxNode) -> String {
    element
        .properties
        .get(1)
        .and_then(FbxProperty::as_str)
        .and_then(|s| s.split('\u{0}').next())
        .unwrap_or("")
        .to_owned()
}

fn subtype(element: &FbxNode) -> String {
    element
        .properties
        .get(2)
        .and_then(FbxProperty::as_str)
        .unwrap_or("")
        .to_owned()
}

/// One object of a record tree rebuilt as an element (id 0), with its
/// children.
struct TreeNode {
    element: FbxNode,
    children: Vec<TreeNode>,
}

fn tree_node(v: &Value, depth: usize) -> Option<TreeNode> {
    if depth > 8 {
        return None;
    }
    let name = v.get("name").and_then(Value::as_str).unwrap_or("");
    let subtype = v.get("subtype")?.as_str()?;
    let block = || FbxNode {
        name: "Properties70".to_string(),
        properties: Vec::new(),
        children: v
            .get("records")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(crate::properties70::json_to_p_record)
            .collect(),
    };
    let blocks = v.get("properties70").and_then(Value::as_u64).unwrap_or(0);
    let mut children = Vec::new();
    if blocks >= 1 {
        children.push(block());
    }
    children.extend(
        v.get("body")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(crate::properties70::json_to_body_leaf),
    );
    if blocks >= 2 {
        children.push(block());
    }
    let mut full = name.as_bytes().to_vec();
    full.extend_from_slice(b"\x00\x01Geometry");
    Some(TreeNode {
        element: FbxNode {
            name: "Geometry".to_string(),
            properties: vec![
                FbxProperty::I64(0),
                FbxProperty::String(full),
                FbxProperty::String(subtype.as_bytes().to_vec()),
            ],
            children,
        },
        children: v
            .get("children")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(|c| tree_node(c, depth + 1))
            .collect(),
    })
}

/// Objects and connections a record tree encodes to.
pub(crate) struct TreeEmit {
    /// The root `Geometry` element (carrying the id it was given).
    pub root: FbxNode,
    /// Every object under it, depth first.
    pub objects: Vec<FbxNode>,
    /// The `Geometry → Geometry` `OO` connections, in tree order.
    pub connections: Vec<FbxNode>,
}

/// Encode-side entry: the objects of a record tree. `root_id` /
/// `root_name` label the root; every other object gets an id from
/// `alloc` and keeps its own name.
pub(crate) fn elements_from_tree(
    v: &Value,
    root_id: i64,
    root_name: &str,
    alloc: &mut dyn FnMut() -> i64,
) -> Option<TreeEmit> {
    let mut root = tree_node(v, 0)?;
    root.element.properties[0] = FbxProperty::I64(root_id);
    let mut full = root_name.as_bytes().to_vec();
    full.extend_from_slice(b"\x00\x01Geometry");
    root.element.properties[1] = FbxProperty::String(full);
    let mut objects = Vec::new();
    let mut connections = Vec::new();
    fn walk(
        node: TreeNode,
        parent: i64,
        alloc: &mut dyn FnMut() -> i64,
        objects: &mut Vec<FbxNode>,
        connections: &mut Vec<FbxNode>,
    ) {
        for mut child in node.children {
            let id = alloc();
            child.element.properties[0] = FbxProperty::I64(id);
            connections.push(FbxNode {
                name: "C".to_string(),
                properties: vec![
                    FbxProperty::String(b"OO".to_vec()),
                    FbxProperty::I64(id),
                    FbxProperty::I64(parent),
                ],
                children: Vec::new(),
            });
            let element = std::mem::take(&mut child.element);
            objects.push(element);
            walk(child, id, alloc, objects, connections);
        }
    }
    let root_element = std::mem::take(&mut root.element);
    walk(root, root_id, alloc, &mut objects, &mut connections);
    Some(TreeEmit {
        root: root_element,
        objects,
        connections,
    })
}

/// The typed trimmed surface of a `TrimNurbsSurface` record tree.
fn trim_from_node(node: &TreeNode) -> Option<FbxTrimSurface> {
    let flip = node
        .element
        .child("FlipNormals")
        .and_then(|n| n.properties.first())
        .and_then(FbxProperty::as_i64)
        .unwrap_or(0)
        != 0;
    let mut surface = None;
    let mut boundaries = Vec::new();
    for child in &node.children {
        match subtype(&child.element).as_str() {
            "NurbsSurface" if surface.is_none() => {
                surface = FbxNurbsSurface::from_element(&child.element);
            }
            "Boundary" => boundaries.push(FbxBoundary::new(
                FbxBoundary::element_is_outer(&child.element),
                child
                    .children
                    .iter()
                    .filter(|e| subtype(&e.element) == "NurbsCurve")
                    .filter_map(|e| FbxNurbsCurve::from_element(&e.element))
                    .collect(),
            )),
            _ => {}
        }
    }
    Some(FbxTrimSurface::new(surface?, boundaries).with_flip_normals(flip))
}

/// The typed trimmed surface a primitive's record tree describes.
pub fn trim_surface_of(prim: &Primitive) -> Option<FbxTrimSurface> {
    let node = tree_node(prim.extras.get(GEOMETRY_TREE_KEY)?, 0)?;
    (subtype(&node.element) == "TrimNurbsSurface")
        .then(|| trim_from_node(&node))
        .flatten()
}

/// The typed surface a primitive's record tree describes.
pub fn nurbs_surface_of(prim: &Primitive) -> Option<FbxNurbsSurface> {
    let node = tree_node(prim.extras.get(GEOMETRY_TREE_KEY)?, 0)?;
    (subtype(&node.element) == "NurbsSurface")
        .then(|| FbxNurbsSurface::from_element(&node.element))
        .flatten()
}

/// The typed line set a primitive's record tree describes.
pub fn line_of(prim: &Primitive) -> Option<FbxLine> {
    let node = tree_node(prim.extras.get(GEOMETRY_TREE_KEY)?, 0)?;
    (subtype(&node.element) == "Line")
        .then(|| FbxLine::from_element(&node.element))
        .flatten()
}

/// Sub-type string of a primitive's record tree.
pub fn tree_subtype(prim: &Primitive) -> Option<&str> {
    prim.extras.get(GEOMETRY_TREE_KEY)?.get("subtype")?.as_str()
}

/// Decode-side entry: the scene-model mesh for a record tree (module
/// docs table). The tree rides on the first primitive.
pub fn mesh_for_tree(tree: Value, name: Option<String>, how: &NurbsTessellation) -> Mesh {
    let node = tree_node(&tree, 0);
    let mut primitives: Vec<Primitive> = match node.as_ref().map(|n| (subtype(&n.element), n)) {
        Some((st, n)) if st == "NurbsSurface" => FbxNurbsSurface::from_element(&n.element)
            .and_then(|s| s.tessellate(how).ok())
            .into_iter()
            .collect(),
        Some((st, n)) if st == "TrimNurbsSurface" => trim_from_node(n)
            .and_then(|t| t.tessellate(how).ok())
            .into_iter()
            .collect(),
        Some((st, n)) if st == "Line" => FbxLine::from_element(&n.element)
            .map(|l| l.to_primitives())
            .unwrap_or_default(),
        _ => Vec::new(),
    };
    if primitives.is_empty() {
        primitives.push(Primitive::new(Topology::Points));
    }
    primitives[0]
        .extras
        .insert(GEOMETRY_TREE_KEY.to_string(), tree);
    let mut mesh = Mesh::new(name);
    mesh.primitives = primitives;
    mesh
}

/// The record tree of an element with no children of its own.
fn leaf_tree(element: &FbxNode, name: &str, children: Vec<Value>) -> Value {
    json!({
        "name": name,
        "subtype": subtype(element),
        "properties70": element.children_named("Properties70").count().min(2),
        "records": crate::properties70::own_records_json(element),
        "body": crate::properties70::body_json(element),
        "children": children,
    })
}

impl FbxNurbsSurface {
    /// A scene-model mesh for this surface (tessellated per `how`,
    /// carrying the record so the encoder writes a `NurbsSurface`).
    pub fn to_mesh(&self, name: Option<String>, how: &NurbsTessellation) -> Mesh {
        mesh_for_tree(
            leaf_tree(&self.to_element(0, ""), "", Vec::new()),
            name,
            how,
        )
    }
}

impl FbxLine {
    /// A scene-model mesh for this line set (one `LineStrip` per
    /// polyline, carrying the record so the encoder writes a `Line`).
    pub fn to_mesh(&self, name: Option<String>) -> Mesh {
        mesh_for_tree(
            leaf_tree(&self.to_element(0, ""), "", Vec::new()),
            name,
            &NurbsTessellation::default(),
        )
    }
}

impl FbxTrimSurface {
    /// A scene-model mesh for this trimmed surface (tessellated per
    /// `how`), carrying the trim object, the untrimmed surface, every
    /// boundary and every edge curve so the encoder writes them all
    /// back with their connections. Sub-objects are named `surface`,
    /// `boundary<i>` and `boundary<i>_edge<j>`.
    pub fn to_mesh(&self, name: Option<String>, how: &NurbsTessellation) -> Mesh {
        let mut children = vec![leaf_tree(
            &self.surface.to_element(0, ""),
            "surface",
            Vec::new(),
        )];
        for (i, b) in self.boundaries.iter().enumerate() {
            let edges = b
                .edges
                .iter()
                .enumerate()
                .map(|(j, e)| {
                    leaf_tree(
                        &e.to_element(0, ""),
                        &format!("boundary{i}_edge{j}"),
                        Vec::new(),
                    )
                })
                .collect();
            children.push(leaf_tree(
                &b.to_element(0, ""),
                &format!("boundary{i}"),
                edges,
            ));
        }
        mesh_for_tree(leaf_tree(&self.to_element(0, ""), "", children), name, how)
    }
}
