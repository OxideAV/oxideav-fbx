//! Opaque object passthrough — `Objects` elements of a class this
//! crate gives no typed home (`CollectionExclusive` display layers,
//! and whatever else a producer writes), carried verbatim on
//! `Scene3D::extras["fbx:opaque_objects"]` and re-emitted by the
//! writer with their connections to typed endpoints rebuilt.
//!
//! Each entry is `{ "class", "name", "subtype", "records": [P…],
//! "body": [leaf…], "connections": [ { "kind": "OO" | "OP", "role":
//! "child" | "parent", "peer": { "node": idx } | { "object": name } |
//! "root", "property"? } ] }` — `records` in the
//! [`crate::properties70::p_record_to_json`] shape, `body` in the
//! array-aware [`crate::properties70::body_json`] shape, and `role`
//! the object's own position in the `C` record. The writer re-creates
//! an edge whose peer is a scene node (by index) or the document
//! root; an `object`-named peer is an element this writer has no id
//! for, so that edge is not re-created (documented lossy edge).

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D};
use serde_json::{json, Value};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// `Scene3D::extras` key (see the module docs).
pub const OPAQUE_OBJECTS_KEY: &str = "fbx:opaque_objects";

/// The `Objects` classes every other module decodes; anything else
/// is opaque.
const TYPED_CLASSES: &[&str] = &[
    "Geometry",
    "Model",
    "Material",
    "Texture",
    "Video",
    "NodeAttribute",
    "Deformer",
    "Pose",
    "AnimationStack",
    "AnimationLayer",
    "AnimationCurveNode",
    "AnimationCurve",
    "Constraint",
];

/// Surface every opaque `Objects` element (document order).
pub fn extract_opaque_objects(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) {
    let Some(objects) = doc.root.child("Objects") else {
        return;
    };
    let mut names: HashMap<i64, String> = HashMap::new();
    let mut opaque: Vec<(i64, &FbxNode)> = Vec::new();
    for child in &objects.children {
        let Some(id) = child.properties.first().and_then(FbxProperty::as_i64) else {
            continue;
        };
        names.insert(id, display_name(child));
        if !TYPED_CLASSES.contains(&child.name.as_str()) {
            opaque.push((id, child));
        }
    }
    if opaque.is_empty() {
        return;
    }
    let ids: HashMap<i64, ()> = opaque.iter().map(|(id, _)| (*id, ())).collect();
    let mut conns: HashMap<i64, Vec<Value>> = HashMap::new();
    if let Some(c_root) = doc.root.child("Connections") {
        for c in c_root.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let prop = c.properties.get(3).and_then(FbxProperty::as_str);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            let peer = |id: i64| -> Value {
                if id == 0 {
                    Value::String("root".into())
                } else if let Some(nid) = model_nodes.get(&id) {
                    json!({ "node": nid.0 })
                } else {
                    json!({ "object": names.get(&id).cloned().unwrap_or_default() })
                }
            };
            for (me, role, other) in [
                (child_id, "child", parent_id),
                (parent_id, "parent", child_id),
            ] {
                if ids.contains_key(&me) {
                    let mut e = json!({ "kind": kind, "role": role, "peer": peer(other) });
                    if let Some(p) = prop {
                        e["property"] = Value::String(p.to_owned());
                    }
                    conns.entry(me).or_default().push(e);
                }
            }
        }
    }
    let entries: Vec<Value> = opaque
        .iter()
        .map(|(id, node)| {
            json!({
                "class": node.name,
                "name": names.get(id).cloned().unwrap_or_default(),
                "subtype": node.properties.get(2).and_then(FbxProperty::as_str).unwrap_or(""),
                "records": crate::properties70::own_records_json(node),
                "body": crate::properties70::body_json(node),
                "connections": conns.remove(id).unwrap_or_default(),
            })
        })
        .collect();
    scene
        .extras
        .entry(OPAQUE_OBJECTS_KEY.to_string())
        .or_insert(Value::Array(entries));
}

/// Rebuild the opaque elements + their typed-endpoint connections.
/// Returns `(objects, connections)`.
pub(crate) fn build_opaque_objects(
    scene: &Scene3D,
    node_fbx_id: impl Fn(usize) -> Option<i64>,
    mut alloc: impl FnMut() -> i64,
) -> (Vec<FbxNode>, Vec<FbxNode>) {
    let mut objects = Vec::new();
    let mut connections = Vec::new();
    for e in scene
        .extras
        .get(OPAQUE_OBJECTS_KEY)
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        let Some(class) = e.get("class").and_then(Value::as_str) else {
            continue;
        };
        let id = alloc();
        let name = e.get("name").and_then(Value::as_str).unwrap_or("");
        let subtype = e.get("subtype").and_then(Value::as_str).unwrap_or("");
        let mut children: Vec<FbxNode> = Vec::new();
        let records: Vec<FbxNode> = e
            .get("records")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(crate::properties70::json_to_p_record)
            .collect();
        if !records.is_empty() {
            children.push(FbxNode {
                name: "Properties70".to_string(),
                properties: Vec::new(),
                children: records,
            });
        }
        children.extend(
            e.get("body")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(crate::properties70::json_to_body_leaf),
        );
        objects.push(FbxNode {
            name: class.to_owned(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(name_class(name, class)),
                FbxProperty::String(subtype.as_bytes().to_vec()),
            ],
            children,
        });
        for c in e
            .get("connections")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
        {
            let kind = c.get("kind").and_then(Value::as_str).unwrap_or("OO");
            let peer_id = match c.get("peer") {
                Some(Value::String(s)) if s == "root" => Some(0),
                Some(p) => p
                    .get("node")
                    .and_then(Value::as_u64)
                    .and_then(|n| node_fbx_id(n as usize)),
                None => None,
            };
            let Some(peer_id) = peer_id else { continue };
            let (child_id, parent_id) = match c.get("role").and_then(Value::as_str) {
                Some("parent") => (peer_id, id),
                _ => (id, peer_id),
            };
            let mut props = vec![
                FbxProperty::String(kind.as_bytes().to_vec()),
                FbxProperty::I64(child_id),
                FbxProperty::I64(parent_id),
            ];
            if let Some(p) = c.get("property").and_then(Value::as_str) {
                props.push(FbxProperty::String(p.as_bytes().to_vec()));
            }
            connections.push(FbxNode {
                name: "C".to_string(),
                properties: props,
                children: Vec::new(),
            });
        }
    }
    (objects, connections)
}

/// `Name\x00\x01Class` → `Name` (binary join) or `Class::Name` → `Name`
/// (ASCII join).
fn display_name(node: &FbxNode) -> String {
    let raw = node
        .properties
        .get(1)
        .and_then(FbxProperty::as_str)
        .unwrap_or("");
    if let Some(i) = raw.find('\u{0}') {
        return raw[..i].to_owned();
    }
    match raw.split_once("::") {
        Some((_, n)) => n.to_owned(),
        None => raw.to_owned(),
    }
}

fn name_class(name: &str, class: &str) -> Vec<u8> {
    let mut v = name.as_bytes().to_vec();
    v.push(0x00);
    v.push(0x01);
    v.extend_from_slice(class.as_bytes());
    v
}
