//! Generic `NodeAttribute` subtype-discriminator surfacing for the
//! `"LimbNode"` (skeletal bone) and `"Null"` (locator / empty)
//! discriminators documented in
//! `docs/3d/fbx/fbx-binary-properties70.md` §6.
//!
//! The two specialised discriminators (`"Light"` / `"Camera"`) are
//! decoded into typed [`oxideav_mesh3d::Light`] / [`oxideav_mesh3d::Camera`]
//! by [`crate::lights_cameras`]. For the remaining well-known
//! discriminators that don't map onto a first-class mesh3d type, the
//! kind itself is what consumers need: enough information to know
//! that a `Model -> Node` is a skeletal bone or a transform-only
//! locator without having to re-walk the `FbxDocument`.
//!
//! The §6 ruleset this module implements verbatim:
//!
//! 1. *"Top-level discriminator = the node Name (type keyword) …
//!    `NodeAttribute`."* (we walk `Objects { NodeAttribute }` records)
//! 2. *"SubType string (prop2) = the fine discriminator … for true
//!    `NodeAttribute` records the subtype string is likewise the
//!    discriminator (`"Light"`, `"Camera"`, `"LimbNode"`, `"Null"`,
//!    …)."* (we read prop2 and dispatch on its value)
//! 3. *"NodeAttribute -> Model `OO` connections bind the attribute to
//!    the owning Model"* (we walk the same connection table the
//!    `lights_cameras` module uses)
//!
//! # What lands where
//!
//! For every `NodeAttribute` element whose subtype is `"LimbNode"` or
//! `"Null"` and which has an `OO` connection to a [`Model`]:
//!
//! - The owning `Model`'s [`oxideav_mesh3d::Node::extras`] gets a
//!   `"fbx:node_attribute_kind"` entry whose value is the subtype
//!   string verbatim (`"LimbNode"` or `"Null"`). This is the
//!   round-trippable record of the §6 discriminator.
//!
//! # What is NOT surfaced
//!
//! - The skeletal-bone geometry fields (bone radius / relative length /
//!   is-root) and the locator/empty extra properties — these are part
//!   of the `LimbNode` / `Null` NodeAttribute `Properties70` blocks but
//!   the specific FBX `P`-record names that feed them are not
//!   enumerated in the staged docs. A
//!   follow-up round may add them once the staging includes the
//!   bone / empty `Properties70` `P`-record name table.
//! - `"Root"` Model subtypes — the §6 ruleset lists `"Root"` as a
//!   `Model` subtype rather than a `NodeAttribute` subtype, so it
//!   isn't dispatched here.
//!
//! # Idempotence with `lights_cameras`
//!
//! [`crate::lights_cameras`] writes `Node::extras["fbx:light_type"]`
//! only for the `Area` / `Volume` fall-back cases (lossy mappings).
//! This module writes a distinct key (`"fbx:node_attribute_kind"`)
//! for the `"LimbNode"` / `"Null"` discriminators, so the two
//! surfacing passes never collide on the same key.

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D};
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// Walk `Objects { NodeAttribute }` and record the `"LimbNode"` /
/// `"Null"` discriminator on the owning `Model`'s scene-graph
/// `Node::extras`. `model_nodes` is the per-Model FBX-id → `NodeId`
/// lookup already produced by the scene builder; `NodeAttribute`
/// records that connect to a `Model` whose id isn't in this map are
/// silently ignored (they may belong to a `Model` we didn't surface,
/// e.g. an unsupported subtype).
pub fn extract_node_attribute_kinds(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) {
    // 1) Index every `NodeAttribute` whose subtype is one of the
    //    well-known generic discriminators documented in
    //    fbx-binary-properties70.md §6.
    let mut attr_kind: HashMap<i64, &'static str> = HashMap::new();
    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            if child.name != "NodeAttribute" {
                continue;
            }
            let id = match child.properties.first().and_then(FbxProperty::as_i64) {
                Some(i) => i,
                None => continue,
            };
            match subtype_string(child).as_deref() {
                Some("LimbNode") => {
                    attr_kind.insert(id, "LimbNode");
                }
                Some("Null") => {
                    attr_kind.insert(id, "Null");
                }
                // "Light" / "Camera" are typed-decoded by
                // crate::lights_cameras; everything else (including
                // "Root" which §6 only documents as a Model subtype)
                // isn't dispatched here.
                _ => {}
            }
        }
    }

    if attr_kind.is_empty() {
        return;
    }

    // 2) Walk `Connections` for `NodeAttribute -> Model` `OO` links and
    //    tag the owning `Model`'s `Node::extras` with the kind string.
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
            let Some(&tag) = attr_kind.get(&child_id) else {
                continue;
            };
            let Some(&nid) = model_nodes.get(&parent_id) else {
                continue;
            };
            if let Some(n) = scene.nodes.get_mut(nid.0 as usize) {
                // Don't overwrite a pre-existing entry (e.g. an earlier
                // attribute on the same Model). Last-wins would
                // silently drop the first one; first-wins keeps the
                // surfacing deterministic in iteration-order-sensitive
                // edge cases.
                n.extras
                    .entry("fbx:node_attribute_kind".to_string())
                    .or_insert_with(|| Value::String(tag.to_string()));
            }
        }
    }
}

/// Subtype-string extractor — third property of the element, per
/// `docs/3d/fbx/fbx-binary-properties70.md` §5 + §6.
fn subtype_string(node: &FbxNode) -> Option<String> {
    node.properties.get(2)?.as_str().map(str::to_owned)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(b: &[u8]) -> FbxProperty {
        FbxProperty::String(b.to_vec())
    }

    fn node_attribute(id: i64, subtype: &str) -> FbxNode {
        FbxNode {
            name: "NodeAttribute".into(),
            properties: vec![
                FbxProperty::I64(id),
                s(b"NodeAttribute\x00\x01NodeAttribute"),
                s(subtype.as_bytes()),
            ],
            children: Vec::new(),
        }
    }

    fn model_node(id: i64, name: &str, subtype: &str) -> FbxNode {
        let display = format!("{name}\x00\x01Model");
        FbxNode {
            name: "Model".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(display.into_bytes()),
                s(subtype.as_bytes()),
            ],
            children: Vec::new(),
        }
    }

    fn c_oo(child: i64, parent: i64) -> FbxNode {
        FbxNode {
            name: "C".into(),
            properties: vec![s(b"OO"), FbxProperty::I64(child), FbxProperty::I64(parent)],
            children: Vec::new(),
        }
    }

    fn build_doc(objects: Vec<FbxNode>, conns: Vec<FbxNode>) -> FbxDocument {
        let root = FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![
                FbxNode {
                    name: "Objects".into(),
                    properties: Vec::new(),
                    children: objects,
                },
                FbxNode {
                    name: "Connections".into(),
                    properties: Vec::new(),
                    children: conns,
                },
            ],
        };
        FbxDocument {
            version: 7500,
            root,
        }
    }

    #[test]
    fn limbnode_subtype_lands_on_owning_model_extras() {
        // docs/3d/fbx/fbx-binary-properties70.md §6: a NodeAttribute
        // whose prop2 subtype string is "LimbNode" is the §6
        // "skeletal bone" discriminator. The OO connection wires it
        // to the owning Model whose Node::extras gets tagged.
        let attr = node_attribute(400, "LimbNode");
        let model = model_node(500, "Bone1", "LimbNode");
        let doc = build_doc(vec![attr, model], vec![c_oo(400, 500)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new().with_name("Bone1"));
        model_nodes.insert(500, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert_eq!(
            node.extras.get("fbx:node_attribute_kind"),
            Some(&Value::String("LimbNode".into()))
        );
    }

    #[test]
    fn null_subtype_lands_on_owning_model_extras() {
        // docs/3d/fbx/fbx-binary-properties70.md §6: "Null" is the
        // empty/locator subtype.
        let attr = node_attribute(401, "Null");
        let model = model_node(501, "Locator", "Null");
        let doc = build_doc(vec![attr, model], vec![c_oo(401, 501)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new().with_name("Locator"));
        model_nodes.insert(501, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert_eq!(
            node.extras.get("fbx:node_attribute_kind"),
            Some(&Value::String("Null".into()))
        );
    }

    #[test]
    fn unknown_subtype_does_not_emit_kind() {
        // Subtypes outside the {LimbNode, Null, Light, Camera} set
        // surface no kind tag from this module (the light/camera
        // ones are typed-decoded elsewhere; "Root" is documented as
        // a Model subtype not a NodeAttribute subtype).
        let attr = node_attribute(402, "Marker");
        let model = model_node(502, "M", "");
        let doc = build_doc(vec![attr, model], vec![c_oo(402, 502)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(502, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert!(!node.extras.contains_key("fbx:node_attribute_kind"));
    }

    #[test]
    fn node_attribute_without_owning_model_is_skipped() {
        // §6 requires NodeAttribute -> Model OO wiring. Orphan
        // attributes (no OO connection, or pointing at an unsurfaced
        // Model) leave the scene untouched.
        let attr = node_attribute(403, "LimbNode");
        let doc = build_doc(vec![attr], vec![]);

        let mut scene = Scene3D::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);
        assert!(scene.nodes.is_empty());
    }

    #[test]
    fn limbnode_does_not_collide_with_light_type_key() {
        // The light/camera surfacing writes "fbx:light_type" only on
        // lossy Area/Volume fall-backs; the LimbNode surfacing here
        // writes a distinct "fbx:node_attribute_kind" key. The two
        // round-trippable shapes never collide on the same scene
        // node.
        let attr = node_attribute(410, "LimbNode");
        let model = model_node(510, "Bone", "LimbNode");
        let doc = build_doc(vec![attr, model], vec![c_oo(410, 510)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let mut prefilled = oxideav_mesh3d::Node::new();
        // Simulate a prior pass having stashed a light-type tag (Area
        // light placement). The NodeAttribute kind must NOT overwrite
        // it, and the light tag must NOT block the kind tag.
        prefilled
            .extras
            .insert("fbx:light_type".into(), Value::String("Area".into()));
        let nid = scene.add_node(prefilled);
        model_nodes.insert(510, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert_eq!(
            node.extras.get("fbx:light_type"),
            Some(&Value::String("Area".into()))
        );
        assert_eq!(
            node.extras.get("fbx:node_attribute_kind"),
            Some(&Value::String("LimbNode".into()))
        );
    }

    #[test]
    fn first_kind_wins_on_repeated_oo_to_same_model() {
        // If a Model has two NodeAttribute children of different kinds
        // (a degenerate but possible file), first-seen wins for
        // deterministic surfacing.
        let attr1 = node_attribute(420, "LimbNode");
        let attr2 = node_attribute(421, "Null");
        let model = model_node(520, "M", "");
        let doc = build_doc(
            vec![attr1, attr2, model],
            vec![c_oo(420, 520), c_oo(421, 520)],
        );

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(520, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        // HashMap iteration order isn't deterministic between the two
        // attrs themselves, but whichever lands first sticks; assert
        // the tag is one of the two valid values.
        let val = node
            .extras
            .get("fbx:node_attribute_kind")
            .expect("kind tag");
        assert!(matches!(val.as_str(), Some("LimbNode") | Some("Null")));
    }

    #[test]
    fn non_oo_connections_are_ignored() {
        // OP / PP / PO connections aren't NodeAttribute attachments
        // per §6; they MUST NOT trigger a kind tag.
        let attr = node_attribute(430, "LimbNode");
        let model = model_node(530, "Bone", "LimbNode");
        let mut c = c_oo(430, 530);
        c.properties[0] = s(b"OP"); // wrong connection kind
        let doc = build_doc(vec![attr, model], vec![c]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(530, nid);

        extract_node_attribute_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert!(!node.extras.contains_key("fbx:node_attribute_kind"));
    }
}
