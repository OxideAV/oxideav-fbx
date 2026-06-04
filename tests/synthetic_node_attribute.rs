//! Round 235 — end-to-end `NodeAttribute` subtype-discriminator
//! surfacing via the full `Mesh3DDecoder::decode` path.
//!
//! Builds a synthetic binary-FBX byte buffer carrying a `LimbNode`
//! and a `Null` `NodeAttribute`, each wired to its own `Model` via
//! `Connections { C "OO" }`, encodes it through the round-3 writer,
//! decodes it through the public [`oxideav_fbx::FbxDecoder`], and
//! asserts the resulting `Scene3D`'s nodes carry the §6 kind tag
//! the new `node_attribute` module surfaces.
//!
//! All record shapes follow `docs/3d/fbx/fbx-binary-properties70.md`
//! §5 / §6 (object record header = id + Name+ClassTag + SubType;
//! the third property is the §6 discriminator).

use std::collections::HashMap;

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_document, FbxDecoder,
};
use oxideav_mesh3d::Mesh3DDecoder;
use serde_json::Value;

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

#[test]
fn limbnode_and_null_node_attributes_round_trip_through_decoder() {
    // Two Models with distinct subtypes, each owning a NodeAttribute
    // of matching kind.
    let limb_attr = node_attribute(600, "LimbNode");
    let null_attr = node_attribute(601, "Null");
    let limb_model = model_node(700, "Bone1", "LimbNode");
    let null_model = model_node(701, "Locator1", "Null");

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![limb_attr, null_attr, limb_model, null_model],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_oo(600, 700), c_oo(601, 701)],
    };
    let root = FbxNode {
        name: String::new(),
        properties: Vec::new(),
        children: vec![objects, conns],
    };
    let doc = FbxDocument {
        version: 7500,
        root,
    };

    let bytes = write_document(&doc).expect("encode synthetic doc");
    let scene = FbxDecoder::new()
        .decode(&bytes)
        .expect("decode synthetic doc");

    // Resolve the two Models by name and verify each one carries the
    // §6 discriminator on its Node::extras.
    let mut by_name: HashMap<&str, &oxideav_mesh3d::Node> = HashMap::new();
    for n in &scene.nodes {
        if let Some(name) = n.name.as_deref() {
            by_name.insert(name, n);
        }
    }

    let bone = by_name.get("Bone1").expect("Bone1 node surfaced");
    assert_eq!(
        bone.extras.get("fbx:node_attribute_kind"),
        Some(&Value::String("LimbNode".into())),
        "LimbNode discriminator missing on Bone1.extras",
    );

    let locator = by_name.get("Locator1").expect("Locator1 node surfaced");
    assert_eq!(
        locator.extras.get("fbx:node_attribute_kind"),
        Some(&Value::String("Null".into())),
        "Null discriminator missing on Locator1.extras",
    );
}
