//! `Documents` + `References` section decoder — document-catalogue
//! metadata surfaced onto [`oxideav_mesh3d::Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-ascii-grammar.md` §7, the FBX top-level
//! section order places `Documents` and `References` between
//! `GlobalSettings` and `Definitions`:
//!
//! ```text
//! Documents:  { ... }   ← Count + Document node(s)
//! References:  { }      ← (empty in sample)
//! ```
//!
//! The staged `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture
//! shows the full observed `Documents` body:
//!
//! ```text
//! Documents:  {
//!   Count: 1
//!   Document: 2359325563280, "", "Scene" {
//!     Properties70:  {
//!       P: "SourceObject", "object", "", ""
//!       P: "ActiveAnimStackName", "KString", "", "", "Take 001"
//!     }
//!     RootNode: 0
//!   }
//! }
//! ```
//!
//! Field grammar, read from the fixture text + the §7c object-line
//! convention:
//!
//! - `Count` — leaf, the number of `Document` records.
//! - `Document: <UID>, "<name>", "<subtype>" { ... }` — one
//!   node-with-body per document, following the §7c three-value object
//!   line (UID / name / subtype). In the sample the name is empty and
//!   the subtype is `"Scene"`.
//! - `Properties70` — a §8 `P`-record block. The two records observed:
//!   `SourceObject` (`"object"` typeName, empty body — the
//!   [`crate::properties70::PropertyMap::as_object_ref`] empty-ref
//!   case) and `ActiveAnimStackName` (`"KString"`), naming the
//!   animation stack that is active when the file opens.
//! - `RootNode` — leaf, the UID of the document's root object (`0` =
//!   the implicit scene root, the same sentinel the §7d `C:` records
//!   use for root attachment).
//!
//! `ActiveAnimStackName` is the join key back to the `Objects`
//! section: it equals the `AnimationStack` *display name* (fixture:
//! `"Take 001"` ⇔ `AnimStack::Take 001`), the same name the
//! [`crate::animation`] module keys each
//! [`oxideav_mesh3d::Animation`] by — and the same name the `Takes`
//! section's `Current` leaf carries (see [`crate::takes`]).
//!
//! # Surfacing
//!
//! Mirroring the [`crate::takes`] convention, the catalogue lands on
//! [`oxideav_mesh3d::Scene3D::extras`]:
//!
//! - `extras["fbx:active_anim_stack"]` — the first document's
//!   `ActiveAnimStackName` string (omitted when absent).
//! - `extras["fbx:documents"]` — a JSON array, one object per
//!   `Document`: `{ "name", "subtype", "active_anim_stack"? }`.
//!   Document UIDs and the `RootNode` UID are **not** surfaced — they
//!   index the source file's private object arena and carry no meaning
//!   once the graph is resolved (a re-encode allocates fresh ids).
//!
//! `References` was observed empty in the sample; there is nothing to
//! surface for it (the [`crate::scene_writer`] encoder still re-emits
//! the empty section so the §7 section set survives a round trip).
//!
//! Both the binary and ASCII front-ends render the identical node tree
//! (`fbx-binary-properties70.md` §4 isomorphism note), so this one
//! walker covers both encodings.

use oxideav_mesh3d::Scene3D;
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode};
use crate::properties70::PropertyMap;

/// FBX top-level node name for the document catalogue (§7 order:
/// after `GlobalSettings`, before `References` / `Definitions`).
pub const DOCUMENTS_NODE: &str = "Documents";

/// FBX top-level node name for the (observed-empty) references
/// section (§7 order: between `Documents` and `Definitions`).
pub const REFERENCES_NODE: &str = "References";

/// Decode the `Documents` section of `doc` and surface its catalogue
/// onto `scene.extras`.
///
/// Returns the number of `Document` records surfaced (zero when the
/// document has no `Documents` node or the node holds no `Document`
/// children).
pub fn extract_documents(doc: &FbxDocument, scene: &mut Scene3D) -> usize {
    let Some(documents_node) = doc.root.child(DOCUMENTS_NODE) else {
        return 0;
    };

    let mut docs_json: Vec<Value> = Vec::new();
    let mut active_stack: Option<String> = None;
    for document in documents_node.children_named("Document") {
        let (json, stack) = document_to_json(document);
        if active_stack.is_none() {
            active_stack = stack;
        }
        docs_json.push(json);
    }

    // First document's ActiveAnimStackName — the §8 KString naming the
    // stack that is active when the file opens.
    if let Some(stack) = active_stack {
        scene
            .extras
            .entry("fbx:active_anim_stack".to_owned())
            .or_insert_with(|| Value::String(stack));
    }

    let count = docs_json.len();
    if count > 0 {
        scene
            .extras
            .entry("fbx:documents".to_owned())
            .or_insert_with(|| Value::Array(docs_json));
    }

    count
}

/// Render one `Document` node as a JSON object; also return its
/// `ActiveAnimStackName` (when present) so the caller can promote the
/// first one to the scene-wide `fbx:active_anim_stack` key.
fn document_to_json(document: &FbxNode) -> (Value, Option<String>) {
    let mut obj = serde_json::Map::new();

    // §7c object-line convention: value 1 = UID (not surfaced —
    // private to the source file's arena), value 2 = "ClassTag::Name"
    // (binary: Name\x00\x01ClassTag), value 3 = subtype string. The
    // fixture's Document line carries an empty name and subtype
    // "Scene".
    obj.insert(
        "name".to_owned(),
        Value::String(element_name(document).unwrap_or_default()),
    );
    obj.insert(
        "subtype".to_owned(),
        Value::String(
            document
                .properties
                .get(2)
                .and_then(|p| p.as_str())
                .unwrap_or("")
                .to_owned(),
        ),
    );

    // `ActiveAnimStackName` — §8 "KString" typeName. The typed
    // accessor keeps a coincidental same-named record of another
    // typeName from being mistaken for the stack name.
    let props = PropertyMap::from_element(document);
    let stack = props
        .as_kstring("ActiveAnimStackName")
        .filter(|s| !s.is_empty())
        .map(str::to_owned);
    if let Some(s) = &stack {
        obj.insert("active_anim_stack".to_owned(), Value::String(s.clone()));
    }

    (Value::Object(obj), stack)
}

/// Read the user-facing name from property[1] of a §7c-shaped object
/// line. Binary joins `Name` + `ClassTag` with `\x00\x01`; ASCII uses
/// `Name::ClassTag` rendered as one string — the fixture's Document
/// name is empty in both forms, so only the `\x00` split needs
/// handling (same convention as the `Objects` walker).
fn element_name(node: &FbxNode) -> Option<String> {
    let raw = match node.properties.get(1)? {
        crate::binary::FbxProperty::String(b) => b,
        _ => return None,
    };
    if let Some(sep) = raw.iter().position(|&b| b == 0x00) {
        std::str::from_utf8(&raw[..sep]).ok().map(str::to_owned)
    } else {
        std::str::from_utf8(raw).ok().map(str::to_owned)
    }
}

/// Convenience: pull the surfaced document catalogue back off a
/// scene's extras. Returns `None` when no `Documents` section was
/// present in the source document.
pub fn documents_from_extras(scene: &Scene3D) -> Option<&[Value]> {
    match scene.extras.get("fbx:documents") {
        Some(Value::Array(v)) => Some(v.as_slice()),
        _ => None,
    }
}

/// Convenience: pull the surfaced active-animation-stack name back
/// off a scene's extras. Returns `None` when no document carried an
/// `ActiveAnimStackName`.
pub fn active_anim_stack_from_extras(scene: &Scene3D) -> Option<&str> {
    match scene.extras.get("fbx:active_anim_stack") {
        Some(Value::String(s)) => Some(s.as_str()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::FbxProperty;

    fn s(text: &str) -> FbxProperty {
        FbxProperty::String(text.as_bytes().to_vec())
    }

    fn leaf(name: &str, props: Vec<FbxProperty>) -> FbxNode {
        FbxNode {
            name: name.to_owned(),
            properties: props,
            children: Vec::new(),
        }
    }

    fn p_record(name: &str, type_name: &str, values: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![s(name), s(type_name), s(""), s("")];
        props.extend(values);
        leaf("P", props)
    }

    /// Build the fixture's `Documents` section as a typed node tree.
    fn fixture_documents() -> FbxNode {
        let props70 = FbxNode {
            name: "Properties70".to_owned(),
            properties: Vec::new(),
            children: vec![
                p_record("SourceObject", "object", Vec::new()),
                p_record("ActiveAnimStackName", "KString", vec![s("Take 001")]),
            ],
        };
        let document = FbxNode {
            name: "Document".to_owned(),
            properties: vec![FbxProperty::I64(2359325563280), s(""), s("Scene")],
            children: vec![props70, leaf("RootNode", vec![FbxProperty::I64(0)])],
        };
        FbxNode {
            name: DOCUMENTS_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![leaf("Count", vec![FbxProperty::I32(1)]), document],
        }
    }

    fn doc_with(documents: FbxNode) -> FbxDocument {
        FbxDocument {
            version: 7500,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![documents],
            },
        }
    }

    #[test]
    fn surfaces_fixture_document_catalogue() {
        let doc = doc_with(fixture_documents());
        let mut scene = Scene3D::new();
        assert_eq!(extract_documents(&doc, &mut scene), 1);

        assert_eq!(active_anim_stack_from_extras(&scene), Some("Take 001"));

        let docs = documents_from_extras(&scene).expect("fbx:documents present");
        assert_eq!(docs.len(), 1);
        let d = &docs[0];
        assert_eq!(d["name"], Value::String(String::new()));
        assert_eq!(d["subtype"], Value::String("Scene".to_owned()));
        assert_eq!(d["active_anim_stack"], Value::String("Take 001".to_owned()));
        // UIDs are not surfaced — they index the source file's private
        // object arena.
        let obj = d.as_object().unwrap();
        assert!(!obj.contains_key("uid"));
        assert!(!obj.contains_key("root_node"));
    }

    #[test]
    fn active_stack_matches_takes_current_join_key() {
        // The ActiveAnimStackName equals the AnimationStack display
        // name / the Takes `Current` name (fixture: "Take 001").
        let doc = doc_with(fixture_documents());
        let mut scene = Scene3D::new();
        extract_documents(&doc, &mut scene);
        assert_eq!(active_anim_stack_from_extras(&scene), Some("Take 001"));
    }

    #[test]
    fn no_documents_node_surfaces_nothing() {
        let doc = FbxDocument {
            version: 7500,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: Vec::new(),
            },
        };
        let mut scene = Scene3D::new();
        assert_eq!(extract_documents(&doc, &mut scene), 0);
        assert!(scene.extras.is_empty());
        assert!(documents_from_extras(&scene).is_none());
        assert!(active_anim_stack_from_extras(&scene).is_none());
    }

    #[test]
    fn document_without_stack_name_omits_key() {
        // A Document whose Properties70 lacks ActiveAnimStackName (or
        // has no Properties70 at all) surfaces name/subtype only.
        let document = FbxNode {
            name: "Document".to_owned(),
            properties: vec![FbxProperty::I64(7), s(""), s("Scene")],
            children: vec![leaf("RootNode", vec![FbxProperty::I64(0)])],
        };
        let documents = FbxNode {
            name: DOCUMENTS_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![document],
        };
        let doc = doc_with(documents);
        let mut scene = Scene3D::new();
        assert_eq!(extract_documents(&doc, &mut scene), 1);
        assert!(active_anim_stack_from_extras(&scene).is_none());
        let d = &documents_from_extras(&scene).unwrap()[0];
        assert!(!d.as_object().unwrap().contains_key("active_anim_stack"));
    }

    #[test]
    fn empty_active_stack_name_treated_as_absent() {
        // An `ActiveAnimStackName` with an empty body (a file saved
        // with no takes) is not promoted to the scene-wide key — an
        // empty stack name selects nothing.
        let props70 = FbxNode {
            name: "Properties70".to_owned(),
            properties: Vec::new(),
            children: vec![p_record("ActiveAnimStackName", "KString", vec![s("")])],
        };
        let document = FbxNode {
            name: "Document".to_owned(),
            properties: vec![FbxProperty::I64(7), s(""), s("Scene")],
            children: vec![props70],
        };
        let documents = FbxNode {
            name: DOCUMENTS_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![document],
        };
        let doc = doc_with(documents);
        let mut scene = Scene3D::new();
        extract_documents(&doc, &mut scene);
        assert!(active_anim_stack_from_extras(&scene).is_none());
    }

    #[test]
    fn first_document_wins_active_stack() {
        // Multiple Document records: the first carrying a stack name
        // provides the scene-wide key; the array keeps them all.
        let make_doc = |name: &str, stack: Option<&str>| {
            let mut children = Vec::new();
            if let Some(st) = stack {
                children.push(FbxNode {
                    name: "Properties70".to_owned(),
                    properties: Vec::new(),
                    children: vec![p_record("ActiveAnimStackName", "KString", vec![s(st)])],
                });
            }
            FbxNode {
                name: "Document".to_owned(),
                properties: vec![FbxProperty::I64(1), s(name), s("Scene")],
                children,
            }
        };
        let documents = FbxNode {
            name: DOCUMENTS_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![
                make_doc("A", None),
                make_doc("B", Some("Walk")),
                make_doc("C", Some("Run")),
            ],
        };
        let doc = doc_with(documents);
        let mut scene = Scene3D::new();
        assert_eq!(extract_documents(&doc, &mut scene), 3);
        assert_eq!(active_anim_stack_from_extras(&scene), Some("Walk"));
        let docs = documents_from_extras(&scene).unwrap();
        assert_eq!(docs.len(), 3);
        assert_eq!(docs[0]["name"].as_str(), Some("A"));
        assert_eq!(docs[2]["active_anim_stack"].as_str(), Some("Run"));
    }

    #[test]
    fn binary_name_classtag_split() {
        // Binary form joins Name + ClassTag with \x00\x01 — the name
        // half is what surfaces (same convention as the Objects
        // walker).
        let mut raw = b"MyDoc".to_vec();
        raw.push(0x00);
        raw.push(0x01);
        raw.extend_from_slice(b"Document");
        let document = FbxNode {
            name: "Document".to_owned(),
            properties: vec![FbxProperty::I64(9), FbxProperty::String(raw), s("Scene")],
            children: Vec::new(),
        };
        let documents = FbxNode {
            name: DOCUMENTS_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![document],
        };
        let doc = doc_with(documents);
        let mut scene = Scene3D::new();
        extract_documents(&doc, &mut scene);
        let d = &documents_from_extras(&scene).unwrap()[0];
        assert_eq!(d["name"].as_str(), Some("MyDoc"));
    }
}
