//! `Takes` section decoder — animation-take time-span metadata
//! surfaced onto [`oxideav_mesh3d::Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-ascii-grammar.md` §7e, the FBX top-level
//! `Takes` node (the last of the §7 ordered sections, after
//! `Connections`) catalogues the file's animation *takes* — the
//! authoring-tool name for what the `Objects` section stores as
//! `AnimationStack` records:
//!
//! ```text
//! Takes:  {
//!   Current: "Take 001"
//!   Take: "Take 001" {
//!     FileName: "Take_001.tak"
//!     LocalTime: 1924423250,230930790000
//!     ReferenceTime: 1924423250,230930790000
//!   }
//! }
//! ```
//!
//! Field grammar, read verbatim from §7e:
//!
//! - `Current` — a leaf carrying the **name of the active take**
//!   (string).
//! - `Take: "<name>" { ... }` — one node-with-body per take. Its
//!   single value is the take name; the §7c object-line convention
//!   does **not** apply here (no UID / classtag triple), the take name
//!   alone is the value-list.
//! - `FileName` — leaf, string. The legacy external `.tak` filename
//!   (always present in the SDK-written sample even when the take is
//!   embedded).
//! - `LocalTime` / `ReferenceTime` — each a **two-integer KTime pair**
//!   `start,stop` (§5 "Two-int time pair": *"KTime values written as
//!   two comma-separated integers"*). `LocalTime` is the take's own
//!   playback span; `ReferenceTime` the reference span the SDK records
//!   alongside it (identical in the sample).
//!
//! The take name is the join key back to the `Objects` section: the
//! §7c `AnimationStack` whose **display name** equals the `Take` name
//! is the same logical clip (fixture: `AnimationStack:
//! "AnimStack::Take 001"` ⇔ `Take: "Take 001"`). The
//! [`crate::animation`] module already keys each
//! [`oxideav_mesh3d::Animation`] by that display name, so a consumer
//! can pair an animation with its take time-span via the
//! [`oxideav_mesh3d::Animation::name`] string and the surfaced
//! `extras["fbx:takes"]` array.
//!
//! # Surfacing
//!
//! `oxideav_mesh3d::Animation` carries no `extras` map (only `name` +
//! `channels`), so the take catalogue is surfaced scene-wide on
//! [`oxideav_mesh3d::Scene3D::extras`], mirroring the
//! [`crate::globals`] `GlobalSettings` convention:
//!
//! - `extras["fbx:current_take"]` — the `Current` leaf string (omitted
//!   when absent).
//! - `extras["fbx:takes"]` — a JSON array, one object per `Take`:
//!   `{ "name", "file_name"?, "local_time": [start, stop]?,
//!   "reference_time": [start, stop]? }`. The KTime integers stay as
//!   JSON numbers (i64-exact: `KTIME_TICKS_PER_SECOND ≈ 4.6e10` is well
//!   outside f32 range — the same reason [`crate::globals`] keeps
//!   `TimeSpanStart` / `TimeSpanStop` as longs); a consumer converts to
//!   seconds with the [`crate::animation::KTIME_TICKS_PER_SECOND`]
//!   constant.
//!
//! Both the binary and ASCII front-ends render the identical node tree
//! (`fbx-binary-properties70.md` §4 isomorphism note), so this one
//! walker covers both encodings.

use oxideav_mesh3d::Scene3D;
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode};

/// FBX top-level node name for the takes catalogue. Sibling of
/// `Objects` / `Connections` / `GlobalSettings` (per
/// `docs/3d/fbx/fbx-ascii-grammar.md` §7 top-level section list).
pub const TAKES_NODE: &str = "Takes";

/// Decode the `Takes` section of `doc` and surface its catalogue onto
/// `scene.extras`.
///
/// Returns the number of `Take` records surfaced (zero when the
/// document has no `Takes` node or the node holds no `Take` children).
/// The `Current` leaf, when present, is surfaced regardless of the
/// `Take` count.
pub fn extract_takes(doc: &FbxDocument, scene: &mut Scene3D) -> usize {
    let Some(takes_node) = doc.root.child(TAKES_NODE) else {
        return 0;
    };

    // `Current: "<name>"` leaf — the active take.
    if let Some(current) = takes_node
        .child("Current")
        .and_then(|n| n.properties.first())
        .and_then(|p| p.as_str())
    {
        scene
            .extras
            .entry("fbx:current_take".to_owned())
            .or_insert_with(|| Value::String(current.to_owned()));
    }

    // One JSON object per `Take: "<name>" { ... }` node.
    let mut takes_json: Vec<Value> = Vec::new();
    for take in takes_node.children_named("Take") {
        takes_json.push(take_to_json(take));
    }

    let count = takes_json.len();
    if count > 0 {
        scene
            .extras
            .entry("fbx:takes".to_owned())
            .or_insert_with(|| Value::Array(takes_json));
    }

    count
}

/// Render one `Take` node as a JSON object per the §7e field grammar.
fn take_to_json(take: &FbxNode) -> Value {
    let mut obj = serde_json::Map::new();

    // value 1 of the `Take:` line is the take name (§7e — the take
    // name alone is the value-list, no UID/classtag triple).
    let name = take
        .properties
        .first()
        .and_then(|p| p.as_str())
        .unwrap_or("");
    obj.insert("name".to_owned(), Value::String(name.to_owned()));

    // `FileName: "<...>"` leaf — string.
    if let Some(file_name) = take
        .child("FileName")
        .and_then(|n| n.properties.first())
        .and_then(|p| p.as_str())
    {
        obj.insert("file_name".to_owned(), Value::String(file_name.to_owned()));
    }

    // `LocalTime` / `ReferenceTime` — two-integer KTime pairs.
    if let Some(pair) = time_pair(take, "LocalTime") {
        obj.insert("local_time".to_owned(), pair);
    }
    if let Some(pair) = time_pair(take, "ReferenceTime") {
        obj.insert("reference_time".to_owned(), pair);
    }

    Value::Object(obj)
}

/// Read a `<name>: start,stop` two-integer KTime leaf as a
/// `[start, stop]` JSON array. Returns `None` when the leaf is absent
/// or doesn't carry exactly two integer values.
///
/// Per §5 ("Two-int time pair") the pair is two comma-separated
/// integers; the ASCII parser surfaces them as two scalar `I32` / `I64`
/// properties and the binary form would carry two `L` (int64) scalars.
/// `FbxProperty::as_i64` widens both losslessly, so the same reader
/// covers either encoding.
fn time_pair(take: &FbxNode, name: &str) -> Option<Value> {
    let leaf = take.child(name)?;
    if leaf.properties.len() != 2 {
        return None;
    }
    let start = leaf.properties[0].as_i64()?;
    let stop = leaf.properties[1].as_i64()?;
    Some(Value::Array(vec![
        Value::Number(start.into()),
        Value::Number(stop.into()),
    ]))
}

/// Convenience: pull the surfaced take catalogue back off a scene's
/// extras as a borrowed slice of JSON objects. Returns `None` when no
/// `Takes` section was present in the source document.
pub fn takes_from_extras(scene: &Scene3D) -> Option<&[Value]> {
    match scene.extras.get("fbx:takes") {
        Some(Value::Array(v)) => Some(v.as_slice()),
        _ => None,
    }
}

/// Convenience: pull the surfaced active-take name back off a scene's
/// extras. Returns `None` when no `Current` leaf was present.
pub fn current_take_from_extras(scene: &Scene3D) -> Option<&str> {
    match scene.extras.get("fbx:current_take") {
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

    /// Build the §7e fixture `Takes` section as a typed node tree.
    fn fixture_takes() -> FbxNode {
        let take = FbxNode {
            name: "Take".to_owned(),
            properties: vec![s("Take 001")],
            children: vec![
                leaf("FileName", vec![s("Take_001.tak")]),
                leaf(
                    "LocalTime",
                    vec![FbxProperty::I64(1924423250), FbxProperty::I64(230930790000)],
                ),
                leaf(
                    "ReferenceTime",
                    vec![FbxProperty::I64(1924423250), FbxProperty::I64(230930790000)],
                ),
            ],
        };
        FbxNode {
            name: TAKES_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![leaf("Current", vec![s("Take 001")]), take],
        }
    }

    fn doc_with(takes: FbxNode) -> FbxDocument {
        FbxDocument {
            version: 7500,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![takes],
            },
        }
    }

    #[test]
    fn surfaces_fixture_take_catalogue() {
        let doc = doc_with(fixture_takes());
        let mut scene = Scene3D::new();
        let count = extract_takes(&doc, &mut scene);
        assert_eq!(count, 1);

        // Current take.
        assert_eq!(current_take_from_extras(&scene), Some("Take 001"));

        // Take object fields.
        let takes = takes_from_extras(&scene).expect("fbx:takes present");
        assert_eq!(takes.len(), 1);
        let t = &takes[0];
        assert_eq!(t["name"], Value::String("Take 001".to_owned()));
        assert_eq!(t["file_name"], Value::String("Take_001.tak".to_owned()));
        assert_eq!(
            t["local_time"],
            Value::Array(vec![
                Value::Number(1924423250i64.into()),
                Value::Number(230930790000i64.into()),
            ])
        );
        assert_eq!(
            t["reference_time"],
            Value::Array(vec![
                Value::Number(1924423250i64.into()),
                Value::Number(230930790000i64.into()),
            ])
        );
    }

    #[test]
    fn take_name_matches_animation_stack_display_name() {
        // §7e join key: the `Take` name equals the AnimationStack
        // display name (fixture `AnimStack::Take 001`).
        let doc = doc_with(fixture_takes());
        let mut scene = Scene3D::new();
        extract_takes(&doc, &mut scene);
        let takes = takes_from_extras(&scene).unwrap();
        assert_eq!(takes[0]["name"].as_str(), Some("Take 001"));
    }

    #[test]
    fn no_takes_node_surfaces_nothing() {
        let doc = FbxDocument {
            version: 7500,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: Vec::new(),
            },
        };
        let mut scene = Scene3D::new();
        assert_eq!(extract_takes(&doc, &mut scene), 0);
        assert!(scene.extras.is_empty());
        assert!(takes_from_extras(&scene).is_none());
        assert!(current_take_from_extras(&scene).is_none());
    }

    #[test]
    fn current_only_takes_node() {
        // A `Takes` block with only `Current` and no `Take` children
        // surfaces the active-take name but reports zero takes.
        let takes = FbxNode {
            name: TAKES_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![leaf("Current", vec![s("Take 001")])],
        };
        let doc = doc_with(takes);
        let mut scene = Scene3D::new();
        assert_eq!(extract_takes(&doc, &mut scene), 0);
        assert_eq!(current_take_from_extras(&scene), Some("Take 001"));
        assert!(takes_from_extras(&scene).is_none());
    }

    #[test]
    fn missing_optional_leaves_omitted() {
        // A take with only a name (no FileName / time pairs) surfaces a
        // single-key object — optional leaves are omitted, not nulled.
        let takes = FbxNode {
            name: TAKES_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![FbxNode {
                name: "Take".to_owned(),
                properties: vec![s("Bare")],
                children: Vec::new(),
            }],
        };
        let doc = doc_with(takes);
        let mut scene = Scene3D::new();
        assert_eq!(extract_takes(&doc, &mut scene), 1);
        let t = &takes_from_extras(&scene).unwrap()[0];
        assert_eq!(t["name"].as_str(), Some("Bare"));
        let obj = t.as_object().unwrap();
        assert!(!obj.contains_key("file_name"));
        assert!(!obj.contains_key("local_time"));
        assert!(!obj.contains_key("reference_time"));
    }

    #[test]
    fn malformed_time_pair_rejected() {
        // A `LocalTime` with one value (not the §5 two-int pair) is
        // not surfaced — partial data is dropped rather than guessed.
        let takes = FbxNode {
            name: TAKES_NODE.to_owned(),
            properties: Vec::new(),
            children: vec![FbxNode {
                name: "Take".to_owned(),
                properties: vec![s("Odd")],
                children: vec![leaf("LocalTime", vec![FbxProperty::I64(42)])],
            }],
        };
        let doc = doc_with(takes);
        let mut scene = Scene3D::new();
        extract_takes(&doc, &mut scene);
        let t = &takes_from_extras(&scene).unwrap()[0];
        assert!(!t.as_object().unwrap().contains_key("local_time"));
    }
}
