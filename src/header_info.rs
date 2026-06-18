//! `FBXHeaderExtension` decoder — file-level authoring metadata
//! surfaced onto [`oxideav_mesh3d::Scene3D`].
//!
//! Per `docs/3d/fbx/fbx-ascii-grammar.md` §7a (and its binary
//! counterpart, `fbx-binary-properties70.md` §5 — the `SceneInfo`
//! object header uses the same `\x00\x01`-delimited name+classtag
//! triple, and §4 for the inner `Properties70` block), the first
//! top-level section is the `FBXHeaderExtension` node, which carries
//! the file's authoring provenance:
//!
//! ```text
//! FBXHeaderExtension:  {
//!   FBXHeaderVersion: 1003
//!   FBXVersion: 7500
//!   CreationTimeStamp:  {
//!     Version: 1000
//!     Year: 2019   Month: 1   Day: 7
//!     Hour: 16     Minute: 17 Second: 31   Millisecond: 730
//!   }
//!   Creator: "FBX SDK/FBX Plugins version 2018.1.1"
//!   SceneInfo: "SceneInfo::GlobalInfo", "UserData" {
//!     Type: "UserData"
//!     Version: 100
//!     MetaData:  {
//!       Version: 100
//!       Title: ""  Subject: ""  Author: ""
//!       Keywords: ""  Revision: ""  Comment: ""
//!     }
//!     Properties70:  {
//!       P: "DocumentUrl", "KString", "Url", "", "U:\...\cubes_with_names.fbx"
//!       P: "Original|ApplicationVendor", "KString", "", "", "Autodesk"
//!       P: "Original|ApplicationName", "KString", "", "", "Maya"
//!       P: "Original|ApplicationVersion", "KString", "", "", "201800"
//!       P: "Original|DateTime_GMT", "DateTime", "", "", "07/01/2019 16:17:31.730"
//!       P: "LastSaved|ApplicationName", "KString", "", "", "Maya"
//!       ...
//!     }
//!   }
//! }
//! ```
//!
//! Field grammar, read verbatim from §7a:
//!
//! - `FBXHeaderVersion` / `FBXVersion` — integer leaves. The latter
//!   echoes the container version byte (`fbx-binary-properties70.md`
//!   §1) the parser already exposes via [`FbxDocument::version`]; we
//!   surface `FBXHeaderVersion` since it is not otherwise reachable.
//! - `Creator` — string leaf naming the writing tool.
//! - `CreationTimeStamp` — a node-with-body holding `Year` / `Month` /
//!   `Day` / `Hour` / `Minute` / `Second` / `Millisecond` integer
//!   leaves (§7a). Composed into an ISO-8601-ish
//!   `YYYY-MM-DDThh:mm:ss.mmm` string so a consumer needn't re-walk the
//!   sub-node.
//! - `SceneInfo` — a §7c-shaped object node (`"SceneInfo::<Name>",
//!   "<SubType>"`) whose body holds the document `MetaData` block and a
//!   `Properties70` of `Original|*` / `LastSaved|*` application
//!   provenance (the `|`-compound-path names of
//!   `fbx-ascii-grammar.md` §8 field 1).
//!
//! # Surfacing
//!
//! `oxideav_mesh3d::Scene3D` carries no typed authoring-metadata
//! fields, so the header is surfaced on
//! [`oxideav_mesh3d::Scene3D::extras`], mirroring the
//! [`crate::globals`] / [`crate::takes`] `"fbx:<snake_case>"`
//! convention:
//!
//! - `extras["fbx:creator"]` — the `Creator` string.
//! - `extras["fbx:header_version"]` — the `FBXHeaderVersion` int.
//! - `extras["fbx:creation_time"]` — the composed timestamp string
//!   (omitted when no `CreationTimeStamp` sub-node is present).
//! - `extras["fbx:meta_<field>"]` — each non-empty `MetaData` field
//!   (`title` / `subject` / `author` / `keywords` / `revision` /
//!   `comment`). Empty strings (the SDK writes `""` for unset fields)
//!   are skipped so the map only carries fields the author actually
//!   filled in.
//! - `extras["fbx:application_name"]` / `["fbx:application_vendor"]` /
//!   `["fbx:application_version"]` — pulled from the `Original|*`
//!   `Properties70` provenance (the file's *creating* application).
//! - `extras["fbx:document_url"]` — the `DocumentUrl` provenance path.
//!
//! Both the binary and ASCII front-ends render the identical node tree
//! (`fbx-binary-properties70.md` §4 isomorphism note), so this one
//! walker covers both encodings.

use oxideav_mesh3d::Scene3D;
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode};
use crate::properties70::PropertyMap;

/// FBX top-level node name for the header-extension section. The first
/// of the §7-ordered sections, sibling of `GlobalSettings` /
/// `Objects` / `Connections` (per
/// `docs/3d/fbx/fbx-ascii-grammar.md` §7 top-level section list).
pub const HEADER_EXTENSION_NODE: &str = "FBXHeaderExtension";

/// Decode the `FBXHeaderExtension` section of `doc` and surface its
/// authoring metadata onto `scene.extras`.
///
/// Returns the number of distinct metadata entries inserted (zero when
/// the document has no `FBXHeaderExtension` node or it holds nothing we
/// recognise). Existing `extras` keys are preserved
/// (`Entry::or_insert`) so a caller may pre-seed values.
pub fn extract_header_info(doc: &FbxDocument, scene: &mut Scene3D) -> usize {
    let Some(ext) = doc.root.child(HEADER_EXTENSION_NODE) else {
        return 0;
    };
    let mut inserted = 0usize;

    // `FBXHeaderVersion` integer leaf.
    if let Some(v) = ext
        .child("FBXHeaderVersion")
        .and_then(|n| n.properties.first())
        .and_then(|p| p.as_i64())
    {
        inserted += insert(scene, "fbx:header_version", Value::Number(v.into()));
    }

    // `Creator` string leaf.
    if let Some(creator) = ext
        .child("Creator")
        .and_then(|n| n.properties.first())
        .and_then(|p| p.as_str())
    {
        inserted += insert(scene, "fbx:creator", Value::String(creator.to_owned()));
    }

    // `CreationTimeStamp` sub-node → ISO-ish composed string.
    if let Some(ts) = ext.child("CreationTimeStamp") {
        if let Some(stamp) = creation_timestamp(ts) {
            inserted += insert(scene, "fbx:creation_time", Value::String(stamp));
        }
    }

    // `SceneInfo` body: document `MetaData` + `Original|*` provenance.
    if let Some(scene_info) = ext.child("SceneInfo") {
        inserted += extract_metadata(scene_info, scene);
        inserted += extract_application_provenance(scene_info, scene);
    }

    inserted
}

/// Compose `CreationTimeStamp` integer sub-leaves into an
/// `YYYY-MM-DDThh:mm:ss.mmm` string. Returns `None` when the node
/// carries none of the date/time leaves (a bare `Version`-only stamp).
///
/// Per §7a the sub-leaves are integers; absent components default to
/// `0`. The composed form is purely a re-rendering of the observed
/// integers (no calendar arithmetic / timezone interpretation — the
/// SDK writes the stamp in its local clock with no zone marker).
fn creation_timestamp(node: &FbxNode) -> Option<String> {
    let any = ["Year", "Month", "Day", "Hour", "Minute", "Second"]
        .iter()
        .any(|k| node.child(k).is_some());
    if !any {
        return None;
    }
    let g = |k: &str| -> i64 {
        node.child(k)
            .and_then(|n| n.properties.first())
            .and_then(|p| p.as_i64())
            .unwrap_or(0)
    };
    Some(format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}.{:03}",
        g("Year"),
        g("Month"),
        g("Day"),
        g("Hour"),
        g("Minute"),
        g("Second"),
        g("Millisecond"),
    ))
}

/// Surface the non-empty fields of the `SceneInfo` `MetaData` block.
///
/// Per §7a the block holds `Title` / `Subject` / `Author` /
/// `Keywords` / `Revision` / `Comment` string leaves (the SDK writes
/// every one, defaulting unset fields to `""`). Empty strings are
/// skipped so the surfaced map only carries author-supplied values.
fn extract_metadata(scene_info: &FbxNode, scene: &mut Scene3D) -> usize {
    let Some(meta) = scene_info.child("MetaData") else {
        return 0;
    };
    let mut inserted = 0usize;
    for field in [
        "Title", "Subject", "Author", "Keywords", "Revision", "Comment",
    ] {
        if let Some(val) = meta
            .child(field)
            .and_then(|n| n.properties.first())
            .and_then(|p| p.as_str())
        {
            if !val.is_empty() {
                let key = format!("fbx:meta_{}", field.to_ascii_lowercase());
                inserted += insert(scene, &key, Value::String(val.to_owned()));
            }
        }
    }
    inserted
}

/// Surface the *creating* application provenance from the `SceneInfo`
/// `Properties70` block.
///
/// Per §7a / §8 the `Original|*` compound-path `P` records name the
/// application that first authored the file:
/// `Original|ApplicationName` / `|ApplicationVendor` /
/// `|ApplicationVersion`, plus the `DocumentUrl` path. (The parallel
/// `LastSaved|*` set records the *last writing* tool; we surface the
/// `Original|*` set as the primary provenance and leave `LastSaved|*`
/// reachable on the raw [`FbxDocument`] for callers that want it.)
fn extract_application_provenance(scene_info: &FbxNode, scene: &mut Scene3D) -> usize {
    let Some(props70) = scene_info.child("Properties70") else {
        return 0;
    };
    let props = PropertyMap::from_properties70(props70);
    let mut inserted = 0usize;
    for (p_name, key) in [
        ("Original|ApplicationName", "fbx:application_name"),
        ("Original|ApplicationVendor", "fbx:application_vendor"),
        ("Original|ApplicationVersion", "fbx:application_version"),
        ("DocumentUrl", "fbx:document_url"),
    ] {
        if let Some(val) = props.as_str(p_name) {
            if !val.is_empty() {
                inserted += insert(scene, key, Value::String(val.to_owned()));
            }
        }
    }
    inserted
}

/// Insert into `scene.extras` only if the key is unset, returning `1`
/// when an insert happened and `0` when an existing entry was kept.
fn insert(scene: &mut Scene3D, key: &str, value: Value) -> usize {
    use std::collections::hash_map::Entry;
    match scene.extras.entry(key.to_owned()) {
        Entry::Vacant(e) => {
            e.insert(value);
            1
        }
        Entry::Occupied(_) => 0,
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
            name: name.to_string(),
            properties: props,
            children: vec![],
        }
    }

    fn int_leaf(name: &str, v: i64) -> FbxNode {
        leaf(name, vec![FbxProperty::I64(v)])
    }

    /// Build a `P` record with `name`, `typeName`, empty label+flags,
    /// and a single string value — the `Original|*` provenance shape
    /// (`fbx-binary-properties70.md` §4).
    fn p_str(name: &str, type_name: &str, value: &str) -> FbxNode {
        leaf("P", vec![s(name), s(type_name), s(""), s(""), s(value)])
    }

    fn doc_with_header(ext: FbxNode) -> FbxDocument {
        FbxDocument {
            version: 7500,
            root: FbxNode {
                name: "__root__".to_string(),
                properties: vec![],
                children: vec![ext],
            },
        }
    }

    /// Reconstruct the fixture's `FBXHeaderExtension` shape
    /// (`tests/fixtures/cubes-ascii-v7500.fbx`, grammar §7a) as an
    /// in-memory node tree.
    fn fixture_header() -> FbxNode {
        let timestamp = FbxNode {
            name: "CreationTimeStamp".to_string(),
            properties: vec![],
            children: vec![
                int_leaf("Version", 1000),
                int_leaf("Year", 2019),
                int_leaf("Month", 1),
                int_leaf("Day", 7),
                int_leaf("Hour", 16),
                int_leaf("Minute", 17),
                int_leaf("Second", 31),
                int_leaf("Millisecond", 730),
            ],
        };
        let metadata = FbxNode {
            name: "MetaData".to_string(),
            properties: vec![],
            children: vec![
                int_leaf("Version", 100),
                leaf("Title", vec![s("")]),
                leaf("Subject", vec![s("My Subject")]),
                leaf("Author", vec![s("Mark")]),
                leaf("Keywords", vec![s("")]),
                leaf("Revision", vec![s("")]),
                leaf("Comment", vec![s("")]),
            ],
        };
        let props70 = FbxNode {
            name: "Properties70".to_string(),
            properties: vec![],
            children: vec![
                p_str("DocumentUrl", "KString", "U:\\Some\\Path\\cubes.fbx"),
                p_str("Original|ApplicationVendor", "KString", "Autodesk"),
                p_str("Original|ApplicationName", "KString", "Maya"),
                p_str("Original|ApplicationVersion", "KString", "201800"),
                p_str(
                    "Original|DateTime_GMT",
                    "DateTime",
                    "07/01/2019 16:17:31.730",
                ),
                p_str("LastSaved|ApplicationName", "KString", "MotionBuilder"),
            ],
        };
        let scene_info = FbxNode {
            name: "SceneInfo".to_string(),
            properties: vec![s("SceneInfo::GlobalInfo"), s("UserData")],
            children: vec![
                leaf("Type", vec![s("UserData")]),
                int_leaf("Version", 100),
                metadata,
                props70,
            ],
        };
        FbxNode {
            name: HEADER_EXTENSION_NODE.to_string(),
            properties: vec![],
            children: vec![
                int_leaf("FBXHeaderVersion", 1003),
                int_leaf("FBXVersion", 7500),
                timestamp,
                leaf("Creator", vec![s("FBX SDK/FBX Plugins version 2018.1.1")]),
                scene_info,
            ],
        }
    }

    fn str_extra<'a>(scene: &'a Scene3D, key: &str) -> Option<&'a str> {
        match scene.extras.get(key) {
            Some(Value::String(s)) => Some(s.as_str()),
            _ => None,
        }
    }

    #[test]
    fn surfaces_creator_and_header_version() {
        let mut scene = Scene3D::new();
        let n = extract_header_info(&doc_with_header(fixture_header()), &mut scene);
        assert!(n > 0);
        assert_eq!(
            str_extra(&scene, "fbx:creator"),
            Some("FBX SDK/FBX Plugins version 2018.1.1")
        );
        assert_eq!(
            scene.extras.get("fbx:header_version"),
            Some(&Value::Number(1003.into()))
        );
    }

    #[test]
    fn composes_creation_timestamp() {
        let mut scene = Scene3D::new();
        extract_header_info(&doc_with_header(fixture_header()), &mut scene);
        assert_eq!(
            str_extra(&scene, "fbx:creation_time"),
            Some("2019-01-07T16:17:31.730")
        );
    }

    #[test]
    fn surfaces_only_nonempty_metadata() {
        let mut scene = Scene3D::new();
        extract_header_info(&doc_with_header(fixture_header()), &mut scene);
        // Filled-in fields surface.
        assert_eq!(str_extra(&scene, "fbx:meta_author"), Some("Mark"));
        assert_eq!(str_extra(&scene, "fbx:meta_subject"), Some("My Subject"));
        // Empty fields (Title/Keywords/Revision/Comment) are skipped.
        assert!(!scene.extras.contains_key("fbx:meta_title"));
        assert!(!scene.extras.contains_key("fbx:meta_keywords"));
        assert!(!scene.extras.contains_key("fbx:meta_comment"));
    }

    #[test]
    fn surfaces_original_application_provenance() {
        let mut scene = Scene3D::new();
        extract_header_info(&doc_with_header(fixture_header()), &mut scene);
        assert_eq!(str_extra(&scene, "fbx:application_name"), Some("Maya"));
        assert_eq!(
            str_extra(&scene, "fbx:application_vendor"),
            Some("Autodesk")
        );
        assert_eq!(str_extra(&scene, "fbx:application_version"), Some("201800"));
        assert_eq!(
            str_extra(&scene, "fbx:document_url"),
            Some("U:\\Some\\Path\\cubes.fbx")
        );
    }

    #[test]
    fn missing_header_returns_zero() {
        let doc = FbxDocument {
            version: 7400,
            root: FbxNode {
                name: "__root__".to_string(),
                properties: vec![],
                children: vec![],
            },
        };
        let mut scene = Scene3D::new();
        assert_eq!(extract_header_info(&doc, &mut scene), 0);
        assert!(scene.extras.is_empty());
    }

    #[test]
    fn existing_extras_are_preserved() {
        let mut scene = Scene3D::new();
        scene
            .extras
            .insert("fbx:creator".to_owned(), Value::String("pre".to_owned()));
        extract_header_info(&doc_with_header(fixture_header()), &mut scene);
        // The pre-seeded value wins (or_insert semantics).
        assert_eq!(str_extra(&scene, "fbx:creator"), Some("pre"));
    }

    #[test]
    fn timestamp_absent_when_no_date_leaves() {
        // A `CreationTimeStamp` with only `Version` → no composed string.
        let ts = FbxNode {
            name: "CreationTimeStamp".to_string(),
            properties: vec![],
            children: vec![int_leaf("Version", 1000)],
        };
        let ext = FbxNode {
            name: HEADER_EXTENSION_NODE.to_string(),
            properties: vec![],
            children: vec![ts],
        };
        let mut scene = Scene3D::new();
        extract_header_info(&doc_with_header(ext), &mut scene);
        assert!(!scene.extras.contains_key("fbx:creation_time"));
    }
}
