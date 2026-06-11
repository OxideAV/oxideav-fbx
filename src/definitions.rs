//! `Definitions` section decoder — per-class instance counts +
//! `PropertyTemplate` default `Properties70` blocks (round 280).
//!
//! Per the observer grammar at `docs/3d/fbx/fbx-ascii-grammar.md` §7b,
//! the top-level `Definitions` section has the shape:
//!
//! ```text
//! Definitions:  {
//!         Version: 100
//!         Count: 13
//!         ObjectType: "Geometry" {
//!                 Count: 4
//!                 PropertyTemplate: "FbxMesh" {
//!                         Properties70:  { P: ... default property set ... }
//!                 }
//!         }
//!         ObjectType: "Material" { Count: 2  PropertyTemplate: "FbxSurfaceLambert" {...} }
//!         ...
//! }
//! ```
//!
//! and the docs state: *"`Count` at the top is the total object count;
//! each `ObjectType:` block names a class, its instance `Count`, and a
//! `PropertyTemplate` holding the default `Properties70` for that
//! class."* Some classes carry no template at all (the cubes fixture's
//! `ObjectType: "GlobalSettings" { Count: 1 }` block has only the
//! count).
//!
//! The binary encoding renders the identical node tree — per
//! `docs/3d/fbx/fbx-binary-properties70.md` §4, *"ASCII and binary are
//! two renderings of the identical node tree"* — so this single
//! [`crate::binary::FbxNode`]-walker covers both front-ends.
//!
//! The decoded [`Definitions`] gives consumers two things:
//!
//! 1. The per-class **default property set** ([`Definitions::template_for`])
//!    that an object's own `Properties70` block overrides. Combined
//!    with [`crate::properties70::PropertyMap::with_template_defaults`]
//!    this resolves the *effective* property value for an object whose
//!    exporter only wrote the non-default subset (the usual case — the
//!    cubes fixture's `Material` instances re-state only 8 of the 17
//!    template records).
//! 2. The declared **instance counts** (total + per class), useful for
//!    pre-sizing arenas and consistency checks.

use std::collections::HashMap;

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::properties70::PropertyMap;

/// One `ObjectType` block from the `Definitions` section.
///
/// Mirrors the docs §7b fields: the class name (the block's single
/// string property), its instance `Count` leaf, and the optional
/// `PropertyTemplate` (template name + decoded default
/// `Properties70`).
#[derive(Clone, Debug)]
pub struct ObjectTypeDefinition {
    /// Class name string — `"Geometry"`, `"Material"`, `"Model"`, …
    /// (the `ObjectType: "<class>"` value per docs §7b).
    pub object_type: String,
    /// Declared instance count for this class (the inner `Count`
    /// leaf). `None` when the block omits the leaf.
    pub count: Option<i64>,
    /// The `PropertyTemplate: "<name>"` string — e.g. `"FbxMesh"`,
    /// `"FbxSurfaceLambert"`, `"FbxNode"`. `None` when the class has
    /// no template block (docs §7b shows `GlobalSettings` carrying
    /// only a `Count`).
    pub template_name: Option<String>,
    /// The template's decoded default `Properties70` block — the
    /// *"default property set"* of docs §7b. `None` when there is no
    /// `PropertyTemplate` child at all; an empty map when the
    /// template exists but holds no `P` records.
    pub template: Option<PropertyMap>,
}

/// Decoded top-level `Definitions` section.
///
/// Build with [`Definitions::from_document`] (or
/// [`Definitions::from_root`] when the caller already holds the root
/// node). A document without a `Definitions` section decodes to the
/// empty value — every lookup returns `None`.
#[derive(Clone, Debug, Default)]
pub struct Definitions {
    /// The section's `Version` leaf (`100` in the docs §7b sample).
    pub version: Option<i64>,
    /// The top-level `Count` leaf — *"the total object count"* per
    /// docs §7b (distinct from each class's own inner `Count`).
    pub total_count: Option<i64>,
    types: HashMap<String, ObjectTypeDefinition>,
}

impl Definitions {
    /// Decode the `Definitions` section of a parsed document.
    pub fn from_document(doc: &FbxDocument) -> Self {
        Self::from_root(&doc.root)
    }

    /// Decode from the synthetic root node (whose children are the
    /// top-level §7 sections). Returns the empty value when no
    /// `Definitions` child exists.
    pub fn from_root(root: &FbxNode) -> Self {
        let Some(defs) = root.child("Definitions") else {
            return Self::default();
        };
        let mut out = Self {
            version: leaf_i64(defs, "Version"),
            total_count: leaf_i64(defs, "Count"),
            types: HashMap::new(),
        };
        for ot in defs.children_named("ObjectType") {
            let Some(class) = ot.properties.first().and_then(FbxProperty::as_str) else {
                // An ObjectType block without its class-name string
                // property doesn't fit the §7b shape — skip it.
                continue;
            };
            let tpl_node = ot.child("PropertyTemplate");
            let def = ObjectTypeDefinition {
                object_type: class.to_owned(),
                count: leaf_i64(ot, "Count"),
                template_name: tpl_node
                    .and_then(|t| t.properties.first())
                    .and_then(FbxProperty::as_str)
                    .map(str::to_owned),
                template: tpl_node.map(PropertyMap::from_element),
            };
            // Last-wins on a repeated class name — the same
            // override shape `PropertyMap` documents for repeated
            // `P` record names.
            out.types.insert(class.to_owned(), def);
        }
        out
    }

    /// Look up one class's `ObjectType` block by class name.
    pub fn get(&self, object_type: &str) -> Option<&ObjectTypeDefinition> {
        self.types.get(object_type)
    }

    /// The default `Properties70` set for a class — the docs §7b
    /// *"`PropertyTemplate` holding the default `Properties70` for
    /// that class"*. `None` when the class is absent or carries no
    /// template block. Feed the result to
    /// [`PropertyMap::with_template_defaults`] to resolve an object's
    /// effective properties.
    pub fn template_for(&self, object_type: &str) -> Option<&PropertyMap> {
        self.types.get(object_type)?.template.as_ref()
    }

    /// Every declared class name, sorted for deterministic iteration.
    pub fn object_types(&self) -> Vec<&str> {
        let mut names: Vec<&str> = self.types.keys().map(String::as_str).collect();
        names.sort_unstable();
        names
    }

    /// Number of `ObjectType` blocks decoded.
    pub fn len(&self) -> usize {
        self.types.len()
    }

    /// True when the document had no (non-empty) `Definitions`
    /// section.
    pub fn is_empty(&self) -> bool {
        self.types.is_empty()
    }
}

/// Read a direct-child leaf's first numeric property (the §7b
/// `Version:` / `Count:` integer leaves).
fn leaf_i64(parent: &FbxNode, name: &str) -> Option<i64> {
    parent
        .child(name)?
        .properties
        .first()
        .and_then(FbxProperty::as_i64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(b: &str) -> FbxProperty {
        FbxProperty::String(b.as_bytes().to_vec())
    }

    fn leaf_i32(name: &str, v: i32) -> FbxNode {
        FbxNode {
            name: name.into(),
            properties: vec![FbxProperty::I32(v)],
            children: Vec::new(),
        }
    }

    fn p_record(name: &str, ty: &str, label: &str, flags: &str, vals: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![s(name), s(ty), s(label), s(flags)];
        props.extend(vals);
        FbxNode {
            name: "P".into(),
            properties: props,
            children: Vec::new(),
        }
    }

    fn property_template(name: &str, records: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: "PropertyTemplate".into(),
            properties: vec![s(name)],
            children: vec![FbxNode {
                name: "Properties70".into(),
                properties: Vec::new(),
                children: records,
            }],
        }
    }

    fn object_type(class: &str, count: i32, template: Option<FbxNode>) -> FbxNode {
        let mut children = vec![leaf_i32("Count", count)];
        children.extend(template);
        FbxNode {
            name: "ObjectType".into(),
            properties: vec![s(class)],
            children,
        }
    }

    fn root_with_definitions(children: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![FbxNode {
                name: "Definitions".into(),
                properties: Vec::new(),
                children,
            }],
        }
    }

    /// Mirrors the docs §7b sample: Version/Count leaves, a class with
    /// a template, and a class without one.
    fn sample_root() -> FbxNode {
        root_with_definitions(vec![
            leaf_i32("Version", 100),
            leaf_i32("Count", 13),
            object_type("GlobalSettings", 1, None),
            object_type(
                "Material",
                2,
                Some(property_template(
                    "FbxSurfaceLambert",
                    vec![
                        p_record("ShadingModel", "KString", "", "", vec![s("Lambert")]),
                        p_record(
                            "DiffuseColor",
                            "Color",
                            "",
                            "A",
                            vec![
                                FbxProperty::F64(0.8),
                                FbxProperty::F64(0.8),
                                FbxProperty::F64(0.8),
                            ],
                        ),
                        p_record(
                            "DiffuseFactor",
                            "Number",
                            "",
                            "A",
                            vec![FbxProperty::F64(1.0)],
                        ),
                    ],
                )),
            ),
        ])
    }

    #[test]
    fn decodes_version_and_total_count() {
        let defs = Definitions::from_root(&sample_root());
        assert_eq!(defs.version, Some(100));
        assert_eq!(defs.total_count, Some(13));
        assert_eq!(defs.len(), 2);
        assert!(!defs.is_empty());
    }

    #[test]
    fn class_with_template_decodes_name_and_records() {
        let defs = Definitions::from_root(&sample_root());
        let mat = defs.get("Material").expect("Material class decoded");
        assert_eq!(mat.object_type, "Material");
        assert_eq!(mat.count, Some(2));
        assert_eq!(mat.template_name.as_deref(), Some("FbxSurfaceLambert"));
        let tpl = defs.template_for("Material").expect("Material template");
        assert_eq!(tpl.len(), 3);
        assert_eq!(tpl.as_vec3("DiffuseColor"), Some([0.8, 0.8, 0.8]));
        assert_eq!(tpl.as_f64("DiffuseFactor"), Some(1.0));
        assert_eq!(tpl.as_str("ShadingModel"), Some("Lambert"));
    }

    #[test]
    fn class_without_template_has_count_only() {
        // Docs §7b / cubes fixture: `ObjectType: "GlobalSettings" {
        // Count: 1 }` carries no PropertyTemplate.
        let defs = Definitions::from_root(&sample_root());
        let gs = defs.get("GlobalSettings").expect("GlobalSettings class");
        assert_eq!(gs.count, Some(1));
        assert_eq!(gs.template_name, None);
        assert!(gs.template.is_none());
        assert!(defs.template_for("GlobalSettings").is_none());
    }

    #[test]
    fn missing_definitions_section_is_empty() {
        let root = FbxNode::default();
        let defs = Definitions::from_root(&root);
        assert!(defs.is_empty());
        assert_eq!(defs.version, None);
        assert_eq!(defs.total_count, None);
        assert!(defs.get("Material").is_none());
        assert!(defs.template_for("Material").is_none());
    }

    #[test]
    fn unknown_class_lookup_returns_none() {
        let defs = Definitions::from_root(&sample_root());
        assert!(defs.get("Texture").is_none());
        assert!(defs.template_for("Texture").is_none());
    }

    #[test]
    fn object_type_without_class_name_is_skipped() {
        let root = root_with_definitions(vec![FbxNode {
            name: "ObjectType".into(),
            properties: Vec::new(), // no class-name string — not the §7b shape
            children: vec![leaf_i32("Count", 1)],
        }]);
        let defs = Definitions::from_root(&root);
        assert!(defs.is_empty());
    }

    #[test]
    fn repeated_class_name_is_last_wins() {
        let root = root_with_definitions(vec![
            object_type("Material", 1, None),
            object_type(
                "Material",
                2,
                Some(property_template("FbxSurfaceLambert", Vec::new())),
            ),
        ]);
        let defs = Definitions::from_root(&root);
        assert_eq!(defs.len(), 1);
        let mat = defs.get("Material").unwrap();
        assert_eq!(mat.count, Some(2));
        assert_eq!(mat.template_name.as_deref(), Some("FbxSurfaceLambert"));
    }

    #[test]
    fn object_types_iteration_is_sorted() {
        let defs = Definitions::from_root(&sample_root());
        assert_eq!(defs.object_types(), vec!["GlobalSettings", "Material"]);
    }

    #[test]
    fn empty_template_block_is_some_empty_map() {
        // A PropertyTemplate with no P records still *exists* — the
        // class declares "no defaults" rather than "no template".
        let root = root_with_definitions(vec![object_type(
            "Texture",
            1,
            Some(property_template("FbxFileTexture", Vec::new())),
        )]);
        let defs = Definitions::from_root(&root);
        let tex = defs.get("Texture").unwrap();
        assert_eq!(tex.template_name.as_deref(), Some("FbxFileTexture"));
        let tpl = tex.template.as_ref().expect("template map exists");
        assert!(tpl.is_empty());
    }
}
