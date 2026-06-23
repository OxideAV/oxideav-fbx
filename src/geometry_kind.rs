//! `Geometry` subtype-discriminator surfacing for the non-`"Mesh"`
//! geometry subtypes documented in
//! `docs/3d/fbx/fbx-binary-properties70.md` §6 point 3.
//!
//! The §6 ruleset lists the `Geometry` prop2 subtype string as the
//! "fine discriminator" within the `Geometry` class:
//!
//! > *"`Geometry … "Mesh"` → a mesh geometry. (Other geometry subtypes
//! > such as `"NurbsCurve"`, `"NurbsSurface"`, `"Shape"`, `"Boundary"`,
//! > `"TrimNurbsSurface"`, `"Line"` would appear here in files that
//! > contain them …)"*
//!
//! The polygonal `"Mesh"` subtype is tessellated into a typed
//! [`oxideav_mesh3d::Mesh`] by [`crate::geometry`] and the
//! `"Shape"` subtype is consumed by the blend-shape path in
//! [`crate::deformer`] (a `Shape` geometry connects to a
//! `BlendShapeChannel`, never to a `Model`). The remaining subtypes —
//! `"NurbsCurve"`, `"NurbsSurface"`, `"Boundary"`, `"TrimNurbsSurface"`,
//! `"Line"` — have no first-class mesh3d tessellation in this crate, so
//! today they are dropped entirely by the scene walker (no `Mesh`, no
//! node tag): a consumer cannot even tell the file carried such a
//! geometry.
//!
//! This module closes that hole the same way round 235 closed the
//! analogous `NodeAttribute` "LimbNode" / "Null" hole: for every
//! non-`Mesh`, non-`Shape` `Geometry` element that has an `OO`
//! connection to a [`Model`], the owning Model's
//! [`oxideav_mesh3d::Node::extras`] gets a `"fbx:geometry_kind"` entry
//! whose value is the §6 prop2 subtype string verbatim
//! (`"NurbsCurve"`, `"NurbsSurface"`, `"Boundary"`,
//! `"TrimNurbsSurface"`, `"Line"`). That is enough for a downstream
//! consumer to know the geometry exists and what kind it is, without
//! re-walking the `FbxDocument`.
//!
//! # What is NOT surfaced
//!
//! - The control-point / knot-vector payloads of the NURBS / Line
//!   geometries — the staged docs enumerate the subtype *names* (§6
//!   point 3) but not the per-subtype `P`-record / sub-record grammar
//!   that would feed a real curve / surface evaluation. Tessellating
//!   them is a follow-up round gated on that grammar being staged.
//! - `"Mesh"` (typed in [`crate::geometry`]) and `"Shape"` (typed in
//!   [`crate::deformer`]) are deliberately excluded so the surfacing
//!   passes never double-claim a geometry that already has a typed
//!   home.
//!
//! # Idempotence with `node_attribute`
//!
//! [`crate::node_attribute`] writes `Node::extras["fbx:node_attribute_kind"]`
//! for the `NodeAttribute` "LimbNode" / "Null" discriminators. This
//! module writes a distinct key (`"fbx:geometry_kind"`) for the
//! `Geometry` subtype discriminator, so the two passes never collide on
//! the same key even when a single Model owns both a NodeAttribute and
//! a (non-mesh) Geometry.

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D};
use serde_json::Value;

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// The non-`Mesh`, non-`Shape` `Geometry` subtype discriminators from
/// `docs/3d/fbx/fbx-binary-properties70.md` §6 point 3. `"Mesh"` is
/// tessellated by [`crate::geometry`]; `"Shape"` is consumed by the
/// blend-shape path in [`crate::deformer`]; both are excluded here.
const SURFACED_GEOMETRY_SUBTYPES: &[&str] = &[
    "NurbsCurve",
    "NurbsSurface",
    "Boundary",
    "TrimNurbsSurface",
    "Line",
];

/// Walk `Objects { Geometry }` and record the non-`Mesh` / non-`Shape`
/// §6 subtype discriminator on the owning `Model`'s scene-graph
/// `Node::extras["fbx:geometry_kind"]`. `model_nodes` is the per-Model
/// FBX-id → `NodeId` lookup already produced by the scene builder;
/// `Geometry` records that connect to a `Model` whose id isn't in this
/// map are silently ignored.
pub fn extract_geometry_kinds(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) {
    // 1) Index every non-`Mesh`/non-`Shape` `Geometry` element by id,
    //    keyed to its §6 subtype string.
    let mut geom_kind: HashMap<i64, &'static str> = HashMap::new();
    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            if child.name != "Geometry" {
                continue;
            }
            let id = match child.properties.first().and_then(FbxProperty::as_i64) {
                Some(i) => i,
                None => continue,
            };
            let st = match subtype_string(child) {
                Some(s) => s,
                None => continue,
            };
            // Match the canonical spelling and stash the &'static so the
            // surfaced value is the docs §6 string, not the raw bytes.
            if let Some(&canon) = SURFACED_GEOMETRY_SUBTYPES
                .iter()
                .find(|&&s| s == st.as_str())
            {
                geom_kind.insert(id, canon);
            }
        }
    }

    if geom_kind.is_empty() {
        return;
    }

    // 2) Walk `Connections` for `Geometry -> Model` `OO` links and tag
    //    the owning `Model`'s `Node::extras` with the kind string.
    //    Mirrors the Geometry -> Model OO walk the scene builder uses
    //    for the `Mesh` attachment (per
    //    `docs/3d/fbx/fbx-binary-properties70.md` §7 Connections).
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
            let Some(&tag) = geom_kind.get(&child_id) else {
                continue;
            };
            let Some(&nid) = model_nodes.get(&parent_id) else {
                continue;
            };
            if let Some(n) = scene.nodes.get_mut(nid.0 as usize) {
                // First-wins keeps the surfacing deterministic when a
                // degenerate file binds two non-mesh geometries to the
                // same Model.
                n.extras
                    .entry("fbx:geometry_kind".to_string())
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

    fn geometry(id: i64, subtype: &str) -> FbxNode {
        FbxNode {
            name: "Geometry".into(),
            properties: vec![
                FbxProperty::I64(id),
                s(b"Geom\x00\x01Geometry"),
                s(subtype.as_bytes()),
            ],
            children: Vec::new(),
        }
    }

    fn model_node(id: i64, name: &str) -> FbxNode {
        let display = format!("{name}\x00\x01Model");
        FbxNode {
            name: "Model".into(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(display.into_bytes()),
                s(b"Null"),
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

    fn make_scene(model_fid: i64) -> (Scene3D, HashMap<i64, NodeId>) {
        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let nid = scene.add_node(oxideav_mesh3d::Node::new());
        model_nodes.insert(model_fid, nid);
        (scene, model_nodes)
    }

    #[test]
    fn nurbs_curve_lands_on_owning_model_extras() {
        // docs/3d/fbx/fbx-binary-properties70.md §6 point 3 lists
        // "NurbsCurve" as a Geometry subtype discriminator. The OO
        // connection wires it to the owning Model whose Node::extras
        // gets tagged.
        let geom = geometry(700, "NurbsCurve");
        let model = model_node(800, "Curve1");
        let doc = build_doc(vec![geom, model], vec![c_oo(700, 800)]);

        let (mut scene, model_nodes) = make_scene(800);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&800];
        assert_eq!(
            scene.nodes[nid.0 as usize].extras.get("fbx:geometry_kind"),
            Some(&Value::String("NurbsCurve".into()))
        );
    }

    #[test]
    fn every_documented_subtype_surfaces() {
        // All five §6 point-3 subtypes other than Mesh (typed) and
        // Shape (blend-shape) round-trip through the kind tag.
        for (i, st) in SURFACED_GEOMETRY_SUBTYPES.iter().enumerate() {
            let gid = 900 + i as i64;
            let mid = 1000 + i as i64;
            let doc = build_doc(
                vec![geometry(gid, st), model_node(mid, "M")],
                vec![c_oo(gid, mid)],
            );
            let (mut scene, model_nodes) = make_scene(mid);
            extract_geometry_kinds(&doc, &mut scene, &model_nodes);
            let nid = model_nodes[&mid];
            assert_eq!(
                scene.nodes[nid.0 as usize].extras.get("fbx:geometry_kind"),
                Some(&Value::String((*st).to_string())),
                "subtype {st} should surface"
            );
        }
    }

    #[test]
    fn mesh_subtype_is_not_tagged() {
        // "Mesh" is tessellated by crate::geometry; this module must
        // NOT claim it (a tagged Mesh would be a double-surface).
        let geom = geometry(701, "Mesh");
        let model = model_node(801, "M");
        let doc = build_doc(vec![geom, model], vec![c_oo(701, 801)]);

        let (mut scene, model_nodes) = make_scene(801);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&801];
        assert!(!scene.nodes[nid.0 as usize]
            .extras
            .contains_key("fbx:geometry_kind"));
    }

    #[test]
    fn shape_subtype_is_not_tagged() {
        // "Shape" is consumed by the blend-shape path in
        // crate::deformer (it connects to a BlendShapeChannel, never a
        // Model); this module must leave it alone.
        let geom = geometry(702, "Shape");
        let model = model_node(802, "M");
        let doc = build_doc(vec![geom, model], vec![c_oo(702, 802)]);

        let (mut scene, model_nodes) = make_scene(802);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&802];
        assert!(!scene.nodes[nid.0 as usize]
            .extras
            .contains_key("fbx:geometry_kind"));
    }

    #[test]
    fn geometry_without_owning_model_is_skipped() {
        // §6 + §7 require a Geometry -> Model OO connection. Orphan
        // geometries (no OO connection, or pointing at an unsurfaced
        // Model) leave the scene untouched.
        let geom = geometry(703, "Line");
        let doc = build_doc(vec![geom], vec![]);

        let mut scene = Scene3D::new();
        let model_nodes: HashMap<i64, NodeId> = HashMap::new();
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);
        assert!(scene.nodes.is_empty());
    }

    #[test]
    fn non_oo_connections_are_ignored() {
        // OP / PP / PO connections aren't attribute attachments per §6
        // point 3 + §7; they MUST NOT trigger a kind tag.
        let geom = geometry(704, "NurbsSurface");
        let model = model_node(804, "M");
        let mut c = c_oo(704, 804);
        c.properties[0] = s(b"OP");
        let doc = build_doc(vec![geom, model], vec![c]);

        let (mut scene, model_nodes) = make_scene(804);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&804];
        assert!(!scene.nodes[nid.0 as usize]
            .extras
            .contains_key("fbx:geometry_kind"));
    }

    #[test]
    fn does_not_collide_with_node_attribute_kind_key() {
        // node_attribute writes "fbx:node_attribute_kind"; this module
        // writes the distinct "fbx:geometry_kind". The two
        // round-trippable shapes coexist on the same node.
        let geom = geometry(705, "Boundary");
        let model = model_node(805, "M");
        let doc = build_doc(vec![geom, model], vec![c_oo(705, 805)]);

        let mut scene = Scene3D::new();
        let mut model_nodes = HashMap::new();
        let mut prefilled = oxideav_mesh3d::Node::new();
        prefilled.extras.insert(
            "fbx:node_attribute_kind".into(),
            Value::String("LimbNode".into()),
        );
        let nid = scene.add_node(prefilled);
        model_nodes.insert(805, nid);

        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let node = &scene.nodes[nid.0 as usize];
        assert_eq!(
            node.extras.get("fbx:node_attribute_kind"),
            Some(&Value::String("LimbNode".into()))
        );
        assert_eq!(
            node.extras.get("fbx:geometry_kind"),
            Some(&Value::String("Boundary".into()))
        );
    }

    #[test]
    fn unknown_subtype_does_not_emit_kind() {
        // A subtype outside the §6 enumeration (and not Mesh/Shape)
        // surfaces no kind tag — we only claim the documented names.
        let geom = geometry(706, "SomeFutureKind");
        let model = model_node(806, "M");
        let doc = build_doc(vec![geom, model], vec![c_oo(706, 806)]);

        let (mut scene, model_nodes) = make_scene(806);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&806];
        assert!(!scene.nodes[nid.0 as usize]
            .extras
            .contains_key("fbx:geometry_kind"));
    }

    #[test]
    fn first_kind_wins_on_repeated_oo_to_same_model() {
        // Degenerate file: two non-mesh geometries bound to the same
        // Model. First-seen wins for deterministic surfacing.
        let g1 = geometry(720, "NurbsCurve");
        let g2 = geometry(721, "Line");
        let model = model_node(820, "M");
        let doc = build_doc(vec![g1, g2, model], vec![c_oo(720, 820), c_oo(721, 820)]);

        let (mut scene, model_nodes) = make_scene(820);
        extract_geometry_kinds(&doc, &mut scene, &model_nodes);

        let nid = model_nodes[&820];
        let val = scene.nodes[nid.0 as usize]
            .extras
            .get("fbx:geometry_kind")
            .expect("kind tag");
        assert!(matches!(val.as_str(), Some("NurbsCurve") | Some("Line")));
    }
}
