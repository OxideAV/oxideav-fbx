//! `Constraint` grammar round-trip (round 439) — the
//! `docs/3d/fbx/fbx-constraint-grammar.md` shapes end-to-end:
//! decode → `Scene3D::extras["fbx:constraints"]` /
//! `["fbx:constraint_templates"]` → encode (binary **and** ASCII)
//! → decode-parity.
//!
//! The synthetic document mirrors the doc's worked example: an
//! `ObjectType: "Constraint"` Definitions block carrying the
//! `FbxConstraintSingleChainIK` per-kind template (doc §1), a
//! constraint object with the display-string sub-type written twice
//! (header + inner `Type:` leaf, doc §2), space-bearing property
//! names and the `"Weight"` property-type string (doc §3), and
//! targets wired exclusively through `OP` records into
//! `"object"`-typed slots (doc §3's load-bearing structural fact).

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_ascii_document, write_document, FbxDecoder,
};
use oxideav_mesh3d::Mesh3DDecoder;

fn s(b: &[u8]) -> FbxProperty {
    FbxProperty::String(b.to_vec())
}

fn element(kind: &str, id: i64, name: &str, class: &str, subtype: &str) -> FbxNode {
    let display = format!("{name}\x00\x01{class}");
    FbxNode {
        name: kind.into(),
        properties: vec![
            FbxProperty::I64(id),
            s(display.as_bytes()),
            s(subtype.as_bytes()),
        ],
        children: Vec::new(),
    }
}

fn leaf(name: &str, prop: FbxProperty) -> FbxNode {
    FbxNode {
        name: name.into(),
        properties: vec![prop],
        children: Vec::new(),
    }
}

fn p(name: &str, ty: &str, label: &str, flags: &str, values: Vec<FbxProperty>) -> FbxNode {
    let mut properties = vec![
        s(name.as_bytes()),
        s(ty.as_bytes()),
        s(label.as_bytes()),
        s(flags.as_bytes()),
    ];
    properties.extend(values);
    FbxNode {
        name: "P".into(),
        properties,
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

fn c_op(child: i64, parent: i64, prop: &str) -> FbxNode {
    FbxNode {
        name: "C".into(),
        properties: vec![
            s(b"OP"),
            FbxProperty::I64(child),
            FbxProperty::I64(parent),
            s(prop.as_bytes()),
        ],
        children: Vec::new(),
    }
}

/// The doc §3 `FbxConstraintSingleChainIK` template body (subset —
/// the shapes that exercise every value form: bool, the `"Weight"`
/// property-type string, value-less `object` slots, enum, `Vector`
/// triple with the `A` flag).
fn ik_template() -> FbxNode {
    FbxNode {
        name: "PropertyTemplate".into(),
        properties: vec![s(b"FbxConstraintSingleChainIK")],
        children: vec![FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: vec![
                p("Active", "bool", "", "", vec![FbxProperty::I32(1)]),
                p("Lock", "bool", "", "", vec![FbxProperty::I32(0)]),
                p("Weight", "Weight", "", "A", vec![FbxProperty::F64(100.0)]),
                p("First Joint", "object", "", "", vec![]),
                p("End Joint", "object", "", "", vec![]),
                p("Effector", "object", "", "", vec![]),
                p("Pole Vector Object", "object", "", "", vec![]),
                p("SolverType", "enum", "", "", vec![FbxProperty::I32(0)]),
                p("PoleVectorType", "enum", "", "", vec![FbxProperty::I32(0)]),
                p(
                    "PoleVector",
                    "Vector",
                    "",
                    "A",
                    vec![
                        FbxProperty::F64(0.0),
                        FbxProperty::F64(1.0),
                        FbxProperty::F64(0.0),
                    ],
                ),
            ],
        }],
    }
}

fn build_doc() -> FbxDocument {
    // Three joint Models + one constraint (doc §2 worked example
    // shape: sub-type display string in the header AND the Type
    // leaf; own Properties70 = only the differs-from-template set).
    let joints: Vec<FbxNode> = [(101, "joint1"), (102, "joint2"), (103, "effector1")]
        .iter()
        .map(|(id, name)| element("Model", *id, name, "Model", "LimbNode"))
        .collect();
    let mut constraint = element(
        "Constraint",
        911,
        "ikHandle1",
        "Constraint",
        "Single Chain IK",
    );
    constraint
        .children
        .push(leaf("Type", s(b"Single Chain IK")));
    constraint
        .children
        .push(leaf("MultiLayer", FbxProperty::I32(0)));
    constraint.children.push(FbxNode {
        name: "Properties70".into(),
        properties: Vec::new(),
        children: vec![
            p("First Joint", "object", "", "", vec![]),
            p("End Joint", "object", "", "", vec![]),
            p("Effector", "object", "", "", vec![]),
            p("SolverType", "enum", "", "", vec![FbxProperty::I32(1)]),
            p(
                "PoleVector",
                "Vector",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(1.0),
                ],
            ),
        ],
    });

    let mut objects_children = joints;
    objects_children.push(constraint);

    let definitions = FbxNode {
        name: "Definitions".into(),
        properties: Vec::new(),
        children: vec![
            leaf("Version", FbxProperty::I32(100)),
            leaf("Count", FbxProperty::I32(4)),
            FbxNode {
                name: "ObjectType".into(),
                properties: vec![s(b"Constraint")],
                children: vec![leaf("Count", FbxProperty::I32(1)), ik_template()],
            },
        ],
    };

    // Doc §3: a constraint is free-floating — only OP edges point at
    // it; the joints attach to the scene root.
    let connections = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(101, 0),
            c_oo(102, 0),
            c_oo(103, 0),
            c_op(101, 911, "First Joint"),
            c_op(102, 911, "End Joint"),
            c_op(103, 911, "Effector"),
        ],
    };

    FbxDocument {
        version: 7500,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![
                definitions,
                FbxNode {
                    name: "Objects".into(),
                    properties: Vec::new(),
                    children: objects_children,
                },
                connections,
            ],
        },
    }
}

fn constraint_entry(scene: &oxideav_mesh3d::Scene3D) -> serde_json::Value {
    scene
        .extras
        .get("fbx:constraints")
        .and_then(|v| v.as_array())
        .expect("fbx:constraints present")[0]
        .clone()
}

#[test]
fn constraint_decodes_with_slots_targets_and_templates() {
    let bytes = write_document(&build_doc()).expect("encode synthetic doc");
    let scene = FbxDecoder::new().decode(&bytes).expect("decode");

    let entry = constraint_entry(&scene);
    assert_eq!(entry["name"], "ikHandle1");
    assert_eq!(entry["kind"], "Single Chain IK");
    assert_eq!(entry["type"], "Single Chain IK");
    assert_eq!(entry["multi_layer"], 0);

    // Own records verbatim — including the space-bearing value-less
    // object slots and the enum override.
    let props = entry["properties"].as_array().unwrap();
    assert_eq!(props.len(), 5);
    assert_eq!(props[0]["name"], "First Joint");
    assert_eq!(props[0]["type"], "object");
    assert_eq!(props[0]["values"].as_array().unwrap().len(), 0);
    assert_eq!(props[3]["name"], "SolverType");
    assert_eq!(props[3]["values"][0]["l"], 1);
    assert_eq!(props[4]["name"], "PoleVector");
    assert_eq!(props[4]["flags"], "A");
    assert_eq!(props[4]["values"][2]["d"], 1.0);

    // Targets resolved through the OP edges to scene node indices.
    let targets = entry["targets"].as_array().unwrap();
    assert_eq!(targets.len(), 3);
    let node_name = |t: &serde_json::Value| {
        let ni = t["node"].as_u64().unwrap() as usize;
        scene.nodes[ni].name.clone().unwrap_or_default()
    };
    assert_eq!(targets[0]["slot"], "First Joint");
    assert_eq!(node_name(&targets[0]), "joint1");
    assert_eq!(targets[1]["slot"], "End Joint");
    assert_eq!(node_name(&targets[1]), "joint2");
    assert_eq!(targets[2]["slot"], "Effector");
    assert_eq!(node_name(&targets[2]), "effector1");

    // Per-kind template captured (doc §1), name following the
    // documented class pattern.
    let templates = scene
        .extras
        .get("fbx:constraint_templates")
        .and_then(|v| v.as_array())
        .expect("templates present");
    assert_eq!(templates.len(), 1);
    assert_eq!(templates[0]["name"], "FbxConstraintSingleChainIK");
    assert_eq!(
        oxideav_fbx::constraint::template_class_for_kind(entry["kind"].as_str().unwrap()),
        "FbxConstraintSingleChainIK"
    );
    let tpl_props = templates[0]["properties"].as_array().unwrap();
    assert_eq!(tpl_props.len(), 10);
    assert_eq!(tpl_props[2]["name"], "Weight");
    assert_eq!(tpl_props[2]["type"], "Weight"); // its own type string
    assert_eq!(tpl_props[2]["values"][0]["d"], 100.0);
}

/// Full closure: decode → encode → decode reproduces the identical
/// constraint catalogue + templates, in both output forms, with the
/// re-encoded document carrying the Definitions multi-template block
/// and the free-floating OP wiring.
#[test]
fn constraint_survives_encode_decode_in_both_forms() {
    let bytes = write_document(&build_doc()).expect("encode synthetic doc");
    let scene = FbxDecoder::new().decode(&bytes).expect("decode");

    let doc2 = oxideav_fbx::scene_writer::encode_scene(&scene);

    // Structure checks on the re-encoded document.
    let objects = doc2.root.child("Objects").expect("Objects");
    let c2 = objects
        .children_named("Constraint")
        .next()
        .expect("Constraint element re-emitted");
    assert_eq!(c2.properties[2].as_str(), Some("Single Chain IK"));
    assert_eq!(
        c2.child("Type")
            .and_then(|n| n.properties.first())
            .and_then(|p| p.as_str()),
        Some("Single Chain IK")
    );
    let defs = doc2.root.child("Definitions").expect("Definitions");
    let constraint_ot = defs
        .children_named("ObjectType")
        .find(|ot| ot.properties.first().and_then(|p| p.as_str()) == Some("Constraint"))
        .expect("ObjectType Constraint block");
    assert_eq!(
        constraint_ot
            .children_named("PropertyTemplate")
            .next()
            .and_then(|t| t.properties.first())
            .and_then(|p| p.as_str()),
        Some("FbxConstraintSingleChainIK")
    );

    // Binary parity.
    let bin = write_document(&doc2).expect("binary re-encode");
    let scene_bin = FbxDecoder::new().decode(&bin).expect("re-decode binary");
    assert_eq!(
        scene.extras.get("fbx:constraints"),
        scene_bin.extras.get("fbx:constraints"),
        "binary constraint catalogue parity"
    );
    assert_eq!(
        scene.extras.get("fbx:constraint_templates"),
        scene_bin.extras.get("fbx:constraint_templates"),
        "binary template parity"
    );

    // ASCII parity.
    let text = write_ascii_document(&doc2).expect("ascii re-encode");
    let scene_ascii = FbxDecoder::new().decode(&text).expect("re-decode ascii");
    assert_eq!(
        scene.extras.get("fbx:constraints"),
        scene_ascii.extras.get("fbx:constraints"),
        "ascii constraint catalogue parity"
    );
    assert_eq!(
        scene.extras.get("fbx:constraint_templates"),
        scene_ascii.extras.get("fbx:constraint_templates"),
        "ascii template parity"
    );
}
