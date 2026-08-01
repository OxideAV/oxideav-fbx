//! Node-transform-chain round trip: `decode → encode → decode`
//! preserves the composed transform AND the authored chain
//! components, in both the binary and ASCII forms.
//!
//! The chain semantics are `docs/3d/fbx/fbx-node-transform-chain.md`
//! §1–§3; record shapes follow `docs/3d/fbx/fbx-binary-properties70.md`
//! §4 / §5.

use std::collections::HashMap;

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_document, FbxDecoder, FbxEncoder, FbxOutputForm,
};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder, Node, Scene3D, Transform};

fn s(b: &[u8]) -> FbxProperty {
    FbxProperty::String(b.to_vec())
}

fn p_vec3(name: &str, type_name: &str, v: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".into(),
        properties: vec![
            s(name.as_bytes()),
            s(type_name.as_bytes()),
            s(b""),
            s(b"A"),
            FbxProperty::F64(v[0]),
            FbxProperty::F64(v[1]),
            FbxProperty::F64(v[2]),
        ],
        children: Vec::new(),
    }
}

fn p_enum(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: "P".into(),
        properties: vec![
            s(name.as_bytes()),
            s(b"enum"),
            s(b""),
            s(b""),
            FbxProperty::I32(v),
        ],
        children: Vec::new(),
    }
}

fn model_with_props(id: i64, name: &str, records: Vec<FbxNode>) -> FbxNode {
    let display = format!("{name}\x00\x01Model");
    FbxNode {
        name: "Model".into(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(display.into_bytes()),
            s(b"Mesh"),
        ],
        children: vec![FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: records,
        }],
    }
}

fn c_oo(child: i64, parent: i64) -> FbxNode {
    FbxNode {
        name: "C".into(),
        properties: vec![s(b"OO"), FbxProperty::I64(child), FbxProperty::I64(parent)],
        children: Vec::new(),
    }
}

/// A document with one fully non-trivial chain, one geometric-TRS
/// carrier, and one `InheritType` carrier.
fn chain_document() -> FbxDocument {
    let full_chain = model_with_props(
        800,
        "FullChain",
        vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.5, -2.0, 3.25]),
            p_vec3("Lcl Rotation", "Lcl Rotation", [30.0, -45.0, 60.0]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [2.0, 0.5, 1.25]),
            p_vec3("RotationOffset", "Vector3D", [0.25, 0.5, -0.75]),
            p_vec3("RotationPivot", "Vector3D", [1.0, 2.0, -1.0]),
            p_vec3("PreRotation", "Vector3D", [10.0, 20.0, -30.0]),
            p_vec3("PostRotation", "Vector3D", [-15.0, 5.0, 25.0]),
            p_vec3("ScalingOffset", "Vector3D", [-0.5, 0.25, 0.125]),
            p_vec3("ScalingPivot", "Vector3D", [0.5, -1.5, 2.5]),
            p_enum("RotationOrder", 3),
        ],
    );
    let geometric = model_with_props(
        801,
        "Geometric",
        vec![
            p_vec3("Lcl Translation", "Lcl Translation", [7.0, 0.0, 0.0]),
            p_vec3("GeometricTranslation", "Vector3D", [0.0, 5.0, 0.0]),
            p_vec3("GeometricRotation", "Vector3D", [0.0, 0.0, 90.0]),
            p_vec3("GeometricScaling", "Vector3D", [2.0, 2.0, 2.0]),
        ],
    );
    let inheriting = model_with_props(
        802,
        "Inheriting",
        vec![
            p_vec3("Lcl Translation", "Lcl Translation", [0.0, 1.0, 0.0]),
            p_enum("InheritType", 1),
        ],
    );

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![full_chain, geometric, inheriting],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_oo(800, 0), c_oo(801, 0), c_oo(802, 0)],
    };
    FbxDocument {
        version: 7500,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, conns],
        },
    }
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn by_name(scene: &Scene3D) -> HashMap<&str, &Node> {
    scene
        .nodes
        .iter()
        .filter_map(|n| n.name.as_deref().map(|name| (name, n)))
        .collect()
}

fn trs(t: &Transform) -> ([f32; 3], [f32; 4], [f32; 3]) {
    match *t {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => (translation, rotation, scale),
        Transform::Matrix(_) => panic!("expected decomposed Trs"),
    }
}

fn assert_transform_close(a: &Transform, b: &Transform, tol: f32) {
    let (ta, qa, sa) = trs(a);
    let (tb, qb, sb) = trs(b);
    for i in 0..3 {
        assert!((ta[i] - tb[i]).abs() < tol, "translation: {ta:?} vs {tb:?}");
        assert!((sa[i] - sb[i]).abs() < tol, "scale: {sa:?} vs {sb:?}");
    }
    // q and −q are the same rotation.
    let dot: f32 = (0..4).map(|i| qa[i] * qb[i]).sum();
    assert!(dot.abs() > 1.0 - 1e-5, "rotation: {qa:?} vs {qb:?}");
}

const CHAIN_KEYS: [&str; 12] = [
    "fbx:lcl_translation",
    "fbx:lcl_rotation",
    "fbx:lcl_scaling",
    "fbx:rotation_offset",
    "fbx:rotation_pivot",
    "fbx:pre_rotation",
    "fbx:post_rotation",
    "fbx:scaling_offset",
    "fbx:scaling_pivot",
    "fbx:rotation_order",
    "fbx:inherit_type",
    "fbx:geometric_translation",
];

fn roundtrip_and_verify(form: FbxOutputForm) {
    let bytes = write_document(&chain_document()).expect("write synthetic doc");
    let first = decode(&bytes);

    let re_encoded = FbxEncoder::new()
        .form(form)
        .encode(&first)
        .expect("re-encode");
    let second = decode(&re_encoded);

    let a = by_name(&first);
    let b = by_name(&second);
    for name in ["FullChain", "Geometric", "Inheriting"] {
        let na = a.get(name).expect("first-decode node");
        let nb = b.get(name).expect("second-decode node");
        assert_transform_close(&na.transform, &nb.transform, 1e-5);
        assert!(
            !nb.extras.contains_key("fbx:transform_incomplete"),
            "{name}: re-decode must not degrade"
        );
        for key in CHAIN_KEYS {
            assert_eq!(
                na.extras.get(key),
                nb.extras.get(key),
                "{name}: extras[{key}] must survive the round trip"
            );
        }
    }

    // Spot-check the authored values survived verbatim (not merely
    // both-absent): the full chain's pivot and order.
    let full = b.get("FullChain").unwrap();
    let pivot = full
        .extras
        .get("fbx:rotation_pivot")
        .and_then(|v| v.as_array())
        .expect("pivot survives");
    assert_eq!(pivot[1].as_f64(), Some(2.0));
    assert_eq!(
        full.extras
            .get("fbx:rotation_order")
            .and_then(|v| v.as_i64()),
        Some(3)
    );
    let geo = b.get("Geometric").unwrap();
    assert_eq!(
        geo.extras
            .get("fbx:geometric_translation")
            .and_then(|v| v.as_array())
            .and_then(|a| a[1].as_f64()),
        Some(5.0)
    );
    assert_eq!(
        b.get("Inheriting")
            .unwrap()
            .extras
            .get("fbx:inherit_type")
            .and_then(|v| v.as_i64()),
        Some(1)
    );
}

#[test]
fn chain_round_trips_binary() {
    roundtrip_and_verify(FbxOutputForm::Binary);
}

#[test]
fn chain_round_trips_ascii() {
    roundtrip_and_verify(FbxOutputForm::Ascii);
}

/// The composed transform of the re-decoded chain must equal the
/// original composition — i.e. the encoder emitted the authored
/// chain, not a double-application of the composed reduction.
#[test]
fn re_encode_does_not_double_apply_pivots() {
    let bytes = write_document(&chain_document()).expect("write");
    let first = decode(&bytes);
    let re_encoded = FbxEncoder::new().encode(&first).expect("re-encode");
    let second = decode(&re_encoded);

    let (t1, _, _) = trs(&by_name(&first)["FullChain"].transform);
    let (t2, _, _) = trs(&by_name(&second)["FullChain"].transform);
    // A double-applied pivot shifts the translation; equality within
    // f32 noise proves the authored chain was re-emitted.
    for i in 0..3 {
        assert!((t1[i] - t2[i]).abs() < 1e-5, "{t1:?} vs {t2:?}");
    }
}

/// A hand-built `Scene3D` (no decode provenance) carrying chain
/// extras synthesises the chain records — the model-carries-them
/// encode direction.
#[test]
fn hand_built_scene_synthesises_chain_records() {
    let mut scene = Scene3D::new();
    let mut node = Node::new().with_name("Pivoted");
    node.extras.insert(
        "fbx:lcl_translation".into(),
        serde_json::json!([10.0, 0.0, 0.0]),
    );
    node.extras.insert(
        "fbx:lcl_rotation".into(),
        serde_json::json!([0.0, 0.0, 90.0]),
    );
    node.extras.insert(
        "fbx:rotation_pivot".into(),
        serde_json::json!([1.0, 0.0, 0.0]),
    );
    node.extras
        .insert("fbx:rotation_order".into(), serde_json::json!(5));
    let nid = scene.add_node(node);
    scene.roots.push(nid);

    let bytes = FbxEncoder::new().encode(&scene).expect("encode");
    let decoded = decode(&bytes);
    let nodes = by_name(&decoded);
    let n = nodes.get("Pivoted").expect("node survives");

    // ZYX order, 90° about Z only → same single-axis rotation; the
    // pivot closed form gives t = T + Rp + Q·(−Rp) = (11, −1, 0).
    let (t, _, _) = trs(&n.transform);
    assert!((t[0] - 11.0).abs() < 1e-5, "t = {t:?}");
    assert!((t[1] + 1.0).abs() < 1e-5, "t = {t:?}");
    assert_eq!(
        n.extras
            .get("fbx:rotation_pivot")
            .and_then(|v| v.as_array())
            .map(|a| a.len()),
        Some(3)
    );
    assert_eq!(
        n.extras.get("fbx:rotation_order").and_then(|v| v.as_i64()),
        Some(5)
    );
}

/// Nodes without chain extras keep the plain decompose path: the
/// encoder must not emit any chain record for them.
#[test]
fn plain_nodes_emit_no_chain_records() {
    let mut scene = Scene3D::new();
    let node = Node::new()
        .with_name("Plain")
        .with_transform(Transform::Trs {
            translation: [1.0, 2.0, 3.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0, 1.0, 1.0],
        });
    let nid = scene.add_node(node);
    scene.roots.push(nid);

    let bytes = FbxEncoder::new().encode(&scene).expect("encode");
    let decoded = decode(&bytes);
    let nodes = by_name(&decoded);
    let n = nodes.get("Plain").expect("node survives");
    let (t, _, _) = trs(&n.transform);
    assert_eq!(t, [1.0, 2.0, 3.0]);
    for key in CHAIN_KEYS {
        assert!(!n.extras.contains_key(key), "unexpected {key}");
    }
}
