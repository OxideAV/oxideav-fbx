//! Round 367 — end-to-end `Model` node local-transform decode via the
//! full `Mesh3DDecoder::decode` (binary front-end) path.
//!
//! Builds a synthetic binary-FBX byte buffer with two `Model` records:
//!
//! - `Placed` carries a `Properties70` with `Lcl Translation`,
//!   `Lcl Rotation` (90° about X), and `Lcl Scaling` — the chain
//!   reduces to `T * R(XYZ) * S`, so its node receives a non-identity
//!   `Transform::Trs`.
//! - `Pivoted` carries a non-zero `PreRotation` `Vector3D` — the
//!   reduced form would be lossy, so its node stays at identity and a
//!   `Node::extras["fbx:transform_incomplete"]` reason marker plus the
//!   raw `Lcl` components surface instead.
//!
//! All record shapes follow `docs/3d/fbx/fbx-binary-properties70.md`
//! §4 / §5 (Properties70 `P` grammar; object record header) and the
//! `Lcl …` typeName enumeration in `docs/3d/fbx/fbx-ascii-grammar.md`
//! §8.

use std::collections::HashMap;

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_document, FbxDecoder,
};
use oxideav_mesh3d::{Mesh3DDecoder, Transform};

fn s(b: &[u8]) -> FbxProperty {
    FbxProperty::String(b.to_vec())
}

/// A `P:` vec3 record `[name, type, "", "A", x, y, z]`.
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

fn properties70(records: Vec<FbxNode>) -> FbxNode {
    FbxNode {
        name: "Properties70".into(),
        properties: Vec::new(),
        children: records,
    }
}

fn model_with_props(id: i64, name: &str, props: FbxNode) -> FbxNode {
    let display = format!("{name}\x00\x01Model");
    FbxNode {
        name: "Model".into(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(display.into_bytes()),
            s(b"Mesh"),
        ],
        children: vec![props],
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
fn model_local_transforms_round_trip_through_binary_decoder() {
    let placed = model_with_props(
        700,
        "Placed",
        properties70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 2.0, 3.0]),
            p_vec3("Lcl Rotation", "Lcl Rotation", [90.0, 0.0, 0.0]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [2.0, 2.0, 2.0]),
        ]),
    );
    let pivoted = model_with_props(
        701,
        "Pivoted",
        properties70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [4.0, 5.0, 6.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 30.0, 0.0]),
        ]),
    );

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![placed, pivoted],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_oo(700, 0), c_oo(701, 0)],
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

    let mut by_name: HashMap<&str, &oxideav_mesh3d::Node> = HashMap::new();
    for n in &scene.nodes {
        if let Some(name) = n.name.as_deref() {
            by_name.insert(name, n);
        }
    }

    // `Placed` reduces to TRS: T=(1,2,3), R=90° about X, S=(2,2,2).
    let placed = by_name.get("Placed").expect("Placed node surfaced");
    match placed.transform {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => {
            assert_eq!(translation, [1.0, 2.0, 3.0]);
            assert_eq!(scale, [2.0, 2.0, 2.0]);
            let h = std::f32::consts::FRAC_1_SQRT_2;
            assert!((rotation[0] - h).abs() < 1e-5, "rot x = {}", rotation[0]);
            assert!((rotation[3] - h).abs() < 1e-5, "rot w = {}", rotation[3]);
        }
        Transform::Matrix(_) => panic!("expected decomposed Trs"),
    }
    assert!(!placed.extras.contains_key("fbx:transform_incomplete"));

    // `Pivoted` has a non-zero PreRotation → stays at identity, marks
    // the lossy reduction, and surfaces the raw Lcl components.
    let pivoted = by_name.get("Pivoted").expect("Pivoted node surfaced");
    assert_eq!(pivoted.transform, Transform::identity());
    assert_eq!(
        pivoted
            .extras
            .get("fbx:transform_incomplete")
            .and_then(|v| v.as_str()),
        Some("pre_rotation"),
    );
    let raw_t = pivoted
        .extras
        .get("fbx:lcl_translation")
        .and_then(|v| v.as_array())
        .expect("raw Lcl Translation surfaced on incomplete node");
    assert_eq!(raw_t[0].as_f64(), Some(4.0));
    assert_eq!(raw_t[2].as_f64(), Some(6.0));
}
