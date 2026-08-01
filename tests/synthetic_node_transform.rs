//! End-to-end `Model` node local-transform decode via the full
//! `Mesh3DDecoder::decode` (binary front-end) path — the complete
//! node-transform chain per `docs/3d/fbx/fbx-node-transform-chain.md`.
//!
//! Builds a synthetic binary-FBX byte buffer with four `Model`
//! records whose composed transforms are verified analytically:
//!
//! - `Placed` carries the plain `Lcl` triple — composes to
//!   `T * R(XYZ) * S` with no chain extras.
//! - `PreRotated` carries a `PreRotation` — the doc §1 chain gives
//!   `Q = Rpre · R`, and the raw chain surfaces on `extras`.
//! - `Pivoted` carries `RotationPivot` + a 90° Z rotation — the doc
//!   §1 closed form gives `t = T + Rp + Q·(−Rp)`.
//! - `Ordered` carries `RotationOrder = 5` (`ZYX`, the doc §3 table)
//!   with a two-axis rotation whose composed action on a basis
//!   vector discriminates ZYX from XYZ.
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

/// A `P:` enum record `[name, "enum", "", "", v]`.
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

/// Rotate a vector by an xyzw quaternion.
fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let mul = |a: [f32; 4], b: [f32; 4]| -> [f32; 4] {
        [
            a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
            a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
            a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
        ]
    };
    let p = [v[0], v[1], v[2], 0.0];
    let c = [-q[0], -q[1], -q[2], q[3]];
    let r = mul(mul(q, p), c);
    [r[0], r[1], r[2]]
}

#[test]
fn model_transform_chain_composes_through_binary_decoder() {
    let placed = model_with_props(
        700,
        "Placed",
        properties70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 2.0, 3.0]),
            p_vec3("Lcl Rotation", "Lcl Rotation", [90.0, 0.0, 0.0]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [2.0, 2.0, 2.0]),
        ]),
    );
    let pre_rotated = model_with_props(
        701,
        "PreRotated",
        properties70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [4.0, 5.0, 6.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 90.0, 0.0]),
        ]),
    );
    let pivoted = model_with_props(
        702,
        "Pivoted",
        properties70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [10.0, 0.0, 0.0]),
            p_vec3("RotationPivot", "Vector3D", [1.0, 0.0, 0.0]),
            p_vec3("Lcl Rotation", "Lcl Rotation", [0.0, 0.0, 90.0]),
        ]),
    );
    let ordered = model_with_props(
        703,
        "Ordered",
        properties70(vec![
            p_enum("RotationOrder", 5),
            p_vec3("Lcl Rotation", "Lcl Rotation", [90.0, 0.0, 90.0]),
        ]),
    );

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![placed, pre_rotated, pivoted, ordered],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_oo(700, 0), c_oo(701, 0), c_oo(702, 0), c_oo(703, 0)],
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

    // `Placed` is the plain triple: T=(1,2,3), R=90° about X,
    // S=(2,2,2); no chain extras.
    let placed = by_name.get("Placed").expect("Placed node surfaced");
    let (translation, rotation, scale) = trs(&placed.transform);
    assert_eq!(translation, [1.0, 2.0, 3.0]);
    assert_eq!(scale, [2.0, 2.0, 2.0]);
    let h = std::f32::consts::FRAC_1_SQRT_2;
    assert!((rotation[0] - h).abs() < 1e-5, "rot x = {}", rotation[0]);
    assert!((rotation[3] - h).abs() < 1e-5, "rot w = {}", rotation[3]);
    assert!(!placed.extras.contains_key("fbx:transform_incomplete"));
    assert!(!placed.extras.contains_key("fbx:lcl_translation"));

    // `PreRotated`: Q = Rpre (90° about Y) since Lcl Rotation is
    // absent; translation is untouched (pivots zero). Raw chain
    // surfaces for re-encode.
    let pre = by_name.get("PreRotated").expect("PreRotated node surfaced");
    let (translation, rotation, _) = trs(&pre.transform);
    assert_eq!(translation, [4.0, 5.0, 6.0]);
    assert!((rotation[1] - h).abs() < 1e-5 && (rotation[3] - h).abs() < 1e-5);
    assert!(!pre.extras.contains_key("fbx:transform_incomplete"));
    let raw_t = pre
        .extras
        .get("fbx:lcl_translation")
        .and_then(|v| v.as_array())
        .expect("raw Lcl Translation surfaced alongside the chain");
    assert_eq!(raw_t[0].as_f64(), Some(4.0));
    assert_eq!(raw_t[2].as_f64(), Some(6.0));
    let raw_pre = pre
        .extras
        .get("fbx:pre_rotation")
        .and_then(|v| v.as_array())
        .expect("raw PreRotation surfaced");
    assert_eq!(raw_pre[1].as_f64(), Some(90.0));

    // `Pivoted`: doc §1 closed form t = T + Rp + Q·(−Rp) with a 90° Z
    // rotation: Q·(−1,0,0) = (0,−1,0) → t = (11,−1,0). The pivot
    // point itself maps back onto T + Rp.
    let piv = by_name.get("Pivoted").expect("Pivoted node surfaced");
    let (translation, rotation, _) = trs(&piv.transform);
    assert!((translation[0] - 11.0).abs() < 1e-5, "t = {translation:?}");
    assert!((translation[1] + 1.0).abs() < 1e-5, "t = {translation:?}");
    assert!(translation[2].abs() < 1e-5);
    // Local action check: the pivot (1,0,0) must land at T + pivot.
    let p = rotate(rotation, [1.0, 0.0, 0.0]);
    let moved = [
        p[0] + translation[0],
        p[1] + translation[1],
        p[2] + translation[2],
    ];
    assert!((moved[0] - 11.0).abs() < 1e-5 && moved[1].abs() < 1e-5);

    // `Ordered` (ZYX): applies Z first — +Y → −X, then the X
    // rotation keeps −X fixed. Raw enum surfaces on extras.
    let ord = by_name.get("Ordered").expect("Ordered node surfaced");
    let (_, rotation, _) = trs(&ord.transform);
    let v = rotate(rotation, [0.0, 1.0, 0.0]);
    assert!(v[0] < -0.999, "ZYX expected +Y → −X, got {v:?}");
    assert_eq!(
        ord.extras
            .get("fbx:rotation_order")
            .and_then(|v| v.as_i64()),
        Some(5)
    );
    assert!(!ord.extras.contains_key("fbx:transform_incomplete"));
}
