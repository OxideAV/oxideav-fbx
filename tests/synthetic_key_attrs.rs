//! Raw `KeyAttrFlags` / `KeyAttrDataFloat` / `KeyAttrRefCount`
//! surfacing (round 439) — the catalogue on
//! `Scene3D::extras["fbx:key_attrs"]`.
//!
//! Per `docs/3d/fbx/GAP-TRACKER.md` the bitfield's value assignment
//! is an open item, so the decode surfaces the arrays **verbatim**
//! (flags / ref_count as integers, the data floats as IEEE-754 bit
//! patterns) with the stack / target / property / axis join key
//! resolved through the same `Connections` chain the animation
//! extractor walks (`docs/3d/fbx/fbx-binary-properties70.md` §5–§7
//! record shapes).

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_document, FbxDecoder,
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

/// Two curves on one `Lcl Translation` curve node: `d|X` carries the
/// three key-attribute sub-records, `d|Y` carries none. Only the
/// attributed curve lands in the catalogue, with its full join key.
#[test]
fn key_attr_arrays_surface_verbatim_with_join_key() {
    let model = element("Model", 900, "Animated", "Model", "Mesh");
    let stack = element("AnimationStack", 910, "Take 001", "AnimStack", "");
    let layer = element("AnimationLayer", 911, "Base", "AnimLayer", "");
    let curve_node = element("AnimationCurveNode", 912, "T", "AnimCurveNode", "");

    let ticks: Vec<i64> = vec![0, 46_186_158_000, 92_372_316_000];
    let mut cx = element("AnimationCurve", 913, "", "AnimCurve", "");
    cx.children
        .push(leaf("KeyTime", FbxProperty::I64Array(ticks.clone())));
    cx.children.push(leaf(
        "KeyValueFloat",
        FbxProperty::F32Array(vec![0.0, 1.0, 4.0]),
    ));
    // Raw payloads — arbitrary integers / bit patterns; the decode
    // must not interpret them, only carry them.
    cx.children.push(leaf(
        "KeyAttrFlags",
        FbxProperty::I32Array(vec![24840, 24968]),
    ));
    cx.children.push(leaf(
        "KeyAttrDataFloat",
        FbxProperty::F32Array(vec![0.25, f32::from_bits(0x7fc0_0001), -1.5]),
    ));
    cx.children
        .push(leaf("KeyAttrRefCount", FbxProperty::I32Array(vec![2, 1])));

    let mut cy = element("AnimationCurve", 914, "", "AnimCurve", "");
    cy.children
        .push(leaf("KeyTime", FbxProperty::I64Array(ticks)));
    cy.children.push(leaf(
        "KeyValueFloat",
        FbxProperty::F32Array(vec![0.0, 2.0, 8.0]),
    ));

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![model, stack, layer, curve_node, cx, cy],
    };
    let connections = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(900, 0),
            c_oo(911, 910),
            c_oo(912, 911),
            c_op(912, 900, "Lcl Translation"),
            c_op(913, 912, "d|X"),
            c_op(914, 912, "d|Y"),
        ],
    };
    let doc = FbxDocument {
        version: 7400,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, connections],
        },
    };

    let bytes = write_document(&doc).expect("encode synthetic doc");
    let scene = FbxDecoder::new().decode(&bytes).expect("decode");

    let catalogue = scene
        .extras
        .get("fbx:key_attrs")
        .and_then(|v| v.as_array())
        .expect("fbx:key_attrs catalogue present");
    assert_eq!(catalogue.len(), 1, "only the attributed curve appears");
    let entry = catalogue[0].as_object().expect("object entry");

    assert_eq!(
        entry.get("stack").and_then(|v| v.as_str()),
        Some("Take 001")
    );
    assert_eq!(
        entry.get("target").and_then(|v| v.as_str()),
        Some("Animated")
    );
    assert_eq!(
        entry.get("property").and_then(|v| v.as_str()),
        Some("Lcl Translation")
    );
    assert_eq!(entry.get("axis").and_then(|v| v.as_str()), Some("d|X"));
    assert_eq!(entry.get("key_count").and_then(|v| v.as_u64()), Some(3));

    let flags: Vec<i64> = entry["flags"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    assert_eq!(flags, vec![24840, 24968]);

    let refc: Vec<i64> = entry["ref_count"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    assert_eq!(refc, vec![2, 1]);

    // Data floats surface as lossless bit patterns — including the
    // NaN payload JSON floats could not carry.
    let bits: Vec<u64> = entry["data_bits"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap())
        .collect();
    assert_eq!(
        bits,
        vec![
            u64::from(0.25f32.to_bits()),
            0x7fc0_0001,
            u64::from((-1.5f32).to_bits()),
        ]
    );

    // The animation itself still decodes (linear sampling unchanged).
    assert_eq!(scene.animations.len(), 1);
}

/// A document whose curves carry no key-attribute records surfaces
/// no catalogue at all.
#[test]
fn absent_key_attrs_leave_no_catalogue() {
    let model = element("Model", 900, "Plain", "Model", "Mesh");
    let stack = element("AnimationStack", 910, "Take 001", "AnimStack", "");
    let layer = element("AnimationLayer", 911, "Base", "AnimLayer", "");
    let curve_node = element("AnimationCurveNode", 912, "T", "AnimCurveNode", "");
    let mut c = element("AnimationCurve", 913, "", "AnimCurve", "");
    c.children.push(leaf(
        "KeyTime",
        FbxProperty::I64Array(vec![0, 46_186_158_000]),
    ));
    c.children
        .push(leaf("KeyValueFloat", FbxProperty::F32Array(vec![0.0, 1.0])));

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![model, stack, layer, curve_node, c],
    };
    let connections = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(900, 0),
            c_oo(911, 910),
            c_oo(912, 911),
            c_op(912, 900, "Lcl Translation"),
            c_op(913, 912, "d|X"),
        ],
    };
    let doc = FbxDocument {
        version: 7400,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, connections],
        },
    };
    let bytes = write_document(&doc).expect("encode synthetic doc");
    let scene = FbxDecoder::new().decode(&bytes).expect("decode");
    assert!(!scene.extras.contains_key("fbx:key_attrs"));
}
