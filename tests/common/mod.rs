//! Shared helpers for the 2026-11 sweep tests: fixture access and
//! tree renditions for bit-exact / value-level comparison.
#![allow(dead_code)]

use std::path::PathBuf;

use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};

/// Every vendored sweep file, `(directory, file name)`.
pub const SWEEP_FILES: &[(&str, &str)] = &[
    ("fbxsdk-keyattr-sweep", "keyattr-sweep-binary-v7400.fbx"),
    ("fbxsdk-keyattr-sweep", "keyattr-sweep-binary-v7500.fbx"),
    ("fbxsdk-keyattr-sweep", "keyattr-sweep-ascii-v7500.fbx"),
    (
        "fbxsdk-keyattr-sweep",
        "keyattr-runs-roundtrip-binary-v7400.fbx",
    ),
    (
        "fbxsdk-keyattr-sweep",
        "keyattr-runs-roundtrip-binary-v7500.fbx",
    ),
    (
        "fbxsdk-keyattr-sweep",
        "keyattr-runs-roundtrip-ascii-v7500.fbx",
    ),
    ("fbxsdk-nurbs", "nurbs-curves-binary-v7400.fbx"),
    ("fbxsdk-nurbs", "nurbs-curves-binary-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-curves-ascii-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-surfaces-binary-v7400.fbx"),
    ("fbxsdk-nurbs", "nurbs-surfaces-binary-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-surfaces-ascii-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-line-binary-v7400.fbx"),
    ("fbxsdk-nurbs", "nurbs-line-binary-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-line-ascii-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-trim-binary-v7400.fbx"),
    ("fbxsdk-nurbs", "nurbs-trim-binary-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-trim-ascii-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-legacy-binary-v7400.fbx"),
    ("fbxsdk-nurbs", "nurbs-legacy-binary-v7500.fbx"),
    ("fbxsdk-nurbs", "nurbs-legacy-ascii-v7500.fbx"),
    (
        "fbxsdk-blendshape-inbetween",
        "blendshape-inbetween-binary-v7400.fbx",
    ),
    (
        "fbxsdk-blendshape-inbetween",
        "blendshape-inbetween-binary-v7500.fbx",
    ),
    (
        "fbxsdk-blendshape-inbetween",
        "blendshape-inbetween-ascii-v7500.fbx",
    ),
    (
        "fbxsdk-texture-placement",
        "texture-placement-sweep-binary-v7400.fbx",
    ),
    (
        "fbxsdk-texture-placement",
        "texture-placement-sweep-binary-v7500.fbx",
    ),
    (
        "fbxsdk-texture-placement",
        "texture-placement-sweep-ascii-v7500.fbx",
    ),
];

pub fn read(dir: &str, name: &str) -> Vec<u8> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/sweeps-2026-11")
        .join(dir)
        .join(name);
    std::fs::read(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

pub fn parse_any(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

/// Bit-exact tree rendition: floats print by bit pattern so NaN
/// payloads (the packed `KeyAttrDataFloat` words) compare.
pub fn exact(n: &FbxNode, out: &mut String) {
    out.push_str(&n.name);
    out.push('(');
    for p in &n.properties {
        match p {
            FbxProperty::F32(v) => out.push_str(&format!("F{:08x},", v.to_bits())),
            FbxProperty::F64(v) => out.push_str(&format!("D{:016x},", v.to_bits())),
            FbxProperty::F32Array(a) => {
                out.push('f');
                a.iter()
                    .for_each(|v| out.push_str(&format!("{:08x} ", v.to_bits())));
            }
            FbxProperty::F64Array(a) => {
                out.push('d');
                a.iter()
                    .for_each(|v| out.push_str(&format!("{:016x} ", v.to_bits())));
            }
            other => out.push_str(&format!("{other:?},")),
        }
    }
    out.push(')');
    out.push('{');
    for c in &n.children {
        exact(c, out);
    }
    out.push('}');
}

/// Value-level rendition for a comparison across the two forms: the
/// ASCII form does not record scalar widths (`I` vs `L`, `F` vs `D`)
/// and prints a whole-valued double without a fraction, so scalars
/// compare by numeric value; arrays keep their element kind.
pub fn loose(n: &FbxNode, out: &mut String) {
    out.push_str(&n.name);
    out.push('(');
    for p in &n.properties {
        match p {
            FbxProperty::I16(v) => out.push_str(&format!("{},", f64::from(*v))),
            FbxProperty::I32(v) => out.push_str(&format!("{},", f64::from(*v))),
            FbxProperty::I64(v) => out.push_str(&format!("{},", *v as f64)),
            FbxProperty::F32(v) => out.push_str(&format!("{},", f64::from(*v))),
            FbxProperty::F64(v) => out.push_str(&format!("{v},")),
            FbxProperty::F32Array(a) => {
                out.push('f');
                a.iter()
                    .for_each(|v| out.push_str(&format!("{:08x} ", v.to_bits())));
            }
            FbxProperty::I32Array(a) => a.iter().for_each(|v| out.push_str(&format!("{v} "))),
            FbxProperty::I64Array(a) => a.iter().for_each(|v| out.push_str(&format!("{v} "))),
            FbxProperty::F64Array(a) => a.iter().for_each(|v| out.push_str(&format!("{v} "))),
            // A binary `R` blob has no ASCII type of its own: it is
            // written as a quoted base64 string.
            FbxProperty::Raw(b) => out.push_str(&format!(
                "{:?},",
                FbxProperty::String(oxideav_fbx::base64::encode(b).into_bytes())
            )),
            other => out.push_str(&format!("{other:?},")),
        }
    }
    out.push(')');
    out.push('{');
    for c in &n.children {
        loose(c, out);
    }
    out.push('}');
}

pub fn render(f: fn(&FbxNode, &mut String), n: &FbxNode) -> String {
    let mut s = String::new();
    f(n, &mut s);
    s
}

/// The three container flavours of one sweep scene.
pub fn flavours(stem: &str) -> [String; 3] {
    [
        format!("{stem}-binary-v7400.fbx"),
        format!("{stem}-binary-v7500.fbx"),
        format!("{stem}-ascii-v7500.fbx"),
    ]
}

/// Display name of an element (`Name\x00\x01Class`).
pub fn display(n: &FbxNode) -> String {
    n.properties
        .get(1)
        .and_then(FbxProperty::as_str)
        .and_then(|s| s.split('\u{0}').next())
        .unwrap_or("")
        .to_owned()
}

/// Sub-type string of an element.
pub fn subtype(n: &FbxNode) -> String {
    n.properties
        .get(2)
        .and_then(FbxProperty::as_str)
        .unwrap_or("")
        .to_owned()
}

/// `true` for an ASCII fixture name.
pub fn is_ascii_name(name: &str) -> bool {
    name.contains("-ascii-")
}
