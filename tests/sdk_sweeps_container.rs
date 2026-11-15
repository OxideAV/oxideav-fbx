//! Container-level checks on the 2026-11 vendor-exporter sweeps
//! (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md`; fixtures vendored
//! under `tests/fixtures/sweeps-2026-11/`, see the README there):
//! every staged file — binary 7400, binary 7500 and ASCII 7500 of each
//! sweep — parses, and re-serialises through both writers to a tree
//! that reads back equal.

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

fn read(dir: &str, name: &str) -> Vec<u8> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/sweeps-2026-11")
        .join(dir)
        .join(name);
    std::fs::read(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

fn parse_any(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

/// Bit-exact tree rendition: floats print by bit pattern so NaN
/// payloads (the packed `KeyAttrDataFloat` words) compare.
fn exact(n: &FbxNode, out: &mut String) {
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
fn loose(n: &FbxNode, out: &mut String) {
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

fn render(f: fn(&FbxNode, &mut String), n: &FbxNode) -> String {
    let mut s = String::new();
    f(n, &mut s);
    s
}

#[test]
fn every_sweep_file_round_trips_through_its_own_form() {
    for (dir, name) in SWEEP_FILES {
        let bytes = read(dir, name);
        let doc = parse_any(&bytes);
        let again = if oxideav_fbx::is_ascii_fbx(&bytes) {
            let text = oxideav_fbx::write_ascii_document(&doc).expect("ascii write");
            oxideav_fbx::ascii::parse(&text).expect("ascii re-parse")
        } else {
            let bin = oxideav_fbx::write_document(&doc).expect("binary write");
            oxideav_fbx::binary::parse(&bin).expect("binary re-parse")
        };
        assert_eq!(doc.version, again.version, "{name}");
        assert_eq!(
            render(exact, &doc.root),
            render(exact, &again.root),
            "{name}: same-form round trip"
        );
    }
}

#[test]
fn every_sweep_file_round_trips_through_the_other_form() {
    for (dir, name) in SWEEP_FILES {
        let bytes = read(dir, name);
        let doc = parse_any(&bytes);
        let again = if oxideav_fbx::is_ascii_fbx(&bytes) {
            let bin = oxideav_fbx::write_document(&doc).expect("binary write");
            oxideav_fbx::binary::parse(&bin).expect("binary re-parse")
        } else {
            let text = oxideav_fbx::write_ascii_document(&doc).expect("ascii write");
            oxideav_fbx::ascii::parse(&text).expect("ascii re-parse")
        };
        let (a, b) = (render(loose, &doc.root), render(loose, &again.root));
        if a != b {
            let at = a
                .bytes()
                .zip(b.bytes())
                .position(|(x, y)| x != y)
                .unwrap_or(a.len().min(b.len()));
            let from = a.floor_char_boundary(at.saturating_sub(120));
            panic!(
                "{name}: cross-form round trip differs at byte {at}:\n  {}\n  {}",
                &a[from..a.floor_char_boundary((at + 80).min(a.len()))],
                &b[from.min(b.len())..b.floor_char_boundary((at + 80).min(b.len()))]
            );
        }
    }
}

/// `docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §1.8: the
/// extrapolation `Type` is one byte holding an ASCII letter (binary
/// `C`), a bare unquoted letter in ASCII — and both forms keep it.
#[test]
fn extrapolation_type_letters_survive_both_forms() {
    for name in [
        "keyattr-sweep-binary-v7400.fbx",
        "keyattr-sweep-binary-v7500.fbx",
        "keyattr-sweep-ascii-v7500.fbx",
    ] {
        let doc = parse_any(&read("fbxsdk-keyattr-sweep", name));
        let objects = doc.root.child("Objects").expect("Objects");
        let mut letters = Vec::new();
        for curve in objects.children_named("AnimationCurve") {
            for which in ["Pre-Extrapolation", "Post-Extrapolation"] {
                if let Some(e) = curve.child(which) {
                    let ty = &e.child("Type").expect("Type").properties[0];
                    let rep = e.child("Repetition").expect("Repetition").properties[0]
                        .as_i64()
                        .expect("Repetition");
                    match ty {
                        FbxProperty::Char(c) => {
                            letters.push((which.starts_with("Pre"), char::from(*c), rep))
                        }
                        other => panic!("{name}: Type is {other:?}"),
                    }
                }
            }
        }
        // notes.md "Extrapolation records": E01 pre R(3) post K(0),
        // E03 post R(0) only, E02 pre M(0) post A(2) — in file order.
        assert_eq!(
            letters,
            vec![
                (true, 'R', 3),
                (false, 'K', 0),
                (false, 'R', 0),
                (true, 'M', 0),
                (false, 'A', 2),
            ],
            "{name}"
        );
    }
}

/// §1.3 "ASCII rendition": `KeyAttrDataFloat` prints each word as the
/// signed decimal integer of its bit pattern; the reader surfaces the
/// same `f` words the binary form carries.
#[test]
fn ascii_key_attr_data_words_are_bit_patterns() {
    let ascii = parse_any(&read(
        "fbxsdk-keyattr-sweep",
        "keyattr-sweep-ascii-v7500.fbx",
    ));
    let binary = parse_any(&read(
        "fbxsdk-keyattr-sweep",
        "keyattr-sweep-binary-v7400.fbx",
    ));
    let words = |doc: &FbxDocument| -> Vec<Vec<u32>> {
        doc.root
            .child("Objects")
            .expect("Objects")
            .children_named("AnimationCurve")
            .map(
                |c| match &c.child("KeyAttrDataFloat").expect("data").properties[0] {
                    FbxProperty::F32Array(a) => a.iter().map(|v| v.to_bits()).collect(),
                    other => panic!("KeyAttrDataFloat is {other:?}"),
                },
            )
            .collect()
    };
    let (a, b) = (words(&ascii), words(&binary));
    assert_eq!(a.len(), 62);
    assert_eq!(a, b);
    // I06 (sixth curve): 1.5, -2.5, 0x0D050D05, 0 per key.
    assert_eq!(&a[5][..4], &[0x3FC0_0000, 0xC020_0000, 0x0D05_0D05, 0]);
}
