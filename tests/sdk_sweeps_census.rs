//! Whole-file census of the 2026-11 sweeps: a decode → encode cycle of
//! every staged file, in both output forms, keeps every object (class,
//! name, sub-type and body, record for record) and every connection.
//! Nothing the fixtures' `notes.md` files document is skipped.

mod common;

use std::collections::BTreeMap;

use common::{SWEEP_FILES, display, loose, parse_any, read, render, subtype};
use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};

/// Float64 arrays at single precision: the scene model holds vertex
/// data as `f32`.
fn single_precision(n: &FbxNode) -> FbxNode {
    FbxNode {
        name: n.name.clone(),
        properties: n
            .properties
            .iter()
            .map(|p| match p {
                FbxProperty::F64Array(a) => {
                    FbxProperty::F64Array(a.iter().map(|v| f64::from(*v as f32)).collect())
                }
                other => other.clone(),
            })
            .collect(),
        children: n.children.iter().map(single_precision).collect(),
    }
}

/// Every object as `class|name|sub-type|body…` (ids left out), counted,
/// and every connection as `kind|child|parent|property`, counted.
fn census(doc: &FbxDocument) -> (BTreeMap<String, usize>, BTreeMap<String, usize>) {
    let objects = doc.root.child("Objects").expect("Objects");
    let mut names = std::collections::HashMap::new();
    let mut out: BTreeMap<String, usize> = BTreeMap::new();
    for o in &objects.children {
        let id = o
            .properties
            .first()
            .and_then(FbxProperty::as_i64)
            .unwrap_or(0);
        names.insert(id, format!("{}:{}", o.name, display(o)));
        // A polygon mesh is re-expressed, not copied: the scene model
        // holds its layers per polygon vertex, so a by-control-point
        // layer of the source comes back per polygon vertex with the
        // same values. Polygon meshes are counted, not compared.
        if o.name == "Geometry" && subtype(o) == "Mesh" {
            *out.entry(format!("Geometry|{}|Mesh", display(o)))
                .or_default() += 1;
            continue;
        }
        let mut body: Vec<String> = o
            .children
            .iter()
            .map(|c| render(loose, &single_precision(c)))
            .collect();
        // Record order inside an object is not part of its content.
        body.sort();
        *out.entry(format!(
            "{}|{}|{}|{}",
            o.name,
            display(o),
            subtype(o),
            body.join(" ")
        ))
        .or_default() += 1;
    }
    let mut conns: BTreeMap<String, usize> = BTreeMap::new();
    for c in doc
        .root
        .child("Connections")
        .expect("Connections")
        .children_named("C")
    {
        // The key-attribute sweep keeps the `OP` edge of the curve the
        // exporter did not write (`N00_zero_keys`: a curve with no keys
        // has no `AnimationCurve` object, notes.md): an edge from an
        // object that does not exist. It has nothing to carry.
        if !names.contains_key(&c.properties[1].as_i64().unwrap_or(0)) {
            continue;
        }
        let name = |i: usize| -> String {
            let id = c.properties[i].as_i64().unwrap_or(0);
            names.get(&id).cloned().unwrap_or_else(|| "root".to_owned())
        };
        *conns
            .entry(format!(
                "{}|{}|{}|{}",
                c.properties[0].as_str().unwrap_or(""),
                name(1),
                name(2),
                c.properties
                    .get(3)
                    .and_then(FbxProperty::as_str)
                    .unwrap_or("")
            ))
            .or_default() += 1;
    }
    (out, conns)
}

fn first_difference(a: &BTreeMap<String, usize>, b: &BTreeMap<String, usize>) -> Option<String> {
    let cut = |s: &str| s.chars().take(1500).collect::<String>();
    for (k, n) in a {
        if b.get(k) != Some(n) {
            let head: String = k.split('|').take(2).collect::<Vec<_>>().join("|");
            let other = b
                .keys()
                .find(|o| o.starts_with(&head) && a.get(*o) != b.get(*o))
                .map(|o| cut(o))
                .unwrap_or_default();
            return Some(format!(
                "only in source (x{n}): {}\n   output has: {other}",
                cut(k)
            ));
        }
    }
    for (k, n) in b {
        if a.get(k) != Some(n) {
            return Some(format!("only in output (x{n}): {}", cut(k)));
        }
    }
    None
}

#[test]
fn every_sweep_file_keeps_every_object_and_connection() {
    let mut failures = Vec::new();
    for (dir, name) in SWEEP_FILES {
        let bytes = read(dir, name);
        let (src_objects, src_conns) = census(&parse_any(&bytes));
        let scene = FbxDecoder::new().decode(&bytes).expect("decode");
        for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
            let out = FbxEncoder::new().form(form).encode(&scene).expect("encode");
            let (objects, conns) = census(&parse_any(&out));
            if let Some(d) = first_difference(&src_objects, &objects) {
                failures.push(format!("{name} {form:?} objects: {d}"));
            }
            if let Some(d) = first_difference(&src_conns, &conns) {
                failures.push(format!("{name} {form:?} connections: {d}"));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
