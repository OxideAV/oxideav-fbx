//! `Constraint` object grammar — decode + re-encode per
//! `docs/3d/fbx/fbx-constraint-grammar.md`.
//!
//! A `Constraint` is an ordinary top-level `Objects` record (doc §1)
//! with the header shape `Constraint: <id>, "Constraint::<name>",
//! "<SubType>"` where the sub-type is a **human-readable display
//! string** (`"Single Chain IK"`, with spaces), repeated in an inner
//! `Type:` leaf (doc §2). Its own `Properties70` block carries only
//! the records that differ from the per-kind `PropertyTemplate`
//! (property names contain spaces — `"First Joint"` — and `"Weight"`
//! is its own property *type* string), and — the load-bearing
//! structural fact of doc §3 — **its targets are not in
//! `Properties70` at all**: `"object"`-typed records are value-less
//! *slots*, filled by `Connections` `OP` records
//! `C: "OP", <sourceObjectId>, <constraintId>, "<slot name>"`.
//!
//! `Definitions` carries `ObjectType: "Constraint"` with **one
//! template per kind present in the scene**, each named for the
//! concrete class (`FbxConstraintSingleChainIK`, and by the same
//! pattern `FbxConstraintAim` / `FbxConstraintParent` /
//! `FbxConstraintPosition` / `FbxConstraintRotation` /
//! `FbxConstraintScale` — doc §1).
//!
//! # Scene surface
//!
//! [`oxideav_mesh3d`] has no first-class constraint type, so the
//! decode surfaces the grammar losslessly on `Scene3D::extras`:
//!
//! - `"fbx:constraints"` — one JSON object per `Constraint` element:
//!   `{ name, kind, type?, multi_layer?, properties: [<raw P
//!   record>…], targets: [{ slot, node: <scene node index> } |
//!   { slot, object: <element name> }…] }`. `properties` keeps the
//!   object's **own** records verbatim (wire-typed value tags, see
//!   below) so re-encode reproduces the authored
//!   differs-from-template set rather than a resolved expansion.
//! - `"fbx:constraint_templates"` — the per-kind `PropertyTemplate`
//!   bodies from `Definitions`, `{ name, properties: [<raw P
//!   record>…] }` each, so the one-template-per-kind rule survives
//!   the round trip.
//!
//! A raw P record serialises as `{ name, type, label, flags,
//! values: [<tagged>…] }` with each value kind-tagged — `{"c": bool}`
//! / `{"l": integer}` / `{"d": float}` / `{"s": string}`. The tags
//! are deliberately **width-normalised** (every integer wire variant
//! `Y`/`I`/`L` lands on `"l"`, every float on `"d"`): the ASCII
//! front-end has no width information in its scalar grammar, so
//! width-preserving tags would make the same constraint decode to
//! different catalogues from the two encodings. Re-emission is
//! deterministic — integers write as `I` when they fit `i32` (the
//! wire form docs §4 records for `enum` / `int` / `bool`) and `L`
//! otherwise; floats write as `D`.
//!
//! # `MarkerSet`
//!
//! The companion ask is answered by doc §5: there is **no
//! `MarkerSet` object type in the FBX file grammar** (the token only
//! occurs inside MotionBuilder blind-data strings), so there is
//! nothing for this crate to implement; the character / control-rig
//! object family it was reaching for stays a docs acquisition item.

use std::collections::HashMap;

use oxideav_mesh3d::{NodeId, Scene3D};
use serde_json::{json, Map, Value};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};

/// `Scene3D::extras` key for the constraint catalogue.
pub const CONSTRAINTS_KEY: &str = "fbx:constraints";
/// `Scene3D::extras` key for the per-kind `PropertyTemplate` bodies.
pub const CONSTRAINT_TEMPLATES_KEY: &str = "fbx:constraint_templates";

/// The concrete template class name for a constraint kind display
/// string, per the doc §1 naming pattern — `"Single Chain IK"` ↔
/// `FbxConstraintSingleChainIK`, and by the same pattern
/// `"Aim"` → `FbxConstraintAim`, `"Parent"` → `FbxConstraintParent`,
/// … (the display string with its spaces removed, prefixed
/// `FbxConstraint`).
pub fn template_class_for_kind(kind: &str) -> String {
    let mut out = String::from("FbxConstraint");
    out.extend(kind.split_whitespace());
    out
}

/// Decode every `Objects { Constraint }` element (plus the
/// `Definitions` per-kind templates) onto `scene.extras` — see the
/// module docs for the surface shape. Returns the number of
/// constraints surfaced.
pub fn extract_constraints(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) -> usize {
    let Some(objects) = doc.root.child("Objects") else {
        return 0;
    };

    // Constraint elements, keyed by FBX id, in document order.
    let mut constraints: Vec<(i64, &FbxNode)> = Vec::new();
    // Element display names for non-Model target resolution.
    let mut element_names: HashMap<i64, String> = HashMap::new();
    for child in &objects.children {
        let Some(id) = element_id(child) else {
            continue;
        };
        if child.name == "Constraint" {
            constraints.push((id, child));
        }
        if let Some(name) = element_name(child) {
            element_names.insert(id, name);
        }
    }
    if constraints.is_empty() {
        return 0;
    }

    // OP edges whose destination is a constraint: doc §3
    // `C: "OP", <sourceObjectId>, <constraintId>, "<slot name>"`.
    let mut targets: HashMap<i64, Vec<(String, i64)>> = HashMap::new();
    let constraint_ids: std::collections::HashSet<i64> =
        constraints.iter().map(|(id, _)| *id).collect();
    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let prop = c.properties.get(3).and_then(FbxProperty::as_str);
            let (Some("OP"), Some(child_id), Some(parent_id), Some(prop)) =
                (kind, child_id, parent_id, prop)
            else {
                continue;
            };
            if constraint_ids.contains(&parent_id) {
                targets
                    .entry(parent_id)
                    .or_default()
                    .push((prop.to_owned(), child_id));
            }
        }
    }

    let mut entries: Vec<Value> = Vec::new();
    for (id, node) in &constraints {
        let mut entry = Map::new();
        entry.insert(
            "name".into(),
            Value::String(element_names.get(id).cloned().unwrap_or_default()),
        );
        // Header prop2 — the kind display string (doc §2).
        let kind = node
            .properties
            .get(2)
            .and_then(FbxProperty::as_str)
            .unwrap_or_default();
        entry.insert("kind".into(), Value::String(kind.to_owned()));
        // Inner `Type:` leaf (observed carrying the same display
        // string; kept separately so a divergent file round-trips).
        if let Some(t) = node
            .child("Type")
            .and_then(|n| n.properties.first())
            .and_then(FbxProperty::as_str)
        {
            entry.insert("type".into(), Value::String(t.to_owned()));
        }
        if let Some(ml) = node
            .child("MultiLayer")
            .and_then(|n| n.properties.first())
            .and_then(FbxProperty::as_i64)
        {
            entry.insert("multi_layer".into(), Value::from(ml));
        }
        // Own P records, verbatim (the differs-from-template set).
        let props = node
            .child("Properties70")
            .map(|p70| {
                p70.children_named("P")
                    .filter_map(p_record_to_json)
                    .collect::<Vec<Value>>()
            })
            .unwrap_or_default();
        entry.insert("properties".into(), Value::Array(props));
        // Targets: slot name + resolved endpoint.
        let mut tj: Vec<Value> = Vec::new();
        for (slot, source_id) in targets.get(id).into_iter().flatten() {
            let mut t = Map::new();
            t.insert("slot".into(), Value::String(slot.clone()));
            if let Some(nid) = model_nodes.get(source_id) {
                t.insert("node".into(), Value::from(nid.0));
            } else if let Some(name) = element_names.get(source_id) {
                t.insert("object".into(), Value::String(name.clone()));
            } else {
                continue;
            }
            tj.push(Value::Object(t));
        }
        entry.insert("targets".into(), Value::Array(tj));
        entries.push(Value::Object(entry));
    }

    let count = entries.len();
    scene
        .extras
        .entry(CONSTRAINTS_KEY.to_string())
        .or_insert(Value::Array(entries));

    // Per-kind Definitions templates (doc §1: one PropertyTemplate
    // per kind under ObjectType "Constraint").
    let templates: Vec<Value> = doc
        .root
        .child("Definitions")
        .into_iter()
        .flat_map(|d| d.children_named("ObjectType"))
        .filter(|ot| ot.properties.first().and_then(FbxProperty::as_str) == Some("Constraint"))
        .flat_map(|ot| ot.children_named("PropertyTemplate"))
        .map(|tpl| {
            let name = tpl
                .properties
                .first()
                .and_then(FbxProperty::as_str)
                .unwrap_or_default();
            let records: Vec<Value> = tpl
                .child("Properties70")
                .map(|p70| {
                    p70.children_named("P")
                        .filter_map(p_record_to_json)
                        .collect()
                })
                .unwrap_or_default();
            json!({ "name": name, "properties": records })
        })
        .collect();
    if !templates.is_empty() {
        scene
            .extras
            .entry(CONSTRAINT_TEMPLATES_KEY.to_string())
            .or_insert(Value::Array(templates));
    }

    count
}

// ---- encode side ------------------------------------------------------

/// Objects + Connections records rebuilt from the
/// `fbx:constraints` extras — the encode-side inverse of
/// [`extract_constraints`]. `node_fbx_id(index)` resolves a scene
/// node index to the FBX `Model` id the caller allocated; `alloc`
/// hands out fresh ids for the constraint elements. Targets recorded
/// as `{ slot, node }` re-emit their OP edge; `{ slot, object }`
/// targets (non-Model endpoints, unobserved in the staged grammar's
/// corpus for anything but joints) are skipped.
pub(crate) fn build_constraint_objects(
    scene: &Scene3D,
    node_fbx_id: impl Fn(usize) -> Option<i64>,
    mut alloc: impl FnMut() -> i64,
) -> (Vec<FbxNode>, Vec<FbxNode>) {
    let Some(entries) = scene.extras.get(CONSTRAINTS_KEY).and_then(Value::as_array) else {
        return (Vec::new(), Vec::new());
    };
    let mut objects = Vec::new();
    let mut connections = Vec::new();
    for entry in entries {
        let Some(entry) = entry.as_object() else {
            continue;
        };
        let name = entry.get("name").and_then(Value::as_str).unwrap_or("");
        let kind = entry.get("kind").and_then(Value::as_str).unwrap_or("");
        let id = alloc();

        let mut children = Vec::new();
        if let Some(t) = entry.get("type").and_then(Value::as_str) {
            children.push(FbxNode {
                name: "Type".to_string(),
                properties: vec![FbxProperty::String(t.as_bytes().to_vec())],
                children: Vec::new(),
            });
        }
        if let Some(ml) = entry.get("multi_layer").and_then(Value::as_i64) {
            children.push(FbxNode {
                name: "MultiLayer".to_string(),
                properties: vec![FbxProperty::I32(ml as i32)],
                children: Vec::new(),
            });
        }
        let records: Vec<FbxNode> = entry
            .get("properties")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
            .filter_map(json_to_p_record)
            .collect();
        if !records.is_empty() {
            children.push(FbxNode {
                name: "Properties70".to_string(),
                properties: Vec::new(),
                children: records,
            });
        }

        let mut display = name.as_bytes().to_vec();
        display.push(0x00);
        display.push(0x01);
        display.extend_from_slice(b"Constraint");
        objects.push(FbxNode {
            name: "Constraint".to_string(),
            properties: vec![
                FbxProperty::I64(id),
                FbxProperty::String(display),
                FbxProperty::String(kind.as_bytes().to_vec()),
            ],
            children,
        });

        for target in entry
            .get("targets")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
        {
            let Some(slot) = target.get("slot").and_then(Value::as_str) else {
                continue;
            };
            let Some(source_fbx) = target
                .get("node")
                .and_then(Value::as_u64)
                .and_then(|ni| node_fbx_id(ni as usize))
            else {
                continue;
            };
            connections.push(FbxNode {
                name: "C".to_string(),
                properties: vec![
                    FbxProperty::String(b"OP".to_vec()),
                    FbxProperty::I64(source_fbx),
                    FbxProperty::I64(id),
                    FbxProperty::String(slot.as_bytes().to_vec()),
                ],
                children: Vec::new(),
            });
        }
    }
    (objects, connections)
}

/// The `PropertyTemplate` nodes for `ObjectType: "Constraint"`,
/// rebuilt from the `fbx:constraint_templates` extras (one per kind,
/// doc §1).
pub(crate) fn constraint_template_nodes(scene: &Scene3D) -> Vec<FbxNode> {
    scene
        .extras
        .get(CONSTRAINT_TEMPLATES_KEY)
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|tpl| {
            let name = tpl.get("name").and_then(Value::as_str)?;
            let records: Vec<FbxNode> = tpl
                .get("properties")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(json_to_p_record)
                .collect();
            Some(FbxNode {
                name: "PropertyTemplate".to_string(),
                properties: vec![FbxProperty::String(name.as_bytes().to_vec())],
                children: vec![FbxNode {
                    name: "Properties70".to_string(),
                    properties: Vec::new(),
                    children: records,
                }],
            })
        })
        .collect()
}

// ---- raw P record <-> JSON -------------------------------------------

/// One `P` record as JSON — `{ name, type, label, flags, values }`
/// with wire-tagged values. `None` for records not matching the docs
/// §4 four-leading-strings shape.
fn p_record_to_json(p: &FbxNode) -> Option<Value> {
    let mut strings = p.properties.iter().take(4).map(|v| match v {
        FbxProperty::String(b) => Some(String::from_utf8_lossy(b).into_owned()),
        _ => None,
    });
    let name = strings.next()??;
    let type_name = strings.next()??;
    let label = strings.next()??;
    let flags = strings.next()??;
    let values: Vec<Value> = p
        .properties
        .iter()
        .skip(4)
        .filter_map(|v| {
            Some(match v {
                // Width-normalised tags — see the module docs.
                FbxProperty::Bool(b) => json!({ "c": b }),
                FbxProperty::I16(n) => json!({ "l": n }),
                FbxProperty::I32(n) => json!({ "l": n }),
                FbxProperty::I64(n) => json!({ "l": n }),
                FbxProperty::F32(x) => json!({ "d": x }),
                FbxProperty::F64(x) => json!({ "d": x }),
                FbxProperty::String(b) => json!({ "s": String::from_utf8_lossy(b) }),
                // Array / raw payloads do not occur in P records
                // (docs §4 value grammar).
                _ => return None,
            })
        })
        .collect();
    Some(json!({
        "name": name,
        "type": type_name,
        "label": label,
        "flags": flags,
        "values": values,
    }))
}

/// Inverse of [`p_record_to_json`].
fn json_to_p_record(v: &Value) -> Option<FbxNode> {
    let obj = v.as_object()?;
    let s = |key: &str| {
        obj.get(key)
            .and_then(Value::as_str)
            .map(|s| FbxProperty::String(s.as_bytes().to_vec()))
    };
    let mut properties = vec![s("name")?, s("type")?, s("label")?, s("flags")?];
    for value in obj
        .get("values")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        let tagged = value.as_object()?;
        let (tag, inner) = tagged.iter().next()?;
        let prop = match tag.as_str() {
            "c" => FbxProperty::Bool(inner.as_bool()?),
            // Integers re-emit as `I` when they fit (the docs §4
            // wire form for `enum` / `int` / `bool`), `L` otherwise.
            "l" => {
                let n = inner.as_i64()?;
                match i32::try_from(n) {
                    Ok(narrow) => FbxProperty::I32(narrow),
                    Err(_) => FbxProperty::I64(n),
                }
            }
            "d" => FbxProperty::F64(inner.as_f64()?),
            "s" => FbxProperty::String(inner.as_str()?.as_bytes().to_vec()),
            _ => return None,
        };
        properties.push(prop);
    }
    Some(FbxNode {
        name: "P".to_string(),
        properties,
        children: Vec::new(),
    })
}

/// Read property[0] (the FBX element id) of an `Objects`-child record.
fn element_id(n: &FbxNode) -> Option<i64> {
    n.properties.first().and_then(FbxProperty::as_i64)
}

/// Property[1] with the binary `\x00\x01` Name/Class join stripped.
fn element_name(n: &FbxNode) -> Option<String> {
    let raw = match n.properties.get(1)? {
        FbxProperty::String(b) => b,
        _ => return None,
    };
    if let Some(sep) = raw.iter().position(|&b| b == 0x00) {
        std::str::from_utf8(&raw[..sep]).ok().map(str::to_owned)
    } else {
        std::str::from_utf8(raw).ok().map(str::to_owned)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn template_class_naming_pattern() {
        assert_eq!(
            template_class_for_kind("Single Chain IK"),
            "FbxConstraintSingleChainIK"
        );
        assert_eq!(template_class_for_kind("Aim"), "FbxConstraintAim");
        assert_eq!(template_class_for_kind("Parent"), "FbxConstraintParent");
        assert_eq!(template_class_for_kind("Position"), "FbxConstraintPosition");
    }

    #[test]
    fn p_record_json_round_trips_every_wire_tag() {
        let p = FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(b"PoleVector".to_vec()),
                FbxProperty::String(b"Vector".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"A".to_vec()),
                FbxProperty::F64(0.0),
                FbxProperty::F64(1.0),
                FbxProperty::F64(0.5),
            ],
            children: Vec::new(),
        };
        let back = json_to_p_record(&p_record_to_json(&p).unwrap()).unwrap();
        assert_eq!(back.name, p.name);
        assert_eq!(back.properties, p.properties);

        // Every scalar wire kind survives with width-normalised
        // tags: `Y`/`I` (and small `L`) re-emit as `I`, wide `L`
        // stays `L`, `F` widens to `D` losslessly, `C`/`S` verbatim.
        let scalar_kinds = FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(b"Mixed".to_vec()),
                FbxProperty::String(b"enum".to_vec()),
                FbxProperty::String(b"label".to_vec()),
                FbxProperty::String(b"AU".to_vec()),
                FbxProperty::I16(-3),
                FbxProperty::Bool(true),
                FbxProperty::I32(7),
                FbxProperty::I64(1 << 40),
                FbxProperty::F32(2.5),
                FbxProperty::F64(-0.125),
                FbxProperty::String(b"text".to_vec()),
            ],
            children: Vec::new(),
        };
        let back = json_to_p_record(&p_record_to_json(&scalar_kinds).unwrap()).unwrap();
        assert_eq!(
            back.properties[4..],
            [
                FbxProperty::I32(-3),
                FbxProperty::Bool(true),
                FbxProperty::I32(7),
                FbxProperty::I64(1 << 40),
                FbxProperty::F64(2.5),
                FbxProperty::F64(-0.125),
                FbxProperty::String(b"text".to_vec()),
            ]
        );
        // And the normalised form is a fixed point: a second pass
        // reproduces it exactly.
        let twice = json_to_p_record(&p_record_to_json(&back).unwrap()).unwrap();
        assert_eq!(twice.properties, back.properties);
    }

    #[test]
    fn value_less_object_slot_round_trips() {
        // Doc §3: `"object"`-typed properties have an empty value —
        // they are slots.
        let p = FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(b"First Joint".to_vec()),
                FbxProperty::String(b"object".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"".to_vec()),
            ],
            children: Vec::new(),
        };
        let j = p_record_to_json(&p).unwrap();
        assert_eq!(j["name"], "First Joint");
        assert_eq!(j["values"].as_array().unwrap().len(), 0);
        let back = json_to_p_record(&j).unwrap();
        assert_eq!(back.properties, p.properties);
    }
}
