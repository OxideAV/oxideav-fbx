//! Animation-curve emission for the [`crate::scene_writer`] encoder.
//!
//! The inverse of [`crate::animation::extract_animations`]: turns each
//! [`oxideav_mesh3d::Animation`] into the FBX
//! `AnimationStack` / `AnimationLayer` / `AnimationCurveNode` /
//! `AnimationCurve` object graph + the `Connections` `OO` / `OP` chain
//! the decode path walks (per `docs/3d/fbx/fbx-binary-properties70.md`
//! §5–§7):
//!
//! ```text
//! AnimationStack  --(OO, child=layer)--> (this stack)
//! AnimationLayer  --(OO)--> AnimationStack
//! AnimationCurveNode --(OP, "Lcl Translation"/"Lcl Rotation"/"Lcl Scaling")--> Model
//! AnimationCurveNode --(OO)--> AnimationLayer
//! AnimationCurve  --(OP, "d|X"/"d|Y"/"d|Z")--> AnimationCurveNode
//! ```
//!
//! # Channel value layout
//!
//! - **Translation / Scale** ([`oxideav_mesh3d::AnimationValues::Vec3`])
//!   split into three `AnimationCurve` records (`d|X` / `d|Y` / `d|Z`),
//!   each carrying the per-keyframe component scalar.
//! - **Rotation** ([`oxideav_mesh3d::AnimationValues::Quat`]) — each
//!   quaternion keyframe is converted to XYZ-Euler degrees (the inverse
//!   of [`crate::animation::euler_xyz_to_quat`], the convention the
//!   decode path reads) and split into the same three component curves.
//! - **MorphWeights** ([`oxideav_mesh3d::AnimationValues::Scalar`]) —
//!   the value table is strided by the target node's morph-target
//!   count (the `oxideav_mesh3d` sampler contract: one weight per
//!   target per keyframe). One `AnimationCurveNode` + one
//!   `"d|DeformPercent"` curve is emitted **per morph-target slot**,
//!   each OP-connected to the node's matching `BlendShapeChannel`
//!   (emitted by [`crate::deformer_writer`], in target order) under
//!   the `"DeformPercent"` property name. Weights are the 0..1 mesh3d
//!   blend factors; the wire curves carry 0..100 `DeformPercent`
//!   percentages (× [`DEFORM_PERCENT_SCALE`] on the way out, the
//!   decode side divides back).
//!
//! # Time units
//!
//! `KeyTime` is FBX KTime ticks — seconds × [`KTIME_TICKS_PER_SECOND`],
//! rounded to the nearest tick and stored as an `l` (i64) array. The
//! decode path divides back by the same constant.

use oxideav_mesh3d::{
    Animation, AnimationChannel, AnimationProperty, AnimationValues, NodeId, Scene3D,
};
use serde_json::Value;

use crate::animation::KTIME_TICKS_PER_SECOND;
use crate::binary::{FbxNode, FbxProperty};
use crate::deformer::DEFORM_PERCENT_SCALE;
use crate::node_transform::TransformChain;
use crate::scene_writer::quat_to_euler_xyz_deg_pub;

/// Output of [`build_animation_objects`]: the element records that go
/// into `Objects` plus the connection records that go into
/// `Connections`.
pub(crate) struct AnimEmit {
    pub objects: Vec<FbxNode>,
    pub connections: Vec<FbxNode>,
}

/// Build the FBX object graph for every [`Animation`] in the scene.
///
/// `node_fbx_id(node_id)` resolves a scene [`NodeId`] to the FBX
/// `Model` element id the [`crate::scene_writer`] assigned (so the
/// `AnimationCurveNode -> Model` OP connection points at the right
/// Model record). `morph_channel_ids(node_id)` resolves a node to its
/// emitted `BlendShapeChannel` element ids in morph-target order (the
/// `DeformPercent` OP targets for MorphWeights channels — one curve
/// per target slot). `alloc` hands out fresh FBX ids for the
/// animation elements.
pub(crate) fn build_animation_objects(
    scene: &Scene3D,
    node_fbx_id: impl Fn(NodeId) -> Option<i64>,
    morph_channel_ids: impl Fn(NodeId) -> Option<Vec<i64>>,
    node_chain: impl Fn(NodeId) -> Option<TransformChain>,
    mut alloc: impl FnMut() -> i64,
) -> AnimEmit {
    let animations = &scene.animations;
    let mut objects = Vec::new();
    let mut connections = Vec::new();

    // The round-tripped stack / layer catalogue
    // (`fbx:anim_stacks`): each stack's own records and layer list,
    // re-emitted verbatim; a stack with no typed `Animation` (a take
    // carrying no curves) is emitted from it too.
    let catalogue: Vec<&Value> = scene
        .extras
        .get(crate::animation::ANIM_STACKS_KEY)
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .collect();
    let catalogue_entry = |name: &str| -> Option<&Value> {
        catalogue
            .iter()
            .copied()
            .find(|e| e.get("name").and_then(Value::as_str) == Some(name))
    };
    // (stack name, layer name) → emitted layer id, for the auxiliary
    // curve nodes' membership below.
    let mut layer_ids: Vec<(String, String, i64)> = Vec::new();
    let key_attrs = KeyAttrs::from_scene(scene);
    let node_name = |nid: NodeId| -> Option<String> {
        scene.nodes.get(nid.0 as usize).and_then(|n| n.name.clone())
    };

    let mut emitted_stacks: Vec<String> = Vec::new();
    for anim in animations {
        let stack_name = anim.name.clone().unwrap_or_default();
        let entry = catalogue_entry(&stack_name);
        let stack_id = alloc();
        objects.push(element(
            "AnimationStack",
            stack_id,
            &stack_name,
            "",
            stack_body(entry, anim),
        ));
        emitted_stacks.push(stack_name.clone());
        // Layers: the catalogue's list (first hosts the channels),
        // else one synthesised `BaseLayer`.
        let layer_specs: Vec<(String, Vec<FbxNode>)> = entry
            .and_then(|e| e.get("layers"))
            .and_then(Value::as_array)
            .map(|ls| {
                ls.iter()
                    .map(|l| {
                        (
                            l.get("name")
                                .and_then(Value::as_str)
                                .unwrap_or("BaseLayer")
                                .to_owned(),
                            records_of(l),
                        )
                    })
                    .collect()
            })
            .filter(|v: &Vec<_>| !v.is_empty())
            .unwrap_or_else(|| vec![("BaseLayer".to_owned(), Vec::new())]);
        let mut first_layer = None;
        for (lname, lrecords) in layer_specs {
            let layer_id = alloc();
            objects.push(element(
                "AnimationLayer",
                layer_id,
                &lname,
                "",
                props70(lrecords),
            ));
            // AnimationLayer -> AnimationStack OO.
            connections.push(conn_oo(layer_id, stack_id));
            layer_ids.push((stack_name.clone(), lname, layer_id));
            first_layer.get_or_insert(layer_id);
        }
        let layer_id = first_layer.expect("at least one layer");

        // Group the T/R/S channels per target node (first-seen order
        // preserved) so a chain-bearing node's channels can be
        // de-composed jointly; morph channels emit inline.
        let mut trs_nodes: Vec<NodeId> = Vec::new();
        let mut trs_groups: Vec<[Option<&AnimationChannel>; 3]> = Vec::new();
        for ch in &anim.channels {
            let slot = match ch.target.property {
                AnimationProperty::Translation => 0usize,
                AnimationProperty::Rotation => 1,
                AnimationProperty::Scale => 2,
                // MorphWeights — one DeformPercent curve per
                // morph-target slot, each targeting the node's
                // matching BlendShapeChannel deformer.
                AnimationProperty::MorphWeights => {
                    let Some(channel_fbxs) = morph_channel_ids(ch.target.node) else {
                        continue;
                    };
                    let times = &ch.sampler.keyframes;
                    // Per the mesh3d sampler contract the value table
                    // is strided by the morph-target count; the typed
                    // read-back rejects a malformed table (`None`) and
                    // yields the authored *value* vector per keyframe
                    // for every interpolation — a CubicSpline sampler's
                    // tangent triples are not FBX curve keys, so its
                    // centre values are what the wire carries.
                    let Some(stride) = ch.sampler.morph_weight_stride() else {
                        continue;
                    };
                    let Some(frames) = ch.sampler.morph_weight_frames() else {
                        continue;
                    };
                    for (slot, channel_fbx) in channel_fbxs.into_iter().enumerate().take(stride) {
                        // 0..1 weights → 0..100 DeformPercent wire values.
                        let slot_vals: Vec<f32> = frames
                            .iter()
                            .map(|frame| (f64::from(frame[slot]) * DEFORM_PERCENT_SCALE) as f32)
                            .collect();
                        let curve_node_id = alloc();
                        objects.push(element(
                            "AnimationCurveNode",
                            curve_node_id,
                            "DeformPercent",
                            "",
                            props70(vec![p_number(
                                "d|DeformPercent",
                                f64::from(slot_vals.first().copied().unwrap_or(0.0)),
                            )]),
                        ));
                        // AnimationCurveNode -> BlendShapeChannel OP.
                        connections.push(conn_op(curve_node_id, channel_fbx, "DeformPercent"));
                        // AnimationCurveNode -> AnimationLayer OO.
                        connections.push(conn_oo(curve_node_id, layer_id));
                        let curve_id = alloc();
                        objects.push(build_curve(curve_id, times, &slot_vals, None));
                        connections.push(conn_op(curve_id, curve_node_id, "d|DeformPercent"));
                    }
                    continue;
                }
            };
            let idx = match trs_nodes.iter().position(|n| *n == ch.target.node) {
                Some(i) => i,
                None => {
                    trs_nodes.push(ch.target.node);
                    trs_groups.push([None; 3]);
                    trs_nodes.len() - 1
                }
            };
            trs_groups[idx][slot] = Some(ch);
        }

        for (node, group) in trs_nodes.iter().zip(&trs_groups) {
            let model_id = match node_fbx_id(*node) {
                Some(id) => id,
                None => continue,
            };
            let join = CurveJoin {
                stack: &stack_name,
                target: node_name(*node),
            };
            let emitted = match node_chain(*node) {
                Some(chain) => decompose_chain_curves(&chain, group),
                None => Vec::new(),
            };
            if !emitted.is_empty() {
                // Chain-bearing node: authored Lcl curves recovered
                // via TransformChain::decompose_sample.
                for (target_prop, times, components) in emitted {
                    emit_trs_curves(
                        &mut objects,
                        &mut connections,
                        &mut alloc,
                        layer_id,
                        model_id,
                        target_prop,
                        &times,
                        &components,
                        &key_attrs,
                        &join,
                    );
                }
                continue;
            }
            // Plain node: each channel's values emit verbatim.
            for (slot, ch) in group.iter().enumerate() {
                let Some(ch) = ch else { continue };
                let target_prop = ["Lcl Translation", "Lcl Rotation", "Lcl Scaling"][slot];
                let times = &ch.sampler.keyframes;
                let components = match channel_components(&ch.sampler.values, times.len()) {
                    Some(c) => c,
                    None => continue,
                };
                emit_trs_curves(
                    &mut objects,
                    &mut connections,
                    &mut alloc,
                    layer_id,
                    model_id,
                    target_prop,
                    times,
                    &components,
                    &key_attrs,
                    &join,
                );
            }
        }
    }

    // Catalogue stacks with no typed Animation — a take carrying no
    // curves — are emitted with their layers so the file's take set
    // (and the Definitions blocks it implies) survives.
    for entry in &catalogue {
        let name = entry.get("name").and_then(Value::as_str).unwrap_or("");
        if emitted_stacks.iter().any(|s| s == name) {
            continue;
        }
        let stack_id = alloc();
        objects.push(element(
            "AnimationStack",
            stack_id,
            name,
            "",
            props70(records_of(entry)),
        ));
        for l in entry
            .get("layers")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
        {
            let lname = l.get("name").and_then(Value::as_str).unwrap_or("BaseLayer");
            let layer_id = alloc();
            objects.push(element(
                "AnimationLayer",
                layer_id,
                lname,
                "",
                props70(records_of(l)),
            ));
            connections.push(conn_oo(layer_id, stack_id));
            layer_ids.push((name.to_owned(), lname.to_owned(), layer_id));
        }
    }

    // Auxiliary curve nodes (`fbx:aux_curve_nodes`), verbatim: the
    // node's records, its Model target + property (an `object`
    // target names an element this writer has no id for — its OP
    // edge is not re-created), its layer membership and every
    // `AnimationCurve` body.
    for aux in scene
        .extras
        .get(crate::animation::AUX_CURVE_NODES_KEY)
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        let cn_id = alloc();
        objects.push(element(
            "AnimationCurveNode",
            cn_id,
            aux.get("name").and_then(Value::as_str).unwrap_or(""),
            "",
            props70(records_of(aux)),
        ));
        if let (Some(nid), Some(prop)) = (
            aux.get("target")
                .and_then(|t| t.get("node"))
                .and_then(Value::as_u64),
            aux.get("property").and_then(Value::as_str),
        ) {
            if let Some(model_id) = node_fbx_id(NodeId(nid as u32)) {
                connections.push(conn_op(cn_id, model_id, prop));
            }
        }
        let stack = aux.get("stack").and_then(Value::as_str);
        let layer = aux.get("layer").and_then(Value::as_str);
        if let Some((_, _, lid)) = layer_ids
            .iter()
            .find(|(s, l, _)| Some(s.as_str()) == stack && Some(l.as_str()) == layer)
        {
            connections.push(conn_oo(cn_id, *lid));
        }
        for curve in aux
            .get("curves")
            .and_then(Value::as_array)
            .into_iter()
            .flatten()
        {
            let curve_id = alloc();
            let body: Vec<FbxNode> = curve
                .get("body")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(crate::properties70::json_to_body_leaf)
                .collect();
            let mut node = element(
                "AnimationCurve",
                curve_id,
                curve.get("name").and_then(Value::as_str).unwrap_or(""),
                "",
                body,
            );
            // The element class tag is `AnimCurve` (the decode side
            // keys on the node name, not the class).
            node.properties[1] = FbxProperty::String(name_class(
                curve.get("name").and_then(Value::as_str).unwrap_or(""),
                "AnimCurve",
            ));
            objects.push(node);
            if let Some(axis) = curve.get("axis").and_then(Value::as_str) {
                connections.push(conn_op(curve_id, cn_id, axis));
            }
        }
    }

    AnimEmit {
        objects,
        connections,
    }
}

/// The `Properties70` records of a catalogue entry (`"records"`).
fn records_of(entry: &Value) -> Vec<FbxNode> {
    entry
        .get("records")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(crate::properties70::json_to_p_record)
        .collect()
}

fn props70(records: Vec<FbxNode>) -> Vec<FbxNode> {
    if records.is_empty() {
        return Vec::new();
    }
    vec![FbxNode {
        name: "Properties70".to_string(),
        properties: Vec::new(),
        children: records,
    }]
}

/// An `AnimationStack`'s body: its round-tripped records when the
/// catalogue carries the stack, else the `LocalStart` / `LocalStop` /
/// `ReferenceStart` / `ReferenceStop` KTime span synthesised from the
/// channels' keyframe range (the records every staged producer
/// writes).
fn stack_body(entry: Option<&Value>, anim: &Animation) -> Vec<FbxNode> {
    if let Some(e) = entry {
        let raw = records_of(e);
        if !raw.is_empty() {
            return props70(raw);
        }
    }
    let mut lo = f32::INFINITY;
    let mut hi = f32::NEG_INFINITY;
    for ch in &anim.channels {
        for t in &ch.sampler.keyframes {
            lo = lo.min(*t);
            hi = hi.max(*t);
        }
    }
    if !(lo.is_finite() && hi.is_finite()) {
        return Vec::new();
    }
    let ticks = |t: f32| (f64::from(t) * KTIME_TICKS_PER_SECOND).round() as i64;
    props70(vec![
        p_ktime("LocalStart", ticks(lo)),
        p_ktime("LocalStop", ticks(hi)),
        p_ktime("ReferenceStart", ticks(lo)),
        p_ktime("ReferenceStop", ticks(hi)),
    ])
}

/// The `fbx:key_attrs` catalogue (see
/// [`crate::animation::extract_key_attr_catalogue`]) indexed for the
/// per-curve lookup: an entry re-emits verbatim onto the curve whose
/// stack / target / property / axis join it and whose emitted key
/// count equals the entry's — the decode side resamples per-axis
/// curves onto a union grid, so a curve whose grid changed keeps no
/// attributes (stretching an uninterpreted per-key table is exactly
/// what the open `KeyAttrFlags` docs gap forbids).
struct KeyAttrs<'a> {
    entries: Vec<&'a Value>,
}

struct CurveJoin<'a> {
    stack: &'a str,
    target: Option<String>,
}

impl<'a> KeyAttrs<'a> {
    fn from_scene(scene: &'a Scene3D) -> Self {
        Self {
            entries: scene
                .extras
                .get("fbx:key_attrs")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .collect(),
        }
    }

    fn entry(&self, join: &CurveJoin<'_>, prop: &str, axis: &str) -> Option<&'a Value> {
        self.entries.iter().copied().find(|e| {
            e.get("stack").and_then(Value::as_str) == Some(join.stack)
                && e.get("target").and_then(Value::as_str) == join.target.as_deref()
                && e.get("property").and_then(Value::as_str) == Some(prop)
                && e.get("axis").and_then(Value::as_str) == Some(axis)
        })
    }

    /// The source curve rebuilt verbatim (id left as 0 for the
    /// caller to set) when linearly sampling it on the union grid
    /// reproduces `values` to 1e-4 — i.e. the typed channel was not
    /// edited — else `None`.
    fn verbatim_curve(
        &self,
        join: &CurveJoin<'_>,
        prop: &str,
        axis: &str,
        times: &[f32],
        values: &[f32],
    ) -> Option<FbxNode> {
        let e = self.entry(join, prop, axis)?;
        let key_times: Vec<i64> = e
            .get("key_times")?
            .as_array()?
            .iter()
            .map(Value::as_i64)
            .collect::<Option<Vec<_>>>()?;
        let key_values: Vec<f32> = e
            .get("key_values")?
            .as_array()?
            .iter()
            .map(|v| {
                v.as_u64()
                    .and_then(|b| u32::try_from(b).ok())
                    .map(f32::from_bits)
            })
            .collect::<Option<Vec<_>>>()?;
        if key_times.is_empty() || key_times.len() != key_values.len() {
            return None;
        }
        let secs: Vec<f64> = key_times
            .iter()
            .map(|t| *t as f64 / KTIME_TICKS_PER_SECOND)
            .collect();
        for (t, v) in times.iter().zip(values) {
            let t = f64::from(*t);
            let sampled = if t <= secs[0] {
                f64::from(key_values[0])
            } else if t >= secs[secs.len() - 1] {
                f64::from(key_values[key_values.len() - 1])
            } else {
                let i = secs.partition_point(|s| *s <= t).max(1);
                let (t0, t1) = (secs[i - 1], secs[i]);
                let (v0, v1) = (f64::from(key_values[i - 1]), f64::from(key_values[i]));
                if t1 > t0 {
                    v0 + (v1 - v0) * (t - t0) / (t1 - t0)
                } else {
                    v1
                }
            };
            if (sampled - f64::from(*v)).abs() > 1.0e-4 * (1.0 + f64::from(v.abs())) {
                return None;
            }
        }
        let default = e
            .get("default")
            .and_then(Value::as_f64)
            .unwrap_or(f64::from(key_values[0]));
        let key_ver = e.get("key_ver").and_then(Value::as_i64).unwrap_or(4008) as i32;
        let mut children = vec![
            FbxNode {
                name: "Default".to_string(),
                properties: vec![FbxProperty::F64(default)],
                children: Vec::new(),
            },
            FbxNode {
                name: "KeyVer".to_string(),
                properties: vec![FbxProperty::I32(key_ver)],
                children: Vec::new(),
            },
            FbxNode {
                name: "KeyTime".to_string(),
                properties: vec![FbxProperty::I64Array(key_times.clone())],
                children: Vec::new(),
            },
            FbxNode {
                name: "KeyValueFloat".to_string(),
                properties: vec![FbxProperty::F32Array(key_values)],
                children: Vec::new(),
            },
        ];
        children.extend(self.leaves(join, prop, axis, key_times.len()));
        Some(FbxNode {
            name: "AnimationCurve".to_string(),
            properties: vec![
                FbxProperty::I64(0),
                FbxProperty::String(name_class("", "AnimCurve")),
                FbxProperty::String(Vec::new()),
            ],
            children,
        })
    }

    fn leaves(&self, join: &CurveJoin<'_>, prop: &str, axis: &str, n_keys: usize) -> Vec<FbxNode> {
        let Some(e) = self
            .entry(join, prop, axis)
            .filter(|e| e.get("key_count").and_then(Value::as_u64) == Some(n_keys as u64))
        else {
            return Vec::new();
        };
        let ints = |key: &str| -> Option<Vec<i32>> {
            e.get(key)?
                .as_array()?
                .iter()
                .map(|v| v.as_i64().and_then(|n| i32::try_from(n).ok()))
                .collect()
        };
        let mut out = Vec::new();
        if let Some(flags) = ints("flags") {
            out.push(FbxNode {
                name: "KeyAttrFlags".to_string(),
                properties: vec![FbxProperty::I32Array(flags)],
                children: Vec::new(),
            });
        }
        if let Some(bits) = e.get("data_bits").and_then(Value::as_array) {
            let floats: Option<Vec<f32>> = bits
                .iter()
                .map(|v| {
                    v.as_u64()
                        .and_then(|b| u32::try_from(b).ok())
                        .map(f32::from_bits)
                })
                .collect();
            if let Some(f) = floats {
                out.push(FbxNode {
                    name: "KeyAttrDataFloat".to_string(),
                    properties: vec![FbxProperty::F32Array(f)],
                    children: Vec::new(),
                });
            }
        }
        if let Some(rc) = ints("ref_count") {
            out.push(FbxNode {
                name: "KeyAttrRefCount".to_string(),
                properties: vec![FbxProperty::I32Array(rc)],
                children: Vec::new(),
            });
        }
        out
    }
}

/// Emit one `AnimationCurveNode` + three per-axis `AnimationCurve`s
/// with their `OO` / `OP` wiring. The curve node carries the
/// `d|X` / `d|Y` / `d|Z` default records (the first key of each
/// component — the shape every staged producer writes).
#[allow(clippy::too_many_arguments)]
fn emit_trs_curves(
    objects: &mut Vec<FbxNode>,
    connections: &mut Vec<FbxNode>,
    alloc: &mut impl FnMut() -> i64,
    layer_id: i64,
    model_id: i64,
    target_prop: &str,
    times: &[f32],
    components: &[Vec<f32>; 3],
    key_attrs: &KeyAttrs<'_>,
    join: &CurveJoin<'_>,
) {
    let curve_node_id = alloc();
    let defaults: Vec<FbxNode> = ["d|X", "d|Y", "d|Z"]
        .iter()
        .zip(components)
        .map(|(axis, vals)| p_number(axis, f64::from(vals.first().copied().unwrap_or(0.0))))
        .collect();
    objects.push(element(
        "AnimationCurveNode",
        curve_node_id,
        target_prop,
        "",
        props70(defaults),
    ));
    // AnimationCurveNode -> Model OP (the property name).
    connections.push(conn_op(curve_node_id, model_id, target_prop));
    // AnimationCurveNode -> AnimationLayer OO.
    connections.push(conn_oo(curve_node_id, layer_id));

    for (axis_tag, values) in [
        ("d|X", &components[0]),
        ("d|Y", &components[1]),
        ("d|Z", &components[2]),
    ] {
        let curve_id = alloc();
        // The source curve verbatim (its own key grid, values,
        // `Default` / `KeyVer` and `KeyAttr*` arrays) whenever the
        // typed channel still samples identically from it; else the
        // union-grid curve, with the attributes only when the grid is
        // unchanged.
        let curve = match key_attrs.verbatim_curve(join, target_prop, axis_tag, times, values) {
            Some(mut original) => {
                original.properties[0] = FbxProperty::I64(curve_id);
                original
            }
            None => {
                let attrs = key_attrs.leaves(join, target_prop, axis_tag, times.len());
                build_curve(curve_id, times, values, Some(attrs))
            }
        };
        objects.push(curve);
        // AnimationCurve -> AnimationCurveNode OP (the axis tag).
        connections.push(conn_op(curve_id, curve_node_id, axis_tag));
    }
}

/// De-compose a chain-bearing node's channel group back to authored
/// `Lcl` component curves.
///
/// The decode side composes `T(t)` / `R(t)` / `S(t)` through the doc
/// §1 product (`docs/3d/fbx/fbx-node-transform-chain.md`); since the
/// re-encoded `Model` carries its pivot / offset / Pre-/PostRotation
/// / `RotationOrder` records again, the curves must carry the
/// **authored** values — emitting the composed samples verbatim would
/// double-apply the chain on the next decode. Per union-grid
/// keyframe, [`TransformChain::decompose_sample`] inverts the closed
/// form (absent channels sample as the chain's static composition);
/// Euler components are unwrapped (±360°) to stay continuous across
/// keys. One `(property, times, components)` tuple is returned per
/// channel present in the group.
fn decompose_chain_curves(
    chain: &TransformChain,
    group: &[Option<&AnimationChannel>; 3],
) -> Vec<AuthoredCurveSet> {
    let [t_ch, r_ch, s_ch] = group;
    if t_ch.is_none() && r_ch.is_none() && s_ch.is_none() {
        return Vec::new();
    }

    // Union time grid over every present channel.
    let mut times: Vec<f32> = Vec::new();
    for ch in group.iter().flatten() {
        times.extend_from_slice(&ch.sampler.keyframes);
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if times.is_empty() {
        return Vec::new();
    }

    // Static composition backs any absent channel.
    let (static_t, static_q, static_s) = chain.compose();

    let mut lcl_t: Vec<[f64; 3]> = Vec::with_capacity(times.len());
    let mut lcl_r: Vec<[f64; 3]> = Vec::with_capacity(times.len());
    let mut lcl_s: Vec<[f64; 3]> = Vec::with_capacity(times.len());
    for &t in &times {
        let ct = sample_vec3_channel(*t_ch, t).unwrap_or(static_t);
        let cq = sample_quat_channel(*r_ch, t).unwrap_or(static_q);
        let cs = sample_vec3_channel(*s_ch, t).unwrap_or(static_s);
        let (at, ar, a_s) = chain.decompose_sample(ct, cq, cs);
        // Unwrap Euler components against the previous key so the
        // linearly-interpolated curve doesn't spin through ±360°.
        let ar = match lcl_r.last() {
            Some(prev) => {
                let mut u = ar;
                for i in 0..3 {
                    while u[i] - prev[i] > 180.0 {
                        u[i] -= 360.0;
                    }
                    while u[i] - prev[i] < -180.0 {
                        u[i] += 360.0;
                    }
                }
                u
            }
            None => ar,
        };
        lcl_t.push(at);
        lcl_r.push(ar);
        lcl_s.push(a_s);
    }

    let split = |vals: &[[f64; 3]]| -> [Vec<f32>; 3] {
        [
            vals.iter().map(|v| v[0] as f32).collect(),
            vals.iter().map(|v| v[1] as f32).collect(),
            vals.iter().map(|v| v[2] as f32).collect(),
        ]
    };

    let mut out = Vec::new();
    if t_ch.is_some() {
        out.push(("Lcl Translation", times.clone(), split(&lcl_t)));
    }
    if r_ch.is_some() {
        out.push(("Lcl Rotation", times.clone(), split(&lcl_r)));
    }
    if s_ch.is_some() {
        out.push(("Lcl Scaling", times.clone(), split(&lcl_s)));
    }
    out
}

/// One authored `Lcl` curve set ready for emission: the FBX target
/// property name, the union keyframe grid, and the three per-axis
/// component series.
type AuthoredCurveSet = (&'static str, Vec<f32>, [Vec<f32>; 3]);

/// Sample a `Vec3` channel at `t` (linear, endpoint-clamped).
/// `None` when the channel is absent or not `Vec3`-valued.
fn sample_vec3_channel(ch: Option<&AnimationChannel>, t: f32) -> Option<[f64; 3]> {
    let ch = ch?;
    let AnimationValues::Vec3(vals) = &ch.sampler.values else {
        return None;
    };
    let (i, j, frac) = bracket(&ch.sampler.keyframes, t)?;
    if vals.len() != ch.sampler.keyframes.len() {
        return None;
    }
    let (a, b) = (vals[i], vals[j]);
    Some([
        f64::from(a[0]) + f64::from(b[0] - a[0]) * frac,
        f64::from(a[1]) + f64::from(b[1] - a[1]) * frac,
        f64::from(a[2]) + f64::from(b[2] - a[2]) * frac,
    ])
}

/// Sample a `Quat` channel at `t` (normalised linear blend on the
/// short arc, endpoint-clamped). `None` when absent / not `Quat`.
fn sample_quat_channel(ch: Option<&AnimationChannel>, t: f32) -> Option<[f64; 4]> {
    let ch = ch?;
    let AnimationValues::Quat(vals) = &ch.sampler.values else {
        return None;
    };
    let (i, j, frac) = bracket(&ch.sampler.keyframes, t)?;
    if vals.len() != ch.sampler.keyframes.len() {
        return None;
    }
    let a = vals[i];
    let mut b = vals[j];
    let dot: f32 = (0..4).map(|k| a[k] * b[k]).sum();
    if dot < 0.0 {
        b = [-b[0], -b[1], -b[2], -b[3]];
    }
    let mut q = [0.0f64; 4];
    for k in 0..4 {
        q[k] = f64::from(a[k]) + f64::from(b[k] - a[k]) * frac;
    }
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm > 0.0 {
        for c in &mut q {
            *c /= norm;
        }
    }
    Some(q)
}

/// Locate `t` on a keyframe grid: returns `(i, j, frac)` with
/// endpoint clamping. `None` for an empty grid.
fn bracket(times: &[f32], t: f32) -> Option<(usize, usize, f64)> {
    if times.is_empty() {
        return None;
    }
    if t <= times[0] {
        return Some((0, 0, 0.0));
    }
    if t >= times[times.len() - 1] {
        let last = times.len() - 1;
        return Some((last, last, 0.0));
    }
    let mut i = 0;
    while i + 1 < times.len() && times[i + 1] < t {
        i += 1;
    }
    let (t0, t1) = (times[i], times[i + 1]);
    let frac = if t1 > t0 {
        f64::from(t - t0) / f64::from(t1 - t0)
    } else {
        0.0
    };
    Some((i, i + 1, frac))
}

/// Decompose a channel's [`AnimationValues`] into three per-keyframe
/// component series `[xs, ys, zs]`. Quaternion rotation channels are
/// converted to XYZ-Euler degrees per keyframe (the decode convention).
/// Returns `None` for a malformed sampler (length mismatch) or a
/// `Scalar` (morph) channel, which this writer doesn't emit.
fn channel_components(values: &AnimationValues, n_keys: usize) -> Option<[Vec<f32>; 3]> {
    match values {
        AnimationValues::Vec3(v) => {
            if v.len() != n_keys {
                return None;
            }
            let xs = v.iter().map(|p| p[0]).collect();
            let ys = v.iter().map(|p| p[1]).collect();
            let zs = v.iter().map(|p| p[2]).collect();
            Some([xs, ys, zs])
        }
        AnimationValues::Quat(q) => {
            if q.len() != n_keys {
                return None;
            }
            let mut xs = Vec::with_capacity(n_keys);
            let mut ys = Vec::with_capacity(n_keys);
            let mut zs = Vec::with_capacity(n_keys);
            for quat in q {
                let e = quat_to_euler_xyz_deg_pub(*quat);
                xs.push(e[0]);
                ys.push(e[1]);
                zs.push(e[2]);
            }
            Some([xs, ys, zs])
        }
        AnimationValues::Scalar(_) => None,
    }
}

/// Build one `AnimationCurve` element carrying a `KeyTime` (l-array,
/// KTime ticks) + `KeyValueFloat` (f-array) pair — the two sub-records
/// the decode path's `read_curve` requires.
fn build_curve(
    id: i64,
    times_secs: &[f32],
    values: &[f32],
    key_attrs: Option<Vec<FbxNode>>,
) -> FbxNode {
    let key_times: Vec<i64> = times_secs
        .iter()
        .map(|t| (*t as f64 * KTIME_TICKS_PER_SECOND).round() as i64)
        .collect();
    // Body shape observed on every staged curve: `Default` (the
    // curve's first value), `KeyVer: 4008`, then the key arrays and —
    // when the round-tripped catalogue supplies them — the `KeyAttr*`
    // arrays verbatim.
    let mut children = vec![
        FbxNode {
            name: "Default".to_string(),
            properties: vec![FbxProperty::F64(f64::from(
                values.first().copied().unwrap_or(0.0),
            ))],
            children: Vec::new(),
        },
        FbxNode {
            name: "KeyVer".to_string(),
            properties: vec![FbxProperty::I32(4008)],
            children: Vec::new(),
        },
        FbxNode {
            name: "KeyTime".to_string(),
            properties: vec![FbxProperty::I64Array(key_times)],
            children: Vec::new(),
        },
        FbxNode {
            name: "KeyValueFloat".to_string(),
            properties: vec![FbxProperty::F32Array(values.to_vec())],
            children: Vec::new(),
        },
    ];
    children.extend(key_attrs.into_iter().flatten());
    FbxNode {
        name: "AnimationCurve".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class("", "AnimCurve")),
            FbxProperty::String(Vec::new()),
        ],
        children,
    }
}

fn p_number(name: &str, v: f64) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"Number".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(v),
        ],
        children: Vec::new(),
    }
}

fn p_ktime(name: &str, v: i64) -> FbxNode {
    FbxNode {
        name: "P".to_string(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"KTime".to_vec()),
            FbxProperty::String(b"Time".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::I64(v),
        ],
        children: Vec::new(),
    }
}

/// Build a generic `Objects` element record with the `[id, name+class,
/// subtype]` property tuple.
fn element(node_name: &str, id: i64, name: &str, subtype: &str, extra: Vec<FbxNode>) -> FbxNode {
    let class = node_name;
    FbxNode {
        name: node_name.to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class(name, class)),
            FbxProperty::String(subtype.as_bytes().to_vec()),
        ],
        children: extra,
    }
}

/// `Name\x00\x01ClassTag` join (binary encoding; the decode path splits
/// on the `\x00`).
fn name_class(name: &str, class: &str) -> Vec<u8> {
    let mut v = name.as_bytes().to_vec();
    v.push(0x00);
    v.push(0x01);
    v.extend_from_slice(class.as_bytes());
    v
}

fn conn_oo(child_id: i64, parent_id: i64) -> FbxNode {
    FbxNode {
        name: "C".to_string(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(child_id),
            FbxProperty::I64(parent_id),
        ],
        children: Vec::new(),
    }
}

fn conn_op(child_id: i64, parent_id: i64, prop: &str) -> FbxNode {
    FbxNode {
        name: "C".to_string(),
        properties: vec![
            FbxProperty::String(b"OP".to_vec()),
            FbxProperty::I64(child_id),
            FbxProperty::I64(parent_id),
            FbxProperty::String(prop.as_bytes().to_vec()),
        ],
        children: Vec::new(),
    }
}
