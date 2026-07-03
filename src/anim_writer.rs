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
//!   one `AnimationCurveNode` per channel, OP-connected to the target
//!   node's first `BlendShapeChannel` (emitted by
//!   [`crate::deformer_writer`]) under the `"DeformPercent"` property
//!   name, with a single `"d|DeformPercent"` curve carrying the raw
//!   scalar keyframes (the decode side reads them back verbatim — no
//!   scaling is applied in either direction).
//!
//! # Time units
//!
//! `KeyTime` is FBX KTime ticks — seconds × [`KTIME_TICKS_PER_SECOND`],
//! rounded to the nearest tick and stored as an `l` (i64) array. The
//! decode path divides back by the same constant.

use oxideav_mesh3d::{Animation, AnimationProperty, AnimationValues, NodeId};

use crate::animation::KTIME_TICKS_PER_SECOND;
use crate::binary::{FbxNode, FbxProperty};
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
/// Model record). `morph_channel_id(node_id)` resolves a node to its
/// first emitted `BlendShapeChannel` element id (the `DeformPercent`
/// OP target for MorphWeights channels). `alloc` hands out fresh FBX
/// ids for the animation elements.
pub(crate) fn build_animation_objects(
    animations: &[Animation],
    node_fbx_id: impl Fn(NodeId) -> Option<i64>,
    morph_channel_id: impl Fn(NodeId) -> Option<i64>,
    mut alloc: impl FnMut() -> i64,
) -> AnimEmit {
    let mut objects = Vec::new();
    let mut connections = Vec::new();

    for anim in animations {
        let stack_id = alloc();
        objects.push(element(
            "AnimationStack",
            stack_id,
            anim.name.as_deref().unwrap_or(""),
            "",
            Vec::new(),
        ));
        let layer_id = alloc();
        objects.push(element(
            "AnimationLayer",
            layer_id,
            "BaseLayer",
            "",
            Vec::new(),
        ));
        // AnimationLayer -> AnimationStack OO.
        connections.push(conn_oo(layer_id, stack_id));

        for ch in &anim.channels {
            let target_prop = match ch.target.property {
                AnimationProperty::Translation => "Lcl Translation",
                AnimationProperty::Rotation => "Lcl Rotation",
                AnimationProperty::Scale => "Lcl Scaling",
                // MorphWeights — a single-curve DeformPercent channel
                // targeting the node's BlendShapeChannel deformer.
                AnimationProperty::MorphWeights => {
                    let Some(channel_fbx) = morph_channel_id(ch.target.node) else {
                        continue;
                    };
                    let AnimationValues::Scalar(vals) = &ch.sampler.values else {
                        continue;
                    };
                    let times = &ch.sampler.keyframes;
                    if vals.len() != times.len() {
                        continue;
                    }
                    let curve_node_id = alloc();
                    objects.push(element(
                        "AnimationCurveNode",
                        curve_node_id,
                        "DeformPercent",
                        "",
                        Vec::new(),
                    ));
                    // AnimationCurveNode -> BlendShapeChannel OP.
                    connections.push(conn_op(curve_node_id, channel_fbx, "DeformPercent"));
                    // AnimationCurveNode -> AnimationLayer OO.
                    connections.push(conn_oo(curve_node_id, layer_id));
                    let curve_id = alloc();
                    objects.push(build_curve(curve_id, times, vals));
                    connections.push(conn_op(curve_id, curve_node_id, "d|DeformPercent"));
                    continue;
                }
            };
            let model_id = match node_fbx_id(ch.target.node) {
                Some(id) => id,
                None => continue,
            };

            // Per-axis (X/Y/Z) component series for this channel.
            let times = &ch.sampler.keyframes;
            let components = match channel_components(&ch.sampler.values, times.len()) {
                Some(c) => c,
                None => continue,
            };

            let curve_node_id = alloc();
            objects.push(element(
                "AnimationCurveNode",
                curve_node_id,
                target_prop,
                "",
                Vec::new(),
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
                objects.push(build_curve(curve_id, times, values));
                // AnimationCurve -> AnimationCurveNode OP (the axis tag).
                connections.push(conn_op(curve_id, curve_node_id, axis_tag));
            }
        }
    }

    AnimEmit {
        objects,
        connections,
    }
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
fn build_curve(id: i64, times_secs: &[f32], values: &[f32]) -> FbxNode {
    let key_times: Vec<i64> = times_secs
        .iter()
        .map(|t| (*t as f64 * KTIME_TICKS_PER_SECOND).round() as i64)
        .collect();
    FbxNode {
        name: "AnimationCurve".to_string(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(name_class("", "AnimCurve")),
            FbxProperty::String(Vec::new()),
        ],
        children: vec![
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
        ],
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
