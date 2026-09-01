//! Animation extraction — `AnimationStack` / `AnimationLayer` /
//! `AnimationCurveNode` / `AnimationCurve` → [`oxideav_mesh3d::Animation`].
//!
//! Animation in an FBX file (per the object graph described in
//! `docs/3d/fbx/fbx-binary-properties70.md` §5–§7) is structured as:
//!
//! - `AnimationStack` — top-level "take" / clip. One stack per
//!   logical animation in the file.
//! - `AnimationLayer` — blend layer owned by a stack. Multiple layers
//!   on a stack composite into the final output; round 2 collapses
//!   every layer's channels into one [`oxideav_mesh3d::Animation`]
//!   keyed by stack name (the per-layer compositing semantics —
//!   weight / blend-mode / additive — are NYI; correct compositing
//!   needs full per-layer scene evaluation).
//! - `AnimationCurveNode` — parameter binding. Connects (a) one or
//!   more `AnimationCurve`s carrying x/y/z component scalars to (b)
//!   one named property on a target `Model` node (e.g.
//!   `Lcl Translation`, `Lcl Rotation`, `Lcl Scaling`,
//!   `DeformPercent`).
//! - `AnimationCurve` — the actual sampled keyframes:
//!   `KeyTime` (`l` array — i64 FBX tick stamps) +
//!   `KeyValueFloat` (`f` array — f32 sample values) and optional
//!   `KeyAttrFlags` / `KeyAttrDataFloat` interpolation metadata.
//!
//! # Connection topology
//!
//! Per `docs/3d/fbx/fbx-binary-properties70.md` §7, animation wiring is
//! conveyed by `OO` (object-object) and `OP` (object-property) records:
//!
//! ```text
//! AnimationCurve --(OP, "d|X" / "d|Y" / "d|Z" / "d|DeformPercent")--> AnimationCurveNode
//! AnimationCurveNode --(OP, "Lcl Translation" / "Lcl Rotation" / ...)--> Model
//! AnimationCurveNode --(OO)--> AnimationLayer
//! AnimationLayer --(OO)--> AnimationStack
//! ```
//!
//! For round 2 we recognise the three FBX node-transform properties
//! (`Lcl Translation`, `Lcl Rotation`, `Lcl Scaling`) and one
//! deformer-targeted property
//! (`DeformPercent`) used by morph-target animation. Other property
//! names round-trip via the underlying [`crate::FbxDocument`] but do
//! not surface as [`oxideav_mesh3d::AnimationChannel`]s.
//!
//! # Time units
//!
//! FBX `KeyTime` is stored in *KTime ticks* — a fixed-point integer
//! that 1 second = 46_186_158_000 ticks (this constant is FBX
//! tooling-community public knowledge; it appears in countless
//! glTF<->FBX bridges and authoring-tool exporter blog posts. We use
//! it here strictly as a numeric scalar — no Autodesk-licensed code
//! is referenced). [`oxideav_mesh3d::AnimationSampler::keyframes`] is
//! seconds (`f32`), so we divide by [`KTIME_TICKS_PER_SECOND`] when
//! materialising the sampler.
//!
//! # Rotation and the transform chain
//!
//! `Lcl Rotation` curves carry **Euler angles in degrees** per
//! `docs/3d/fbx/fbx-node-transform-chain.md`; the default rotation
//! order is XYZ when no `RotationOrder` property is set on the bound
//! Model. Rotation samplers surface as
//! [`oxideav_mesh3d::AnimationValues::Quat`] xyzw quaternions.
//!
//! When the bound Model's transform chain carries **extensions**
//! (pivots / offsets / Pre-/PostRotation / non-XYZ `RotationOrder` —
//! see [`crate::node_transform::TransformChain`]), the animated `Lcl`
//! values slot into the middle of the doc §1 product exactly as the
//! static ones do: per merged keyframe the chain is re-composed with
//! the sampled `T(t)` / `R(t)` / `S(t)` (unanimated properties hold
//! their static `Lcl` values), so the emitted translation channel
//! picks up the pivot swing of an animated rotation and the rotation
//! channel is `Rpre · R(t) · Rpost⁻¹` in the Model's rotation order.
//! For trivial chains the per-property fast path emits each channel
//! independently via [`euler_xyz_to_quat`] (identical output —
//! `Q = R(XYZ)` and the translation term reduces to `T`).

use std::collections::HashMap;

use oxideav_mesh3d::{
    Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationTarget,
    AnimationValues, Interpolation, NodeId,
};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::deformer::{MorphChannelBinding, DEFORM_PERCENT_SCALE};
use crate::node_transform::TransformChain;

/// FBX KTime constant — number of fixed-point ticks per real-world
/// second. Public knowledge in the FBX-tooling community; used here
/// strictly to convert `KeyTime` integers to `oxideav_mesh3d`'s
/// seconds-as-f32 sampler convention.
pub const KTIME_TICKS_PER_SECOND: f64 = 46_186_158_000.0;

/// Names this round recognises as targets of an `AnimationCurveNode`
/// connected to a Model: the FBX node-transform `P`-records
/// `"Lcl Translation"` / `"Lcl Rotation"` / `"Lcl Scaling"` plus the
/// deformer-channel `"DeformPercent"` used by morph animation.
const TARGET_TRANSLATION: &str = "Lcl Translation";
const TARGET_ROTATION: &str = "Lcl Rotation";
const TARGET_SCALING: &str = "Lcl Scaling";
const TARGET_DEFORM_PERCENT: &str = "DeformPercent";

/// Per-stack collected channel data, keyed by `(target_node_fbx_id,
/// AnimationProperty)`. The Vec3 components arrive on three separate
/// `AnimationCurve` records (one per axis); we accumulate them here
/// and merge into one `AnimationChannel` per (target, property) at
/// the end of [`extract_animations`].
#[derive(Default)]
struct ComponentCurves {
    /// `Some(curve)` once the X (or scalar) curve has been seen.
    x: Option<RawCurve>,
    /// `Some(curve)` once the Y curve has been seen.
    y: Option<RawCurve>,
    /// `Some(curve)` once the Z curve has been seen.
    z: Option<RawCurve>,
}

/// A single `AnimationCurve` — raw `KeyTime` integers + `KeyValueFloat`
/// scalars, fresh out of the binary document.
#[derive(Clone)]
struct RawCurve {
    /// Keyframe times in seconds (already divided by
    /// [`KTIME_TICKS_PER_SECOND`]).
    times_secs: Vec<f32>,
    /// Sample values, one per keyframe.
    values: Vec<f32>,
}

/// Walk the [`FbxDocument`] and produce one [`Animation`] per
/// `AnimationStack`. Channels are wired onto the supplied
/// `model_nodes` map (FBX `Model` id → [`NodeId`] in the
/// [`oxideav_mesh3d::Scene3D`]).
///
/// Models referenced by an animation but not present in `model_nodes`
/// are skipped (with no error — animations on hidden helper bones
/// that the scene-walker hasn't surfaced are common in the wild).
///
/// `deformer_targets` / `node_morph_statics` come from
/// [`crate::deformer::extract_deformers`]: the former resolves each
/// `BlendShapeChannel` FBX id to its node + morph-target slot, the
/// latter supplies the per-node static (rest) weights that fill
/// unanimated slots of a multi-target `MorphWeights` sampler.
pub fn extract_animations(
    doc: &FbxDocument,
    model_nodes: &HashMap<i64, NodeId>,
    deformer_targets: &HashMap<i64, MorphChannelBinding>,
    node_morph_statics: &HashMap<NodeId, Vec<f32>>,
) -> Vec<Animation> {
    // Index Objects by FBX id for every animation-related element
    // type we touch.
    let mut stacks: HashMap<i64, String> = HashMap::new();
    // Document order of the stacks — the emitted `Animation` list
    // follows it, so a decode is deterministic (a hash-map walk
    // would reorder stacks from one run to the next).
    let mut stack_order: Vec<i64> = Vec::new();
    let mut layers: HashMap<i64, String> = HashMap::new();
    let mut curve_nodes: HashMap<i64, String> = HashMap::new();
    let mut curves: HashMap<i64, RawCurve> = HashMap::new();

    if let Some(objects) = doc.root.child("Objects") {
        for child in &objects.children {
            match child.name.as_str() {
                "AnimationStack" => {
                    if let Some(id) = element_id(child) {
                        if stacks
                            .insert(id, element_name(child).unwrap_or_default())
                            .is_none()
                        {
                            stack_order.push(id);
                        }
                    }
                }
                "AnimationLayer" => {
                    if let Some(id) = element_id(child) {
                        layers.insert(id, element_name(child).unwrap_or_default());
                    }
                }
                "AnimationCurveNode" => {
                    if let Some(id) = element_id(child) {
                        curve_nodes.insert(id, element_name(child).unwrap_or_default());
                    }
                }
                "AnimationCurve" => {
                    if let Some(id) = element_id(child) {
                        if let Some(c) = read_curve(child) {
                            curves.insert(id, c);
                        }
                    }
                }
                _ => {}
            }
        }
    }

    if stacks.is_empty() {
        return Vec::new();
    }

    // Walk Connections to reconstruct the curve-node → property-on-Model
    // bindings + the layer-membership chain.
    //
    // Maps:
    //   curve_id -> (curve_node_id, axis_tag)         (OP records)
    //   curve_node_id -> (target_id, target_prop_str) (OP records)
    //   curve_node_id -> layer_id                     (OO records)
    //   layer_id -> stack_id                          (OO records)
    let mut curve_to_node: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_target: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_layer: HashMap<i64, i64> = HashMap::new();
    let mut layer_to_stack: HashMap<i64, i64> = HashMap::new();

    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let prop_name = c.properties.get(3).and_then(FbxProperty::as_str);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            match kind {
                "OP" => {
                    let prop = match prop_name {
                        Some(p) => p.to_owned(),
                        None => continue,
                    };
                    // Curve -> CurveNode, where prop_name is "d|X" / "d|Y"
                    // / "d|Z" / "d|DeformPercent".
                    if curves.contains_key(&child_id) && curve_nodes.contains_key(&parent_id) {
                        curve_to_node.insert(child_id, (parent_id, prop));
                        continue;
                    }
                    // CurveNode -> Model (or other element), where prop_name
                    // is "Lcl Translation" / "Lcl Rotation" / etc.
                    if curve_nodes.contains_key(&child_id) {
                        node_to_target.insert(child_id, (parent_id, prop));
                    }
                }
                "OO" => {
                    if curve_nodes.contains_key(&child_id) && layers.contains_key(&parent_id) {
                        node_to_layer.insert(child_id, parent_id);
                        continue;
                    }
                    if layers.contains_key(&child_id) && stacks.contains_key(&parent_id) {
                        layer_to_stack.insert(child_id, parent_id);
                    }
                }
                _ => {}
            }
        }
    }

    // Per-stack accumulator: (target_node_id, property tag u8) -> ComponentCurves.
    // The tag encodes which `AnimationProperty` variant the bucket
    // belongs to; it's an internal key only — see `prop_tag` /
    // `prop_from_tag`.
    let mut per_stack: HashMap<i64, HashMap<(NodeId, u8), ComponentCurves>> = HashMap::new();
    // Per-stack accumulator for morph-target (`DeformPercent`)
    // animations, separate from the Vec3/Quat path because they fan
    // out across morph-target slots rather than X/Y/Z components:
    // node → (target slot → raw curve). BTreeMap keeps slot
    // iteration deterministic.
    #[allow(clippy::type_complexity)]
    let mut per_stack_morph: HashMap<
        i64,
        HashMap<NodeId, std::collections::BTreeMap<usize, RawCurve>>,
    > = HashMap::new();

    for (curve_id, curve) in &curves {
        let (curve_node_id, axis_tag) = match curve_to_node.get(curve_id) {
            Some(t) => t,
            None => continue,
        };
        let (target_id, target_prop) = match node_to_target.get(curve_node_id) {
            Some(t) => t,
            None => continue,
        };
        let layer_id = match node_to_layer.get(curve_node_id) {
            Some(l) => l,
            None => continue,
        };
        let stack_id = match layer_to_stack.get(layer_id) {
            Some(s) => *s,
            None => continue,
        };

        // Vec3-targeted properties on a Model node.
        if let Some(prop) = property_for(target_prop) {
            let node_id = match model_nodes.get(target_id) {
                Some(&nid) => nid,
                None => continue,
            };
            let bucket = per_stack
                .entry(stack_id)
                .or_default()
                .entry((node_id, prop_tag(prop)))
                .or_default();
            match axis_tag.as_str() {
                "d|X" => bucket.x = Some(curve.clone()),
                "d|Y" => bucket.y = Some(curve.clone()),
                "d|Z" => bucket.z = Some(curve.clone()),
                _ => {}
            }
            continue;
        }

        // Morph-target (Scalar) — DeformPercent on a BlendShapeChannel
        // sub-deformer. The deformer module pre-resolved the
        // BlendShapeChannel FBX id to its node + morph-target slot;
        // we bucket the curve under that slot here so every channel
        // of a multi-target mesh contributes to one strided
        // MorphWeights sampler below.
        if target_prop == TARGET_DEFORM_PERCENT && axis_tag == "d|DeformPercent" {
            if let Some(binding) = deformer_targets.get(target_id) {
                per_stack_morph
                    .entry(stack_id)
                    .or_default()
                    .entry(binding.target.node)
                    .or_default()
                    .insert(binding.target_index, curve.clone());
            }
        }
    }

    // Per-node resolved transform chains — a node whose chain carries
    // extensions (pivots / Pre-/PostRotation / rotation order) needs
    // its animated Lcl values composed through the doc §1 product
    // rather than emitted verbatim.
    let node_chains: HashMap<NodeId, TransformChain> = crate::node_transform::model_chains(doc)
        .into_iter()
        .filter_map(|(fid, chain)| model_nodes.get(&fid).map(|nid| (*nid, chain)))
        .collect();

    // Materialise one Animation per stack.
    let mut animations: Vec<Animation> = Vec::new();
    for stack_id in &stack_order {
        let name = &stacks[stack_id];
        let mut anim = Animation::new(if name.is_empty() {
            None
        } else {
            Some(name.clone())
        });

        if let Some(channels) = per_stack.get(stack_id) {
            // Group the Vec3-property buckets per node so
            // extension-bearing chains can compose them jointly.
            let mut grouped: HashMap<NodeId, [Option<&ComponentCurves>; 3]> = HashMap::new();
            for ((node_id, tag), comps) in channels {
                if *tag <= 2 {
                    grouped.entry(*node_id).or_default()[usize::from(*tag)] = Some(comps);
                }
            }
            // Channel order is deterministic: ascending target node
            // id, T / R / S within a node (the writer's emission
            // order, so decode → encode → decode is a fixed point).
            let mut grouped: Vec<(NodeId, [Option<&ComponentCurves>; 3])> =
                grouped.into_iter().collect();
            grouped.sort_by_key(|(n, _)| n.0);
            for (node_id, [t_c, r_c, s_c]) in grouped {
                match node_chains.get(&node_id) {
                    Some(chain) if chain.has_extensions() => {
                        anim.channels
                            .extend(build_composed_channels(node_id, chain, t_c, r_c, s_c));
                    }
                    _ => {
                        for (tag, comps) in [(0u8, t_c), (1, r_c), (2, s_c)] {
                            if let Some(comps) = comps {
                                if let Some(channel) =
                                    build_channel(node_id, prop_from_tag(tag), comps)
                                {
                                    anim.channels.push(channel);
                                }
                            }
                        }
                    }
                }
            }
        }
        if let Some(morphs) = per_stack_morph.get(stack_id) {
            // Deterministic node order for the emitted channel list.
            let mut node_ids: Vec<NodeId> = morphs.keys().copied().collect();
            node_ids.sort_by_key(|n| n.0);
            for node_id in node_ids {
                let slot_curves = &morphs[&node_id];
                if let Some(channel) =
                    build_morph_channel(node_id, slot_curves, node_morph_statics.get(&node_id))
                {
                    anim.channels.push(channel);
                }
            }
        }

        if !anim.channels.is_empty() {
            animations.push(anim);
        }
    }
    animations
}

/// Walk every `AnimationCurve` element and surface its raw key
/// attribute sub-records — `KeyAttrFlags` / `KeyAttrDataFloat` /
/// `KeyAttrRefCount` — as a JSON catalogue for
/// `Scene3D::extras["fbx:key_attrs"]`.
///
/// Per `docs/3d/fbx/GAP-TRACKER.md` ("`KeyAttrFlags` interpolation /
/// tangent modes — OPEN"), `KeyAttrFlags` is a packed bitfield
/// selecting interpolation mode (constant / linear / cubic), tangent
/// mode (auto / TCB / user / break / clamped) and weighting, with
/// `KeyAttrDataFloat` and `KeyAttrRefCount` alongside — but **no
/// value assignment is recorded in the staged docs**, so this crate
/// deliberately does not interpret a single bit. What it does do is
/// stop dropping the payload: each curve carrying any of the three
/// sub-records contributes one catalogue entry with its stack /
/// target / property / axis join key (resolved through the same
/// `Connections` chain [`extract_animations`] walks) and the raw
/// arrays:
///
/// - `flags` / `ref_count` — the integer arrays verbatim;
/// - `data_bits` — `KeyAttrDataFloat` as per-element IEEE-754 bit
///   patterns (`u32`), the lossless surface for a payload whose
///   semantics are unpinned (interpreted floats could not represent
///   a NaN bit pattern in JSON);
/// - `key_count` — the curve's `KeyTime` length, so a consumer can
///   check the `ref_count` run-length relationship itself.
///
/// Returns `None` when no curve carries key-attribute records (the
/// synthetic output of this crate's own encoder, for instance).
/// Re-encode does **not** re-emit these records: the decode side
/// merges per-axis curves onto a union keyframe grid, and stretching
/// an uninterpreted per-key attribute table onto a resampled grid
/// would require exactly the semantics the docs don't pin. See the
/// README "Notes & limitations".
pub fn extract_key_attr_catalogue(doc: &FbxDocument) -> Option<serde_json::Value> {
    let objects = doc.root.child("Objects")?;

    // Element indices for the join-key resolution.
    let mut stacks: HashMap<i64, String> = HashMap::new();
    let mut layers: HashMap<i64, ()> = HashMap::new();
    let mut curve_nodes: HashMap<i64, ()> = HashMap::new();
    let mut model_names: HashMap<i64, String> = HashMap::new();
    let mut curve_elements: Vec<(i64, &FbxNode)> = Vec::new();
    for child in &objects.children {
        let Some(id) = element_id(child) else {
            continue;
        };
        match child.name.as_str() {
            "AnimationStack" => {
                stacks.insert(id, element_name(child).unwrap_or_default());
            }
            "AnimationLayer" => {
                layers.insert(id, ());
            }
            "AnimationCurveNode" => {
                curve_nodes.insert(id, ());
            }
            "AnimationCurve" => curve_elements.push((id, child)),
            "Model" => {
                model_names.insert(id, element_name(child).unwrap_or_default());
            }
            _ => {}
        }
    }
    if curve_elements.is_empty() {
        return None;
    }

    let mut curve_to_node: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_target: HashMap<i64, (i64, String)> = HashMap::new();
    let mut node_to_layer: HashMap<i64, i64> = HashMap::new();
    let mut layer_to_stack: HashMap<i64, i64> = HashMap::new();
    if let Some(conns) = doc.root.child("Connections") {
        for c in conns.children_named("C") {
            let kind = c.properties.first().and_then(FbxProperty::as_str);
            let child_id = c.properties.get(1).and_then(FbxProperty::as_i64);
            let parent_id = c.properties.get(2).and_then(FbxProperty::as_i64);
            let prop_name = c.properties.get(3).and_then(FbxProperty::as_str);
            let (Some(kind), Some(child_id), Some(parent_id)) = (kind, child_id, parent_id) else {
                continue;
            };
            match (kind, prop_name) {
                ("OP", Some(prop)) => {
                    if curve_nodes.contains_key(&parent_id) {
                        curve_to_node.insert(child_id, (parent_id, prop.to_owned()));
                    } else if curve_nodes.contains_key(&child_id) {
                        node_to_target.insert(child_id, (parent_id, prop.to_owned()));
                    }
                }
                ("OO", _) => {
                    if curve_nodes.contains_key(&child_id) && layers.contains_key(&parent_id) {
                        node_to_layer.insert(child_id, parent_id);
                    } else if layers.contains_key(&child_id) && stacks.contains_key(&parent_id) {
                        layer_to_stack.insert(child_id, parent_id);
                    }
                }
                _ => {}
            }
        }
    }

    let mut entries: Vec<serde_json::Value> = Vec::new();
    for (curve_id, node) in curve_elements {
        let flags = int_array(node, "KeyAttrFlags");
        let ref_count = int_array(node, "KeyAttrRefCount");
        let data_bits = f32_bits_array(node, "KeyAttrDataFloat");
        if flags.is_none() && ref_count.is_none() && data_bits.is_none() {
            continue;
        }
        let mut entry = serde_json::Map::new();
        // Join key: stack name / target element name / target
        // property / axis tag, each present when the Connections
        // chain resolves that hop.
        let wiring = curve_to_node.get(&curve_id);
        if let Some((cn_id, axis)) = wiring {
            entry.insert("axis".into(), serde_json::Value::String(axis.clone()));
            if let Some((target_id, prop)) = node_to_target.get(cn_id) {
                entry.insert("property".into(), serde_json::Value::String(prop.clone()));
                if let Some(name) = model_names.get(target_id) {
                    entry.insert("target".into(), serde_json::Value::String(name.clone()));
                }
            }
            if let Some(stack_id) = node_to_layer.get(cn_id).and_then(|l| layer_to_stack.get(l)) {
                if let Some(name) = stacks.get(stack_id) {
                    entry.insert("stack".into(), serde_json::Value::String(name.clone()));
                }
            }
        }
        let key_count = node
            .child("KeyTime")
            .and_then(|n| n.properties.first())
            .and_then(|p| match p {
                FbxProperty::I64Array(a) => Some(a.len()),
                FbxProperty::I32Array(a) => Some(a.len()),
                _ => None,
            });
        if let Some(n) = key_count {
            entry.insert("key_count".into(), serde_json::Value::from(n));
        }
        if let Some(v) = flags {
            entry.insert("flags".into(), v);
        }
        if let Some(v) = data_bits {
            entry.insert("data_bits".into(), v);
        }
        if let Some(v) = ref_count {
            entry.insert("ref_count".into(), v);
        }
        entries.push(serde_json::Value::Object(entry));
    }
    if entries.is_empty() {
        None
    } else {
        Some(serde_json::Value::Array(entries))
    }
}

/// An integer-array sub-record (`i` / `l` wire variants) as a JSON
/// array of integers.
fn int_array(node: &FbxNode, name: &str) -> Option<serde_json::Value> {
    let arr = match node.child(name)?.properties.first()? {
        FbxProperty::I32Array(a) => a.iter().map(|v| serde_json::Value::from(*v)).collect(),
        FbxProperty::I64Array(a) => a.iter().map(|v| serde_json::Value::from(*v)).collect(),
        _ => return None,
    };
    Some(serde_json::Value::Array(arr))
}

/// An `f`-array sub-record as its per-element IEEE-754 bit patterns
/// (lossless; see [`extract_key_attr_catalogue`]). An `i`-wire
/// variant (some producers write the attr data as raw ints already)
/// surfaces its unsigned reinterpretation so the representation is
/// uniform.
fn f32_bits_array(node: &FbxNode, name: &str) -> Option<serde_json::Value> {
    let arr = match node.child(name)?.properties.first()? {
        FbxProperty::F32Array(a) => a
            .iter()
            .map(|v| serde_json::Value::from(v.to_bits()))
            .collect(),
        FbxProperty::I32Array(a) => a
            .iter()
            .map(|v| serde_json::Value::from(u32::from_ne_bytes(v.to_ne_bytes())))
            .collect(),
        _ => return None,
    };
    Some(serde_json::Value::Array(arr))
}

/// Map an FBX target-property string to a typed
/// [`AnimationProperty`]. Returns `None` for properties this round
/// doesn't surface.
fn property_for(s: &str) -> Option<AnimationProperty> {
    match s {
        TARGET_TRANSLATION => Some(AnimationProperty::Translation),
        TARGET_ROTATION => Some(AnimationProperty::Rotation),
        TARGET_SCALING => Some(AnimationProperty::Scale),
        _ => None,
    }
}

/// Compact the four-variant `AnimationProperty` enum to a `u8` so we
/// can use it as a HashMap key (the upstream enum doesn't `derive`
/// `Hash`).
fn prop_tag(p: AnimationProperty) -> u8 {
    match p {
        AnimationProperty::Translation => 0,
        AnimationProperty::Rotation => 1,
        AnimationProperty::Scale => 2,
        AnimationProperty::MorphWeights => 3,
    }
}

/// Inverse of [`prop_tag`]. Internal — only called with values
/// produced by [`prop_tag`], so the panic branch is unreachable.
fn prop_from_tag(t: u8) -> AnimationProperty {
    match t {
        0 => AnimationProperty::Translation,
        1 => AnimationProperty::Rotation,
        2 => AnimationProperty::Scale,
        3 => AnimationProperty::MorphWeights,
        _ => unreachable!("prop_from_tag called with invalid tag {t}"),
    }
}

/// Combine three component curves into one Vec3-or-Quat
/// [`AnimationChannel`].
///
/// FBX writers don't guarantee that the X/Y/Z curves share keyframe
/// times — components animate independently. We produce the union of
/// keyframe times across the three components and linearly interpolate
/// each component into that merged grid. This loses no fidelity at
/// the original keyframes (each is hit exactly) and the in-between
/// values match what an `Interpolation::Linear` consumer would
/// compute anyway.
fn build_channel(
    node: NodeId,
    property: AnimationProperty,
    comps: &ComponentCurves,
) -> Option<AnimationChannel> {
    let xs = comps.x.as_ref();
    let ys = comps.y.as_ref();
    let zs = comps.z.as_ref();

    // Build the merged time axis over whichever components exist —
    // exporters animate components independently, so a Z-only (or
    // Y-only) curve set is a legitimate channel.
    let mut merged_times: Vec<f32> = Vec::new();
    for c in [xs, ys, zs].into_iter().flatten() {
        push_times(&mut merged_times, &c.times_secs);
    }
    merged_times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    merged_times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if merged_times.is_empty() {
        return None;
    }

    // Resample each component onto the merged grid (absent → 0.0).
    let resample = |c: Option<&RawCurve>| -> Vec<f32> {
        merged_times
            .iter()
            .map(|t| c.map(|c| sample_linear(c, *t)).unwrap_or(0.0))
            .collect()
    };
    let xv = resample(xs);
    let yv = resample(ys);
    let zv = resample(zs);

    let values = match property {
        AnimationProperty::Rotation => {
            // FBX Lcl Rotation is degrees, default XYZ Euler order. Map
            // to xyzw quaternion per the docs.
            let q: Vec<[f32; 4]> = xv
                .iter()
                .zip(yv.iter())
                .zip(zv.iter())
                .map(|((x, y), z)| euler_xyz_to_quat([*x, *y, *z]))
                .collect();
            AnimationValues::Quat(q)
        }
        AnimationProperty::Translation | AnimationProperty::Scale => {
            let v: Vec<[f32; 3]> = xv
                .iter()
                .zip(yv.iter())
                .zip(zv.iter())
                .map(|((x, y), z)| [*x, *y, *z])
                .collect();
            AnimationValues::Vec3(v)
        }
        AnimationProperty::MorphWeights => return None,
    };

    Some(AnimationChannel {
        target: AnimationTarget { node, property },
        sampler: AnimationSampler {
            keyframes: merged_times,
            values,
            interpolation: Interpolation::Linear,
        },
    })
}

/// Combine every `DeformPercent` curve targeting one node's
/// blend-shape channels into a single `MorphWeights`
/// [`AnimationChannel`], per the [`oxideav_mesh3d`] sampler contract:
/// the `Scalar` value table is strided by the bound mesh's
/// morph-target count (one weight per target per keyframe).
///
/// FBX writers animate channels independently (each
/// `BlendShapeChannel` gets its own curve with its own key grid), so
/// — exactly like [`build_channel`]'s X/Y/Z merge — the union of
/// keyframe times is taken and every animated slot is linearly
/// resampled onto it. Slots without a curve hold their **static**
/// `DeformPercent` value (the same rest weight the deformer module
/// decoded into `Mesh::weights`). Wire values are 0..100 percentages;
/// the emitted weights are divided by [`DEFORM_PERCENT_SCALE`] to the
/// mesh3d §3.7.2.2 blend-factor convention.
fn build_morph_channel(
    node: NodeId,
    slot_curves: &std::collections::BTreeMap<usize, RawCurve>,
    statics: Option<&Vec<f32>>,
) -> Option<AnimationChannel> {
    let statics: &[f32] = statics.map_or(&[], Vec::as_slice);
    // Stride = the mesh's morph-target count: every static slot plus
    // any animated slot beyond them (defensive — the deformer module
    // fills statics for every materialised target).
    let max_animated = slot_curves.keys().copied().max()?;
    let n_targets = statics.len().max(max_animated + 1);

    let mut merged_times: Vec<f32> = Vec::new();
    for c in slot_curves.values() {
        push_times(&mut merged_times, &c.times_secs);
    }
    merged_times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    merged_times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if merged_times.is_empty() {
        return None;
    }

    // One full weight vector per merged keyframe — the typed
    // `AnimationSampler::morph_weights` synthesis path flattens them
    // row-major into the strided `Scalar` table; it returns `None`
    // only for a grid `validate` would reject (empty / non-increasing
    // times, ragged frames), which the merge above rules out.
    let frames: Vec<Vec<f32>> = merged_times
        .iter()
        .map(|&t| {
            (0..n_targets)
                .map(|slot| match slot_curves.get(&slot) {
                    Some(curve) => {
                        (f64::from(sample_linear(curve, t)) / DEFORM_PERCENT_SCALE) as f32
                    }
                    None => statics.get(slot).copied().unwrap_or(0.0),
                })
                .collect()
        })
        .collect();
    let sampler = AnimationSampler::morph_weights(merged_times, frames, Interpolation::Linear)?;
    Some(AnimationChannel::new(
        node,
        AnimationProperty::MorphWeights,
        sampler,
    ))
}

/// Build the coupled channel set for a node whose transform chain
/// carries extensions (`docs/3d/fbx/fbx-node-transform-chain.md` §1).
///
/// The animated `Lcl` values replace the static ones **inside** the
/// chain product, so per merged keyframe the full chain is
/// re-composed via [`TransformChain::compose`]:
///
/// - the **translation** channel is always emitted — its composed
///   value `T + Roff + Rp + Q·(Soff + Sp − Rp − S∘Sp)` varies with
///   any of the three animated properties (the pivot swing of an
///   animated rotation lands here);
/// - the **rotation** channel (`Rpre · R(t) · Rpost⁻¹` in the
///   Model's rotation order) is emitted iff rotation curves exist;
/// - the **scale** channel (`S(t)` verbatim) is emitted iff scaling
///   curves exist.
///
/// Unanimated properties hold their static `Lcl` values; a missing
/// component inside an animated property samples as `0.0` (the same
/// convention [`build_channel`] applies). Successive quaternion keys
/// are sign-aligned so a linear/slerp consumer takes the short arc.
fn build_composed_channels(
    node: NodeId,
    chain: &TransformChain,
    t_curves: Option<&ComponentCurves>,
    r_curves: Option<&ComponentCurves>,
    s_curves: Option<&ComponentCurves>,
) -> Vec<AnimationChannel> {
    // Union time grid across every present component curve.
    let mut times: Vec<f32> = Vec::new();
    for curves in [t_curves, r_curves, s_curves].into_iter().flatten() {
        for c in [&curves.x, &curves.y, &curves.z].into_iter().flatten() {
            push_times(&mut times, &c.times_secs);
        }
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if times.is_empty() {
        return Vec::new();
    }

    let sample_vec3 = |curves: Option<&ComponentCurves>, statics: [f64; 3], t: f32| -> [f64; 3] {
        match curves {
            None => statics,
            Some(cc) => {
                let comp = |c: &Option<RawCurve>| {
                    c.as_ref().map_or(0.0, |c| f64::from(sample_linear(c, t)))
                };
                [comp(&cc.x), comp(&cc.y), comp(&cc.z)]
            }
        }
    };

    let mut tv: Vec<[f32; 3]> = Vec::with_capacity(times.len());
    let mut rv: Vec<[f32; 4]> = Vec::with_capacity(times.len());
    let mut sv: Vec<[f32; 3]> = Vec::with_capacity(times.len());
    for &t in &times {
        let mut c = chain.clone();
        c.lcl_translation = sample_vec3(t_curves, chain.lcl_translation, t);
        c.lcl_rotation = sample_vec3(r_curves, chain.lcl_rotation, t);
        c.lcl_scaling = sample_vec3(s_curves, chain.lcl_scaling, t);
        let (ct, cq, cs) = c.compose();
        tv.push([ct[0] as f32, ct[1] as f32, ct[2] as f32]);
        let mut q = [cq[0] as f32, cq[1] as f32, cq[2] as f32, cq[3] as f32];
        // Sign-align with the previous key (q and −q are the same
        // rotation; alignment keeps interpolation on the short arc).
        if let Some(prev) = rv.last() {
            let dot: f32 = (0..4).map(|i| prev[i] * q[i]).sum();
            if dot < 0.0 {
                q = [-q[0], -q[1], -q[2], -q[3]];
            }
        }
        rv.push(q);
        sv.push([cs[0] as f32, cs[1] as f32, cs[2] as f32]);
    }

    let channel = |property, values| AnimationChannel {
        target: AnimationTarget { node, property },
        sampler: AnimationSampler {
            keyframes: times.clone(),
            values,
            interpolation: Interpolation::Linear,
        },
    };
    let mut out = vec![channel(
        AnimationProperty::Translation,
        AnimationValues::Vec3(tv),
    )];
    if r_curves.is_some() {
        out.push(channel(
            AnimationProperty::Rotation,
            AnimationValues::Quat(rv),
        ));
    }
    if s_curves.is_some() {
        out.push(channel(AnimationProperty::Scale, AnimationValues::Vec3(sv)));
    }
    out
}

/// Append every time from `src` into `dst` (deduping happens after a
/// global sort).
fn push_times(dst: &mut Vec<f32>, src: &[f32]) {
    dst.extend_from_slice(src);
}

/// Linear sampler — clamps at the endpoints, linearly interpolates
/// between adjacent keyframes. We don't honour `KeyAttrFlags` /
/// `KeyAttrDataFloat` (cubic / step / TCB) because the docs we have
/// do not specify their bit layout; emitting `Interpolation::Linear`
/// matches the common baked-animation downstream representation.
fn sample_linear(curve: &RawCurve, t: f32) -> f32 {
    if curve.times_secs.is_empty() {
        return 0.0;
    }
    if t <= curve.times_secs[0] {
        return curve.values[0];
    }
    let last = curve.times_secs.len() - 1;
    if t >= curve.times_secs[last] {
        return curve.values[last];
    }
    // Binary search for the bracketing pair.
    let idx = curve
        .times_secs
        .binary_search_by(|x| x.partial_cmp(&t).unwrap_or(std::cmp::Ordering::Equal));
    match idx {
        Ok(i) => curve.values[i],
        Err(i) => {
            let t0 = curve.times_secs[i - 1];
            let t1 = curve.times_secs[i];
            let v0 = curve.values[i - 1];
            let v1 = curve.values[i];
            let dt = t1 - t0;
            if dt.abs() < f32::EPSILON {
                v0
            } else {
                let alpha = (t - t0) / dt;
                v0 + (v1 - v0) * alpha
            }
        }
    }
}

/// Read an `AnimationCurve` element record into a [`RawCurve`].
///
/// Looks for the `KeyTime` (`l` array — i64 ticks) and `KeyValueFloat`
/// (`f` array — f32 values) sub-records. Returns `None` when either
/// is missing or the lengths disagree.
fn read_curve(node: &FbxNode) -> Option<RawCurve> {
    let times_node = node.child("KeyTime")?;
    let vals_node = node.child("KeyValueFloat")?;
    let times_raw: Vec<i64> = match times_node.properties.first()? {
        FbxProperty::I64Array(a) => a.clone(),
        FbxProperty::I32Array(a) => a.iter().map(|v| *v as i64).collect(),
        _ => return None,
    };
    let values: Vec<f32> = match vals_node.properties.first()? {
        FbxProperty::F32Array(a) => a.clone(),
        FbxProperty::F64Array(a) => a.iter().map(|v| *v as f32).collect(),
        _ => return None,
    };
    if times_raw.len() != values.len() {
        return None;
    }
    let times_secs: Vec<f32> = times_raw
        .iter()
        .map(|t| ((*t as f64) / KTIME_TICKS_PER_SECOND) as f32)
        .collect();
    Some(RawCurve { times_secs, values })
}

/// Read property[0] (the FBX element id) of an Objects-child record.
fn element_id(n: &FbxNode) -> Option<i64> {
    n.properties.first().and_then(FbxProperty::as_i64)
}

/// Read property[1] of an Objects-child record and strip the
/// `\0\1`-separated `Name::SubType` joiner (binary FBX convention) to
/// return only the leading name.
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

/// Convert an XYZ-order Euler triplet (degrees) to a quaternion in
/// xyzw layout. XYZ is the FBX default `RotationOrder`.
///
/// Composition order is `R = Rz * Ry * Rx` so that vectors transform
/// as `v' = Rz * (Ry * (Rx * v))` — this is the standard XYZ
/// extrinsic / ZYX intrinsic interpretation used by every glTF
/// converter. The quaternion equivalent is
/// `q = qz * qy * qx` (Hamilton product, applied right-to-left).
pub fn euler_xyz_to_quat(deg: [f32; 3]) -> [f32; 4] {
    let to_rad = std::f32::consts::PI / 180.0;
    let (hx, hy, hz) = (
        deg[0] * to_rad * 0.5,
        deg[1] * to_rad * 0.5,
        deg[2] * to_rad * 0.5,
    );
    let (sx, cx) = (hx.sin(), hx.cos());
    let (sy, cy) = (hy.sin(), hy.cos());
    let (sz, cz) = (hz.sin(), hz.cos());
    // Per-axis quaternions in xyzw form:
    //   qx = (sx, 0,  0, cx)
    //   qy = (0,  sy, 0, cy)
    //   qz = (0,  0,  sz, cz)
    // Compose q = qz * (qy * qx) so that the resulting rotation
    // applies Rx first, then Ry, then Rz.
    let qx = (sx, 0.0_f32, 0.0_f32, cx);
    let qy = (0.0_f32, sy, 0.0_f32, cy);
    let qz = (0.0_f32, 0.0_f32, sz, cz);
    let yx = quat_mul(qy, qx);
    let zyx = quat_mul(qz, yx);
    [zyx.0, zyx.1, zyx.2, zyx.3]
}

/// Hamilton quaternion product in xyzw layout.
fn quat_mul(a: (f32, f32, f32, f32), b: (f32, f32, f32, f32)) -> (f32, f32, f32, f32) {
    let (ax, ay, az, aw) = a;
    let (bx, by, bz, bw) = b;
    (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ktime_constant_round_trips_one_second() {
        let one_sec_ticks = KTIME_TICKS_PER_SECOND as i64;
        let secs = (one_sec_ticks as f64) / KTIME_TICKS_PER_SECOND;
        assert!((secs - 1.0).abs() < 1e-9);
    }

    #[test]
    fn property_mapping() {
        assert_eq!(
            property_for("Lcl Translation"),
            Some(AnimationProperty::Translation)
        );
        assert_eq!(
            property_for("Lcl Rotation"),
            Some(AnimationProperty::Rotation)
        );
        assert_eq!(property_for("Lcl Scaling"), Some(AnimationProperty::Scale));
        assert_eq!(property_for("Visibility"), None);
    }

    #[test]
    fn euler_identity_to_identity_quat() {
        let q = euler_xyz_to_quat([0.0, 0.0, 0.0]);
        assert!((q[0]).abs() < 1e-6);
        assert!((q[1]).abs() < 1e-6);
        assert!((q[2]).abs() < 1e-6);
        assert!((q[3] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn euler_90_about_x_matches_known_quat() {
        // 90 degrees about X = (sin 45, 0, 0, cos 45).
        let q = euler_xyz_to_quat([90.0, 0.0, 0.0]);
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert!((q[0] - s).abs() < 1e-5, "x = {} expected {}", q[0], s);
        assert!(q[1].abs() < 1e-5);
        assert!(q[2].abs() < 1e-5);
        assert!((q[3] - s).abs() < 1e-5);
    }

    #[test]
    fn linear_sample_clamps() {
        let c = RawCurve {
            times_secs: vec![1.0, 2.0],
            values: vec![10.0, 20.0],
        };
        assert_eq!(sample_linear(&c, 0.0), 10.0);
        assert_eq!(sample_linear(&c, 3.0), 20.0);
        assert_eq!(sample_linear(&c, 1.5), 15.0);
    }
}
