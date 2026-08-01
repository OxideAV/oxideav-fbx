//! Animated transform-chain composition: a Model whose chain carries
//! a `RotationPivot` and an animated `Lcl Rotation` must emit
//! channels composed through the doc §1 product
//! (`docs/3d/fbx/fbx-node-transform-chain.md`) — the rotation channel
//! is `Rpre · R(t) · Rpost⁻¹` and the translation channel picks up
//! the pivot swing `T + Rp + Q(t)·(−Rp)`.
//!
//! Record shapes per `docs/3d/fbx/fbx-binary-properties70.md` §4 / §5
//! (Properties70 `P` grammar; object record headers; the
//! `AnimationStack → Layer → CurveNode → Curve` OO/OP wiring of §7).

use std::collections::HashMap;

use oxideav_fbx::{
    animation::KTIME_TICKS_PER_SECOND,
    binary::{FbxDocument, FbxNode, FbxProperty},
    write_document, FbxDecoder,
};
use oxideav_mesh3d::{AnimationProperty, AnimationValues, Mesh3DDecoder};

fn s(b: &[u8]) -> FbxProperty {
    FbxProperty::String(b.to_vec())
}

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

fn element(kind: &str, id: i64, name: &str, class: &str, subtype: &str) -> FbxNode {
    let display = format!("{name}\x00\x01{class}");
    FbxNode {
        name: kind.into(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(display.into_bytes()),
            s(subtype.as_bytes()),
        ],
        children: Vec::new(),
    }
}

fn curve(id: i64, times_ticks: &[i64], values: &[f32]) -> FbxNode {
    let mut n = element("AnimationCurve", id, "Curve", "AnimCurve", "");
    n.children = vec![
        FbxNode {
            name: "KeyTime".into(),
            properties: vec![FbxProperty::I64Array(times_ticks.to_vec())],
            children: Vec::new(),
        },
        FbxNode {
            name: "KeyValueFloat".into(),
            properties: vec![FbxProperty::F32Array(values.to_vec())],
            children: Vec::new(),
        },
    ];
    n
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

#[test]
fn animated_rotation_composes_through_pivot_chain() {
    // Model 900: RotationPivot (1,0,0), static Lcl Translation
    // (10,0,0), animated Lcl Rotation Z: 0° → 90° over one second.
    let mut model = element("Model", 900, "Swinger", "Model", "Mesh");
    model.children = vec![FbxNode {
        name: "Properties70".into(),
        properties: Vec::new(),
        children: vec![
            p_vec3("Lcl Translation", "Lcl Translation", [10.0, 0.0, 0.0]),
            p_vec3("RotationPivot", "Vector3D", [1.0, 0.0, 0.0]),
        ],
    }];

    let one_sec = KTIME_TICKS_PER_SECOND as i64;
    let stack = element("AnimationStack", 910, "Take 001", "AnimStack", "");
    let layer = element("AnimationLayer", 911, "BaseLayer", "AnimLayer", "");
    let curve_node = element("AnimationCurveNode", 912, "R", "AnimCurveNode", "");
    let cz = curve(913, &[0, one_sec], &[0.0, 90.0]);

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![model, stack, layer, curve_node, cz],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(900, 0),
            c_oo(911, 910),                 // layer -> stack
            c_oo(912, 911),                 // curve node -> layer
            c_op(912, 900, "Lcl Rotation"), // curve node -> model property
            c_op(913, 912, "d|Z"),          // curve -> curve node axis
        ],
    };
    let doc = FbxDocument {
        version: 7500,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, conns],
        },
    };

    let bytes = write_document(&doc).expect("encode synthetic doc");
    let scene = FbxDecoder::new()
        .decode(&bytes)
        .expect("decode synthetic doc");

    assert_eq!(scene.animations.len(), 1);
    let anim = &scene.animations[0];

    let mut channels: HashMap<u8, &oxideav_mesh3d::AnimationChannel> = HashMap::new();
    for ch in &anim.channels {
        let tag = match ch.target.property {
            AnimationProperty::Translation => 0,
            AnimationProperty::Rotation => 1,
            AnimationProperty::Scale => 2,
            AnimationProperty::MorphWeights => 3,
        };
        channels.insert(tag, ch);
    }

    // The rotation channel: identity at t=0, 90° about Z at t=1.
    let rot = channels.get(&1).expect("rotation channel emitted");
    assert_eq!(rot.sampler.keyframes.len(), 2);
    assert!((rot.sampler.keyframes[1] - 1.0).abs() < 1e-4);
    let AnimationValues::Quat(q) = &rot.sampler.values else {
        panic!("rotation channel must be Quat");
    };
    assert!(
        (q[0][3] - 1.0).abs() < 1e-5,
        "t0 must be identity: {:?}",
        q[0]
    );
    let h = std::f32::consts::FRAC_1_SQRT_2;
    assert!(
        (q[1][2] - h).abs() < 1e-5 && (q[1][3] - h).abs() < 1e-5,
        "t1 must be 90° about Z: {:?}",
        q[1]
    );

    // The translation channel carries the pivot swing: doc §1 closed
    // form t = T + Rp + Q·(−Rp).
    //   t=0: Q = I      → (10,0,0) + (1,0,0) + (−1, 0,0) = (10, 0,0)
    //   t=1: Q = Rz(90) → (10,0,0) + (1,0,0) + ( 0,−1,0) = (11,−1,0)
    let tr = channels.get(&0).expect("translation channel emitted");
    let AnimationValues::Vec3(tvals) = &tr.sampler.values else {
        panic!("translation channel must be Vec3");
    };
    assert_eq!(tr.sampler.keyframes.len(), 2);
    assert!(
        (tvals[0][0] - 10.0).abs() < 1e-4 && tvals[0][1].abs() < 1e-4,
        "t0: {:?}",
        tvals[0]
    );
    assert!(
        (tvals[1][0] - 11.0).abs() < 1e-4 && (tvals[1][1] + 1.0).abs() < 1e-4,
        "t1: {:?}",
        tvals[1]
    );

    // Scale was never animated → no scale channel.
    assert!(!channels.contains_key(&2), "no scale channel expected");

    // The rest transform still composes the static chain: the static
    // rotation is identity, so the rest translation is (10,0,0).
    let node = scene
        .nodes
        .iter()
        .find(|n| n.name.as_deref() == Some("Swinger"))
        .expect("node surfaced");
    match node.transform {
        oxideav_mesh3d::Transform::Trs { translation, .. } => {
            assert!((translation[0] - 10.0).abs() < 1e-5);
        }
        oxideav_mesh3d::Transform::Matrix(_) => panic!("expected Trs"),
    }
}

/// A trivial-chain Model keeps the per-property fast path: only the
/// animated property's channel is emitted.
#[test]
fn trivial_chain_keeps_independent_channels() {
    let mut model = element("Model", 900, "Plain", "Model", "Mesh");
    model.children = vec![FbxNode {
        name: "Properties70".into(),
        properties: Vec::new(),
        children: vec![p_vec3(
            "Lcl Translation",
            "Lcl Translation",
            [10.0, 0.0, 0.0],
        )],
    }];

    let one_sec = KTIME_TICKS_PER_SECOND as i64;
    let stack = element("AnimationStack", 910, "Take 001", "AnimStack", "");
    let layer = element("AnimationLayer", 911, "BaseLayer", "AnimLayer", "");
    let curve_node = element("AnimationCurveNode", 912, "R", "AnimCurveNode", "");
    let cz = curve(913, &[0, one_sec], &[0.0, 90.0]);

    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![model, stack, layer, curve_node, cz],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(900, 0),
            c_oo(911, 910),
            c_oo(912, 911),
            c_op(912, 900, "Lcl Rotation"),
            c_op(913, 912, "d|Z"),
        ],
    };
    let doc = FbxDocument {
        version: 7500,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, conns],
        },
    };

    let bytes = write_document(&doc).expect("encode synthetic doc");
    let scene = FbxDecoder::new()
        .decode(&bytes)
        .expect("decode synthetic doc");

    assert_eq!(scene.animations.len(), 1);
    let anim = &scene.animations[0];
    assert_eq!(anim.channels.len(), 1, "only the rotation channel");
    assert!(matches!(
        anim.channels[0].target.property,
        AnimationProperty::Rotation
    ));
}

/// `decode → encode → decode` of an animated chain-bearing node must
/// reproduce the same composed channels — the encoder de-composes
/// the channels back to authored `Lcl` curves (the re-encoded Model
/// carries its pivot records again, so emitting composed values
/// verbatim would double-apply the chain).
#[test]
fn animated_chain_round_trips_without_double_application() {
    use oxideav_mesh3d::Mesh3DEncoder;

    // Same fixture as `animated_rotation_composes_through_pivot_chain`.
    let mut model = element("Model", 900, "Swinger", "Model", "Mesh");
    model.children = vec![FbxNode {
        name: "Properties70".into(),
        properties: Vec::new(),
        children: vec![
            p_vec3("Lcl Translation", "Lcl Translation", [10.0, 0.0, 0.0]),
            p_vec3("RotationPivot", "Vector3D", [1.0, 0.0, 0.0]),
        ],
    }];
    let one_sec = KTIME_TICKS_PER_SECOND as i64;
    let stack = element("AnimationStack", 910, "Take 001", "AnimStack", "");
    let layer = element("AnimationLayer", 911, "BaseLayer", "AnimLayer", "");
    let curve_node = element("AnimationCurveNode", 912, "R", "AnimCurveNode", "");
    let cz = curve(913, &[0, one_sec], &[0.0, 90.0]);
    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![model, stack, layer, curve_node, cz],
    };
    let conns = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![
            c_oo(900, 0),
            c_oo(911, 910),
            c_oo(912, 911),
            c_op(912, 900, "Lcl Rotation"),
            c_op(913, 912, "d|Z"),
        ],
    };
    let doc = FbxDocument {
        version: 7500,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, conns],
        },
    };

    let bytes = write_document(&doc).expect("encode synthetic doc");
    let first = FbxDecoder::new().decode(&bytes).expect("first decode");
    let re_encoded = oxideav_fbx::FbxEncoder::new()
        .encode(&first)
        .expect("re-encode");
    let second = FbxDecoder::new().decode(&re_encoded).expect("re-decode");

    type ChannelData = (Vec<f32>, Vec<[f32; 4]>, Vec<[f32; 3]>);
    let channels_of = |scene: &oxideav_mesh3d::Scene3D| -> HashMap<u8, ChannelData> {
        let mut out = HashMap::new();
        for anim in &scene.animations {
            for ch in &anim.channels {
                let tag = match ch.target.property {
                    AnimationProperty::Translation => 0u8,
                    AnimationProperty::Rotation => 1,
                    AnimationProperty::Scale => 2,
                    AnimationProperty::MorphWeights => 3,
                };
                let (mut quats, mut vecs) = (Vec::new(), Vec::new());
                match &ch.sampler.values {
                    AnimationValues::Quat(q) => quats = q.clone(),
                    AnimationValues::Vec3(v) => vecs = v.clone(),
                    AnimationValues::Scalar(_) => {}
                }
                out.insert(tag, (ch.sampler.keyframes.clone(), quats, vecs));
            }
        }
        out
    };

    let a = channels_of(&first);
    let b = channels_of(&second);
    assert_eq!(a.len(), b.len(), "channel sets must match");

    // Translation channel: same composed values — in particular the
    // t=1 pivot swing (11, −1, 0) must NOT be double-applied.
    let (ta, _, tva) = a.get(&0).expect("first translation");
    let (tb, _, tvb) = b.get(&0).expect("second translation");
    assert_eq!(ta.len(), tb.len());
    for (va, vb) in tva.iter().zip(tvb) {
        for i in 0..3 {
            assert!(
                (va[i] - vb[i]).abs() < 1e-4,
                "translation drifted: {tva:?} vs {tvb:?}"
            );
        }
    }
    assert!((tvb.last().unwrap()[0] - 11.0).abs() < 1e-4);
    assert!((tvb.last().unwrap()[1] + 1.0).abs() < 1e-4);

    // Rotation channel: same rotations (q ≡ −q allowed).
    let (_, qa, _) = a.get(&1).expect("first rotation");
    let (_, qb, _) = b.get(&1).expect("second rotation");
    assert_eq!(qa.len(), qb.len());
    for (x, y) in qa.iter().zip(qb) {
        let dot: f32 = (0..4).map(|i| x[i] * y[i]).sum();
        assert!(dot.abs() > 1.0 - 1e-4, "rotation drifted: {x:?} vs {y:?}");
    }

    // And the re-decoded rest transform is unchanged.
    let node = second
        .nodes
        .iter()
        .find(|n| n.name.as_deref() == Some("Swinger"))
        .expect("node survives");
    match node.transform {
        oxideav_mesh3d::Transform::Trs { translation, .. } => {
            assert!((translation[0] - 10.0).abs() < 1e-4, "rest drifted");
        }
        oxideav_mesh3d::Transform::Matrix(_) => panic!("expected Trs"),
    }
}
