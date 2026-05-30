//! Hand-authored binary FBX fixtures exercising round-2 round 2:
//! animation extraction (`AnimationStack` / `AnimationLayer` /
//! `AnimationCurveNode` / `AnimationCurve`) and skin-deformer
//! extraction (`Deformer{Skin}` / `Deformer{Cluster}`).
//!
//! Both fixtures share the round-1 quad-Geometry + Mesh-Model
//! topology; the two test modules below extend the `Objects` /
//! `Connections` lists with the new element kinds.

use oxideav_fbx::{
    animation::{euler_xyz_to_quat, KTIME_TICKS_PER_SECOND},
    FbxDecoder, FBX_MAGIC,
};
use oxideav_mesh3d::{AnimationProperty, AnimationValues, Interpolation, Mesh3DDecoder};

const NULL_RECORD_BYTES_32: usize = 13;

/// Same `Rec` builder as the round-1 fixture — duplicated here to
/// keep each integration test file self-contained (cargo compiles
/// every `tests/*.rs` as its own crate).
struct Rec {
    name: String,
    props: Vec<u8>,
    num_props: u32,
    children: Vec<Rec>,
}

impl Rec {
    fn new(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            props: Vec::new(),
            num_props: 0,
            children: Vec::new(),
        }
    }
    fn with_prop_string(mut self, s: &[u8]) -> Self {
        self.props.push(b'S');
        self.props
            .extend_from_slice(&(s.len() as u32).to_le_bytes());
        self.props.extend_from_slice(s);
        self.num_props += 1;
        self
    }
    fn with_prop_i64(mut self, v: i64) -> Self {
        self.props.push(b'L');
        self.props.extend_from_slice(&v.to_le_bytes());
        self.num_props += 1;
        self
    }
    fn with_prop_f64_array(mut self, arr: &[f64]) -> Self {
        self.props.push(b'd');
        self.props
            .extend_from_slice(&(arr.len() as u32).to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes()); // encoding 0
        self.props.extend_from_slice(&0u32.to_le_bytes()); // comp_len 0
        for &v in arr {
            self.props.extend_from_slice(&v.to_le_bytes());
        }
        self.num_props += 1;
        self
    }
    fn with_prop_f32_array(mut self, arr: &[f32]) -> Self {
        self.props.push(b'f');
        self.props
            .extend_from_slice(&(arr.len() as u32).to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        for &v in arr {
            self.props.extend_from_slice(&v.to_le_bytes());
        }
        self.num_props += 1;
        self
    }
    fn with_prop_i32_array(mut self, arr: &[i32]) -> Self {
        self.props.push(b'i');
        self.props
            .extend_from_slice(&(arr.len() as u32).to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        for &v in arr {
            self.props.extend_from_slice(&v.to_le_bytes());
        }
        self.num_props += 1;
        self
    }
    fn with_prop_i64_array(mut self, arr: &[i64]) -> Self {
        self.props.push(b'l');
        self.props
            .extend_from_slice(&(arr.len() as u32).to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        self.props.extend_from_slice(&0u32.to_le_bytes());
        for &v in arr {
            self.props.extend_from_slice(&v.to_le_bytes());
        }
        self.num_props += 1;
        self
    }
    fn with_child(mut self, child: Rec) -> Self {
        self.children.push(child);
        self
    }
}

fn serialize_node(rec: &Rec, out: &mut Vec<u8>, base: usize) {
    let header_off = out.len();
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&rec.num_props.to_le_bytes());
    out.extend_from_slice(&(rec.props.len() as u32).to_le_bytes());
    out.push(rec.name.len() as u8);
    out.extend_from_slice(rec.name.as_bytes());
    out.extend_from_slice(&rec.props);
    if !rec.children.is_empty() {
        for child in &rec.children {
            serialize_node(child, out, base);
        }
        out.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    }
    let end_offset_abs = (base + out.len()) as u32;
    out[header_off..header_off + 4].copy_from_slice(&end_offset_abs.to_le_bytes());
}

/// Build the quad-Geometry (FBX id 100) shared by every test below.
fn quad_geometry() -> Rec {
    let vertices = Rec::new("Vertices")
        .with_prop_f64_array(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0]);
    let pvi = Rec::new("PolygonVertexIndex").with_prop_i32_array(&[0, 1, 2, -4]);
    Rec::new("Geometry")
        .with_prop_i64(100)
        .with_prop_string(b"Quad\x00\x01Geometry")
        .with_prop_string(b"Mesh")
        .with_child(vertices)
        .with_child(pvi)
}

fn quad_model() -> Rec {
    Rec::new("Model")
        .with_prop_i64(200)
        .with_prop_string(b"QuadModel\x00\x01Model")
        .with_prop_string(b"Mesh")
}

fn connection(kind: &[u8], child: i64, parent: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(kind)
        .with_prop_i64(child)
        .with_prop_i64(parent)
}

fn connection_op(kind: &[u8], child: i64, parent: i64, prop: &[u8]) -> Rec {
    Rec::new("C")
        .with_prop_string(kind)
        .with_prop_i64(child)
        .with_prop_i64(parent)
        .with_prop_string(prop)
}

fn assemble(version: u32, top_levels: Vec<Rec>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(FBX_MAGIC);
    buf.extend_from_slice(&[0x1A, 0x00]);
    buf.extend_from_slice(&version.to_le_bytes());
    let base = buf.len();
    let mut body = Vec::new();
    for r in &top_levels {
        serialize_node(r, &mut body, base);
    }
    body.extend_from_slice(&[0u8; NULL_RECORD_BYTES_32]);
    buf.extend_from_slice(&body);
    buf
}

// ---------------------------------------------------------------------
// Animation
// ---------------------------------------------------------------------

/// Build an `AnimationCurve` with the given times (seconds, will be
/// converted to KTime ticks) and values.
fn anim_curve(id: i64, times_secs: &[f32], values: &[f32]) -> Rec {
    let key_time: Vec<i64> = times_secs
        .iter()
        .map(|t| (*t as f64 * KTIME_TICKS_PER_SECOND) as i64)
        .collect();
    Rec::new("AnimationCurve")
        .with_prop_i64(id)
        .with_prop_string(b"AnimCurve\x00\x01AnimCurve")
        .with_prop_string(b"")
        .with_child(Rec::new("KeyTime").with_prop_i64_array(&key_time))
        .with_child(Rec::new("KeyValueFloat").with_prop_f32_array(values))
}

fn anim_curve_node(id: i64) -> Rec {
    Rec::new("AnimationCurveNode")
        .with_prop_i64(id)
        .with_prop_string(b"T\x00\x01AnimCurveNode")
        .with_prop_string(b"")
}

fn anim_layer(id: i64) -> Rec {
    Rec::new("AnimationLayer")
        .with_prop_i64(id)
        .with_prop_string(b"BaseLayer\x00\x01AnimLayer")
        .with_prop_string(b"")
}

fn anim_stack(id: i64) -> Rec {
    Rec::new("AnimationStack")
        .with_prop_i64(id)
        .with_prop_string(b"Take 001\x00\x01AnimStack")
        .with_prop_string(b"")
}

#[test]
fn animation_translation_three_components_decoded() {
    // Animate Lcl Translation with t=0.0..1.0 going (0,0,0)..(2,3,4).
    let cx = anim_curve(1000, &[0.0, 1.0], &[0.0, 2.0]);
    let cy = anim_curve(1001, &[0.0, 1.0], &[0.0, 3.0]);
    let cz = anim_curve(1002, &[0.0, 1.0], &[0.0, 4.0]);
    let cn = anim_curve_node(2000);
    let layer = anim_layer(3000);
    let stack = anim_stack(4000);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(cx)
        .with_child(cy)
        .with_child(cz)
        .with_child(cn)
        .with_child(layer)
        .with_child(stack);
    let connections = Rec::new("Connections")
        // Geometry/Model wiring (round 1).
        .with_child(connection(b"OO", 100, 200))
        .with_child(connection(b"OO", 200, 0))
        // Animation wiring.
        .with_child(connection_op(b"OP", 1000, 2000, b"d|X"))
        .with_child(connection_op(b"OP", 1001, 2000, b"d|Y"))
        .with_child(connection_op(b"OP", 1002, 2000, b"d|Z"))
        .with_child(connection_op(b"OP", 2000, 200, b"Lcl Translation"))
        .with_child(connection(b"OO", 2000, 3000))
        .with_child(connection(b"OO", 3000, 4000));

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("animation fixture decodes");

    assert_eq!(scene.animations.len(), 1, "one animation per stack");
    let anim = &scene.animations[0];
    assert_eq!(anim.name.as_deref(), Some("Take 001"));
    assert_eq!(anim.channels.len(), 1, "one Translation channel");
    let ch = &anim.channels[0];
    assert_eq!(ch.target.property, AnimationProperty::Translation);
    assert_eq!(ch.sampler.interpolation, Interpolation::Linear);
    match &ch.sampler.values {
        AnimationValues::Vec3(v) => {
            assert_eq!(v.len(), 2);
            assert_eq!(v[0], [0.0, 0.0, 0.0]);
            assert!((v[1][0] - 2.0).abs() < 1e-3);
            assert!((v[1][1] - 3.0).abs() < 1e-3);
            assert!((v[1][2] - 4.0).abs() < 1e-3);
        }
        other => panic!("expected Vec3 values, got {other:?}"),
    }
    assert_eq!(ch.sampler.keyframes.len(), 2);
    assert!((ch.sampler.keyframes[0]).abs() < 1e-3);
    assert!((ch.sampler.keyframes[1] - 1.0).abs() < 1e-3);
}

#[test]
fn animation_rotation_to_quaternion() {
    // Single keyframe rotating 90 degrees about X.
    let cx = anim_curve(1000, &[0.0], &[90.0]);
    let cy = anim_curve(1001, &[0.0], &[0.0]);
    let cz = anim_curve(1002, &[0.0], &[0.0]);
    let cn = anim_curve_node(2000);
    let layer = anim_layer(3000);
    let stack = anim_stack(4000);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(cx)
        .with_child(cy)
        .with_child(cz)
        .with_child(cn)
        .with_child(layer)
        .with_child(stack);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200))
        .with_child(connection(b"OO", 200, 0))
        .with_child(connection_op(b"OP", 1000, 2000, b"d|X"))
        .with_child(connection_op(b"OP", 1001, 2000, b"d|Y"))
        .with_child(connection_op(b"OP", 1002, 2000, b"d|Z"))
        .with_child(connection_op(b"OP", 2000, 200, b"Lcl Rotation"))
        .with_child(connection(b"OO", 2000, 3000))
        .with_child(connection(b"OO", 3000, 4000));

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("rotation fixture decodes");

    let anim = &scene.animations[0];
    let ch = &anim.channels[0];
    assert_eq!(ch.target.property, AnimationProperty::Rotation);
    let expected = euler_xyz_to_quat([90.0, 0.0, 0.0]);
    match &ch.sampler.values {
        AnimationValues::Quat(q) => {
            assert_eq!(q.len(), 1);
            for k in 0..4 {
                assert!(
                    (q[0][k] - expected[k]).abs() < 1e-4,
                    "quat[{k}] = {} expected {}",
                    q[0][k],
                    expected[k]
                );
            }
        }
        other => panic!("expected Quat, got {other:?}"),
    }
}

// ---------------------------------------------------------------------
// Skin deformer
// ---------------------------------------------------------------------

fn skin_deformer(id: i64) -> Rec {
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"Skin\x00\x01Deformer")
        .with_prop_string(b"Skin")
}

fn cluster_deformer(id: i64, indices: &[i32], weights: &[f64]) -> Rec {
    // Identity Transform + TransformLink → identity inverse-bind.
    let identity_16 = [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ];
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"Cluster\x00\x01Deformer")
        .with_prop_string(b"Cluster")
        .with_child(Rec::new("Indexes").with_prop_i32_array(indices))
        .with_child(Rec::new("Weights").with_prop_f64_array(weights))
        .with_child(Rec::new("Transform").with_prop_f64_array(&identity_16))
        .with_child(Rec::new("TransformLink").with_prop_f64_array(&identity_16))
}

fn limb_node(id: i64, name: &[u8]) -> Rec {
    let mut full = name.to_vec();
    full.extend_from_slice(b"\x00\x01Model");
    Rec::new("Model")
        .with_prop_i64(id)
        .with_prop_string(&full)
        .with_prop_string(b"LimbNode")
}

#[test]
fn skin_deformer_wires_skeleton_and_per_corner_weights() {
    // 2-bone skin: bone A (id 300) influences vertices 0,1 with weight 1.0
    //              bone B (id 301) influences vertices 2,3 with weight 1.0
    // Each shared vertex appears once in the corner buffer (the quad
    // fan-triangulates as [0,1,2, 0,2,3] so v0 appears at corners 0 & 3,
    // v2 at corners 2 & 4, etc — verifying the per-corner expansion).
    let bone_a = limb_node(300, b"BoneA");
    let bone_b = limb_node(301, b"BoneB");
    let cluster_a = cluster_deformer(400, &[0, 1], &[1.0, 1.0]);
    let cluster_b = cluster_deformer(401, &[2, 3], &[1.0, 1.0]);
    let skin = skin_deformer(500);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(bone_a)
        .with_child(bone_b)
        .with_child(cluster_a)
        .with_child(cluster_b)
        .with_child(skin);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200)) // Geometry → Model
        .with_child(connection(b"OO", 200, 0)) // Model → root
        .with_child(connection(b"OO", 300, 0)) // BoneA → root
        .with_child(connection(b"OO", 301, 0)) // BoneB → root
        .with_child(connection(b"OO", 500, 100)) // Skin → Geometry
        .with_child(connection(b"OO", 400, 500)) // ClusterA → Skin
        .with_child(connection(b"OO", 401, 500)) // ClusterB → Skin
        .with_child(connection(b"OO", 400, 300)) // ClusterA → BoneA
        .with_child(connection(b"OO", 401, 301)); // ClusterB → BoneB

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("skin fixture decodes");

    assert_eq!(scene.skeletons.len(), 1, "one skeleton");
    assert_eq!(scene.skins.len(), 1, "one skin");
    let skel = &scene.skeletons[0];
    assert_eq!(skel.joints.len(), 2);
    assert_eq!(skel.inverse_bind_matrices.len(), 2);

    // Mesh primitive should now carry joints + weights buffers.
    let mesh = &scene.meshes[0];
    let prim = &mesh.primitives[0];
    let joints = prim.joints.as_ref().expect("joints set on skinned prim");
    let weights = prim.weights.as_ref().expect("weights set on skinned prim");
    assert_eq!(joints.len(), 6, "6 corner positions, 6 joint quads");
    assert_eq!(weights.len(), 6);

    // Corner 0 (v0 = vertex shared-index 0) → BoneA only.
    assert_eq!(joints[0][0], 0);
    assert!((weights[0][0] - 1.0).abs() < 1e-6);
    // Corner 2 (v2) → BoneB only.
    assert_eq!(joints[2][0], 1);
    assert!((weights[2][0] - 1.0).abs() < 1e-6);
    // Corner 5 (v3) → BoneB only.
    assert_eq!(joints[5][0], 1);
    assert!((weights[5][0] - 1.0).abs() < 1e-6);

    // Mesh-bearing node should have skin attached.
    let model_node = scene
        .nodes
        .iter()
        .find(|n| n.mesh.is_some())
        .expect("Model node present");
    assert!(
        model_node.skin.is_some(),
        "Skin attached to mesh-bearing node"
    );
}

// ---------------------------------------------------------------------
// Blend shape (morph target) deformer
// ---------------------------------------------------------------------

fn blend_deformer(id: i64) -> Rec {
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"BlendShape\x00\x01Deformer")
        .with_prop_string(b"BlendShape")
}

fn blend_channel(id: i64) -> Rec {
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"Smile\x00\x01BlendShapeChannel")
        .with_prop_string(b"BlendShapeChannel")
}

fn shape_geometry(id: i64, indexes: &[i32], deltas: &[f64]) -> Rec {
    Rec::new("Geometry")
        .with_prop_i64(id)
        .with_prop_string(b"Smile\x00\x01Shape")
        .with_prop_string(b"Shape")
        .with_child(Rec::new("Indexes").with_prop_i32_array(indexes))
        .with_child(Rec::new("Vertices").with_prop_f64_array(deltas))
}

#[test]
fn blend_shape_emits_morph_target_with_per_corner_deltas() {
    // Single morph target shifting v2 by +0.5 in Z.
    let shape = shape_geometry(600, &[2], &[0.0, 0.0, 0.5]);
    let channel = blend_channel(700);
    let blend = blend_deformer(800);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(shape)
        .with_child(channel)
        .with_child(blend);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200))
        .with_child(connection(b"OO", 200, 0))
        .with_child(connection(b"OO", 800, 100)) // BlendShape → Geometry
        .with_child(connection(b"OO", 700, 800)) // Channel → BlendShape
        .with_child(connection(b"OO", 600, 700)); // Shape → Channel

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("blend-shape fixture decodes");

    let mesh = &scene.meshes[0];
    assert_eq!(mesh.primitives[0].targets.len(), 1, "one morph target");
    assert_eq!(mesh.weights, vec![0.0]);
    let tgt = &mesh.primitives[0].targets[0];
    let pos = tgt.position.as_ref().expect("position deltas present");
    assert_eq!(pos.len(), 6, "per-corner deltas (6 corners)");
    // v2 lives at corners 2 and 4 of the fan-triangulated quad.
    assert_eq!(pos[2], [0.0, 0.0, 0.5]);
    assert_eq!(pos[4], [0.0, 0.0, 0.5]);
    // Other corners stay zero.
    assert_eq!(pos[0], [0.0, 0.0, 0.0]);
    assert_eq!(pos[1], [0.0, 0.0, 0.0]);
    assert_eq!(pos[3], [0.0, 0.0, 0.0]);
    assert_eq!(pos[5], [0.0, 0.0, 0.0]);
}

#[test]
fn blend_shape_animation_wires_morph_weight_channel() {
    // Combine a BlendShape with an AnimationCurveNode driving the
    // channel's DeformPercent. Expect one Animation with one
    // Scalar-valued MorphWeights channel.
    let shape = shape_geometry(600, &[2], &[0.0, 0.0, 0.5]);
    let channel = blend_channel(700);
    let blend = blend_deformer(800);

    // Animation: t=0..1 going 0..100 (FBX DeformPercent is 0..100).
    let curve = anim_curve(1000, &[0.0, 1.0], &[0.0, 100.0]);
    let cn = Rec::new("AnimationCurveNode")
        .with_prop_i64(2000)
        .with_prop_string(b"DeformPercent\x00\x01AnimCurveNode")
        .with_prop_string(b"");
    let layer = anim_layer(3000);
    let stack = anim_stack(4000);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(shape)
        .with_child(channel)
        .with_child(blend)
        .with_child(curve)
        .with_child(cn)
        .with_child(layer)
        .with_child(stack);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200))
        .with_child(connection(b"OO", 200, 0))
        .with_child(connection(b"OO", 800, 100))
        .with_child(connection(b"OO", 700, 800))
        .with_child(connection(b"OO", 600, 700))
        // Animation plumbing: Curve -> CurveNode (d|DeformPercent),
        // CurveNode -> Channel (DeformPercent), CurveNode -> Layer,
        // Layer -> Stack.
        .with_child(connection_op(b"OP", 1000, 2000, b"d|DeformPercent"))
        .with_child(connection_op(b"OP", 2000, 700, b"DeformPercent"))
        .with_child(connection(b"OO", 2000, 3000))
        .with_child(connection(b"OO", 3000, 4000));

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("blend+anim fixture decodes");

    assert_eq!(scene.animations.len(), 1);
    let anim = &scene.animations[0];
    assert_eq!(anim.channels.len(), 1, "one DeformPercent channel");
    let ch = &anim.channels[0];
    assert_eq!(ch.target.property, AnimationProperty::MorphWeights);
    match &ch.sampler.values {
        AnimationValues::Scalar(v) => {
            assert_eq!(v.len(), 2);
            assert_eq!(v[0], 0.0);
            assert_eq!(v[1], 100.0);
        }
        other => panic!("expected Scalar, got {other:?}"),
    }
}
