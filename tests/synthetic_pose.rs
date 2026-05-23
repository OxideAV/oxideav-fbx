//! Hand-authored binary FBX fixture exercising round-97 bind-pose
//! (`Pose` element, subtype `"BindPose"`) surfacing.
//!
//! Builds the round-1 quad-Geometry + Mesh-Model topology, adds a
//! single-bone skin whose `Cluster` deliberately *omits* the
//! `TransformLink` sub-record (so the deformer module defaults that
//! joint's inverse-bind to identity), then supplies the bone's world
//! matrix through a `Pose : "BindPose" { PoseNode { Node, Matrix } }`
//! element. The decoded scene should:
//!
//! 1. Carry the bind-pose world matrix on the bone node's
//!    `extras["fbx:bind_pose"]`.
//! 2. Have refined the skeleton's identity inverse-bind to
//!    `inverse(bone_to_world)`.
//!
//! Per `docs/3d/fbx/ufbx/reference.html` §`ufbx_pose` /
//! §`ufbx_bone_pose`: a bind pose stores each bone's world transform
//! ("FBX only stores world transformations"), and the inverse-bind is
//! approximated from it when the cluster lacks an explicit link matrix.

use oxideav_fbx::{FbxDecoder, FBX_MAGIC};
use oxideav_mesh3d::Mesh3DDecoder;

const NULL_RECORD_BYTES_32: usize = 13;

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

fn limb_node(id: i64, name: &[u8]) -> Rec {
    let mut full = name.to_vec();
    full.extend_from_slice(b"\x00\x01Model");
    Rec::new("Model")
        .with_prop_i64(id)
        .with_prop_string(&full)
        .with_prop_string(b"LimbNode")
}

fn skin_deformer(id: i64) -> Rec {
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"Skin\x00\x01Deformer")
        .with_prop_string(b"Skin")
}

/// Cluster *without* a `TransformLink` sub-record — the deformer
/// module defaults the joint's inverse-bind to identity, leaving the
/// bind-pose refinement to fill it in.
fn cluster_no_link(id: i64, indices: &[i32], weights: &[f64]) -> Rec {
    Rec::new("Deformer")
        .with_prop_i64(id)
        .with_prop_string(b"Cluster\x00\x01Deformer")
        .with_prop_string(b"Cluster")
        .with_child(Rec::new("Indexes").with_prop_i32_array(indices))
        .with_child(Rec::new("Weights").with_prop_f64_array(weights))
}

/// `Pose : "BindPose" { PoseNode { Node : bone_id, Matrix : d[16] } }`.
fn bind_pose(id: i64, bone_id: i64, world_row_major: &[f64; 16]) -> Rec {
    let pose_node = Rec::new("PoseNode")
        .with_child(Rec::new("Node").with_prop_i64(bone_id))
        .with_child(Rec::new("Matrix").with_prop_f64_array(world_row_major));
    Rec::new("Pose")
        .with_prop_i64(id)
        .with_prop_string(b"BindPose\x00\x01Pose")
        .with_prop_string(b"BindPose")
        .with_child(pose_node)
}

fn connection(kind: &[u8], child: i64, parent: i64) -> Rec {
    Rec::new("C")
        .with_prop_string(kind)
        .with_prop_i64(child)
        .with_prop_i64(parent)
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

#[test]
fn bind_pose_refines_inverse_bind_and_stashes_node_extras() {
    // Single bone (id 300) influencing all 4 quad vertices. The
    // cluster carries Indexes/Weights but no TransformLink → identity
    // inverse-bind from the deformer module. The bind pose places the
    // bone at world translation (4, 6, -2).
    let bone = limb_node(300, b"Bone");
    let cluster = cluster_no_link(400, &[0, 1, 2, 3], &[1.0, 1.0, 1.0, 1.0]);
    let skin = skin_deformer(500);
    let world = [
        1.0, 0.0, 0.0, 4.0, //
        0.0, 1.0, 0.0, 6.0, //
        0.0, 0.0, 1.0, -2.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    let pose = bind_pose(900, 300, &world);

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(bone)
        .with_child(cluster)
        .with_child(skin)
        .with_child(pose);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200)) // Geometry → Model
        .with_child(connection(b"OO", 200, 0)) // Model → root
        .with_child(connection(b"OO", 300, 0)) // Bone → root
        .with_child(connection(b"OO", 500, 100)) // Skin → Geometry
        .with_child(connection(b"OO", 400, 500)) // Cluster → Skin
        .with_child(connection(b"OO", 400, 300)); // Cluster → Bone

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("pose fixture decodes");

    // One skeleton, one joint.
    assert_eq!(scene.skeletons.len(), 1, "one skeleton");
    let skel = &scene.skeletons[0];
    assert_eq!(skel.joints.len(), 1);
    assert_eq!(skel.inverse_bind_matrices.len(), 1);

    // Inverse-bind refined from the bind pose: inverse of pure
    // translation (4, 6, -2) is (-4, -6, 2).
    let ibm = skel.inverse_bind_matrices[0];
    assert!((ibm[0][3] + 4.0).abs() < 1e-5, "ibm = {ibm:?}");
    assert!((ibm[1][3] + 6.0).abs() < 1e-5, "ibm = {ibm:?}");
    assert!((ibm[2][3] - 2.0).abs() < 1e-5, "ibm = {ibm:?}");

    // Bone node carries the bind-pose world matrix in extras.
    let bone_node = scene
        .nodes
        .iter()
        .find(|n| n.extras.contains_key("fbx:bind_pose"))
        .expect("bone node carries fbx:bind_pose extra");
    let arr = bone_node.extras["fbx:bind_pose"]
        .as_array()
        .expect("bind pose is a JSON array");
    assert_eq!(arr.len(), 16);
    assert_eq!(arr[3].as_f64(), Some(4.0));
    assert_eq!(arr[7].as_f64(), Some(6.0));
    assert_eq!(arr[11].as_f64(), Some(-2.0));

    // Scene stays internally consistent (skeleton IBM affine etc.).
    scene.validate().expect("bind-pose scene validates clean");
}
