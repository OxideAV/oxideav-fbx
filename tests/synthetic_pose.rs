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

    // The refined inverse-bind is affine (last row [0, 0, 0, 1]) — the
    // skinning math relies on this. Asserted directly here rather than
    // via `Scene3D::validate()` so the test stays buildable against the
    // published `oxideav-mesh3d` (whose `validate` arrived post-publish).
    assert_eq!(ibm[3], [0.0, 0.0, 0.0, 1.0]);
}

/// Round 226 — bind-pose parent-space (`bone_to_parent`) derivation
/// surfacing.
///
/// Builds a two-bone chain (parent at world translation (10, 0, 0),
/// child at world translation (10, 5, 0)) with a `Pose: BindPose`
/// element posing both bones. After decode every posed bone carries
/// both `fbx:bind_pose` (world matrix, round 97) AND
/// `fbx:bind_pose_parent_local` (parent-space form derived from the
/// scene-graph parent chain, round 226):
///
/// * the parent bone has the implicit-root parent → parent-local
///   equals world;
/// * the child bone's parent-local is `inverse(parent_world) *
///   child_world` — for pure translations that's the difference, i.e.
///   a translation of (0, 5, 0).
///
/// Per `docs/3d/fbx/ufbx/reference.html` §`ufbx_bone_pose.bone_to_parent`:
/// *"Matrix from node local space to parent space. FBX only stores
/// world transformations so this is approximated from the parent
/// world transform."*
#[test]
fn bind_pose_parent_local_chains_through_scene_graph() {
    // Parent bone (id 300) at world (10, 0, 0); child bone (id 301)
    // at world (10, 5, 0). The Connections wire 301 → 300 → root so
    // the scene-graph parent map links them.
    let bone_parent = limb_node(300, b"Root");
    let bone_child = limb_node(301, b"Tip");
    // Skin so that the geometry is bound to both joints — keeps the
    // fixture similar in shape to the first test, exercising the same
    // pose-after-deformer ordering inside `scene::build_scene`.
    let cluster_parent = cluster_no_link(401, &[0, 1], &[0.5, 0.5]);
    let cluster_child = cluster_no_link(402, &[2, 3], &[0.5, 0.5]);
    let skin = skin_deformer(500);
    let parent_world: [f64; 16] = [
        1.0, 0.0, 0.0, 10.0, //
        0.0, 1.0, 0.0, 0.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    let child_world: [f64; 16] = [
        1.0, 0.0, 0.0, 10.0, //
        0.0, 1.0, 0.0, 5.0, //
        0.0, 0.0, 1.0, 0.0, //
        0.0, 0.0, 0.0, 1.0,
    ];
    let pose = Rec::new("Pose")
        .with_prop_i64(900)
        .with_prop_string(b"BindPose\x00\x01Pose")
        .with_prop_string(b"BindPose")
        .with_child(
            Rec::new("PoseNode")
                .with_child(Rec::new("Node").with_prop_i64(300))
                .with_child(Rec::new("Matrix").with_prop_f64_array(&parent_world)),
        )
        .with_child(
            Rec::new("PoseNode")
                .with_child(Rec::new("Node").with_prop_i64(301))
                .with_child(Rec::new("Matrix").with_prop_f64_array(&child_world)),
        );

    let objects = Rec::new("Objects")
        .with_child(quad_geometry())
        .with_child(quad_model())
        .with_child(bone_parent)
        .with_child(bone_child)
        .with_child(cluster_parent)
        .with_child(cluster_child)
        .with_child(skin)
        .with_child(pose);
    let connections = Rec::new("Connections")
        .with_child(connection(b"OO", 100, 200)) // Geometry → Model
        .with_child(connection(b"OO", 200, 0)) // Model → root
        .with_child(connection(b"OO", 300, 0)) // Root bone → root
        .with_child(connection(b"OO", 301, 300)) // Tip bone → Root bone (scene-graph parent)
        .with_child(connection(b"OO", 500, 100)) // Skin → Geometry
        .with_child(connection(b"OO", 401, 500)) // Parent cluster → Skin
        .with_child(connection(b"OO", 402, 500)) // Child cluster → Skin
        .with_child(connection(b"OO", 401, 300)) // Parent cluster → root bone
        .with_child(connection(b"OO", 402, 301)); // Child cluster → tip bone

    let bytes = assemble(7400, vec![objects, connections]);
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("two-bone fixture decodes");

    // Bind-pose world matrix must be on both bones (round 97).
    let posed_nodes: Vec<&oxideav_mesh3d::Node> = scene
        .nodes
        .iter()
        .filter(|n| n.extras.contains_key("fbx:bind_pose"))
        .collect();
    assert_eq!(posed_nodes.len(), 2, "two bones posed");

    // Both bones must carry the parent-local form (round 226).
    let parent_local_count = scene
        .nodes
        .iter()
        .filter(|n| n.extras.contains_key("fbx:bind_pose_parent_local"))
        .count();
    assert_eq!(parent_local_count, 2, "two bones have parent-local form");

    // Identify the bones by their world translation column: the root
    // bone has translation (10, 0, 0), the tip has (10, 5, 0).
    let mut root_bone_local: Option<&serde_json::Value> = None;
    let mut tip_bone_local: Option<&serde_json::Value> = None;
    for node in &scene.nodes {
        let Some(world) = node.extras.get("fbx:bind_pose") else {
            continue;
        };
        let arr = world.as_array().expect("array");
        let ty = arr[7].as_f64().expect("y-translation");
        let local = node
            .extras
            .get("fbx:bind_pose_parent_local")
            .expect("posed bone has parent-local");
        if ty == 0.0 {
            root_bone_local = Some(local);
        } else if (ty - 5.0).abs() < 1e-5 {
            tip_bone_local = Some(local);
        }
    }
    let root_local = root_bone_local
        .expect("root bone matched")
        .as_array()
        .unwrap();
    let tip_local = tip_bone_local
        .expect("tip bone matched")
        .as_array()
        .unwrap();

    // Root bone: parent is the implicit scene root (identity world),
    // so parent-local == world. Translation column = (10, 0, 0).
    assert!((root_local[3].as_f64().unwrap() - 10.0).abs() < 1e-5);
    assert!(root_local[7].as_f64().unwrap().abs() < 1e-5);
    assert!(root_local[11].as_f64().unwrap().abs() < 1e-5);

    // Tip bone: parent-local = inverse(parent_world) * child_world.
    // For pure translations, that's translation by the difference:
    // (10 - 10, 5 - 0, 0 - 0) = (0, 5, 0).
    assert!(tip_local[3].as_f64().unwrap().abs() < 1e-5);
    assert!((tip_local[7].as_f64().unwrap() - 5.0).abs() < 1e-5);
    assert!(tip_local[11].as_f64().unwrap().abs() < 1e-5);
    // Linear part is identity for pure-translation composition.
    assert!((tip_local[0].as_f64().unwrap() - 1.0).abs() < 1e-5);
    assert!((tip_local[5].as_f64().unwrap() - 1.0).abs() < 1e-5);
    assert!((tip_local[10].as_f64().unwrap() - 1.0).abs() < 1e-5);
    // Last row affine.
    assert_eq!(tip_local[15].as_f64(), Some(1.0));
}
