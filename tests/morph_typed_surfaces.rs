//! Round-trip pins for the `oxideav-mesh3d` 0.0.6 typed morph
//! surfaces: `Mesh::target_names`, the sampled-`MorphWeights`
//! synthesis path (`AnimationSampler::morph_weights` /
//! `morph_weights_cubic` + `Animation::with_channel`) and its
//! lossless read-back, and `Scene3D::validate` on both the authored
//! and the re-decoded scene, through the binary and ASCII forms.

use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{
    Animation, AnimationProperty, AnimationSampler, Interpolation, Mesh, Mesh3DDecoder,
    Mesh3DEncoder, MorphTarget, Node, NodeId, Primitive, Scene3D, Topology,
};

fn quad(name: &str) -> Mesh {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
    let mut mesh = Mesh::new(Some(name.to_string()));
    mesh.primitives.push(prim);
    mesh
}

fn delta(corner: usize, d: [f32; 3]) -> MorphTarget {
    let mut pos = vec![[0.0f32; 3]; 6];
    pos[corner] = d;
    MorphTarget::with_deltas(Some(pos), None, None)
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

/// Three named targets + a typed linear `MorphWeights` sampler:
/// the authored scene validates, both wire forms re-decode to a
/// validating scene, `find_target` resolves the same slots, and
/// `morph_weight_frames` reads back the authored key vectors
/// (0..100 wire percentages round-trip to f32 blend factors).
#[test]
fn typed_names_and_sampled_weights_validate_both_forms() {
    let mut scene = Scene3D::new();
    let mut mesh = quad("Face").with_target_names(["Smile", "Frown", "Blink"]);
    {
        let prim = &mut mesh.primitives[0];
        prim.targets.push(delta(0, [0.0, 0.0, 1.5]));
        prim.targets.push(delta(2, [-0.5, 0.0, 0.0]));
        prim.targets.push(delta(5, [0.0, 0.25, 0.0]));
    }
    mesh.weights = vec![0.25, 0.0, 1.0];
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_name("FaceNode").with_mesh(mid));
    scene.roots.push(nid);

    let frames = vec![
        vec![0.0f32, 1.0, 0.5],
        vec![0.5, 0.75, 0.5],
        vec![1.0, 0.0, 0.25],
    ];
    let sampler =
        AnimationSampler::morph_weights(vec![0.0, 0.5, 1.0], frames.clone(), Interpolation::Linear)
            .expect("well-formed sampler");
    scene.add_animation(Animation::new(Some("Talk".to_string())).with_channel(
        nid,
        AnimationProperty::MorphWeights,
        sampler,
    ));
    assert_eq!(scene.validate(), Ok(()), "authored scene validates");

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let scene2 = decode(&encode(&scene, form));
        assert_eq!(scene2.validate(), Ok(()), "{form:?} re-decode validates");

        let mesh2 = &scene2.meshes[0];
        assert_eq!(
            mesh2.target_names,
            vec!["Smile".to_string(), "Frown".into(), "Blink".into()]
        );
        assert_eq!(mesh2.find_target("Blink"), Some(2));
        assert_eq!(mesh2.target_name(1), Some("Frown"));
        assert_eq!(mesh2.primitives[0].targets.len(), 3);
        assert!(
            !mesh2.primitives[0]
                .extras
                .contains_key("fbx:morph_target_names"),
            "names live on the typed field only"
        );
        assert_eq!(mesh2.weights.len(), 3);
        assert!((mesh2.weights[0] - 0.25).abs() < 1e-6);
        assert!((mesh2.weights[2] - 1.0).abs() < 1e-6);

        let node2 = scene2
            .nodes
            .iter()
            .position(|n| n.mesh.is_some())
            .map(|i| NodeId(i as u32))
            .expect("mesh node");
        let ch = scene2.animations[0]
            .channel_for(node2, AnimationProperty::MorphWeights)
            .expect("one MorphWeights channel");
        assert_eq!(ch.sampler.interpolation, Interpolation::Linear);
        assert_eq!(ch.sampler.morph_weight_stride(), Some(3));
        let back = ch.sampler.morph_weight_frames().expect("read-back");
        assert_eq!(back.len(), 3);
        for (k, (got, want)) in back.iter().zip(&frames).enumerate() {
            assert!((ch.sampler.keyframes[k] - [0.0f32, 0.5, 1.0][k]).abs() < 1e-6);
            for (g, w) in got.iter().zip(want) {
                assert!(
                    (g - w).abs() < 1e-6,
                    "{form:?} frame {k}: {got:?} vs {want:?}"
                );
            }
        }
    }
}

/// A `CubicSpline` `MorphWeights` sampler has no FBX curve-key
/// equivalent for its tangent triples; the wire carries the centre
/// *value* vectors, so the re-decoded (linear) sampler's frames equal
/// the authored `values` table — never a 3×-strided one.
#[test]
fn cubic_sampler_emits_centre_values() {
    let mut scene = Scene3D::new();
    let mut mesh = quad("Cubic").with_target_names(["A", "B"]);
    mesh.primitives[0].targets.push(delta(1, [0.0, 0.0, 1.0]));
    mesh.primitives[0].targets.push(delta(3, [1.0, 0.0, 0.0]));
    mesh.weights = vec![0.0, 0.0];
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_name("CubicNode").with_mesh(mid));
    scene.roots.push(nid);

    let values = vec![vec![0.0f32, 1.0], vec![1.0, 0.0]];
    let sampler = AnimationSampler::morph_weights_cubic(
        vec![0.0, 1.0],
        vec![vec![0.0, 0.0], vec![0.0, 0.0]],
        values.clone(),
        vec![vec![0.5, -0.5], vec![0.5, -0.5]],
    )
    .expect("well-formed cubic sampler");
    assert_eq!(sampler.morph_weight_frame(1), Some(&[1.0f32, 0.0][..]));
    scene.add_animation(Animation::new(Some("Cubic".to_string())).with_channel(
        nid,
        AnimationProperty::MorphWeights,
        sampler,
    ));
    assert_eq!(scene.validate(), Ok(()));

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let scene2 = decode(&encode(&scene, form));
        assert_eq!(scene2.validate(), Ok(()));
        let ch = scene2.animations[0]
            .channels
            .iter()
            .find(|c| c.target.property == AnimationProperty::MorphWeights)
            .expect("MorphWeights channel");
        assert_eq!(ch.sampler.interpolation, Interpolation::Linear);
        assert_eq!(ch.sampler.morph_weight_stride(), Some(2));
        let back = ch.sampler.morph_weight_frames().expect("read-back");
        assert_eq!(back.len(), 2, "{form:?}: two keys, not six");
        for (got, want) in back.iter().zip(&values) {
            for (g, w) in got.iter().zip(want) {
                assert!((g - w).abs() < 1e-6);
            }
        }
    }
}

/// The pre-0.0.6 `Primitive::extras["fbx:morph_target_names"]`
/// side-channel still drives the emitted channel names when the typed
/// field is empty, and the re-decode surfaces them typed.
#[test]
fn extras_side_channel_fallback_lands_on_typed_names() {
    let mut scene = Scene3D::new();
    let mut mesh = quad("Legacy");
    mesh.primitives[0].targets.push(delta(0, [0.0, 0.0, 1.0]));
    mesh.primitives[0].extras.insert(
        "fbx:morph_target_names".to_string(),
        serde_json::json!(["Legacy"]),
    );
    mesh.weights = vec![0.5];
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_name("LegacyNode").with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode(&scene, FbxOutputForm::Binary));
    assert_eq!(scene2.validate(), Ok(()));
    assert_eq!(scene2.meshes[0].target_names, vec!["Legacy".to_string()]);
}

/// Typed names win over a stale extras side-channel when both are
/// present.
#[test]
fn typed_names_take_precedence_over_extras() {
    let mut scene = Scene3D::new();
    let mut mesh = quad("Both").with_target_names(["Typed"]);
    mesh.primitives[0].targets.push(delta(0, [0.0, 0.0, 1.0]));
    mesh.primitives[0].extras.insert(
        "fbx:morph_target_names".to_string(),
        serde_json::json!(["Stale"]),
    );
    mesh.weights = vec![0.0];
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_name("BothNode").with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode(&scene, FbxOutputForm::Ascii));
    assert_eq!(scene2.meshes[0].target_names, vec!["Typed".to_string()]);
}

/// A `Node::weights` per-instance override is what the emitted
/// `DeformPercent` rest record carries, and the re-decoded scene —
/// where the override has been folded into the (now unshared) mesh's
/// rest weights — validates.
#[test]
fn node_weight_override_folds_into_rest_state_and_validates() {
    let mut scene = Scene3D::new();
    let mut mesh = quad("Override").with_target_names(["Open"]);
    mesh.primitives[0].targets.push(delta(4, [0.0, 0.0, 2.0]));
    mesh.weights = vec![0.1];
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(
        Node::new()
            .with_name("OverrideNode")
            .with_mesh(mid)
            .with_weights([0.8f32]),
    );
    scene.roots.push(nid);
    assert_eq!(scene.validate(), Ok(()));

    let scene2 = decode(&encode(&scene, FbxOutputForm::Binary));
    assert_eq!(scene2.validate(), Ok(()));
    assert!((scene2.meshes[0].weights[0] - 0.8).abs() < 1e-6);
    assert_eq!(scene2.meshes[0].target_names, vec!["Open".to_string()]);
}
