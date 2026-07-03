//! Full `Scene3D` → FBX bytes → `Scene3D` round-trip integration tests
//! for the round-377 encoder (`FbxEncoder` / `scene_writer`).
//!
//! Each test builds a `Scene3D` with the `oxideav_mesh3d` typed API,
//! encodes it to bytes via the public [`oxideav_fbx::FbxEncoder`]
//! (`Mesh3DEncoder`), decodes the bytes back through
//! [`oxideav_fbx::FbxDecoder`] (`Mesh3DDecoder`), and asserts the
//! survived scene reproduces the authored geometry / attributes.
//!
//! Provenance: these tests are clean-room — the `Scene3D` shapes are
//! hand-authored against the `oxideav_mesh3d` public API, and the
//! emitted node tree follows the grammar in
//! `docs/3d/fbx/fbx-binary-properties70.md` §4–§7 +
//! `docs/3d/fbx/fbx-ascii-grammar.md` §7b–§8.

use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{
    AlphaMode, Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationTarget,
    AnimationValues, Interpolation, Material, Mesh, Mesh3DDecoder, Mesh3DEncoder, Node, Primitive,
    Scene3D, Topology, Transform,
};

/// A unit quad (two triangles) with per-corner normals + one UV set.
fn quad_with_normals_and_uvs(name: &str) -> Mesh {
    let mut prim = Primitive::new(Topology::Triangles);
    // Two triangles forming a quad in the XY plane.
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
    prim.uvs = vec![vec![
        [0.0, 0.0],
        [1.0, 0.0],
        [1.0, 1.0],
        [0.0, 0.0],
        [1.0, 1.0],
        [0.0, 1.0],
    ]];
    let mut mesh = Mesh::new(Some(name.to_string()));
    mesh.primitives.push(prim);
    mesh
}

fn encode_binary(scene: &Scene3D) -> Vec<u8> {
    FbxEncoder::new().encode(scene).expect("binary encode")
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

#[test]
fn quad_normals_uvs_survive_round_trip() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("Quad"));
    let nid = scene.add_node(Node::new().with_name("QuadNode").with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));

    assert_eq!(scene2.meshes.len(), 1);
    let prim = &scene2.meshes[0].primitives[0];
    assert_eq!(prim.topology, Topology::Triangles);
    assert_eq!(prim.positions.len(), 6, "two-triangle quad → 6 corners");

    // Positions exact.
    assert_eq!(prim.positions[0], [0.0, 0.0, 0.0]);
    assert_eq!(prim.positions[2], [1.0, 1.0, 0.0]);
    assert_eq!(prim.positions[5], [0.0, 1.0, 0.0]);

    // Normals survived as a per-corner buffer all pointing +Z.
    let normals = prim.normals.as_ref().expect("normals round-tripped");
    assert_eq!(normals.len(), 6);
    for n in normals {
        assert!((n[2] - 1.0).abs() < 1e-5, "normal should be +Z, got {n:?}");
    }

    // UV set 0 survived.
    assert_eq!(prim.uvs.len(), 1, "one UV set round-tripped");
    assert_eq!(prim.uvs[0].len(), 6);
    assert_eq!(prim.uvs[0][0], [0.0, 0.0]);
    assert_eq!(prim.uvs[0][2], [1.0, 1.0]);
}

#[test]
fn parent_child_hierarchy_round_trips() {
    // Root → child node tree. The child's mesh-binding + the
    // parent/child edge must survive the Connections walk.
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("Child"));
    let child = scene.add_node(Node::new().with_name("Child").with_mesh(mid));
    let mut parent_node = Node::new().with_name("Parent");
    parent_node.children.push(child);
    let parent = scene.add_node(parent_node);
    scene.roots.push(parent);

    let scene2 = decode(&encode_binary(&scene));

    assert_eq!(scene2.nodes.len(), 2, "parent + child");
    // Exactly one root.
    assert_eq!(scene2.roots.len(), 1, "only the parent is a root");
    let root = &scene2.nodes[scene2.roots[0].0 as usize];
    assert_eq!(root.name.as_deref(), Some("Parent"));
    assert_eq!(root.children.len(), 1, "parent owns the child");
    let kid = &scene2.nodes[root.children[0].0 as usize];
    assert_eq!(kid.name.as_deref(), Some("Child"));
    assert!(kid.mesh.is_some(), "child keeps its mesh");
}

#[test]
fn multiple_materials_round_trip() {
    let mut scene = Scene3D::new();
    let red = scene.add_material(
        Material::new()
            .with_base_color([0.9, 0.1, 0.1, 1.0])
            .with_name("Red"),
    );
    let glass = {
        let mut m = Material::new().with_base_color([0.2, 0.5, 0.9, 0.4]);
        m.alpha_mode = AlphaMode::Blend;
        m.name = Some("Glass".to_string());
        scene.add_material(m)
    };

    let mut mesh_a = quad_with_normals_and_uvs("A");
    mesh_a.primitives[0].material = Some(red);
    let mut mesh_b = quad_with_normals_and_uvs("B");
    mesh_b.primitives[0].material = Some(glass);
    let ma = scene.add_mesh(mesh_a);
    let mb = scene.add_mesh(mesh_b);
    let na = scene.add_node(Node::new().with_mesh(ma));
    let nb = scene.add_node(Node::new().with_mesh(mb));
    scene.roots.push(na);
    scene.roots.push(nb);

    let scene2 = decode(&encode_binary(&scene));

    assert_eq!(scene2.materials.len(), 2, "both materials round-trip");
    // Find the glass material (alpha < 1).
    let glass_mat = scene2
        .materials
        .iter()
        .find(|m| m.base_color[3] < 0.9)
        .expect("blended material survives");
    assert!(matches!(glass_mat.alpha_mode, AlphaMode::Blend));
    assert!((glass_mat.base_color[2] - 0.9).abs() < 1e-2);

    // Each mesh's primitive binds a material.
    for mesh in &scene2.meshes {
        assert!(
            mesh.primitives[0].material.is_some(),
            "mesh `{:?}` kept its material binding",
            mesh.name
        );
    }
}

#[test]
fn transform_translation_scale_survive() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let node = Node::new().with_mesh(mid).with_transform(Transform::Trs {
        translation: [10.0, -5.0, 2.5],
        rotation: [0.0, 0.0, 0.0, 1.0],
        scale: [3.0, 3.0, 3.0],
    });
    let nid = scene.add_node(node);
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    match scene2.nodes[0].transform {
        Transform::Trs {
            translation, scale, ..
        } => {
            assert!((translation[0] - 10.0).abs() < 1e-4);
            assert!((translation[1] + 5.0).abs() < 1e-4);
            assert!((translation[2] - 2.5).abs() < 1e-4);
            assert!((scale[0] - 3.0).abs() < 1e-4);
            assert!((scale[2] - 3.0).abs() < 1e-4);
        }
        other => panic!("expected Trs, got {other:?}"),
    }
}

#[test]
fn ascii_form_round_trips_geometry() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("AsciiQuad"));
    let nid = scene.add_node(Node::new().with_name("N").with_mesh(mid));
    scene.roots.push(nid);

    let bytes = FbxEncoder::new()
        .form(FbxOutputForm::Ascii)
        .encode(&scene)
        .expect("ascii encode");
    let text = std::str::from_utf8(&bytes).expect("ascii output is utf-8");
    assert!(text.starts_with("; FBX"), "ASCII banner present");
    assert!(text.contains("Objects"), "Objects section emitted");

    let scene2 = decode(&bytes);
    assert_eq!(scene2.meshes.len(), 1);
    assert_eq!(scene2.meshes[0].primitives[0].positions.len(), 6);
    assert_eq!(scene2.meshes[0].name.as_deref(), Some("AsciiQuad"));
}

#[test]
fn translation_animation_round_trips() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let nid = scene.add_node(Node::new().with_name("Animated").with_mesh(mid));
    scene.roots.push(nid);

    // A two-keyframe translation channel: origin → (10,0,0) over 1 s.
    let mut anim = Animation::new(Some("Take 001".to_string()));
    anim.channels.push(AnimationChannel {
        target: AnimationTarget {
            node: nid,
            property: AnimationProperty::Translation,
        },
        sampler: AnimationSampler {
            keyframes: vec![0.0, 1.0],
            values: AnimationValues::Vec3(vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]]),
            interpolation: Interpolation::Linear,
        },
    });
    scene.add_animation(anim);

    let scene2 = decode(&encode_binary(&scene));

    assert_eq!(scene2.animations.len(), 1, "animation round-tripped");
    let a = &scene2.animations[0];
    assert_eq!(a.name.as_deref(), Some("Take 001"));
    // One translation channel targeting the animated node.
    let ch = a
        .channels
        .iter()
        .find(|c| c.target.property == AnimationProperty::Translation)
        .expect("translation channel survived");
    // The X component should sweep 0 → 10 across the keyframes.
    match &ch.sampler.values {
        AnimationValues::Vec3(v) => {
            let first_x = v.first().unwrap()[0];
            let last_x = v.last().unwrap()[0];
            assert!(first_x.abs() < 1e-3, "starts at x≈0, got {first_x}");
            assert!((last_x - 10.0).abs() < 1e-2, "ends at x≈10, got {last_x}");
        }
        other => panic!("expected Vec3 translation values, got {other:?}"),
    }
    // Keyframe times survived (0 s and ~1 s).
    assert_eq!(ch.sampler.keyframes.len(), 2);
    assert!((ch.sampler.keyframes[1] - 1.0).abs() < 1e-3);
}

#[test]
fn rotation_animation_round_trips() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let nid = scene.add_node(Node::new().with_name("Spin").with_mesh(mid));
    scene.roots.push(nid);

    // Rotate 90° about Z across 1 s. Build the quaternions via the
    // public mesh3d helper is not exposed, so author them directly:
    // identity → 90° Z (xyzw = (0,0,sin45,cos45)).
    let s = (std::f32::consts::FRAC_PI_4).sin();
    let c = (std::f32::consts::FRAC_PI_4).cos();
    let mut anim = Animation::new(Some("Spin".to_string()));
    anim.channels.push(AnimationChannel {
        target: AnimationTarget {
            node: nid,
            property: AnimationProperty::Rotation,
        },
        sampler: AnimationSampler {
            keyframes: vec![0.0, 1.0],
            values: AnimationValues::Quat(vec![[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, s, c]]),
            interpolation: Interpolation::Linear,
        },
    });
    scene.add_animation(anim);

    let bytes = FbxEncoder::new().encode(&scene).expect("encode");
    let scene2 = decode(&bytes);

    assert_eq!(scene2.animations.len(), 1);
    let ch = scene2.animations[0]
        .channels
        .iter()
        .find(|c| c.target.property == AnimationProperty::Rotation)
        .expect("rotation channel survived");
    // Last keyframe should be a ~90° Z rotation (quaternion z ≈ sin45).
    match &ch.sampler.values {
        AnimationValues::Quat(q) => {
            let last = q.last().unwrap();
            assert!(
                (last[2].abs() - s).abs() < 1e-2,
                "expected z≈{s}, got {last:?}"
            );
        }
        other => panic!("expected Quat rotation values, got {other:?}"),
    }
}

#[test]
fn deflate_compressed_binary_round_trips() {
    // A bigger mesh so the array-deflate threshold actually engages.
    let mut prim = Primitive::new(Topology::Triangles);
    for i in 0..300 {
        let f = i as f32 * 0.01;
        prim.positions.push([f, f * 2.0, f * 3.0]);
    }
    // Pad to a multiple of 3 corners.
    while prim.positions.len() % 3 != 0 {
        prim.positions.push([0.0, 0.0, 0.0]);
    }
    let mut mesh = Mesh::new(Some("Big".to_string()));
    mesh.primitives.push(prim);

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let raw = FbxEncoder::new().encode(&scene).expect("raw encode");
    let compressed = FbxEncoder::new()
        .compress_arrays_at(256)
        .encode(&scene)
        .expect("compressed encode");
    assert!(
        compressed.len() < raw.len(),
        "deflate should shrink the big vertex array (raw {} vs compressed {})",
        raw.len(),
        compressed.len()
    );

    let scene2 = decode(&compressed);
    let n = scene2.meshes[0].primitives[0].positions.len();
    assert_eq!(n % 3, 0);
    assert!(n >= 300, "all corner positions survived");
}

// ---------------------------------------------------------------------
// Round 384 — encoder attribute-layer completeness (multi-UV / vertex
// colours / tangents).
// ---------------------------------------------------------------------

/// Two UV sets (diffuse + lightmap, the canonical pair) each become a
/// `LayerElementUV` record and decode back as two `Primitive::uvs`
/// entries in the same order.
#[test]
fn two_uv_sets_survive_round_trip() {
    let mut mesh = quad_with_normals_and_uvs("MultiUv");
    let set1: Vec<[f32; 2]> = vec![
        [0.0, 0.5],
        [0.5, 0.5],
        [0.5, 1.0],
        [0.0, 0.5],
        [0.5, 1.0],
        [0.0, 1.0],
    ];
    mesh.primitives[0].uvs.push(set1.clone());
    let set0 = mesh.primitives[0].uvs[0].clone();

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    let prim = &scene2.meshes[0].primitives[0];
    assert_eq!(prim.uvs.len(), 2, "both UV sets survive");
    for (a, b) in prim.uvs[0].iter().zip(&set0) {
        assert!((a[0] - b[0]).abs() < 1e-6 && (a[1] - b[1]).abs() < 1e-6);
    }
    for (a, b) in prim.uvs[1].iter().zip(&set1) {
        assert!((a[0] - b[0]).abs() < 1e-6 && (a[1] - b[1]).abs() < 1e-6);
    }
}

/// Vertex-colour sets become `LayerElementColor` records (RGBA
/// `Colors` d-array) and decode back onto `Primitive::colors`.
#[test]
fn vertex_color_sets_survive_round_trip() {
    let mut mesh = quad_with_normals_and_uvs("Colored");
    let set0: Vec<[f32; 4]> = vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
        [1.0, 1.0, 0.0, 0.5],
    ];
    let set1: Vec<[f32; 4]> = vec![[0.25, 0.5, 0.75, 1.0]; 6];
    mesh.primitives[0].colors.push(set0.clone());
    mesh.primitives[0].colors.push(set1.clone());

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    let prim = &scene2.meshes[0].primitives[0];
    assert_eq!(prim.colors.len(), 2, "both colour sets survive");
    for (got, want) in [(&prim.colors[0], &set0), (&prim.colors[1], &set1)] {
        for (a, b) in got.iter().zip(want) {
            for c in 0..4 {
                assert!((a[c] - b[c]).abs() < 1e-6, "colour {a:?} vs {b:?}");
            }
        }
    }
}

/// The canonical glTF-style tangent slot round-trips through the FBX
/// `Tangents` (xyz) + `TangentsW` (handedness sign) split, including a
/// mixed-handedness buffer.
#[test]
fn tangents_survive_round_trip() {
    let mut mesh = quad_with_normals_and_uvs("Tangent");
    let tangents: Vec<[f32; 4]> = vec![
        [1.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, -1.0],
        [0.0, 0.0, 1.0, -1.0],
    ];
    mesh.primitives[0].tangents = Some(tangents.clone());

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    let prim = &scene2.meshes[0].primitives[0];
    let got = prim.tangents.as_ref().expect("tangents survive");
    assert_eq!(got.len(), tangents.len());
    for (a, b) in got.iter().zip(&tangents) {
        for c in 0..4 {
            assert!((a[c] - b[c]).abs() < 1e-6, "tangent {a:?} vs {b:?}");
        }
    }
}

/// An indexed primitive expands its UV / colour / tangent sets through
/// the index buffer exactly like the position stream.
#[test]
fn indexed_attributes_expand_through_index_buffer() {
    use oxideav_mesh3d::Indices;
    let mut prim = Primitive::new(Topology::Triangles);
    // 4 shared vertices, 2 triangles.
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    prim.indices = Some(Indices::U16(vec![0, 1, 2, 0, 2, 3]));
    prim.uvs = vec![vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]];
    prim.colors = vec![vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0, 1.0],
    ]];
    let mut mesh = Mesh::new(Some("Indexed".to_string()));
    mesh.primitives.push(prim);

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    let prim = &scene2.meshes[0].primitives[0];
    assert_eq!(prim.positions.len(), 6, "index buffer expanded");
    assert_eq!(prim.uvs.len(), 1);
    assert_eq!(prim.uvs[0].len(), 6);
    // Corner 4 references shared vertex 2 → uv (1,1) / colour blue.
    assert!((prim.uvs[0][4][0] - 1.0).abs() < 1e-6);
    assert!((prim.uvs[0][4][1] - 1.0).abs() < 1e-6);
    assert_eq!(prim.colors.len(), 1);
    assert!((prim.colors[0][4][2] - 1.0).abs() < 1e-6, "blue expanded");
}

/// A two-material mesh (per-face slot split) re-emits the
/// `LayerElementMaterial` `ByPolygon` table + slot-ordered
/// `Material -> Model` OO connections, and the decode side rebuilds
/// the same slot tables.
#[test]
fn multi_material_slot_table_survives_round_trip() {
    let mut scene = Scene3D::new();
    let mat_a = scene.add_material(
        Material::new()
            .with_name("A")
            .with_base_color([0.9, 0.1, 0.1, 1.0]),
    );
    let mat_b = scene.add_material(
        Material::new()
            .with_name("B")
            .with_base_color([0.1, 0.1, 0.9, 1.0]),
    );

    let mut mesh = quad_with_normals_and_uvs("TwoMat");
    {
        let prim = &mut mesh.primitives[0];
        prim.material = Some(mat_a);
        // Triangle 0 → slot 0, triangle 1 → slot 1 (per-corner form,
        // the shape the decode side stashes).
        prim.extras.insert(
            "fbx:face_material_slots".to_string(),
            serde_json::json!([0, 0, 0, 1, 1, 1]),
        );
        prim.extras.insert(
            "fbx:material_slots".to_string(),
            serde_json::json!([mat_a.0, mat_b.0]),
        );
    }
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    assert_eq!(scene2.materials.len(), 2, "both materials survive");
    let prim = &scene2.meshes[0].primitives[0];
    // Slot 0 binding preserved.
    let m0 = prim.material.expect("slot-0 material bound");
    assert!(
        (scene2.materials[m0.0 as usize].base_color[0] - 0.9).abs() < 1e-3,
        "slot 0 is the red material"
    );
    // Full slot table rebuilt from the OO connections.
    assert_eq!(
        prim.extras.get("fbx:material_slots"),
        Some(&serde_json::json!([m0.0, 1 - m0.0])),
    );
    // Per-face table rebuilt from LayerElementMaterial.
    assert_eq!(
        prim.extras.get("fbx:face_material_slots"),
        Some(&serde_json::json!([0, 0, 0, 1, 1, 1])),
    );
    assert_eq!(
        prim.extras
            .get("fbx:material_mapping")
            .and_then(|v| v.as_str()),
        Some("ByPolygon"),
    );
}
