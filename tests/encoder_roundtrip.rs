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
fn offset_width_parity_32bit_vs_64bit() {
    // The same authored scene must survive identically whether the
    // encoder writes the pre-7500 32-bit Node Record offset layout
    // (`EndOffset` / `NumProperties` / `PropertyListLen` as u32) or the
    // >= 7500 64-bit layout (those three widen to u64). This proves the
    // 32-bit and 64-bit reader/writer offset variants are equivalent
    // (per docs/3d/fbx/blender-fbx-binary-format.html
    // §"Version-dependent quirks").
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("Quad"));
    let nid = scene.add_node(Node::new().with_name("QuadNode").with_mesh(mid));
    scene.roots.push(nid);

    let bytes32 = FbxEncoder::new().version(7400).encode(&scene).unwrap();
    let bytes64 = FbxEncoder::new().version(7700).encode(&scene).unwrap();

    // Header versions differ, so the byte streams must not be identical.
    assert_ne!(bytes32, bytes64, "32-bit and 64-bit layouts differ on disk");

    let s32 = decode(&bytes32);
    let s64 = decode(&bytes64);

    let p32 = &s32.meshes[0].primitives[0];
    let p64 = &s64.meshes[0].primitives[0];

    // Geometry, normals, and UVs decode identically from both layouts.
    assert_eq!(p32.positions, p64.positions, "positions parity");
    assert_eq!(p32.normals, p64.normals, "normals parity");
    assert_eq!(p32.uvs, p64.uvs, "uv parity");
    assert_eq!(p64.positions.len(), 6);
    assert_eq!(p64.normals.as_ref().unwrap().len(), 6);
}

#[test]
fn offset_width_parity_survives_with_deflate() {
    // Same parity check, but with array deflate (`Encoding == 1`) turned
    // on, so the compressed-array path is exercised under both the
    // 32-bit and 64-bit Node Record layouts.
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("Quad"));
    let nid = scene.add_node(Node::new().with_name("QuadNode").with_mesh(mid));
    scene.roots.push(nid);

    let enc = |ver: u32| {
        FbxEncoder::new()
            .version(ver)
            .compress_arrays_at(1)
            .encode(&scene)
            .unwrap()
    };
    let s32 = decode(&enc(7400));
    let s64 = decode(&enc(7700));

    let p32 = &s32.meshes[0].primitives[0];
    let p64 = &s64.meshes[0].primitives[0];
    assert_eq!(p32.positions, p64.positions);
    assert_eq!(p32.normals, p64.normals);
    assert_eq!(p32.uvs, p64.uvs);
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

// ---------------------------------------------------------------------
// Round 384 — encoder deformer emission (skin + blend shapes) and
// morph-weight animation channels.
// ---------------------------------------------------------------------

/// A two-bone skinned quad survives decode → encode → decode: the
/// Skin / Cluster tree rebuilds the skeleton (joint order preserved),
/// the exact inverse-bind matrices, the per-corner top-4 joint /
/// weight buffers, and the node's skin binding.
#[test]
fn skinned_mesh_survives_round_trip() {
    use oxideav_mesh3d::{Skeleton, Skin};

    let mut scene = Scene3D::new();
    let mut mesh = quad_with_normals_and_uvs("Skinned");
    {
        let prim = &mut mesh.primitives[0];
        // Corners 0..2 → joint 0 only; corners 3..5 → blended 0.75/0.25.
        prim.joints = Some(vec![
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 0, 0],
        ]);
        prim.weights = Some(vec![
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.75, 0.25, 0.0, 0.0],
            [0.75, 0.25, 0.0, 0.0],
            [0.75, 0.25, 0.0, 0.0],
        ]);
    }
    let mid = scene.add_mesh(mesh);
    let bone0 = scene.add_node(Node::new().with_name("B0"));
    let bone1 = scene.add_node(Node::new().with_name("B1"));

    let mut skel = Skeleton::new();
    skel.joints.push(bone0);
    skel.joints.push(bone1);
    // Distinct, exactly-representable inverse-bind translations.
    let ib0 = [
        [1.0, 0.0, 0.0, -2.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    let ib1 = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -3.5],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    skel.inverse_bind_matrices.push(ib0);
    skel.inverse_bind_matrices.push(ib1);
    let skel_id = scene.add_skeleton(skel);
    let skin_id = scene.add_skin(Skin::new(skel_id));

    let mut mesh_node = Node::new().with_name("SkinnedNode").with_mesh(mid);
    mesh_node.skin = Some(skin_id);
    let nid = scene.add_node(mesh_node);
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    assert_eq!(scene2.skins.len(), 1, "one skin rebuilt");
    assert_eq!(scene2.skeletons.len(), 1, "one skeleton rebuilt");
    let skel2 = &scene2.skeletons[0];
    assert_eq!(skel2.joints.len(), 2, "both joints rebuilt");
    // Joint order preserved (Cluster -> Skin connection order).
    assert_eq!(
        scene2.nodes[skel2.joints[0].0 as usize].name.as_deref(),
        Some("B0")
    );
    assert_eq!(
        scene2.nodes[skel2.joints[1].0 as usize].name.as_deref(),
        Some("B1")
    );
    // Inverse-bind matrices survive exactly (Transform = inverse-bind,
    // TransformLink = identity — no float inversion on either side).
    for (got, want) in [
        (&skel2.inverse_bind_matrices[0], &ib0),
        (&skel2.inverse_bind_matrices[1], &ib1),
    ] {
        for r in 0..4 {
            for c in 0..4 {
                assert!(
                    (got[r][c] - want[r][c]).abs() < 1e-6,
                    "inverse-bind [{r}][{c}]: {} vs {}",
                    got[r][c],
                    want[r][c]
                );
            }
        }
    }
    // The mesh's node carries the skin.
    let skinned = scene2
        .nodes
        .iter()
        .find(|n| n.name.as_deref() == Some("SkinnedNode"))
        .expect("mesh node survives");
    assert!(skinned.skin.is_some(), "node.skin rebound");
    // Per-corner buffers.
    let prim = &scene2.meshes[0].primitives[0];
    let joints = prim.joints.as_ref().expect("joints buffer");
    let weights = prim.weights.as_ref().expect("weights buffer");
    assert_eq!(joints.len(), 6);
    assert!((weights[0][0] - 1.0).abs() < 1e-6, "corner 0 fully joint 0");
    assert_eq!(joints[0][0], 0);
    assert!((weights[3][0] - 0.75).abs() < 1e-6, "corner 3 blended");
    assert!((weights[3][1] - 0.25).abs() < 1e-6);
    assert_eq!(joints[3][0], 0);
    assert_eq!(joints[3][1], 1);
}

/// A morph target (sparse position + normal deltas) and its
/// MorphWeights animation channel survive the round trip through the
/// BlendShape / BlendShapeChannel / Geometry{Shape} tree + the
/// DeformPercent curve chain.
#[test]
fn morph_target_and_weight_animation_survive_round_trip() {
    use oxideav_mesh3d::MorphTarget;

    let mut scene = Scene3D::new();
    let mut mesh = quad_with_normals_and_uvs("Morphed");
    {
        let prim = &mut mesh.primitives[0];
        let mut tgt = MorphTarget::new();
        // Sparse deltas: corners 0 and 4 move; the rest stay.
        let mut pos = vec![[0.0f32; 3]; 6];
        pos[0] = [0.0, 0.0, 1.5];
        pos[4] = [0.25, -0.5, 0.0];
        let mut nrm = vec![[0.0f32; 3]; 6];
        nrm[0] = [0.0, 1.0, 0.0];
        tgt.position = Some(pos.clone());
        tgt.normal = Some(nrm.clone());
        prim.targets.push(tgt);
        mesh.weights.push(0.0);
    }
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_name("MorphNode").with_mesh(mid));
    scene.roots.push(nid);

    // MorphWeights animation: DeformPercent ramps 0 → 100.
    let mut anim = Animation::new(Some("MorphClip".to_string()));
    anim.channels.push(AnimationChannel {
        target: AnimationTarget {
            node: nid,
            property: AnimationProperty::MorphWeights,
        },
        sampler: AnimationSampler {
            keyframes: vec![0.0, 1.0, 2.0],
            values: AnimationValues::Scalar(vec![0.0, 50.0, 100.0]),
            interpolation: Interpolation::Linear,
        },
    });
    scene.add_animation(anim);

    let scene2 = decode(&encode_binary(&scene));

    // Morph target rebuilt on the primitive.
    let prim = &scene2.meshes[0].primitives[0];
    assert_eq!(prim.targets.len(), 1, "one morph target");
    let tgt = &prim.targets[0];
    let pos = tgt.position.as_ref().expect("position deltas");
    assert_eq!(pos.len(), 6);
    assert!((pos[0][2] - 1.5).abs() < 1e-6);
    assert!((pos[4][0] - 0.25).abs() < 1e-6);
    assert!((pos[4][1] + 0.5).abs() < 1e-6);
    assert_eq!(pos[1], [0.0, 0.0, 0.0], "untouched corner stays zero");
    let nrm = tgt.normal.as_ref().expect("normal deltas");
    assert!((nrm[0][1] - 1.0).abs() < 1e-6);

    // MorphWeights channel rebuilt through the DeformPercent chain.
    assert_eq!(scene2.animations.len(), 1);
    let anim2 = &scene2.animations[0];
    assert_eq!(anim2.name.as_deref(), Some("MorphClip"));
    let ch = anim2
        .channels
        .iter()
        .find(|c| c.target.property == AnimationProperty::MorphWeights)
        .expect("MorphWeights channel survives");
    assert_eq!(ch.sampler.keyframes.len(), 3);
    match &ch.sampler.values {
        AnimationValues::Scalar(v) => {
            assert!((v[0] - 0.0).abs() < 1e-4);
            assert!((v[1] - 50.0).abs() < 1e-4);
            assert!((v[2] - 100.0).abs() < 1e-4);
        }
        other => panic!("expected Scalar values, got {other:?}"),
    }
    // The channel targets the mesh's node.
    assert_eq!(
        scene2.nodes[ch.target.node.0 as usize].name.as_deref(),
        Some("MorphNode")
    );
}

// ---------------------------------------------------------------------
// Round 384 — encoder light / camera NodeAttribute emission.
// ---------------------------------------------------------------------

/// Point / Directional / Spot lights round-trip through the
/// `NodeAttribute : "Light"` P-record set (LightType / Color /
/// Intensity ×100 / DecayType + DecayStart / cone angles).
#[test]
fn lights_survive_round_trip() {
    use oxideav_mesh3d::Light;

    let mut scene = Scene3D::new();
    let point = scene.add_light(Light::Point {
        color: [1.0, 0.5, 0.25],
        intensity: 2.0,
        range: Some(12.5),
    });
    let sun = scene.add_light(Light::Directional {
        color: [1.0, 1.0, 0.9],
        intensity: 1.5,
    });
    let spot = scene.add_light(Light::Spot {
        color: [0.0, 1.0, 0.0],
        intensity: 0.75,
        range: None,
        inner_cone_angle: 0.25,
        outer_cone_angle: 0.5,
    });
    for (name, lid) in [("P", point), ("D", sun), ("S", spot)] {
        let mut node = Node::new().with_name(name);
        node.light = Some(lid);
        let nid = scene.add_node(node);
        scene.roots.push(nid);
    }

    let scene2 = decode(&encode_binary(&scene));
    assert_eq!(scene2.lights.len(), 3, "all three lights survive");
    let by_name = |n: &str| -> &Light {
        let node = scene2
            .nodes
            .iter()
            .find(|x| x.name.as_deref() == Some(n))
            .expect("node");
        &scene2.lights[node.light.expect("light bound").0 as usize]
    };
    match by_name("P") {
        Light::Point {
            color,
            intensity,
            range,
        } => {
            assert!((color[1] - 0.5).abs() < 1e-6);
            assert!((intensity - 2.0).abs() < 1e-5);
            assert!((range.expect("range survives") - 12.5).abs() < 1e-5);
        }
        other => panic!("expected Point, got {other:?}"),
    }
    match by_name("D") {
        Light::Directional { color, intensity } => {
            assert!((color[2] - 0.9).abs() < 1e-6);
            assert!((intensity - 1.5).abs() < 1e-5);
        }
        other => panic!("expected Directional, got {other:?}"),
    }
    match by_name("S") {
        Light::Spot {
            intensity,
            inner_cone_angle,
            outer_cone_angle,
            ..
        } => {
            assert!((intensity - 0.75).abs() < 1e-5);
            assert!((inner_cone_angle - 0.25).abs() < 1e-5);
            assert!((outer_cone_angle - 0.5).abs() < 1e-5);
        }
        other => panic!("expected Spot, got {other:?}"),
    }
}

/// Perspective + orthographic cameras round-trip through the
/// `NodeAttribute : "Camera"` P-record set (projection type /
/// FieldOfViewY / near-far planes / aspect pair / OrthoZoom).
#[test]
fn cameras_survive_round_trip() {
    use oxideav_mesh3d::Camera;

    let mut scene = Scene3D::new();
    let persp = scene.add_camera(Camera::Perspective {
        aspect_ratio: Some(16.0 / 9.0),
        yfov: 1.0,
        znear: 0.5,
        zfar: Some(500.0),
    });
    let ortho = scene.add_camera(Camera::Orthographic {
        xmag: 4.0,
        ymag: 2.0,
        znear: 0.1,
        zfar: 100.0,
    });
    for (name, cid) in [("Persp", persp), ("Ortho", ortho)] {
        let mut node = Node::new().with_name(name);
        node.camera = Some(cid);
        let nid = scene.add_node(node);
        scene.roots.push(nid);
    }

    let scene2 = decode(&encode_binary(&scene));
    assert_eq!(scene2.cameras.len(), 2, "both cameras survive");
    let by_name = |n: &str| -> &Camera {
        let node = scene2
            .nodes
            .iter()
            .find(|x| x.name.as_deref() == Some(n))
            .expect("node");
        &scene2.cameras[node.camera.expect("camera bound").0 as usize]
    };
    match by_name("Persp") {
        Camera::Perspective {
            aspect_ratio,
            yfov,
            znear,
            zfar,
        } => {
            assert!((yfov - 1.0).abs() < 1e-5, "yfov {yfov}");
            assert!((znear - 0.5).abs() < 1e-6);
            assert!((zfar.expect("far plane") - 500.0).abs() < 1e-3);
            assert!(
                (aspect_ratio.expect("aspect") - 16.0 / 9.0).abs() < 1e-5,
                "aspect {aspect_ratio:?}"
            );
        }
        other => panic!("expected Perspective, got {other:?}"),
    }
    match by_name("Ortho") {
        Camera::Orthographic {
            xmag,
            ymag,
            znear,
            zfar,
        } => {
            assert!((xmag - 4.0).abs() < 1e-5, "xmag {xmag}");
            assert!((ymag - 2.0).abs() < 1e-5, "ymag {ymag}");
            assert!((znear - 0.1).abs() < 1e-6);
            assert!((zfar - 100.0).abs() < 1e-4);
        }
        other => panic!("expected Orthographic, got {other:?}"),
    }
}

// ---------------------------------------------------------------------
// Round 384 — encoder Takes + FBXHeaderExtension metadata emission.
// ---------------------------------------------------------------------

/// Authoring metadata (creator / creation time / document MetaData /
/// application provenance) and the Takes catalogue survive a
/// decode → encode → decode cycle via the round-tripped extras.
#[test]
fn header_metadata_and_takes_survive_round_trip() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    for (k, v) in [
        ("fbx:creator", serde_json::json!("OxideAV test writer")),
        ("fbx:header_version", serde_json::json!(1003)),
        (
            "fbx:creation_time",
            serde_json::json!("2019-01-07T16:17:31.730"),
        ),
        ("fbx:meta_title", serde_json::json!("A quad")),
        ("fbx:meta_author", serde_json::json!("A. Author")),
        ("fbx:application_name", serde_json::json!("Maya")),
        ("fbx:application_vendor", serde_json::json!("Autodesk")),
        ("fbx:application_version", serde_json::json!("201800")),
        ("fbx:document_url", serde_json::json!("/tmp/a.fbx")),
        ("fbx:current_take", serde_json::json!("Take 001")),
        (
            "fbx:takes",
            serde_json::json!([{
                "name": "Take 001",
                "file_name": "Take_001.tak",
                "local_time": [1924423250i64, 230930790000i64],
                "reference_time": [1924423250i64, 230930790000i64],
            }]),
        ),
    ] {
        scene.extras.insert(k.to_string(), v);
    }

    let scene2 = decode(&encode_binary(&scene));
    let s = |k: &str| {
        scene2
            .extras
            .get(k)
            .and_then(|v| v.as_str())
            .map(str::to_owned)
    };
    assert_eq!(s("fbx:creator").as_deref(), Some("OxideAV test writer"));
    assert_eq!(
        scene2
            .extras
            .get("fbx:header_version")
            .and_then(|v| v.as_i64()),
        Some(1003)
    );
    assert_eq!(
        s("fbx:creation_time").as_deref(),
        Some("2019-01-07T16:17:31.730")
    );
    assert_eq!(s("fbx:meta_title").as_deref(), Some("A quad"));
    assert_eq!(s("fbx:meta_author").as_deref(), Some("A. Author"));
    assert_eq!(s("fbx:application_name").as_deref(), Some("Maya"));
    assert_eq!(s("fbx:application_vendor").as_deref(), Some("Autodesk"));
    assert_eq!(s("fbx:application_version").as_deref(), Some("201800"));
    assert_eq!(s("fbx:document_url").as_deref(), Some("/tmp/a.fbx"));
    assert_eq!(s("fbx:current_take").as_deref(), Some("Take 001"));
    let takes = scene2
        .extras
        .get("fbx:takes")
        .and_then(|v| v.as_array())
        .expect("takes survive");
    assert_eq!(takes.len(), 1);
    assert_eq!(
        takes[0].get("name").and_then(|v| v.as_str()),
        Some("Take 001")
    );
    assert_eq!(
        takes[0].get("file_name").and_then(|v| v.as_str()),
        Some("Take_001.tak")
    );
    assert_eq!(
        takes[0].get("local_time"),
        Some(&serde_json::json!([1924423250i64, 230930790000i64]))
    );
}

/// The same metadata survives through the ASCII writer form too (one
/// walker covers both front-ends).
#[test]
fn header_metadata_survives_ascii_round_trip() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);
    scene.extras.insert(
        "fbx:creator".to_string(),
        serde_json::json!("ASCII writer test"),
    );
    scene
        .extras
        .insert("fbx:current_take".to_string(), serde_json::json!("T1"));

    let bytes = FbxEncoder::new()
        .form(FbxOutputForm::Ascii)
        .encode(&scene)
        .expect("ascii encode");
    let scene2 = decode(&bytes);
    assert_eq!(
        scene2.extras.get("fbx:creator").and_then(|v| v.as_str()),
        Some("ASCII writer test")
    );
    assert_eq!(
        scene2
            .extras
            .get("fbx:current_take")
            .and_then(|v| v.as_str()),
        Some("T1")
    );
}

// ---------------------------------------------------------------------
// Round 384 — encoder GlobalSettings parity + NodeAttribute kind
// markers.
// ---------------------------------------------------------------------

/// The full decode-side GlobalSettings recognised-name set survives a
/// decode → encode → decode cycle: time-mode enums, i64-exact KTime
/// spans, doubles, DefaultCamera, AmbientColor, and a non-canonical
/// UnitScaleFactor (2.54 — neither cm nor m) round-trip via extras.
#[test]
fn global_settings_parity_survives_round_trip() {
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(quad_with_normals_and_uvs("M"));
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    for (k, v) in [
        ("fbx:up_axis", serde_json::json!(1)),
        ("fbx:up_axis_sign", serde_json::json!(1)),
        ("fbx:original_up_axis", serde_json::json!(2)),
        ("fbx:original_up_axis_sign", serde_json::json!(-1)),
        ("fbx:time_mode", serde_json::json!(6)),
        ("fbx:time_protocol", serde_json::json!(2)),
        ("fbx:snap_on_frame_mode", serde_json::json!(0)),
        ("fbx:current_time_marker", serde_json::json!(-1)),
        ("fbx:time_span_start", serde_json::json!(1924423250i64)),
        ("fbx:time_span_stop", serde_json::json!(230930790000i64)),
        ("fbx:original_unit_scale_factor", serde_json::json!(2.54)),
        ("fbx:custom_frame_rate", serde_json::json!(-1.0)),
        (
            "fbx:default_camera",
            serde_json::json!("Producer Perspective"),
        ),
        ("fbx:ambient_color", serde_json::json!([0.1, 0.2, 0.3])),
        ("fbx:unit_scale_factor", serde_json::json!(2.54)),
    ] {
        scene.extras.insert(k.to_string(), v);
    }

    let scene2 = decode(&encode_binary(&scene));
    let gi = |k: &str| scene2.extras.get(k).and_then(|v| v.as_i64());
    assert_eq!(gi("fbx:up_axis"), Some(1));
    assert_eq!(gi("fbx:original_up_axis"), Some(2));
    assert_eq!(gi("fbx:original_up_axis_sign"), Some(-1));
    assert_eq!(gi("fbx:time_mode"), Some(6));
    assert_eq!(gi("fbx:time_protocol"), Some(2));
    assert_eq!(gi("fbx:snap_on_frame_mode"), Some(0));
    assert_eq!(gi("fbx:current_time_marker"), Some(-1));
    assert_eq!(gi("fbx:time_span_start"), Some(1924423250));
    assert_eq!(gi("fbx:time_span_stop"), Some(230930790000));
    let gf = |k: &str| scene2.extras.get(k).and_then(|v| v.as_f64());
    assert_eq!(gf("fbx:original_unit_scale_factor"), Some(2.54));
    assert_eq!(gf("fbx:custom_frame_rate"), Some(-1.0));
    // The non-canonical factor survives verbatim (the decode side
    // leaves scene.unit at default and stashes the raw value).
    assert_eq!(gf("fbx:unit_scale_factor"), Some(2.54));
    assert_eq!(
        scene2
            .extras
            .get("fbx:default_camera")
            .and_then(|v| v.as_str()),
        Some("Producer Perspective")
    );
    assert_eq!(
        scene2.extras.get("fbx:ambient_color"),
        Some(&serde_json::json!([0.1, 0.2, 0.3]))
    );
}

/// A bone (LimbNode) / locator (Null) kind marker on a node survives
/// re-encode via the NodeAttribute element.
#[test]
fn node_attribute_kind_markers_survive_round_trip() {
    let mut scene = Scene3D::new();
    let mut bone = Node::new().with_name("Bone");
    bone.extras.insert(
        "fbx:node_attribute_kind".to_string(),
        serde_json::json!("LimbNode"),
    );
    let mut locator = Node::new().with_name("Loc");
    locator.extras.insert(
        "fbx:node_attribute_kind".to_string(),
        serde_json::json!("Null"),
    );
    let b = scene.add_node(bone);
    let l = scene.add_node(locator);
    scene.roots.push(b);
    scene.roots.push(l);

    let scene2 = decode(&encode_binary(&scene));
    let kind = |n: &str| {
        scene2
            .nodes
            .iter()
            .find(|x| x.name.as_deref() == Some(n))
            .and_then(|x| x.extras.get("fbx:node_attribute_kind"))
            .and_then(|v| v.as_str())
            .map(str::to_owned)
    };
    assert_eq!(kind("Bone").as_deref(), Some("LimbNode"));
    assert_eq!(kind("Loc").as_deref(), Some("Null"));
}

/// Explicitly-authored binormals (extras-borne, xyz + w sign) and an
/// additional normal layer survive re-encode as
/// `LayerElementBinormal` / second `LayerElementNormal` records.
#[test]
fn binormals_and_extra_normal_layer_survive_round_trip() {
    let mut mesh = quad_with_normals_and_uvs("ExtraLayers");
    {
        let prim = &mut mesh.primitives[0];
        // One binormal layer: 6 corners × [x, y, z, w].
        let mut flat = Vec::new();
        for c in 0..6 {
            flat.extend([0.0, 1.0, 0.0, if c % 2 == 0 { 1.0 } else { -1.0 }]);
        }
        prim.extras
            .insert("fbx:binormals".to_string(), serde_json::json!([flat]));
        prim.extras.insert(
            "fbx:binormals_mapping".to_string(),
            serde_json::json!(["ByPolygonVertex"]),
        );
        // One extra normal layer (all +X), TypedIndex 1.
        let extra: Vec<f64> = (0..6).flat_map(|_| [1.0, 0.0, 0.0]).collect();
        prim.extras
            .insert("fbx:extra_normals".to_string(), serde_json::json!([extra]));
        prim.extras.insert(
            "fbx:extra_normals_typed_index".to_string(),
            serde_json::json!([1]),
        );
    }

    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let nid = scene.add_node(Node::new().with_mesh(mid));
    scene.roots.push(nid);

    let scene2 = decode(&encode_binary(&scene));
    let prim = &scene2.meshes[0].primitives[0];

    // Binormal layer rebuilt.
    let bn = prim
        .extras
        .get("fbx:binormals")
        .and_then(|v| v.as_array())
        .expect("binormals survive");
    assert_eq!(bn.len(), 1, "one binormal layer");
    let flat = bn[0].as_array().expect("flat buffer");
    assert_eq!(flat.len(), 24, "6 corners x 4 components");
    assert_eq!(flat[1].as_f64(), Some(1.0), "y component");
    assert_eq!(flat[3].as_f64(), Some(1.0), "corner 0 w sign");
    assert_eq!(flat[7].as_f64(), Some(-1.0), "corner 1 w sign");

    // Extra normal layer rebuilt (canonical slot untouched).
    let n0 = prim.normals.as_ref().expect("canonical normals");
    assert!((n0[0][2] - 1.0).abs() < 1e-6, "canonical layer is +Z");
    let extra = prim
        .extras
        .get("fbx:extra_normals")
        .and_then(|v| v.as_array())
        .expect("extra normal layer survives");
    assert_eq!(extra.len(), 1);
    let eflat = extra[0].as_array().expect("flat buffer");
    assert_eq!(eflat.len(), 18, "6 corners x 3 components");
    assert_eq!(eflat[0].as_f64(), Some(1.0), "extra layer is +X");
    assert_eq!(
        prim.extras.get("fbx:extra_normals_typed_index"),
        Some(&serde_json::json!([1]))
    );
}
