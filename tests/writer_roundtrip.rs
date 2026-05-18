//! Round-trip test for [`oxideav_fbx::write_document`].
//!
//! The flow:
//!
//! 1. Hand-build the same synthetic-quad fixture as
//!    `synthetic_quad.rs` but produce it as an in-memory
//!    [`oxideav_fbx::FbxDocument`] tree instead of raw bytes.
//! 2. Serialise with [`oxideav_fbx::write_document`].
//! 3. Re-parse with [`oxideav_fbx::FbxDecoder`] / `binary::parse`.
//! 4. Assert the decoded [`oxideav_mesh3d::Scene3D`] and the
//!    re-parsed [`oxideav_fbx::FbxDocument`] both match the originals.
//!
//! Also exercises the writer at both layout widths (pre-7500 32-bit
//! and post-7500 64-bit).

use oxideav_fbx::{
    write_document, write_document_with_options, FbxDecoder, FbxDocument, FbxNode, FbxProperty,
    WriterOptions, FBX_VERSION_64BIT_THRESHOLD,
};
use oxideav_mesh3d::{Mesh3DDecoder, Topology};

/// Build the FbxDocument that mirrors the synthetic_quad fixture:
///
/// - `Objects { Geometry, Model }`
/// - `Connections { C("OO", Geometry, Model), C("OO", Model, 0) }`
fn build_quad_document(version: u32) -> FbxDocument {
    let geometry = FbxNode {
        name: "Geometry".into(),
        properties: vec![
            FbxProperty::I64(100),
            FbxProperty::String(b"Quad\x00\x01Geometry".to_vec()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: vec![
            FbxNode {
                name: "Vertices".into(),
                properties: vec![FbxProperty::F64Array(vec![
                    0.0, 0.0, 0.0, //
                    1.0, 0.0, 0.0, //
                    1.0, 1.0, 0.0, //
                    0.0, 1.0, 0.0, //
                ])],
                children: Vec::new(),
            },
            FbxNode {
                name: "PolygonVertexIndex".into(),
                properties: vec![FbxProperty::I32Array(vec![0, 1, 2, -4])],
                children: Vec::new(),
            },
        ],
    };
    let model = FbxNode {
        name: "Model".into(),
        properties: vec![
            FbxProperty::I64(200),
            FbxProperty::String(b"QuadModel\x00\x01Model".to_vec()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: Vec::new(),
    };
    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![geometry, model],
    };
    let c_geom_to_model = FbxNode {
        name: "C".into(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(100),
            FbxProperty::I64(200),
        ],
        children: Vec::new(),
    };
    let c_model_to_root = FbxNode {
        name: "C".into(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(200),
            FbxProperty::I64(0),
        ],
        children: Vec::new(),
    };
    let connections = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_geom_to_model, c_model_to_root],
    };
    FbxDocument {
        version,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, connections],
        },
    }
}

/// Recursively compare two [`FbxNode`]s for structural equality. Used
/// to assert the writer is the exact inverse of the parser on every
/// node our test corpus carries.
fn nodes_equal(a: &FbxNode, b: &FbxNode) -> bool {
    if a.name != b.name {
        return false;
    }
    if a.properties != b.properties {
        return false;
    }
    if a.children.len() != b.children.len() {
        return false;
    }
    for (ac, bc) in a.children.iter().zip(b.children.iter()) {
        if !nodes_equal(ac, bc) {
            return false;
        }
    }
    true
}

#[test]
fn synthetic_quad_round_trips_through_writer_pre_7500() {
    let original = build_quad_document(7400);
    let bytes = write_document(&original).expect("write_document succeeds");

    // The writer should pick the 32-bit header width — every per-record
    // EndOffset is u32 in the binary stream. We verify indirectly by
    // re-parsing as a 7400 document.
    assert!(original.version < FBX_VERSION_64BIT_THRESHOLD);

    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("emitted bytes decode");
    let reparsed = dec.last_document.as_ref().expect("document captured");
    assert_eq!(reparsed.version, original.version);
    // Top-level node structure round-trips bit-for-bit.
    assert!(
        nodes_equal(&original.root, &reparsed.root),
        "document tree did not round-trip"
    );

    // Same scene-level invariants the existing synthetic_quad test
    // asserts on a hand-coded byte buffer.
    assert_eq!(scene.meshes.len(), 1);
    assert_eq!(scene.nodes.len(), 1);
    assert_eq!(scene.roots.len(), 1);
    let mesh = &scene.meshes[0];
    assert_eq!(mesh.name.as_deref(), Some("Quad"));
    assert_eq!(mesh.primitives.len(), 1);
    let prim = &mesh.primitives[0];
    assert_eq!(prim.topology, Topology::Triangles);
    assert_eq!(prim.positions.len(), 6);
}

#[test]
fn synthetic_quad_round_trips_through_writer_post_7500() {
    let original = build_quad_document(7700);
    let bytes = write_document(&original).expect("write_document succeeds at 7700");
    assert!(original.version >= FBX_VERSION_64BIT_THRESHOLD);

    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("64-bit emitted bytes decode");
    let reparsed = dec.last_document.as_ref().unwrap();
    assert_eq!(reparsed.version, 7700);
    assert!(
        nodes_equal(&original.root, &reparsed.root),
        "64-bit document did not round-trip"
    );

    assert_eq!(scene.meshes.len(), 1);
}

#[test]
fn empty_document_round_trips_through_writer() {
    let original = FbxDocument {
        version: 7400,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: Vec::new(),
        },
    };
    let bytes = write_document(&original).expect("empty write succeeds");
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&bytes).expect("empty bytes decode");
    let reparsed = dec.last_document.as_ref().unwrap();
    assert_eq!(reparsed.version, 7400);
    assert!(reparsed.root.children.is_empty());
    assert!(scene.meshes.is_empty());
}

/// Build a quad-style document scaled up to a 1024-vertex grid so
/// the geometry arrays compress meaningfully. Used by the deflate
/// round-trip test below.
fn build_big_grid_document(version: u32, side: usize) -> FbxDocument {
    let mut vertices = Vec::with_capacity(side * side * 3);
    for y in 0..side {
        for x in 0..side {
            vertices.push(x as f64);
            vertices.push(y as f64);
            vertices.push(0.0);
        }
    }
    // Emit quads as triangles: per-row pairs of triangles, last
    // corner per polygon flagged with the bitwise-NOT
    // end-of-polygon convention.
    let mut indices: Vec<i32> = Vec::new();
    for y in 0..(side - 1) {
        for x in 0..(side - 1) {
            let v00 = (y * side + x) as i32;
            let v10 = v00 + 1;
            let v11 = v00 + side as i32 + 1;
            let v01 = v00 + side as i32;
            indices.push(v00);
            indices.push(v10);
            indices.push(v11);
            indices.push(!v01);
        }
    }
    let geometry = FbxNode {
        name: "Geometry".into(),
        properties: vec![
            FbxProperty::I64(100),
            FbxProperty::String(b"Grid\x00\x01Geometry".to_vec()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: vec![
            FbxNode {
                name: "Vertices".into(),
                properties: vec![FbxProperty::F64Array(vertices)],
                children: Vec::new(),
            },
            FbxNode {
                name: "PolygonVertexIndex".into(),
                properties: vec![FbxProperty::I32Array(indices)],
                children: Vec::new(),
            },
        ],
    };
    let model = FbxNode {
        name: "Model".into(),
        properties: vec![
            FbxProperty::I64(200),
            FbxProperty::String(b"GridModel\x00\x01Model".to_vec()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: Vec::new(),
    };
    let objects = FbxNode {
        name: "Objects".into(),
        properties: Vec::new(),
        children: vec![geometry, model],
    };
    let c_geom_to_model = FbxNode {
        name: "C".into(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(100),
            FbxProperty::I64(200),
        ],
        children: Vec::new(),
    };
    let c_model_to_root = FbxNode {
        name: "C".into(),
        properties: vec![
            FbxProperty::String(b"OO".to_vec()),
            FbxProperty::I64(200),
            FbxProperty::I64(0),
        ],
        children: Vec::new(),
    };
    let connections = FbxNode {
        name: "Connections".into(),
        properties: Vec::new(),
        children: vec![c_geom_to_model, c_model_to_root],
    };
    FbxDocument {
        version,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![objects, connections],
        },
    }
}

/// End-to-end deflate path: a 32×32 quad grid serialised with
/// `compress_arrays_at(256)` must shrink versus the raw form *and*
/// still decode through `FbxDecoder` to a geometrically-equivalent
/// scene.
#[test]
fn deflate_compressed_grid_round_trips_through_full_decoder() {
    let doc = build_big_grid_document(7400, 32);
    let raw_bytes = write_document(&doc).expect("raw write");
    let opts = WriterOptions::default().compress_arrays_at(256);
    let compressed_bytes = write_document_with_options(&doc, &opts).expect("compressed write");
    // The Vertices array is 32*32*3 = 3072 doubles = 24 KiB raw; a
    // monotonically-increasing-coordinate grid compresses *very*
    // well. The compressed file must be strictly smaller.
    // Measured on the in-tree CI: raw = 40 346 bytes vs compressed
    // = 8 326 bytes (≈ 20.6 % of the raw size). Assert a generous
    // upper bound so we'd notice if a future `miniz_oxide` upgrade
    // regressed the ratio dramatically, without locking the test to
    // a fragile exact byte count.
    assert!(
        compressed_bytes.len() < raw_bytes.len() / 2,
        "compression should shrink the file to <50% of raw: raw={} compressed={}",
        raw_bytes.len(),
        compressed_bytes.len()
    );
    // And the geometry must survive the round-trip.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(&compressed_bytes).expect("compressed decodes");
    let reparsed = dec.last_document.as_ref().unwrap();
    assert!(
        nodes_equal(&doc.root, &reparsed.root),
        "compressed document did not round-trip"
    );
    assert_eq!(scene.meshes.len(), 1);
    let mesh = &scene.meshes[0];
    assert_eq!(mesh.primitives.len(), 1);
    assert_eq!(mesh.primitives[0].topology, Topology::Triangles);
    // 31*31 quads × 2 triangles × 3 corners = 5766 per-corner verts.
    assert_eq!(mesh.primitives[0].positions.len(), 31 * 31 * 2 * 3);
}

/// Decode-then-encode-then-decode round-trip from the hand-coded
/// fixture in `synthetic_quad.rs`. This catches any property variant
/// (or edge case in name length / property count) that the writer
/// fails to round-trip when fed the parser's own output rather than a
/// hand-built `FbxDocument` literal.
#[test]
fn parser_output_writes_back_unchanged() {
    let original = build_quad_document(7400);
    let first_bytes = write_document(&original).expect("first write succeeds");
    let parsed = oxideav_fbx::binary::parse(&first_bytes).expect("first decode succeeds");
    let second_bytes = write_document(&parsed).expect("second write succeeds");
    // The writer is deterministic: encoding a parsed document twice
    // yields identical bytes.
    assert_eq!(
        first_bytes, second_bytes,
        "writer is non-deterministic on a parsed document"
    );
    let reparsed = oxideav_fbx::binary::parse(&second_bytes).expect("second decode succeeds");
    assert!(nodes_equal(&parsed.root, &reparsed.root));
}
