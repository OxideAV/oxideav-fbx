//! Whole-file tests over the staged `box-binary-v7400.fbx` binary
//! fixture — the primary sample `docs/3d/fbx/fbx-binary-properties70.md`
//! was observer-derived from (SHA-256
//! `ad2d79fe89d4d646bc7930dc952eb28e69976a321b387bf7851ecd3f37e667f8`,
//! 17200 bytes; provenance in `tests/fixtures/README.md`).
//!
//! The headline closure is **byte-faithful re-encoding**: `parse` +
//! `parse_footer` capture everything the file contains (record tree +
//! per-file footer id), and `write_document_with_options` with the
//! captured id reproduces the input byte-for-byte — header, every
//! Node Record, the `References` empty-body form, the top-level NULL
//! record, the footer alignment padding, the version echo, and the
//! trailer signature.

use oxideav_fbx::{
    parse_footer, write_document_with_options, FbxDecoder, WriterOptions, FOOTER_TRAILER,
};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};

const FIXTURE: &[u8] = include_bytes!("fixtures/box-binary-v7400.fbx");

#[test]
fn fixture_parses_with_documented_topline_facts() {
    // The doc's §1 facts: version 7400, first record
    // `FBXHeaderExtension`, 11 top-level records ending at `Takes`.
    let doc = oxideav_fbx::binary::parse(FIXTURE).expect("fixture parses");
    assert_eq!(doc.version, 7400);
    let names: Vec<&str> = doc.root.children.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "FBXHeaderExtension",
            "FileId",
            "CreationTime",
            "Creator",
            "GlobalSettings",
            "Documents",
            "References",
            "Definitions",
            "Objects",
            "Connections",
            "Takes",
        ]
    );
    // §2 worked example: FBXHeaderExtension's first child leaf is
    // `FBXHeaderVersion` = 1003.
    let header = &doc.root.children[0];
    let hv = header
        .child("FBXHeaderVersion")
        .expect("FBXHeaderVersion leaf");
    assert_eq!(hv.properties[0].as_i64(), Some(1003));
    // The `References` record is the fixture's only property-less
    // child-less node (the empty-body canon witness).
    let refs = doc.root.child("References").expect("References");
    assert!(refs.properties.is_empty() && refs.children.is_empty());
}

#[test]
fn fixture_footer_decodes_to_the_observed_bytes() {
    let footer = parse_footer(FIXTURE).expect("footer decodes");
    // The 16 observed id bytes at offset 17037 of the staged sample.
    assert_eq!(
        footer.id,
        [
            0xfa, 0xbc, 0xaf, 0x0f, 0xd2, 0xc0, 0xd8, 0x63, 0xb2, 0x78, 0xf4, 0x89, 0x14, 0xf3,
            0x26, 0x75,
        ]
    );
    assert_eq!(footer.id_hex(), "fabcaf0fd2c0d863b278f48914f32675");
    // The file's final 16 bytes are the constant trailer signature.
    assert_eq!(&FIXTURE[FIXTURE.len() - 16..], FOOTER_TRAILER);
}

#[test]
fn fixture_reencodes_byte_for_byte() {
    let doc = oxideav_fbx::binary::parse(FIXTURE).expect("fixture parses");
    let footer = parse_footer(FIXTURE).expect("footer decodes");
    let opts = WriterOptions::default().footer_id(footer.id);
    let bytes = write_document_with_options(&doc, &opts).expect("re-encode");
    assert_eq!(bytes.len(), FIXTURE.len(), "re-encoded length matches");
    // Locate the first divergence (if any) for a useful failure
    // message rather than a 17200-byte assert_eq dump.
    if let Some(pos) = bytes.iter().zip(FIXTURE.iter()).position(|(a, b)| a != b) {
        panic!(
            "re-encode diverges at offset {pos}: got 0x{:02x}, fixture has 0x{:02x}",
            bytes[pos], FIXTURE[pos]
        );
    }
}

#[test]
fn fixture_decodes_to_a_single_cube_scene() {
    // Black-box sanity through the full decoder: the box sample is
    // one cube Geometry bound to one Model with one Material (per the
    // doc's §5/§7 object + connection listings).
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("scene decodes");
    assert_eq!(scene.meshes.len(), 1);
    let prim = &scene.meshes[0].primitives[0];
    // 12 triangles = 36 corners from the cube's 6 quads (§3b:
    // `PolygonVertexIndex` count 36 after fan-triangulation of the
    // doc's observed 72-double Vertices / 36-int index arrays).
    assert_eq!(prim.positions.len(), 36);
    assert_eq!(scene.materials.len(), 1);
    // The footer id is surfaced for encoder-side round-tripping.
    assert_eq!(
        scene.extras.get("fbx:footer_id").and_then(|v| v.as_str()),
        Some("fabcaf0fd2c0d863b278f48914f32675")
    );
}

#[test]
fn top_level_provenance_records_survive_the_scene_round_trip() {
    // The v7400 fixture's top-level siblings of FBXHeaderExtension:
    // FileId (16-byte R blob), CreationTime, Creator. Surfaced on
    // Scene3D extras and re-emitted by the encoder in fixture order.
    let mut dec = FbxDecoder::new();
    let scene = dec.decode(FIXTURE).expect("scene decodes");
    let s = |k: &str| {
        scene
            .extras
            .get(k)
            .and_then(|v| v.as_str())
            .map(str::to_owned)
    };
    assert_eq!(
        s("fbx:file_id").as_deref(),
        Some("2cb52ce5b829c3c9b3ccb724a124f3fd")
    );
    assert_eq!(
        s("fbx:file_creation_time").as_deref(),
        Some("2017-09-25 13:03:42:782")
    );
    assert_eq!(
        s("fbx:file_creator").as_deref(),
        Some("FBX SDK/FBX Plugins version 2017.1 build=20161007")
    );

    // Scene3D round trip: encode → decode preserves all three, and
    // the emitted document carries the records in the observed
    // top-level order.
    let bytes = oxideav_fbx::FbxEncoder::new()
        .encode(&scene)
        .expect("re-encode");
    let doc = oxideav_fbx::binary::parse(&bytes).expect("parses");
    let names: Vec<&str> = doc.root.children.iter().map(|c| c.name.as_str()).collect();
    let pos = |n: &str| names.iter().position(|&x| x == n).unwrap_or(usize::MAX);
    assert!(pos("FBXHeaderExtension") < pos("FileId"));
    assert!(pos("FileId") < pos("CreationTime"));
    assert!(pos("CreationTime") < pos("Creator"));
    assert!(pos("Creator") < pos("GlobalSettings"));

    let scene2 = FbxDecoder::new().decode(&bytes).expect("re-decode");
    for k in ["fbx:file_id", "fbx:file_creation_time", "fbx:file_creator"] {
        assert_eq!(
            scene2.extras.get(k).and_then(|v| v.as_str()),
            scene.extras.get(k).and_then(|v| v.as_str()),
            "{k} survives"
        );
    }
}
