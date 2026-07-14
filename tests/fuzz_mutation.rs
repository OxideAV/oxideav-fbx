//! Deterministic bounded fuzz sweeps over both FBX front-ends.
//!
//! Not a libFuzzer harness — a fixed-seed xorshift PRNG drives byte
//! mutations, truncations, and splices over corpus files derived from
//! the staged `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx` fixture and
//! this crate's own encoder output, so every CI run replays the exact
//! same hostile-input population. The invariant under test is the
//! decoder's total-function contract: **every** byte string produces
//! `Ok` or `Err` — never a panic, never an abort (allocation bomb /
//! stack overflow).
//!
//! The round-413 hardening these sweeps lock in:
//! - truncated `Y` (i16) scalar → bounds-checked read (was a panic),
//! - hostile `NumProperties` → clamped preallocation (was multi-GiB),
//! - binary + ASCII nesting depth bombs → `MAX_NODE_DEPTH` (was a
//!   stack-overflow abort).
//!
//! Each iteration runs under `catch_unwind` so a failure reports the
//! seed / iteration that produced it (the sweep being deterministic,
//! the report is directly replayable).

use std::panic::catch_unwind;

use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{
    Material, Mesh, Mesh3DDecoder, Mesh3DEncoder, Node, Primitive, Scene3D, Topology,
};

const FIXTURE: &[u8] = include_bytes!("fixtures/cubes-ascii-v7500.fbx");

/// xorshift64* — tiny deterministic PRNG (public-domain construction),
/// good enough to spread mutations; no crypto claims.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

/// Decode must return without panicking; the result value is
/// irrelevant (mutated inputs are usually invalid — `Err` is fine).
fn assert_total(bytes: &[u8], what: &str) {
    let owned = bytes.to_vec();
    let outcome = catch_unwind(move || {
        let _ = FbxDecoder::new().decode(&owned);
    });
    assert!(outcome.is_ok(), "decoder panicked on {what}");
}

/// A small authored scene exercising the encoder's material / light /
/// hierarchy paths — its encoded bytes are the second corpus family.
fn synthetic_scene() -> Scene3D {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
    prim.uvs = vec![vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]];
    let mut mesh = Mesh::new(Some("Tri".to_string()));
    mesh.primitives.push(prim);

    let mut scene = Scene3D::new();
    let mat = scene.add_material(Material::new().with_base_color([0.2, 0.4, 0.8, 1.0]));
    scene.meshes.push(mesh);
    scene.meshes[0].primitives[0].material = Some(mat);
    let mid = oxideav_mesh3d::MeshId(0);
    let light = scene.add_light(oxideav_mesh3d::Light::Point {
        color: [1.0, 1.0, 1.0],
        intensity: 2.0,
        range: None,
    });
    let mut node = Node::new().with_name("TriNode").with_mesh(mid);
    node.light = Some(light);
    let nid = scene.add_node(node);
    scene.roots.push(nid);
    scene
}

/// The corpus families: the staged ASCII fixture, its binary
/// round-trip through our own writer, and a synthetic encode in both
/// forms.
fn corpora() -> Vec<(&'static str, Vec<u8>)> {
    let scene = FbxDecoder::new().decode(FIXTURE).expect("fixture decodes");
    let fixture_binary = FbxEncoder::new().encode(&scene).expect("binary re-encode");
    let synth = synthetic_scene();
    let synth_binary = FbxEncoder::new().encode(&synth).expect("synthetic binary");
    let synth_ascii = FbxEncoder::new()
        .form(FbxOutputForm::Ascii)
        .encode(&synth)
        .expect("synthetic ascii");
    vec![
        ("fixture-ascii", FIXTURE.to_vec()),
        ("fixture-binary", fixture_binary),
        ("synthetic-binary", synth_binary),
        ("synthetic-ascii", synth_ascii),
    ]
}

#[test]
fn prefix_truncation_sweep_never_panics() {
    // Every truncation point in small corpora; strided in the large
    // fixture so the sweep stays fast (a byte-level EOF bug does not
    // hide between strides — the interesting boundaries are record /
    // token edges, which the stride plus the ±1 offsets sample well).
    for (name, corpus) in corpora() {
        let stride = if corpus.len() > 8192 { 251 } else { 1 };
        let mut cut = 0usize;
        while cut <= corpus.len() {
            assert_total(&corpus[..cut], &format!("{name} truncated at {cut}"));
            for delta in [1usize, 2] {
                if cut >= delta {
                    let c = cut - delta;
                    assert_total(&corpus[..c], &format!("{name} truncated at {c}"));
                }
            }
            cut += stride;
        }
        assert_total(&corpus, &format!("{name} full"));
    }
}

#[test]
fn byte_mutation_sweep_never_panics() {
    // 1..=8 random byte overwrites per iteration, 400 iterations per
    // corpus, fixed seed per corpus name — fully replayable.
    for (ci, (name, corpus)) in corpora().into_iter().enumerate() {
        let mut rng = Rng::new(0x413_0000 + ci as u64);
        for iter in 0..400 {
            let mut mutated = corpus.clone();
            let n_mut = 1 + rng.below(8);
            for _ in 0..n_mut {
                let pos = rng.below(mutated.len());
                mutated[pos] = (rng.next() & 0xFF) as u8;
            }
            assert_total(&mutated, &format!("{name} mutation iter {iter}"));
        }
    }
}

#[test]
fn chunk_splice_sweep_never_panics() {
    // Structural mutations: duplicate, delete, or swap random chunks.
    // These reshuffle record boundaries wholesale, hunting for
    // offset-confusion bugs the single-byte sweep can't reach.
    for (ci, (name, corpus)) in corpora().into_iter().enumerate() {
        let mut rng = Rng::new(0x413_1000 + ci as u64);
        for iter in 0..150 {
            let mut mutated = corpus.clone();
            let a = rng.below(mutated.len());
            let len = 1 + rng.below(64.min(mutated.len() - a));
            match rng.below(3) {
                0 => {
                    // Duplicate chunk [a..a+len] at a random spot.
                    let chunk: Vec<u8> = mutated[a..a + len].to_vec();
                    let at = rng.below(mutated.len());
                    mutated.splice(at..at, chunk);
                }
                1 => {
                    // Delete the chunk.
                    mutated.drain(a..a + len);
                }
                _ => {
                    // Swap with another same-length chunk.
                    let b = rng.below(mutated.len().saturating_sub(len).max(1));
                    for i in 0..len {
                        mutated.swap(a + i, b + i);
                    }
                }
            }
            assert_total(&mutated, &format!("{name} splice iter {iter}"));
        }
    }
}

#[test]
fn random_tail_after_valid_magic_never_panics() {
    // Random garbage behind each front-end's valid signature, so the
    // sweep spends its budget past the sniffer instead of bouncing off
    // it. 300 iterations each, growing lengths.
    let mut rng = Rng::new(0x413_2000);
    let binary_header: &[u8] = &{
        let mut h = Vec::new();
        h.extend_from_slice(oxideav_fbx::FBX_MAGIC);
        h.extend_from_slice(&[0x1A, 0x00]);
        h.extend_from_slice(&7400u32.to_le_bytes());
        h
    };
    let ascii_banner: &[u8] = b"; FBX 7.5.0 project file\n";
    for header in [binary_header, ascii_banner] {
        for iter in 0..300 {
            let mut bytes = header.to_vec();
            let tail_len = rng.below(1 + iter * 8);
            for _ in 0..tail_len {
                bytes.push((rng.next() & 0xFF) as u8);
            }
            assert_total(&bytes, &format!("random tail iter {iter}"));
        }
    }
}

#[test]
fn structured_document_write_parse_closure_holds() {
    // Generative round-trip: random typed FbxDocument trees written by
    // our binary writer must re-parse to the identical tree. This is a
    // writer/reader closure check over shapes no fixture covers
    // (random names, every property variant, mixed nesting).
    use oxideav_fbx::{write_document, FbxDocument, FbxNode, FbxProperty};

    fn random_property(rng: &mut Rng) -> FbxProperty {
        match rng.below(10) {
            0 => FbxProperty::I16(rng.next() as i16),
            1 => FbxProperty::Bool(rng.next() & 1 == 1),
            2 => FbxProperty::I32(rng.next() as i32),
            3 => FbxProperty::F32(f32::from_bits((rng.next() as u32) & 0x7F7F_FFFF)),
            4 => FbxProperty::F64(f64::from_bits(rng.next() & 0x7FEF_FFFF_FFFF_FFFF)),
            5 => FbxProperty::I64(rng.next() as i64),
            6 => {
                let n = rng.below(16);
                FbxProperty::I32Array((0..n).map(|_| rng.next() as i32).collect())
            }
            7 => {
                let n = rng.below(16);
                FbxProperty::F64Array(
                    (0..n)
                        .map(|_| f64::from_bits(rng.next() & 0x7FEF_FFFF_FFFF_FFFF))
                        .collect(),
                )
            }
            8 => {
                let n = rng.below(24);
                FbxProperty::String((0..n).map(|_| b'a' + (rng.next() % 26) as u8).collect())
            }
            _ => {
                let n = rng.below(24);
                FbxProperty::Raw((0..n).map(|_| (rng.next() & 0xFF) as u8).collect())
            }
        }
    }

    fn random_node(rng: &mut Rng, depth: usize) -> FbxNode {
        let name_len = 1 + rng.below(8);
        let name: String = (0..name_len)
            .map(|_| (b'A' + (rng.next() % 26) as u8) as char)
            .collect();
        let properties = (0..rng.below(4)).map(|_| random_property(rng)).collect();
        let children = if depth < 4 {
            (0..rng.below(3))
                .map(|_| random_node(rng, depth + 1))
                .collect()
        } else {
            Vec::new()
        };
        FbxNode {
            name,
            properties,
            children,
        }
    }

    for (seed, version) in [(1u64, 7400u32), (2, 7500), (3, 7700), (4, 7400), (5, 7500)] {
        let mut rng = Rng::new(0x413_3000 + seed);
        for iter in 0..40 {
            let root = FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: (0..1 + rng.below(4))
                    .map(|_| random_node(&mut rng, 0))
                    .collect(),
            };
            let doc = FbxDocument {
                version,
                root: root.clone(),
            };
            let bytes = write_document(&doc).expect("random doc writes");
            let reparsed = oxideav_fbx::binary::parse(&bytes)
                .unwrap_or_else(|e| panic!("seed {seed} iter {iter} failed to re-parse: {e}"));
            assert_eq!(
                format!("{root:?}"),
                format!("{:?}", reparsed.root),
                "seed {seed} iter {iter}: write/parse closure broke"
            );
        }
    }
}
