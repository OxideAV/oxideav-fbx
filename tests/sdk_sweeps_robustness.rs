//! Deterministic mutation sweep over the 2026-11 sweep fixtures: the
//! typed readers added for them (key attributes and the curve
//! evaluator, NURBS records and tessellators, trim regions, in-between
//! stations, texture records) are total — a damaged file decodes to
//! `Ok` or `Err`, never a panic — and whatever decodes also encodes.
//!
//! Binary files get random byte overwrites (most land in array
//! payloads, the bulk of these files); ASCII files get digit-for-digit
//! and digit-for-sign substitutions, which keep the grammar valid and
//! so reach the typed layers with hostile numbers (counts, orders,
//! run lengths, knot values, indices).

mod common;

use std::panic::catch_unwind;

use common::{SWEEP_FILES, read};
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm, SceneDecodeOptions};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};

/// xorshift64* — a small deterministic generator.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }
}

fn exercise(bytes: Vec<u8>, what: String) {
    let label = what.clone();
    let outcome = catch_unwind(move || {
        // The default decode, and one sampling NURBS by tolerance.
        for options in [
            SceneDecodeOptions::new(),
            SceneDecodeOptions::new()
                .with_nurbs_tolerance(0.5)
                .with_curve_tolerance(0.05),
        ] {
            if let Ok(scene) = FbxDecoder::new().with_scene_options(options).decode(&bytes) {
                for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
                    let _ = FbxEncoder::new().form(form).encode(&scene);
                }
            }
        }
    });
    assert!(outcome.is_ok(), "panic on {label}");
}

#[test]
fn mutated_sweep_files_never_panic() {
    const ROUNDS: usize = 16;
    for (index, (dir, name)) in SWEEP_FILES.iter().enumerate() {
        let original = read(dir, name);
        let ascii = oxideav_fbx::is_ascii_fbx(&original);
        // Positions worth hitting in an ASCII file: digits past the
        // header sections (object bodies and connections).
        let digits: Vec<usize> = if ascii {
            let from = original
                .windows(8)
                .position(|w| w == b"Objects:")
                .unwrap_or(0);
            (from..original.len())
                .filter(|i| original[*i].is_ascii_digit())
                .collect()
        } else {
            Vec::new()
        };
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ ((index as u64 + 1) << 17));
        for round in 0..ROUNDS {
            let mut bytes = original.clone();
            let edits = 1 + rng.below(4);
            for _ in 0..edits {
                if ascii && !digits.is_empty() {
                    let at = digits[rng.below(digits.len())];
                    bytes[at] = match rng.below(12) {
                        10 => b'-',
                        11 => b'9',
                        d => b'0' + d as u8,
                    };
                } else {
                    // Past the 27-byte header, so the magic survives.
                    let at = 27 + rng.below(bytes.len() - 27);
                    bytes[at] = rng.next() as u8;
                }
            }
            exercise(bytes, format!("{name} round {round}"));
        }
    }
}

/// Hostile values aimed at the typed layers directly, as ASCII edits
/// of the staged files: each replaces one authored number.
#[test]
fn targeted_hostile_values_never_panic() {
    let edit = |dir: &str, name: &str, from: &str, to: &str| {
        let text = String::from_utf8(read(dir, name)).expect("utf-8");
        assert!(text.contains(from), "{name}: `{from}` not found");
        exercise(
            text.replacen(from, to, 1).into_bytes(),
            format!("{name}: {from} -> {to}"),
        );
    };
    let key = "keyattr-sweep-ascii-v7500.fbx";
    let kdir = "fbxsdk-keyattr-sweep";
    // Run lengths: zero, negative, enormous; a missing array.
    edit(kdir, key, "a: 1,1,1,1\n", "a: 0,0,0,0\n");
    edit(kdir, key, "a: 1,1,1,1\n", "a: -5,1,1,1\n");
    edit(kdir, key, "a: 1,1,1,1\n", "a: 2147483647,2147483647,1,1\n");
    edit(kdir, key, "KeyAttrRefCount: *4", "KeyAttrRefCountX: *4");
    // Coincident and decreasing key times; flag words with every bit.
    edit(
        kdir,
        key,
        "a: 0,46186158000,92372316000,138558474000",
        "a: 0,0,0,0",
    );
    edit(
        kdir,
        key,
        "a: 0,46186158000,92372316000,138558474000",
        "a: 138558474000,92372316000,46186158000,0",
    );
    edit(kdir, key, "a: 1032,1032,1032,1032", "a: -1,-1,-1,-1");
    // Weighted flags over a zero weight word, and NaN slope words.
    edit(
        kdir,
        key,
        "a: 50332680,50332680,50332680,50332680",
        "a: 50332680,50332680,50332680,50332680\n\t\t} \n\t\tJunk: *1 {\n\t\t\ta: 0",
    );
    edit(
        kdir,
        key,
        "a: 1069547520,-1071644672,218434821,0,",
        "a: 2143289344,-4194304,0,0,",
    );

    let ndir = "fbxsdk-nurbs";
    let curves = "nurbs-curves-ascii-v7500.fbx";
    edit(ndir, curves, "Order: 4", "Order: 0");
    edit(ndir, curves, "Order: 4", "Order: 1");
    edit(ndir, curves, "Order: 4", "Order: 2000000000");
    edit(ndir, curves, "Form: \"Periodic\"", "Form: \"Sideways\"");
    edit(ndir, curves, "a: 1,10,100,1,", "a: 1,10,100,0,");
    edit(ndir, curves, "a: 1,10,100,1,", "a: 1,10,100,-1,");
    let surfaces = "nurbs-surfaces-ascii-v7500.fbx";
    edit(ndir, surfaces, "Dimensions: 5,4", "Dimensions: 0,0");
    edit(ndir, surfaces, "Dimensions: 5,4", "Dimensions: 4,5");
    edit(ndir, surfaces, "Dimensions: 5,4", "Dimensions: 65536,65536");
    edit(
        ndir,
        surfaces,
        "NurbsSurfaceOrder: 4,3",
        "NurbsSurfaceOrder: 1,1",
    );
    edit(ndir, surfaces, "Step: 3,5", "Step: 2000000000,2000000000");
    let line = "nurbs-line-ascii-v7500.fbx";
    edit(
        ndir,
        line,
        "a: 0,1,-3,4,3,-1",
        "a: 9999,-9999,-2147483648,2147483647,3,-1",
    );
    let trim = "nurbs-trim-ascii-v7500.fbx";
    edit(
        ndir,
        trim,
        "Type: \"Boundary\"",
        "Type: \"Boundary\"\n\t\tPoints: *0 {\n\t\t\ta: \n\t\t}",
    );

    let bdir = "fbxsdk-blendshape-inbetween";
    let blend = "blendshape-inbetween-ascii-v7500.fbx";
    edit(bdir, blend, "a: 100,30,60", "a: 100,100,100");
    edit(bdir, blend, "a: 100,30,60", "a: 0,0,0");
    edit(bdir, blend, "a: 100,30,60", "a: 1e308,-1e308,60");
    edit(bdir, blend, "a: 25,50,100", "a: 25");
}
