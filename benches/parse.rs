//! Parse-time and peak-memory measurement for the container readers.
//!
//! ```text
//! cargo bench --bench parse
//! ```
//!
//! Inputs: the largest staged binary and ASCII files (the 2026-11
//! key-attribute sweep, vendored under `tests/fixtures/sweeps-2026-11/`)
//! and one synthetic document per form — a single `Geometry` with
//! `STRESS_POINTS` control points, written by this crate's own writers
//! — so the array paths dominate. For each input the bench prints the
//! median wall time of `ROUNDS` parses and the peak heap growth of one
//! parse (bytes live above the level before the call, measured by a
//! counting allocator).

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};

struct Counting;

static LIVE: AtomicUsize = AtomicUsize::new(0);
static PEAK: AtomicUsize = AtomicUsize::new(0);

// SAFETY: defers to the system allocator; only adds byte accounting.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            let live = LIVE.fetch_add(layout.size(), Ordering::Relaxed) + layout.size();
            PEAK.fetch_max(live, Ordering::Relaxed);
        }
        p
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size(), Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let p = unsafe { System.realloc(ptr, layout, new_size) };
        if !p.is_null() {
            if new_size >= layout.size() {
                let grow = new_size - layout.size();
                let live = LIVE.fetch_add(grow, Ordering::Relaxed) + grow;
                PEAK.fetch_max(live, Ordering::Relaxed);
            } else {
                LIVE.fetch_sub(layout.size() - new_size, Ordering::Relaxed);
            }
        }
        p
    }
}

#[global_allocator]
static ALLOC: Counting = Counting;

const ROUNDS: usize = 41;
const STRESS_POINTS: usize = 200_000;

fn parse(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

fn measure(label: &str, bytes: &[u8]) {
    // Peak heap growth of one parse.
    let before = LIVE.load(Ordering::Relaxed);
    PEAK.store(before, Ordering::Relaxed);
    let doc = parse(bytes);
    let peak = PEAK.load(Ordering::Relaxed) - before;
    let retained = LIVE.load(Ordering::Relaxed) - before;
    drop(doc);
    // Median wall time.
    let mut times: Vec<f64> = (0..ROUNDS)
        .map(|_| {
            let t = Instant::now();
            let doc = parse(bytes);
            let dt = t.elapsed().as_secs_f64();
            std::hint::black_box(&doc);
            dt
        })
        .collect();
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = times[ROUNDS / 2];
    println!(
        "{label:<34} {:>9} B  median {:>9.1} us  {:>7.1} MB/s  peak {:>10} B  retained {:>10} B",
        bytes.len(),
        median * 1.0e6,
        bytes.len() as f64 / median / 1.0e6,
        peak,
        retained,
    );
}

fn leaf(name: &str, p: FbxProperty) -> FbxNode {
    FbxNode {
        name: name.to_string(),
        properties: vec![p],
        children: Vec::new(),
    }
}

/// One `Geometry` with `STRESS_POINTS` control points in triangles.
fn stress_document() -> FbxDocument {
    let n = STRESS_POINTS;
    let vertices: Vec<f64> = (0..n * 3).map(|i| (i as f64) * 0.125 - 1000.5).collect();
    let indices: Vec<i32> = (0..n as i32)
        .map(|i| if i % 3 == 2 { !i } else { i })
        .collect();
    let normals: Vec<f64> = (0..n * 3).map(|i| ((i % 7) as f64) / 7.0).collect();
    let geometry = FbxNode {
        name: "Geometry".to_string(),
        properties: vec![
            FbxProperty::I64(1),
            FbxProperty::String(b"Stress\x00\x01Geometry".to_vec()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: vec![
            leaf("Vertices", FbxProperty::F64Array(vertices)),
            leaf("PolygonVertexIndex", FbxProperty::I32Array(indices)),
            leaf("Normals", FbxProperty::F64Array(normals)),
        ],
    };
    let header = FbxNode {
        name: "FBXHeaderExtension".to_string(),
        properties: Vec::new(),
        children: vec![leaf("FBXVersion", FbxProperty::I32(7400))],
    };
    let objects = FbxNode {
        name: "Objects".to_string(),
        properties: Vec::new(),
        children: vec![geometry],
    };
    FbxDocument {
        version: 7400,
        root: FbxNode {
            name: String::new(),
            properties: Vec::new(),
            children: vec![header, objects],
        },
    }
}

fn main() {
    let dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/sweeps-2026-11/fbxsdk-keyattr-sweep");
    for name in [
        "keyattr-sweep-binary-v7500.fbx",
        "keyattr-sweep-ascii-v7500.fbx",
    ] {
        let bytes = std::fs::read(dir.join(name)).expect("fixture");
        measure(name, &bytes);
    }
    let doc = stress_document();
    let binary = oxideav_fbx::write_document(&doc).expect("binary write");
    measure("stress 200k points, binary", &binary);
    let ascii = oxideav_fbx::write_ascii_document(&doc).expect("ascii write");
    measure("stress 200k points, ASCII", &ascii);
}
