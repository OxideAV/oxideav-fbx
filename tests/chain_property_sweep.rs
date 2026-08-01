//! Deterministic randomized sweep over the node-transform-chain math
//! (`docs/3d/fbx/fbx-node-transform-chain.md` §1/§3) — fixed-seed and
//! replayable, in the style of `tests/fuzz_mutation.rs`:
//!
//! 1. `compose` ≡ the literal 11-factor doc §1 matrix product, for
//!    random chains in every rotation order;
//! 2. `decompose_sample` inverts `compose` for random chains;
//! 3. full-document `decode → encode → decode` fixed-point: random
//!    chain records survive with the composed transform intact.

use std::collections::HashMap;

use oxideav_fbx::{
    binary::{FbxDocument, FbxNode, FbxProperty},
    node_transform::{euler_to_quat, RotationOrder, TransformChain},
    write_document, FbxDecoder, FbxEncoder,
};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder, Transform};

/// Tiny deterministic LCG (same recipe as `tests/fuzz_mutation.rs`
/// uses for replayable sweeps).
struct Lcg(u64);

impl Lcg {
    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 32) as u32
    }
    /// Uniform-ish f64 in [-max, max], quantised so values survive
    /// f64 wire encoding exactly.
    fn value(&mut self, max: f64) -> f64 {
        let v = f64::from(self.next_u32()) / f64::from(u32::MAX); // 0..1
        ((v * 2.0 - 1.0) * max * 16.0).round() / 16.0
    }
    fn vec3(&mut self, max: f64) -> [f64; 3] {
        [self.value(max), self.value(max), self.value(max)]
    }
}

fn random_chain(rng: &mut Lcg) -> TransformChain {
    // Scales kept away from zero (a zero scale is non-invertible for
    // the decompose check and degenerate in authoring practice).
    let scale = |rng: &mut Lcg| 0.25 + (f64::from(rng.next_u32() % 64) + 1.0) / 16.0;
    TransformChain {
        lcl_translation: rng.vec3(10.0),
        lcl_rotation: rng.vec3(180.0),
        lcl_scaling: [scale(rng), scale(rng), scale(rng)],
        rotation_offset: rng.vec3(4.0),
        rotation_pivot: rng.vec3(4.0),
        pre_rotation: rng.vec3(90.0),
        post_rotation: rng.vec3(90.0),
        scaling_offset: rng.vec3(4.0),
        scaling_pivot: rng.vec3(4.0),
        rotation_order: RotationOrder::from_enum_int(i64::from(rng.next_u32() % 7)).unwrap(),
    }
}

// ---- matrix oracle (duplicated from the unit tests: each tests/*.rs
// file is its own crate) --------------------------------------------

type Mat4 = [[f64; 4]; 4];

fn mat_identity() -> Mat4 {
    let mut m = [[0.0; 4]; 4];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

fn mat_mul(a: Mat4, b: Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn mat_translate(v: [f64; 3]) -> Mat4 {
    let mut m = mat_identity();
    m[0][3] = v[0];
    m[1][3] = v[1];
    m[2][3] = v[2];
    m
}

fn mat_scale(v: [f64; 3]) -> Mat4 {
    let mut m = mat_identity();
    m[0][0] = v[0];
    m[1][1] = v[1];
    m[2][2] = v[2];
    m
}

fn mat_rot_axis(axis: usize, deg: f64) -> Mat4 {
    let r = deg.to_radians();
    let (s, c) = (r.sin(), r.cos());
    let mut m = mat_identity();
    match axis {
        0 => {
            m[1][1] = c;
            m[1][2] = -s;
            m[2][1] = s;
            m[2][2] = c;
        }
        1 => {
            m[0][0] = c;
            m[0][2] = s;
            m[2][0] = -s;
            m[2][2] = c;
        }
        _ => {
            m[0][0] = c;
            m[0][1] = -s;
            m[1][0] = s;
            m[1][1] = c;
        }
    }
    m
}

fn mat_euler(deg: [f64; 3], order: RotationOrder) -> Mat4 {
    let [a, b, c] = order.application_axes();
    mat_mul(
        mat_rot_axis(c, deg[c]),
        mat_mul(mat_rot_axis(b, deg[b]), mat_rot_axis(a, deg[a])),
    )
}

fn mat_transpose(m: Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = m[j][i];
        }
    }
    out
}

fn quat_to_mat(q: [f64; 4]) -> Mat4 {
    let [x, y, z, w] = q;
    let mut m = mat_identity();
    m[0][0] = 1.0 - 2.0 * (y * y + z * z);
    m[0][1] = 2.0 * (x * y - z * w);
    m[0][2] = 2.0 * (x * z + y * w);
    m[1][0] = 2.0 * (x * y + z * w);
    m[1][1] = 1.0 - 2.0 * (x * x + z * z);
    m[1][2] = 2.0 * (y * z - x * w);
    m[2][0] = 2.0 * (x * z - y * w);
    m[2][1] = 2.0 * (y * z + x * w);
    m[2][2] = 1.0 - 2.0 * (x * x + y * y);
    m
}

fn chain_matrix_literal(c: &TransformChain) -> Mat4 {
    let neg = |v: [f64; 3]| [-v[0], -v[1], -v[2]];
    let factors = [
        mat_translate(c.lcl_translation),
        mat_translate(c.rotation_offset),
        mat_translate(c.rotation_pivot),
        mat_euler(c.pre_rotation, RotationOrder::Xyz),
        mat_euler(c.lcl_rotation, c.rotation_order),
        mat_transpose(mat_euler(c.post_rotation, RotationOrder::Xyz)),
        mat_translate(neg(c.rotation_pivot)),
        mat_translate(c.scaling_offset),
        mat_translate(c.scaling_pivot),
        mat_scale(c.lcl_scaling),
        mat_translate(neg(c.scaling_pivot)),
    ];
    factors.into_iter().fold(mat_identity(), mat_mul)
}

/// 1. Closed form == literal matrix product, 256 random chains.
#[test]
fn sweep_compose_matches_literal_product() {
    let mut rng = Lcg(0x436_c0de);
    for case in 0..256 {
        let chain = random_chain(&mut rng);
        let (t, q, s) = chain.compose();
        let composed = mat_mul(mat_translate(t), mat_mul(quat_to_mat(q), mat_scale(s)));
        let literal = chain_matrix_literal(&chain);
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (composed[i][j] - literal[i][j]).abs() < 1e-8,
                    "case {case}: [{i}][{j}] {} vs {} for {chain:?}",
                    composed[i][j],
                    literal[i][j],
                );
            }
        }
    }
}

/// 2. `decompose_sample` inverts `compose`, 256 random chains.
#[test]
fn sweep_decompose_inverts_compose() {
    let mut rng = Lcg(0xdec0_bead);
    for case in 0..256 {
        let chain = random_chain(&mut rng);
        let (t, q, s) = chain.compose();
        let (lt, lr, ls) = chain.decompose_sample(t, q, s);
        for i in 0..3 {
            assert!(
                (lt[i] - chain.lcl_translation[i]).abs() < 1e-8,
                "case {case}: T {lt:?} vs {:?}",
                chain.lcl_translation
            );
            assert!((ls[i] - chain.lcl_scaling[i]).abs() < 1e-12);
        }
        let qa = euler_to_quat(lr, chain.rotation_order);
        let qb = euler_to_quat(chain.lcl_rotation, chain.rotation_order);
        let dot: f64 = (0..4).map(|i| qa[i] * qb[i]).sum();
        assert!(dot.abs() > 1.0 - 1e-9, "case {case}: R {lr:?}");
    }
}

// ---- document round trip ------------------------------------------

fn p_vec3(name: &str, type_name: &str, v: [f64; 3]) -> FbxNode {
    FbxNode {
        name: "P".into(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(type_name.as_bytes().to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(b"A".to_vec()),
            FbxProperty::F64(v[0]),
            FbxProperty::F64(v[1]),
            FbxProperty::F64(v[2]),
        ],
        children: Vec::new(),
    }
}

fn p_enum(name: &str, v: i32) -> FbxNode {
    FbxNode {
        name: "P".into(),
        properties: vec![
            FbxProperty::String(name.as_bytes().to_vec()),
            FbxProperty::String(b"enum".to_vec()),
            FbxProperty::String(Vec::new()),
            FbxProperty::String(Vec::new()),
            FbxProperty::I32(v),
        ],
        children: Vec::new(),
    }
}

fn chain_model(id: i64, name: &str, chain: &TransformChain) -> FbxNode {
    let records = vec![
        p_vec3("Lcl Translation", "Lcl Translation", chain.lcl_translation),
        p_vec3("Lcl Rotation", "Lcl Rotation", chain.lcl_rotation),
        p_vec3("Lcl Scaling", "Lcl Scaling", chain.lcl_scaling),
        p_vec3("RotationOffset", "Vector3D", chain.rotation_offset),
        p_vec3("RotationPivot", "Vector3D", chain.rotation_pivot),
        p_vec3("PreRotation", "Vector3D", chain.pre_rotation),
        p_vec3("PostRotation", "Vector3D", chain.post_rotation),
        p_vec3("ScalingOffset", "Vector3D", chain.scaling_offset),
        p_vec3("ScalingPivot", "Vector3D", chain.scaling_pivot),
        p_enum("RotationOrder", chain.rotation_order.to_enum_int() as i32),
    ];
    let display = format!("{name}\x00\x01Model");
    FbxNode {
        name: "Model".into(),
        properties: vec![
            FbxProperty::I64(id),
            FbxProperty::String(display.into_bytes()),
            FbxProperty::String(b"Mesh".to_vec()),
        ],
        children: vec![FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: records,
        }],
    }
}

/// 3. Random chain documents survive `decode → encode → decode` with
/// the composed transform intact (32 documents × 4 models each).
#[test]
fn sweep_chain_documents_round_trip() {
    let mut rng = Lcg(0xf1f7_0e5);
    for doc_case in 0..32 {
        let mut models = Vec::new();
        let mut conns = Vec::new();
        let mut expected: HashMap<String, TransformChain> = HashMap::new();
        for m in 0..4 {
            let id = 1000 + m;
            let name = format!("N{m}");
            let chain = random_chain(&mut rng);
            models.push(chain_model(id, &name, &chain));
            conns.push(FbxNode {
                name: "C".into(),
                properties: vec![
                    FbxProperty::String(b"OO".to_vec()),
                    FbxProperty::I64(id),
                    FbxProperty::I64(0),
                ],
                children: Vec::new(),
            });
            expected.insert(name, chain);
        }
        let doc = FbxDocument {
            version: 7500,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![
                    FbxNode {
                        name: "Objects".into(),
                        properties: Vec::new(),
                        children: models,
                    },
                    FbxNode {
                        name: "Connections".into(),
                        properties: Vec::new(),
                        children: conns,
                    },
                ],
            },
        };

        let bytes = write_document(&doc).expect("write");
        let first = FbxDecoder::new().decode(&bytes).expect("decode");
        let re_encoded = FbxEncoder::new().encode(&first).expect("re-encode");
        let second = FbxDecoder::new().decode(&re_encoded).expect("re-decode");

        for scene in [&first, &second] {
            for node in &scene.nodes {
                let Some(name) = node.name.as_deref() else {
                    continue;
                };
                let Some(chain) = expected.get(name) else {
                    continue;
                };
                let (et, eq, es) = chain.compose();
                let Transform::Trs {
                    translation,
                    rotation,
                    scale,
                } = node.transform
                else {
                    panic!("doc {doc_case} node {name}: expected Trs");
                };
                for i in 0..3 {
                    assert!(
                        (f64::from(translation[i]) - et[i]).abs() < 1e-3,
                        "doc {doc_case} node {name}: t {translation:?} vs {et:?}"
                    );
                    assert!((f64::from(scale[i]) - es[i]).abs() < 1e-4);
                }
                let dot: f64 = (0..4).map(|i| f64::from(rotation[i]) * eq[i]).sum();
                assert!(
                    dot.abs() > 1.0 - 1e-5,
                    "doc {doc_case} node {name}: q {rotation:?} vs {eq:?}"
                );
                assert!(!node.extras.contains_key("fbx:transform_incomplete"));
            }
        }
    }
}
