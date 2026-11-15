//! The vendor-exporter in-between blend-shape sweep
//! (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §4,
//! `fixtures/fbxsdk-blendshape-inbetween/notes.md`): thirteen channels
//! with one, two, three or no target shapes, their full-weight
//! percents, the typed in-between model, its piecewise evaluation, and
//! the writer.

mod common;

use common::{display, flavours, loose, parse_any, read, render, subtype};
use oxideav_fbx::anim_curve::bound_curves;
use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};
use oxideav_fbx::deformer::BLEND_CHANNELS_KEY;
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{
    Inbetween, Mesh, Mesh3DDecoder, Mesh3DEncoder, MorphTarget, Node, Primitive, Scene3D, Topology,
};
use serde_json::Value;

const DIR: &str = "fbxsdk-blendshape-inbetween";

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

/// The base quad's control points (notes.md "Authored scene").
const BASE: [[f32; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [10.0, 0.0, 0.0],
    [10.0, 10.0, 0.0],
    [0.0, 10.0, 0.0],
];

/// The delta a per-corner buffer holds at control point `v` (every
/// corner of one control point carries the same value).
fn at_vertex(prim: &Primitive, buffer: &[[f32; 3]], v: usize) -> [f32; 3] {
    let corners: Vec<usize> = prim
        .positions
        .iter()
        .enumerate()
        .filter(|(_, p)| **p == BASE[v])
        .map(|(i, _)| i)
        .collect();
    assert!(!corners.is_empty(), "control point {v}");
    let first = buffer[corners[0]];
    for c in &corners {
        assert_eq!(buffer[*c], first, "control point {v}");
    }
    first
}

fn slot(mesh: &Mesh, name: &str) -> usize {
    mesh.target_names
        .iter()
        .position(|n| n == name)
        .unwrap_or_else(|| panic!("channel {name}"))
}

const CHANNELS: [&str; 13] = [
    "ChA_inbetween_added_100_30_60",
    "ChB_single_default_percent_with_normals",
    "ChC_inbetween_added_25_50_100",
    "ChD_two_shapes_both_default_percent",
    "ChE_shape_legacy_style_flag",
    "ChF_shape_absolute_mode_flag",
    "ChG_shape_identical_to_base",
    "ChH_weights_over_100_and_negative",
    "ChI_no_shapes",
    "ChJ_shape_legacy_style_off",
    "ChK_shape_legacy_off_absolute_on",
    "ChL_shape_normals_distinct",
    "ChM_shape_moves_all_four_vertices",
];

/// notes.md "Channel records": shapes in connection order with their
/// `FullWeights`, and where each went in the typed model.
#[test]
fn channels_decode_with_their_station_tables() {
    type Row = (
        &'static str,
        f32,
        &'static [(&'static str, f64, &'static str)],
    );
    let table: [Row; 13] = [
        (
            CHANNELS[0],
            0.45,
            &[
                ("A_full100", 100.0, "primary"),
                ("A_at30", 30.0, "inbetween"),
                ("A_at60", 60.0, "inbetween"),
            ],
        ),
        (CHANNELS[1], 0.0, &[("B_only", 100.0, "primary")]),
        (
            CHANNELS[2],
            1.0,
            &[
                ("C_at25", 25.0, "inbetween"),
                ("C_at50", 50.0, "inbetween"),
                ("C_full100", 100.0, "primary"),
            ],
        ),
        (
            CHANNELS[3],
            0.0,
            &[("D_first", 100.0, "primary"), ("D_second", 100.0, "extra")],
        ),
        (CHANNELS[4], 0.0, &[("E_legacy", 100.0, "primary")]),
        (CHANNELS[5], 0.0, &[("F_absolute", 100.0, "primary")]),
        (CHANNELS[6], 0.0, &[("G_nodelta", 100.0, "primary")]),
        (
            CHANNELS[7],
            -0.125,
            &[
                ("H_at_minus50", -50.0, "inbetween"),
                ("H_at_150", 150.0, "inbetween"),
            ],
        ),
        (CHANNELS[8], 0.0, &[]),
        (CHANNELS[9], 0.0, &[("J_nonlegacy", 100.0, "primary")]),
        (
            CHANNELS[10],
            0.0,
            &[("K_nonlegacy_absolute", 100.0, "primary")],
        ),
        (CHANNELS[11], 0.0, &[("L_normals", 100.0, "primary")]),
        (CHANNELS[12], 0.0, &[("M_all", 100.0, "primary")]),
    ];
    for name in flavours("blendshape-inbetween") {
        let scene = decode(&read(DIR, &name));
        assert_eq!((scene.nodes.len(), scene.meshes.len()), (1, 1), "{name}");
        let mesh = &scene.meshes[0];
        let prim = &mesh.primitives[0];
        assert_eq!(mesh.target_names, CHANNELS, "{name}");
        assert_eq!(prim.targets.len(), 13, "{name}");
        let entries = prim.extras[BLEND_CHANNELS_KEY].as_array().expect("entries");
        assert_eq!(entries.len(), 13);
        for (i, (channel, weight, shapes)) in table.iter().enumerate() {
            assert_eq!(mesh.weights[i], *weight, "{name} {channel}");
            let e = &entries[i];
            assert_eq!(e["deformer"], "BS");
            assert_eq!(e["station_table"], true, "{name} {channel}");
            let got: Vec<(String, f64, String)> = e["shapes"]
                .as_array()
                .unwrap()
                .iter()
                .map(|s| {
                    (
                        s["name"].as_str().unwrap().to_owned(),
                        s["full_weight"].as_f64().unwrap(),
                        s["role"].as_str().unwrap().to_owned(),
                    )
                })
                .collect();
            let want: Vec<(String, f64, String)> = shapes
                .iter()
                .map(|(n, w, r)| ((*n).to_owned(), *w, (*r).to_owned()))
                .collect();
            assert_eq!(got, want, "{name} {channel}");
            let stations: Vec<(Option<&str>, f32)> = prim.targets[i]
                .inbetweens
                .iter()
                .map(|ib| (ib.name.as_deref(), ib.weight))
                .collect();
            let want: Vec<(Option<&str>, f32)> = shapes
                .iter()
                .filter(|s| s.2 == "inbetween")
                .map(|s| (Some(s.0), (s.1 / 100.0) as f32))
                .collect();
            assert_eq!(stations, want, "{name} {channel}");
        }
        // `F_absolute`, `J_nonlegacy` and `K_nonlegacy_absolute` carry
        // an empty `Properties70`; no other shape does.
        let with_block: Vec<&str> = entries
            .iter()
            .flat_map(|e| e["shapes"].as_array().unwrap())
            .filter(|s| s["properties70"] == true)
            .map(|s| s["name"].as_str().unwrap())
            .collect();
        assert_eq!(
            with_block,
            ["F_absolute", "J_nonlegacy", "K_nonlegacy_absolute"]
        );
    }
}

/// notes.md "Shape records": every shape stores its delta against the
/// base mesh; the typed targets hold those deltas and evaluate
/// piecewise through their stations.
#[test]
fn targets_hold_the_authored_deltas_and_evaluate_piecewise() {
    for name in flavours("blendshape-inbetween") {
        let scene = decode(&read(DIR, &name));
        let mesh = &scene.meshes[0];
        let prim = &mesh.primitives[0];
        let target = |channel: &str| &prim.targets[slot(mesh, channel)];
        let pos =
            |t: &MorphTarget, v: usize| at_vertex(prim, t.position.as_ref().expect("position"), v);
        let near = |a: [f32; 3], b: [f32; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-5);

        // ChA: full shape v2 → +8; at 30 %: v1 +1, v2 +2; at 60 %: v2 +5.
        let a = target(CHANNELS[0]);
        assert_eq!(pos(a, 2), [0.0, 0.0, 8.0]);
        assert_eq!(pos(a, 1), [0.0; 3]);
        let ib = |t: &MorphTarget, k: usize, v: usize| {
            at_vertex(
                prim,
                t.inbetweens[k].position.as_ref().expect("position"),
                v,
            )
        };
        assert_eq!(
            (ib(a, 0, 1), ib(a, 0, 2)),
            ([0.0, 0.0, 1.0], [0.0, 0.0, 2.0])
        );
        assert_eq!((ib(a, 1, 1), ib(a, 1, 2)), ([0.0; 3], [0.0, 0.0, 5.0]));
        // Piecewise through 0 → 30 → 60 → 100.
        for (w, v1, v2) in [
            (0.0, 0.0, 0.0),
            (0.15, 0.5, 1.0),
            (0.3, 1.0, 2.0),
            (0.45, 0.5, 3.5),
            (0.6, 0.0, 5.0),
            (0.8, 0.0, 6.5),
            (1.0, 0.0, 8.0),
        ] {
            let at = a.at_weight(w);
            assert!(
                near(pos(&at, 1), [0.0, 0.0, v1]),
                "{name} ChA v1 at {w}: {:?}",
                pos(&at, 1)
            );
            assert!(
                near(pos(&at, 2), [0.0, 0.0, v2]),
                "{name} ChA v2 at {w}: {:?}",
                pos(&at, 2)
            );
        }
        // ChC: stations 25 / 50 / 100 on v3: 1, 2, 4.
        let c = target(CHANNELS[2]);
        for (w, z) in [
            (0.125, 0.5),
            (0.25, 1.0),
            (0.5, 2.0),
            (0.75, 3.0),
            (1.0, 4.0),
        ] {
            assert!(
                near(pos(&c.at_weight(w), 3), [0.0, 0.0, z]),
                "{name} ChC at {w}"
            );
        }
        // ChH: stations at −50 % (v2 −4) and 150 % (v2 +12); no shape at
        // 100 %, so the target's own deltas are the station table read
        // at weight 1: 12 · (1 / 1.5) = 8.
        let h = target(CHANNELS[7]);
        assert!(near(pos(h, 2), [0.0, 0.0, 8.0]), "{:?}", pos(h, 2));
        for (w, z) in [
            (-0.5, -4.0),
            (-0.125, -1.0),
            (0.0, 0.0),
            (0.75, 6.0),
            (1.0, 8.0),
            (1.5, 12.0),
        ] {
            assert!(
                near(pos(&h.at_weight(w), 2), [0.0, 0.0, z]),
                "{name} ChH at {w}"
            );
        }
        // ChD: two shapes at 100 % — the first is the target.
        assert_eq!(pos(target(CHANNELS[3]), 1), [1.0, 0.0, 0.0]);
        // ChB / ChL: normal deltas.
        let norm =
            |t: &MorphTarget, v: usize| at_vertex(prim, t.normal.as_ref().expect("normal"), v);
        let b = target(CHANNELS[1]);
        assert_eq!(pos(b, 0), [-1.0, -2.0, -3.0]);
        assert_eq!(norm(b, 0), [0.0, 1.0, -1.0]);
        let l = target(CHANNELS[11]);
        assert_eq!((pos(l, 0), pos(l, 2)), ([1.0, 2.0, 3.0], [3.0, 4.0, 15.0]));
        assert!(near(norm(l, 0), [0.6, 0.8, -1.0]));
        assert!(near(norm(l, 2), [0.4, 0.0, -0.8]));
        // ChG moves nothing; ChI has no shape at all; ChM moves all four.
        for ch in [CHANNELS[6], CHANNELS[8]] {
            assert!(
                target(ch)
                    .position
                    .as_ref()
                    .unwrap()
                    .iter()
                    .all(|d| *d == [0.0; 3])
            );
        }
        let m = target(CHANNELS[12]);
        assert_eq!(
            [pos(m, 0)[2], pos(m, 1)[2], pos(m, 2)[2], pos(m, 3)[2]],
            [2.0, 4.0, 3.0, 1.0]
        );
        // The mesh at its static weights: ChA at 45 %, ChC at 100 %,
        // ChH at −12.5 %.
        let posed = prim.apply_morph_weights(&mesh.weights);
        let moved = |v: usize| -> [f32; 3] {
            let i = prim.positions.iter().position(|p| *p == BASE[v]).unwrap();
            let p = posed.positions[i];
            [p[0] - BASE[v][0], p[1] - BASE[v][1], p[2] - BASE[v][2]]
        };
        assert!(near(moved(1), [0.0, 0.0, 0.5]), "{:?}", moved(1));
        assert!(near(moved(2), [0.0, 0.0, 3.5 - 1.0]), "{:?}", moved(2));
        assert!(near(moved(3), [0.0, 0.0, 4.0]), "{:?}", moved(3));
    }
}

/// The typed model holds deltas as `f32`; compare float64 arrays at
/// that precision.
fn single_precision(n: &FbxNode) -> FbxNode {
    FbxNode {
        name: n.name.clone(),
        properties: n
            .properties
            .iter()
            .map(|p| match p {
                FbxProperty::F64Array(a) => {
                    FbxProperty::F64Array(a.iter().map(|v| f64::from(*v as f32)).collect())
                }
                other => other.clone(),
            })
            .collect(),
        children: n.children.iter().map(single_precision).collect(),
    }
}

/// `(name, sub-type, body)` of every deformer and shape, and the
/// shape → channel connections in file order.
type Census = (Vec<(String, String, Vec<String>)>, Vec<(String, String)>);

fn census(doc: &FbxDocument) -> Census {
    let objects = doc.root.child("Objects").expect("Objects");
    let mut names = std::collections::HashMap::new();
    let mut out = Vec::new();
    for o in &objects.children {
        let Some(id) = o.properties.first().and_then(FbxProperty::as_i64) else {
            continue;
        };
        names.insert(id, display(o));
        if o.name == "Deformer" || (o.name == "Geometry" && subtype(o) == "Shape") {
            out.push((
                display(o),
                subtype(o),
                o.children
                    .iter()
                    .map(|c| render(loose, &single_precision(c)))
                    .collect(),
            ));
        }
    }
    let conns = doc
        .root
        .child("Connections")
        .expect("Connections")
        .children_named("C")
        .filter(|c| c.properties.first().and_then(FbxProperty::as_str) == Some("OO"))
        .filter_map(|c| {
            let child = names.get(&c.properties.get(1)?.as_i64()?)?;
            let parent = names.get(&c.properties.get(2)?.as_i64()?)?;
            parent
                .starts_with("Ch")
                .then(|| (child.clone(), parent.clone()))
        })
        .collect();
    (out, conns)
}

/// Scene → file → document: the blend-shape deformer, every channel
/// (its `Properties70`, `DeformPercent`, `FullWeights`) and every shape
/// come back record for record, shapes connected in the same order —
/// and the `DeformPercent` curve with them.
#[test]
fn scene_re_encode_keeps_channels_and_shapes() {
    for name in flavours("blendshape-inbetween") {
        let bytes = read(DIR, &name);
        let source = parse_any(&bytes);
        let (src_objects, src_conns) = census(&source);
        assert_eq!(src_objects.len(), 14 + 18, "{name}");
        let src_curves = bound_curves(&source);
        assert_eq!(src_curves.len(), 1);
        let scene = decode(&bytes);
        for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
            let out = encode(&scene, form);
            let again = parse_any(&out);
            let (objects, conns) = census(&again);
            let sorted = |mut v: Vec<(String, String, Vec<String>)>| {
                v.sort();
                v
            };
            let (a, b) = (sorted(src_objects.clone()), sorted(objects));
            assert_eq!(a.len(), b.len(), "{name} {form:?}");
            for (x, y) in a.iter().zip(&b) {
                assert_eq!(x, y, "{name} {form:?}");
            }
            // Per channel, the shapes in attachment order.
            let per_channel = |c: &[(String, String)], ch: &str| -> Vec<String> {
                c.iter()
                    .filter(|(_, p)| p == ch)
                    .map(|(s, _)| s.clone())
                    .collect()
            };
            for ch in CHANNELS {
                assert_eq!(
                    per_channel(&conns, ch),
                    per_channel(&src_conns, ch),
                    "{name} {form:?} {ch}"
                );
            }
            assert_eq!(
                per_channel(&conns, CHANNELS[0]),
                ["A_full100", "A_at30", "A_at60"]
            );
            // The channel-percent curve: 0 at 0 s, 100 at 1 s, linear
            // (0x104), `Default` 45.
            let curves = bound_curves(&again);
            assert_eq!(curves.len(), 1, "{name} {form:?}");
            assert_eq!(curves[0].curve, src_curves[0].curve, "{name} {form:?}");
            assert_eq!(curves[0].target.as_deref(), Some(CHANNELS[0]));
            assert_eq!(curves[0].property.as_deref(), Some("DeformPercent"));
            assert_eq!(curves[0].curve.default, 45.0);
            // And the second decode is the same typed model.
            let scene2 = decode(&out);
            let (p, q) = (
                &scene.meshes[0].primitives[0],
                &scene2.meshes[0].primitives[0],
            );
            assert_eq!(p.targets, q.targets, "{name} {form:?}");
            assert_eq!(scene.meshes[0].weights, scene2.meshes[0].weights);
            assert_eq!(p.extras[BLEND_CHANNELS_KEY], q.extras[BLEND_CHANNELS_KEY]);
        }
    }
}

/// In-betweens authored through the scene model: the writer emits one
/// channel with the target at 100 % followed by its in-betweens, one
/// `FullWeights` percent per shape, and the decode reads them back.
#[test]
fn authored_inbetweens_write_a_station_table() {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut target = MorphTarget::new();
    target.position = Some(vec![[0.0, 0.0, 4.0], [0.0; 3], [0.0; 3]]);
    target.inbetweens = vec![
        Inbetween::new(0.25)
            .with_name("quarter")
            .with_position(vec![[0.0, 0.0, 3.0], [0.0, 0.0, 1.0], [0.0; 3]]),
        Inbetween::new(0.5).with_name("half").with_position(vec![
            [0.0, 0.0, 3.5],
            [0.0; 3],
            [0.0; 3],
        ]),
    ];
    prim.targets.push(target.clone());
    let mut mesh = Mesh::new(Some("Tri".to_string()));
    mesh.primitives.push(prim);
    mesh.weights = vec![0.375];
    mesh.target_names = vec!["Smile".to_string()];
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let node = scene.add_node(Node::new().with_name("Tri").with_mesh(mid));
    scene.roots.push(node);

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let doc = parse_any(&out);
        let objects = doc.root.child("Objects").unwrap();
        let channel: &FbxNode = objects
            .children_named("Deformer")
            .find(|d| subtype(d) == "BlendShapeChannel")
            .expect("channel");
        assert_eq!(display(channel), "Smile");
        let floats = |n: &FbxNode, name: &str| -> Vec<f64> {
            match &n.child(name).expect(name).properties[0] {
                FbxProperty::F64Array(a) => a.clone(),
                other => panic!("{name}: {other:?}"),
            }
        };
        assert_eq!(floats(channel, "FullWeights"), vec![100.0, 25.0, 50.0]);
        // The percent lives in a property (the authoritative place).
        let props = oxideav_fbx::properties70::PropertyMap::from_element(channel);
        assert_eq!(props.as_f64("DeformPercent"), Some(37.5));
        let shapes: Vec<&FbxNode> = objects
            .children_named("Geometry")
            .filter(|g| subtype(g) == "Shape")
            .collect();
        assert_eq!(
            shapes.iter().map(|s| display(s)).collect::<Vec<_>>(),
            ["Smile", "quarter", "half"]
        );
        // Sparse, against the base mesh, normals present and zero.
        assert_eq!(
            floats(shapes[1], "Vertices"),
            vec![0.0, 0.0, 3.0, 0.0, 0.0, 1.0]
        );
        assert_eq!(floats(shapes[1], "Normals"), vec![0.0; 6]);

        let back = decode(&out);
        let got = &back.meshes[0].primitives[0].targets[0];
        assert_eq!(got.position, target.position, "{form:?}");
        assert_eq!(got.inbetweens, target.inbetweens, "{form:?}");
        assert_eq!(back.meshes[0].weights, vec![0.375]);
        let entry: &Value = &back.meshes[0].primitives[0].extras[BLEND_CHANNELS_KEY][0];
        assert_eq!(entry["station_table"], true);
    }
}
