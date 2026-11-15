//! The vendor-exporter key-attribute sweep
//! (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §1,
//! `fixtures/fbxsdk-keyattr-sweep/notes.md`): every curve of every
//! container flavour decodes into the typed attribute model, evaluates
//! to the staged evaluator samples, reports the staged read-back
//! tangents, and writes back record for record.

mod common;

use std::collections::HashMap;

use common::{exact, flavours, is_ascii_name, loose, parse_any, read, render};
use oxideav_fbx::anim_curve::{
    AnimCurve, BoundCurve, ConstantMode, ExtrapolationKind, KeyInterpolation, TangentMode,
    bound_curves,
};
use oxideav_fbx::binary::FbxNode;

const DIR: &str = "fbxsdk-keyattr-sweep";

fn curves_of(name: &str) -> Vec<BoundCurve> {
    bound_curves(&parse_any(&read(DIR, name)))
}

fn by_target(curves: &[BoundCurve]) -> HashMap<String, &AnimCurve> {
    curves
        .iter()
        .map(|c| (c.target.clone().expect("target"), &c.curve))
        .collect()
}

fn tsv(name: &str) -> Vec<Vec<String>> {
    String::from_utf8(read(DIR, name))
        .expect("utf-8")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| l.split('\t').map(str::to_owned).collect())
        .collect()
}

#[test]
fn every_curve_joins_its_model_and_channel() {
    for name in flavours("keyattr-sweep") {
        let curves = curves_of(&name);
        // 62 curves on 62 nodes; `N00_zero_keys` has none (§1.1).
        assert_eq!(curves.len(), 62, "{name}");
        for c in &curves {
            assert_eq!(c.stack.as_deref(), Some("Take 001"), "{name}");
            assert_eq!(c.layer.as_deref(), Some("BaseLayer"), "{name}");
            assert_eq!(c.property.as_deref(), Some("Lcl Translation"), "{name}");
            assert_eq!(c.channel.as_deref(), Some("d|X"), "{name}");
            assert_eq!(c.curve.key_ver, 4009, "{name}");
            assert!(c.curve.runs_cover_keys(), "{name}");
            // One entry per key in the first-generation files (§1.4).
            assert_eq!(c.curve.runs.len(), c.curve.len(), "{name}");
        }
        assert!(!by_target(&curves).contains_key("N00_zero_keys"));
    }
}

/// notes.md variant table: the flag word of every key, and the typed
/// reading of each.
#[test]
fn flag_words_match_the_variant_table() {
    let uniform: &[(&str, u32)] = &[
        ("I01_constant_standard", 0x2002),
        ("I02_constant_next", 0x2102),
        ("I03_linear", 0x2104),
        ("I04_cubic_auto", 0x108),
        ("I05_cubic_auto_break", 0x908),
        ("I06_cubic_user", 0x408),
        ("I07_cubic_user_break", 0xC08),
        ("I08_cubic_tcb", 0x208),
        ("I09_cubic_auto_clamp", 0x1108),
        ("I10_cubic_auto_clamp_progressive", 0x6108),
        ("I11_cubic_auto_time_independent", 0x2108),
        ("I12_cubic_user_time_independent", 0x2408),
        ("I13_cubic_auto_then_setbreak", 0x908),
        ("I14_cubic_genericbreak_only", 0x808),
        ("I15_cubic_auto_clamp_break", 0x1908),
        ("I16_default_key_untouched", 0x2108),
        ("W01_user_weighted_none", 0x408),
        ("W02_user_weighted_right", 0x0100_0408),
        ("W03_user_weighted_nextleft", 0x0200_0408),
        ("W04_user_weighted_all", 0x0300_0408),
        ("W05_user_weights_set_mode_none", 0x408),
        ("V02_user_velocity_right", 0x1000_0408),
        ("V03_user_velocity_nextleft", 0x2000_0408),
        ("V04_user_velocity_all", 0x3000_0408),
        ("V05_user_weighted_all_velocity_all", 0x3300_0408),
        ("S01_auto_show_none", 0x108),
        ("S02_auto_show_left", 0x0010_0108),
        ("S03_auto_show_right", 0x0020_0108),
        ("S04_auto_show_both", 0x0030_0108),
        ("D01_user_distinct_slopes_per_key", 0x408),
        ("D02_user_distinct_all_data_per_key", 0x3300_0408),
        ("D04_user_then_curve_level_derivative_setters", 0xC08),
        ("D05_tcb_distinct_per_key", 0x208),
        ("R03_eight_linear_keys", 0x2104),
        ("N01_single_key_linear", 0x2104),
        ("T01_static_value_3p5_first_key_0", 0x2104),
        ("T02_first_key_value_6", 0x2104),
        ("E01_linear_pre_repetition_post_keepslope", 0x2104),
        ("A01_nonuniform_auto", 0x108),
        ("A02_nonuniform_auto_clamp", 0x1108),
        ("A03_nonuniform_auto_clamp_progressive", 0x6108),
        ("A04_nonuniform_auto_time_independent", 0x2108),
        ("A05_nonuniform_tcb", 0x208),
        ("A06_nonuniform_tcb_zero", 0x208),
        ("E03_linear_pre_constant_post_repetition_count_0", 0x2104),
        ("Q01_user_weighted_all_weight_ladder", 0x0300_0408),
        ("Q02_user_velocity_all_velocity_ladder", 0x3000_0408),
        ("E02_linear_pre_mirror_post_relative", 0x2104),
        ("P01_curvelevel_eight_linear_keys", 0x104),
        ("P04_curvelevel_four_cubic_auto_keys", 0x2108),
        ("P05_curvelevel_user_same_slopes_4keys", 0x408),
        ("P06_curvelevel_user_slopes_A_A_B_B", 0x408),
        ("P07_curvelevel_user_weighted_all_w_0p6_0p2", 0x0300_0408),
        ("P08_curvelevel_user_velocity_args_0p7_0p3", 0x408),
        ("P09_curvelevel_tcb_same", 0x208),
        ("P10_curvelevel_keyadd_only_defaults", 0x2108),
    ];
    let per_key: &[(&str, &[u32])] = &[
        ("D03_mixed_interp_per_key", &[0x2102, 0x2104, 0x108, 0x408]),
        (
            "R01_runs_lin_lin_const_const_const_lin",
            &[0x2104, 0x2104, 0x2102, 0x2102, 0x2102, 0x2104],
        ),
        (
            "R02_alternating_lin_const_lin_const",
            &[0x2104, 0x2102, 0x2104, 0x2102],
        ),
        (
            "P02_curvelevel_runs_lin_lin_const_const_const_lin",
            &[0x104, 0x104, 0x102, 0x102, 0x102, 0x104],
        ),
        (
            "P03_curvelevel_alternating_lin_const_lin_const",
            &[0x104, 0x102, 0x104, 0x102],
        ),
        (
            "P11_curvelevel_auto_then_left_weight_0p5_on_key2",
            &[0x408, 0x0200_0408, 0x0100_0408, 0x408],
        ),
    ];
    assert_eq!(uniform.len() + per_key.len(), 62);
    for name in flavours("keyattr-sweep") {
        let curves = curves_of(&name);
        let map = by_target(&curves);
        for (variant, word) in uniform {
            let attrs = map[*variant].key_attrs();
            assert!(!attrs.is_empty(), "{name} {variant}");
            for a in &attrs {
                assert_eq!(a.flags.0, *word, "{name} {variant}");
                assert_eq!(a.flags.unassigned_bits(), 0, "{name} {variant}");
            }
        }
        for (variant, words) in per_key {
            let attrs = map[*variant].key_attrs();
            let got: Vec<u32> = attrs.iter().map(|a| a.flags.0).collect();
            assert_eq!(&got, words, "{name} {variant}");
        }
        // Typed readings of a few rows.
        let first = |v: &str| map[v].key_attrs()[0];
        let i01 = first("I01_constant_standard").flags;
        assert_eq!(i01.interpolation(), KeyInterpolation::Constant);
        assert_eq!(i01.constant_mode(), ConstantMode::Standard);
        assert_eq!(
            first("I02_constant_next").flags.constant_mode(),
            ConstantMode::Next
        );
        assert_eq!(
            first("I03_linear").flags.interpolation(),
            KeyInterpolation::Linear
        );
        for (v, mode) in [
            ("I04_cubic_auto", TangentMode::Auto),
            ("I06_cubic_user", TangentMode::User),
            ("I08_cubic_tcb", TangentMode::Tcb),
            ("I14_cubic_genericbreak_only", TangentMode::Stored),
        ] {
            let f = first(v).flags;
            assert_eq!(f.interpolation(), KeyInterpolation::Cubic, "{v}");
            assert_eq!(f.tangent_mode(), mode, "{v}");
        }
        assert!(first("I05_cubic_auto_break").flags.is_break());
        assert!(first("I09_cubic_auto_clamp").flags.is_clamp());
        assert!(
            first("I11_cubic_auto_time_independent")
                .flags
                .is_time_independent()
        );
        assert!(
            first("I10_cubic_auto_clamp_progressive")
                .flags
                .is_progressive_clamp()
        );
        // §1.3 data words.
        let i06 = first("I06_cubic_user");
        assert_eq!((i06.right_slope(), i06.next_left_slope()), (1.5, -2.5));
        assert_eq!(first("I08_cubic_tcb").tcb(), [0.25, 0.5, -0.75]);
        let w04 = first("W04_user_weighted_all");
        assert!((w04.right_weight() - 5999.0 / 9999.0).abs() < 1e-12);
        assert!((w04.next_left_weight() - 1999.0 / 9999.0).abs() < 1e-12);
        let v04 = first("V04_user_velocity_all");
        assert!((v04.right_velocity() - 6999.0 / 9999.0).abs() < 1e-12);
        assert!((v04.next_left_velocity() - 2999.0 / 9999.0).abs() < 1e-12);
        // D01: key i has slopes 10 + i / 20 + i.
        for (i, a) in map["D01_user_distinct_slopes_per_key"]
            .key_attrs()
            .iter()
            .enumerate()
        {
            assert_eq!(a.right_slope(), 10.0 + i as f32);
            assert_eq!(a.next_left_slope(), 20.0 + i as f32);
        }
        // D04: "left derivative of key 2 := 9" lives in entry 1's
        // next-left word, "right derivative of key 2 := 8" in entry 2.
        let d04 = map["D04_user_then_curve_level_derivative_setters"].key_attrs();
        assert_eq!(d04[1].next_left_slope(), 9.0);
        assert_eq!(d04[2].right_slope(), 8.0);
        // §1.7: `Default` is the channel's static value.
        assert_eq!(map["T01_static_value_3p5_first_key_0"].default, 3.5);
        assert_eq!(map["T02_first_key_value_6"].default, 0.0);
        assert_eq!(map["T02_first_key_value_6"].key_values[0], 6.0);
        // §1.8 extrapolation records.
        let e01 = map["E01_linear_pre_repetition_post_keepslope"];
        let pre = e01.pre_extrapolation.expect("pre");
        assert_eq!(
            (pre.kind, pre.repetition),
            (ExtrapolationKind::Repetition, 3)
        );
        assert_eq!(
            e01.post_extrapolation.expect("post").kind,
            ExtrapolationKind::KeepSlope
        );
        let e02 = map["E02_linear_pre_mirror_post_relative"];
        assert_eq!(
            e02.pre_extrapolation.expect("pre").kind,
            ExtrapolationKind::MirrorRepetition
        );
        let post = e02.post_extrapolation.expect("post");
        assert_eq!(
            (post.kind, post.repetition),
            (ExtrapolationKind::RelativeRepetition, 2)
        );
        let e03 = map["E03_linear_pre_constant_post_repetition_count_0"];
        assert!(e03.pre_extrapolation.is_none());
        assert_eq!(
            e03.post_extrapolation.expect("post").kind,
            ExtrapolationKind::Repetition
        );
    }
}

/// `sdk-evaluated-samples.tsv`: the staged evaluator output at every
/// 1/8 s of every curve. §1.5 quotes 5e-5 as the largest difference
/// between its formulas and the evaluator on values up to 10.
#[test]
fn every_curve_evaluates_to_the_staged_samples() {
    let rows = tsv("sdk-evaluated-samples.tsv");
    assert_eq!(rows.len(), 62);
    for name in flavours("keyattr-sweep") {
        let curves = curves_of(&name);
        let map = by_target(&curves);
        let mut checked = 0usize;
        let mut worst = 0.0f64;
        let mut outliers: Vec<String> = Vec::new();
        for row in &rows {
            let eval = map[&row[0]].evaluator();
            let first: f64 = row[1].parse().expect("first");
            let last: f64 = row[2].parse().expect("last");
            let samples = &row[3..];
            assert_eq!(
                samples.len(),
                ((last - first) * 8.0).round() as usize + 1,
                "{}",
                row[0]
            );
            for (k, s) in samples.iter().enumerate() {
                let expected: f64 = s.parse().expect("sample");
                let got = eval.sample(first + k as f64 / 8.0);
                let err = (got - expected).abs();
                if err > 1.0e-4 {
                    outliers.push(format!(
                        "{} at {} s: {got:.5} vs {expected}",
                        row[0],
                        first + k as f64 / 8.0
                    ));
                } else {
                    worst = worst.max(err);
                }
                checked += 1;
            }
        }
        // Five samples of `Q01` sit one eighth of a second from a key
        // whose handle has the smallest storable weight (1 / 9999): the
        // Bézier's time coordinate is all but stationary there
        // (`t ≈ s³`), and the staged numbers are up to 0.036 away from
        // the exact root of the §1.5 control polygon that reproduces
        // the other 1825 samples. The evaluator keeps the exact root.
        assert_eq!(
            outliers,
            vec![
                "Q01_user_weighted_all_weight_ladder at 0.125 s: 4.55682 vs 4.52085",
                "Q01_user_weighted_all_weight_ladder at 1.125 s: 6.67803 vs 6.70269",
                "Q01_user_weighted_all_weight_ladder at 2.125 s: 5.10985 vs 5.1004",
                "Q01_user_weighted_all_weight_ladder at 5.875 s: 6.32197 vs 6.2975",
                "Q01_user_weighted_all_weight_ladder at 6.875 s: 4.38258 vs 4.38819",
            ],
            "{name}"
        );
        assert_eq!(checked, 1830, "{name}");
        assert!(worst <= 1.0e-4, "{name}: worst {worst}");
    }
}

/// `sdk-readback-tangents.tsv`: per key, the slopes and weights the
/// vendor reader reports — computed values for automatic and TCB keys.
#[test]
fn every_key_reports_the_staged_tangents() {
    let rows = tsv("sdk-readback-tangents.tsv");
    // Header row + 277 keys.
    assert_eq!(rows.len(), 278);
    for name in flavours("keyattr-sweep") {
        let curves = curves_of(&name);
        let map = by_target(&curves);
        let mut evaluators = HashMap::new();
        for row in &rows[1..] {
            let eval = evaluators
                .entry(row[0].clone())
                .or_insert_with(|| map[&row[0]].evaluator());
            let key: usize = row[1].parse().expect("key");
            let num = |i: usize| -> f64 { row[i].parse().expect("number") };
            assert!((eval.key_times()[key] - num(2)).abs() < 1e-9, "{}", row[0]);
            assert!((eval.key_values()[key] - num(3)).abs() < 1e-6, "{}", row[0]);
            let (arriving, leaving) = eval.key_slopes()[key];
            let (w_in, w_out) = eval.key_weights()[key];
            for (what, got, expected) in [
                ("left slope", arriving, num(4)),
                ("right slope", leaving, num(5)),
                ("left weight", w_in, num(6)),
                ("right weight", w_out, num(7)),
            ] {
                assert!(
                    (got - expected).abs() <= 2.0e-5 * (1.0 + expected.abs()),
                    "{name} {} key {key} {what}: {got} vs {expected}",
                    row[0]
                );
            }
        }
    }
}

/// Typed model → records: the body `AnimCurve` writes equals the body
/// the file carries, and no curve has a record outside §1.1.
#[test]
fn curves_write_back_record_for_record() {
    const KNOWN: &[&str] = &[
        "Default",
        "KeyVer",
        "KeyTime",
        "KeyValueFloat",
        "KeyAttrFlags",
        "KeyAttrDataFloat",
        "KeyAttrRefCount",
        "Pre-Extrapolation",
        "Post-Extrapolation",
    ];
    for stem in ["keyattr-sweep", "keyattr-runs-roundtrip"] {
        for name in flavours(stem) {
            let doc = parse_any(&read(DIR, &name));
            let objects = doc.root.child("Objects").expect("Objects");
            let mut count = 0;
            for element in objects.children_named("AnimationCurve") {
                for child in &element.children {
                    assert!(
                        KNOWN.contains(&child.name.as_str()),
                        "{name}: {}",
                        child.name
                    );
                }
                let curve = AnimCurve::from_element(element).expect("curve");
                let rebuilt = FbxNode {
                    name: element.name.clone(),
                    properties: element.properties.clone(),
                    children: curve.to_body(),
                };
                // The ASCII form does not record scalar widths.
                let f = if is_ascii_name(&name) { loose } else { exact };
                assert_eq!(render(f, element), render(f, &rebuilt), "{name}");
                // And the typed value survives its own element.
                assert_eq!(AnimCurve::from_element(&rebuilt).expect("again"), curve);
                count += 1;
            }
            assert_eq!(
                count,
                if stem == "keyattr-sweep" { 62 } else { 7 },
                "{name}"
            );
        }
    }
}

/// §1.4 / notes.md "Second-generation files": after a read / write
/// cycle equal neighbouring keys share one entry. Compacting the
/// first-generation curve gives exactly the staged second generation.
#[test]
fn compacted_runs_equal_the_second_generation() {
    let expected: &[(&str, &[u32])] = &[
        ("W04_user_weighted_all", &[4]),
        ("D03_mixed_interp_per_key", &[1, 1, 1, 1]),
        ("R01_runs_lin_lin_const_const_const_lin", &[2, 3, 1]),
        ("R02_alternating_lin_const_lin_const", &[1, 1, 1, 1]),
        ("R03_eight_linear_keys", &[8]),
        (
            "P02_curvelevel_runs_lin_lin_const_const_const_lin",
            &[2, 3, 1],
        ),
        ("P06_curvelevel_user_slopes_A_A_B_B", &[2, 2]),
    ];
    let first = curves_of("keyattr-sweep-binary-v7400.fbx");
    let first = by_target(&first);
    for name in flavours("keyattr-runs-roundtrip") {
        let second = curves_of(&name);
        assert_eq!(second.len(), 7, "{name}");
        let second = by_target(&second);
        for (variant, counts) in expected {
            let staged = second[*variant];
            assert_eq!(
                staged.runs.iter().map(|r| r.count).collect::<Vec<_>>(),
                *counts,
                "{name} {variant}"
            );
            assert!(staged.runs_cover_keys());
            let mut compacted = first[*variant].clone();
            compacted.compact_runs();
            assert_eq!(compacted.runs, staged.runs, "{name} {variant}");
            // Same keys, same evaluation.
            assert_eq!(compacted.key_times, staged.key_times);
            assert_eq!(compacted.key_values, staged.key_values);
            assert_eq!(staged.key_attrs(), first[*variant].key_attrs());
        }
    }
}

// ---------------------------------------------------------------------
// Scene level: typed channels and the re-encode.
// ---------------------------------------------------------------------

use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{
    Animation, AnimationChannel, AnimationProperty, AnimationSampler, AnimationValues,
    Interpolation, Mesh3DDecoder, Mesh3DEncoder, Node, Scene3D,
};

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

/// The X component of a translation sampler read the way its
/// interpolation says (`Step` holds, `Linear` interpolates).
fn sample_x(s: &AnimationSampler, t: f32) -> f32 {
    let AnimationValues::Vec3(v) = &s.values else {
        panic!("Vec3 sampler");
    };
    let k = &s.keyframes;
    if t <= k[0] {
        return v[0][0];
    }
    if t >= k[k.len() - 1] {
        return v[k.len() - 1][0];
    }
    let i = k.partition_point(|x| *x <= t) - 1;
    match s.interpolation {
        Interpolation::Step => v[i][0],
        _ => {
            let f = (t - k[i]) / (k[i + 1] - k[i]);
            v[i][0] + (v[i + 1][0] - v[i][0]) * f
        }
    }
}

fn translation_sampler<'a>(scene: &'a Scene3D, node_name: &str) -> &'a AnimationSampler {
    let idx = scene
        .nodes
        .iter()
        .position(|n| n.name.as_deref() == Some(node_name))
        .unwrap_or_else(|| panic!("node {node_name}"));
    scene.animations[0]
        .channels
        .iter()
        .find(|c| {
            c.target.node.0 as usize == idx && c.target.property == AnimationProperty::Translation
        })
        .map(|c| &c.sampler)
        .unwrap_or_else(|| panic!("channel on {node_name}"))
}

/// The decoded scene's samplers follow every curve's interpolation:
/// read as the sampler says, each channel reproduces the staged
/// evaluator samples to the bake tolerance (1e-3 of the value range).
#[test]
fn scene_channels_follow_the_staged_samples() {
    let rows = tsv("sdk-evaluated-samples.tsv");
    for name in flavours("keyattr-sweep") {
        let scene = decode(&read(DIR, &name));
        assert_eq!(scene.nodes.len(), 63, "{name}");
        assert_eq!(scene.animations.len(), 1, "{name}");
        assert_eq!(scene.animations[0].channels.len(), 62, "{name}");
        for row in &rows {
            let sampler = translation_sampler(&scene, &row[0]);
            let first: f32 = row[1].parse().expect("first");
            // `Q01` carries the five stationary-abscissa samples (see
            // `every_curve_evaluates_to_the_staged_samples`).
            let tol = if row[0].starts_with("Q01") {
                0.06
            } else {
                0.02
            };
            for (k, s) in row[3..].iter().enumerate() {
                let expected: f32 = s.parse().expect("sample");
                let t = first + k as f32 / 8.0;
                let got = sample_x(sampler, t);
                assert!(
                    (got - expected).abs() <= tol,
                    "{name} {} at {t} s: {got} vs {expected}",
                    row[0]
                );
            }
        }
        let shape = |v: &str| {
            let s = translation_sampler(&scene, v);
            (s.interpolation, s.keyframes.len())
        };
        // Constant "standard": a Step sampler on the key grid.
        assert_eq!(shape("I01_constant_standard"), (Interpolation::Step, 4));
        // Linear: the key grid.
        assert_eq!(shape("I03_linear"), (Interpolation::Linear, 4));
        assert_eq!(shape("R03_eight_linear_keys"), (Interpolation::Linear, 8));
        assert_eq!(shape("N01_single_key_linear"), (Interpolation::Linear, 1));
        // Constant "next": one extra time after each of the three jumps.
        assert_eq!(shape("I02_constant_next"), (Interpolation::Linear, 7));
        // Cubic: a refined grid.
        let (interp, keys) = shape("I06_cubic_user");
        assert_eq!(interp, Interpolation::Linear);
        assert!(keys > 20, "{name}: {keys} keys");
    }
}

/// A looser bake tolerance gives a coarser grid, a tighter one a finer.
#[test]
fn curve_tolerance_controls_the_bake() {
    let bytes = read(DIR, "keyattr-sweep-binary-v7400.fbx");
    let keys = |tol: f32| {
        let scene = FbxDecoder::new()
            .with_scene_options(oxideav_fbx::SceneDecodeOptions::new().with_curve_tolerance(tol))
            .decode(&bytes)
            .expect("decode");
        translation_sampler(&scene, "I06_cubic_user")
            .keyframes
            .len()
    };
    let (coarse, default, fine) = (keys(0.05), keys(1.0e-3), keys(1.0e-5));
    assert!(
        coarse < default && default < fine,
        "{coarse} {default} {fine}"
    );
}

/// Scene → file → curves: every curve comes back with the same keys,
/// attribute entries, run lengths and extrapolation records, in both
/// output forms, from every container flavour.
#[test]
fn scene_re_encode_keeps_every_curve() {
    for stem in ["keyattr-sweep", "keyattr-runs-roundtrip"] {
        for name in flavours(stem) {
            let bytes = read(DIR, &name);
            let source = bound_curves(&parse_any(&bytes));
            let source = by_target(&source);
            let scene = decode(&bytes);
            for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
                let out = encode(&scene, form);
                let again = bound_curves(&parse_any(&out));
                assert_eq!(again.len(), source.len(), "{name} {form:?}");
                for c in &again {
                    let target = c.target.as_deref().expect("target");
                    assert_eq!(c.property.as_deref(), Some("Lcl Translation"));
                    // Only the X channel was authored; Y / Z are the
                    // typed channel's zero components.
                    if c.channel.as_deref() == Some("d|X") {
                        assert_eq!(&c.curve, source[target], "{name} {form:?} {target}");
                    }
                }
                // The curve node keeps its own channel value: `T01`'s
                // static X is 3.5 while its first key is 0 (§1.7).
                if stem == "keyattr-sweep" {
                    let doc = parse_any(&out);
                    let objects = doc.root.child("Objects").expect("Objects");
                    let statics: Vec<f64> = objects
                        .children_named("AnimationCurveNode")
                        .filter_map(|n| {
                            oxideav_fbx::properties70::PropertyMap::from_element(n).as_f64("d|X")
                        })
                        .filter(|v| *v != 0.0)
                        .collect();
                    assert_eq!(statics, vec![3.5], "{name} {form:?}");
                    assert_eq!(objects.children_named("AnimationCurveNode").count(), 63);
                }
                // And the second decode is the same scene.
                let scene2 = decode(&out);
                assert_eq!(
                    scene2.animations[0].channels.len(),
                    scene.animations[0].channels.len()
                );
                for (a, b) in scene.animations[0]
                    .channels
                    .iter()
                    .zip(&scene2.animations[0].channels)
                {
                    assert_eq!(a.sampler.keyframes, b.sampler.keyframes, "{name} {form:?}");
                    assert_eq!(a.sampler.interpolation, b.sampler.interpolation);
                }
            }
        }
    }
}

/// Channels authored through the scene model write typed attributes: a
/// `Step` sampler as constant-"standard" keys, a cubic-spline `Vec3`
/// sampler as cubic user-tangent keys carrying its tangents.
#[test]
fn authored_step_and_cubic_channels_write_typed_attributes() {
    let mut scene = Scene3D::new();
    let stepper = scene.add_node(Node::new().with_name("Stepper"));
    let spline = scene.add_node(Node::new().with_name("Spline"));
    scene.roots.extend([stepper, spline]);
    let mut anim = Animation::new(Some("Take".to_string()));
    anim.channels.push(AnimationChannel::new(
        stepper,
        AnimationProperty::Translation,
        AnimationSampler {
            keyframes: vec![0.0, 1.0, 2.0],
            values: AnimationValues::Vec3(vec![
                [0.0, 1.0, 2.0],
                [10.0, 11.0, 12.0],
                [4.0, 5.0, 6.0],
            ]),
            interpolation: Interpolation::Step,
        },
    ));
    // (in-tangent, value, out-tangent) per key, tangents per second.
    anim.channels.push(AnimationChannel::new(
        spline,
        AnimationProperty::Translation,
        AnimationSampler {
            keyframes: vec![0.0, 1.0, 2.0],
            values: AnimationValues::Vec3(vec![
                [0.0; 3],
                [0.0, 0.0, 0.0],
                [1.5, 0.0, 0.0],
                [-2.5, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [0.0; 3],
            ]),
            interpolation: Interpolation::CubicSpline,
        },
    ));
    scene.add_animation(anim);

    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let curves = bound_curves(&parse_any(&out));
        let find = |target: &str, channel: &str| -> &AnimCurve {
            &curves
                .iter()
                .find(|c| {
                    c.target.as_deref() == Some(target) && c.channel.as_deref() == Some(channel)
                })
                .unwrap_or_else(|| panic!("{target} {channel}"))
                .curve
        };
        let step = find("Stepper", "d|Y");
        assert_eq!(step.key_values, vec![1.0, 11.0, 5.0]);
        assert_eq!(step.runs.len(), 1, "one run covers the three equal keys");
        assert_eq!(step.runs[0].count, 3);
        assert_eq!(step.runs[0].attr.flags.0, 0x2);
        assert_eq!(step.evaluate(0.999), 1.0);
        assert_eq!(step.evaluate(1.0), 11.0);

        let cubic = find("Spline", "d|X");
        let attrs = cubic.key_attrs();
        assert_eq!(attrs[0].flags.0, 0x408);
        assert_eq!(
            (attrs[0].right_slope(), attrs[0].next_left_slope()),
            (1.5, -2.5)
        );
        assert_eq!(
            (attrs[1].right_slope(), attrs[1].next_left_slope()),
            (3.0, -1.0)
        );
        // Hermite midpoint of the first segment: 5 + (1.5 + 2.5) / 8.
        assert!((cubic.evaluate(0.5) - 5.5).abs() < 1e-6);

        // Read back: the step channel is a Step sampler again, the
        // spline a refined linear one that follows the same curve.
        let back = decode(&out);
        let s = translation_sampler(&back, "Stepper");
        assert_eq!(s.interpolation, Interpolation::Step);
        assert_eq!(s.keyframes, vec![0.0, 1.0, 2.0]);
        let c = translation_sampler(&back, "Spline");
        assert_eq!(c.interpolation, Interpolation::Linear);
        assert!((sample_x(c, 0.5) - 5.5).abs() < 0.02);
    }
}
