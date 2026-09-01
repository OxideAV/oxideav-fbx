//! Decode → encode → decode **fixed-point** harness over the whole
//! staged clean-room corpus at `docs/3d/fbx/fixtures/` (binary and
//! ASCII, round 455).
//!
//! Two oracles, both in-tree (no third-party FBX consumer is
//! installed on the round's machine, so the crate's own reader is the
//! black box):
//!
//! 1. **Typed-scene fingerprint** — every field of the decoded
//!    [`Scene3D`] (nodes / meshes / primitives / materials / textures
//!    / skeletons / skins / animations / cameras / lights / axis +
//!    unit / every `extras` key at every level) is flattened into a
//!    `path → value` map with floats rounded to 1e-4, so two decodes
//!    can be diffed feature-by-feature. Generation 1 is the fixture's
//!    own decode; generation 2 decodes our re-encode of generation 1;
//!    generation 3 decodes the re-encode of generation 2. The
//!    **fixed-point** law is `fp(gen2) == fp(gen3)` (the writer
//!    converges after one pass), and the **parity** law is
//!    `fp(gen1) ⊆ fp(gen2)` (nothing the reader surfaced is dropped or
//!    degraded by the writer).
//! 2. **Wire-record census** — the multiset of record paths in the
//!    fixture's own `FbxDocument` (`Objects/Geometry/LayerElementUV`,
//!    `Objects/Model/Properties70/P:Lcl Translation`, …) against the
//!    re-encoded document's, so a decode-side feature the writer
//!    silently drops shows up as a missing wire path even when the
//!    typed scene happens not to expose it.
//!
//! Every test skips cleanly when the docs corpus is not on the
//! machine (standalone-crate CI has no docs checkout).

use std::collections::BTreeMap;
use std::path::PathBuf;

use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};
use oxideav_fbx::{FbxDecoder, FbxEncoder, FbxOutputForm};
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder, Scene3D};

fn fixtures_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("OXIDEAV_FBX_DOCS_FIXTURES") {
        let p = PathBuf::from(dir);
        return p.is_dir().then_some(p);
    }
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/3d/fbx/fixtures");
    p.is_dir().then_some(p)
}

fn fixture(name: &str) -> Option<Vec<u8>> {
    let dir = fixtures_dir()?;
    match std::fs::read(dir.join(name)) {
        Ok(bytes) => Some(bytes),
        Err(_) => {
            eprintln!("skipping: {name} not present under {}", dir.display());
            None
        }
    }
}

const ALL_FIXTURES: &[&str] = &[
    "box-binary-v7400.fbx",
    "box-binary-v7500.fbx",
    "camera-attr-binary-v7400.fbx",
    "skin-anim-binary-v7400.fbx",
    "cubes-ascii-v7500.fbx",
    "cubes-pivots-ascii-v7500.fbx",
    "texture-video-ascii-v7500.fbx",
];

// ---------------------------------------------------------------------
// Typed-scene fingerprint
// ---------------------------------------------------------------------

/// Re-format every float literal inside a `Debug` rendering to four
/// decimals so `f32 → f64 → f32` and Euler ↔ quaternion churn does
/// not register as a diff. Integer literals are left verbatim.
fn round_floats(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        let starts_number = c.is_ascii_digit()
            || (c == '-'
                && i + 1 < bytes.len()
                && (bytes[i + 1] as char).is_ascii_digit()
                && !(i > 0 && (bytes[i - 1] as char).is_ascii_alphanumeric()));
        if !starts_number || (i > 0 && (bytes[i - 1] as char).is_ascii_alphanumeric()) {
            out.push(c);
            i += 1;
            continue;
        }
        let start = i;
        i += 1;
        while i < bytes.len() {
            let d = bytes[i] as char;
            let ok = d.is_ascii_digit()
                || d == '.'
                || ((d == 'e' || d == 'E')
                    && i + 1 < bytes.len()
                    && matches!(bytes[i + 1] as char, '0'..='9' | '-' | '+'))
                || ((d == '-' || d == '+') && matches!(bytes[i - 1] as char, 'e' | 'E'));
            if !ok {
                break;
            }
            i += 1;
        }
        let tok = &s[start..i];
        if tok.contains('.') || tok.contains('e') || tok.contains('E') {
            match tok.parse::<f64>() {
                Ok(v) => {
                    let r = (v * 1e4).round() / 1e4;
                    let r = if r == 0.0 { 0.0 } else { r };
                    out.push_str(&format!("{r:.4}"));
                }
                Err(_) => out.push_str(tok),
            }
        } else {
            out.push_str(tok);
        }
    }
    out
}

/// `q` and `−q` are the same rotation; pin the sign so an Euler
/// round trip that lands on the antipode does not register.
fn canon_quat(q: [f32; 4]) -> [f32; 4] {
    let first_nonzero = q.iter().rev().find(|c| **c != 0.0).copied().unwrap_or(1.0);
    if first_nonzero < 0.0 {
        [-q[0], -q[1], -q[2], -q[3]]
    } else {
        q
    }
}

fn canon_transform(t: oxideav_mesh3d::Transform) -> oxideav_mesh3d::Transform {
    match t {
        oxideav_mesh3d::Transform::Trs {
            translation,
            rotation,
            scale,
        } => oxideav_mesh3d::Transform::Trs {
            translation,
            rotation: canon_quat(rotation),
            scale,
        },
        other => other,
    }
}

fn canon_json(v: &serde_json::Value) -> String {
    round_floats(&v.to_string())
}

type Fingerprint = BTreeMap<String, String>;

fn put<T: std::fmt::Debug>(fp: &mut Fingerprint, key: String, v: T) {
    fp.insert(key, round_floats(&format!("{v:?}")));
}

fn put_extras(
    fp: &mut Fingerprint,
    prefix: &str,
    extras: &std::collections::HashMap<String, serde_json::Value>,
) {
    for (k, v) in extras {
        fp.insert(format!("{prefix}.extras[{k}]"), canon_json(v));
    }
}

/// Flatten a decoded scene into `path → rounded value`.
fn fingerprint(s: &Scene3D) -> Fingerprint {
    let mut fp = Fingerprint::new();
    put(&mut fp, "scene.up_axis".into(), s.up_axis);
    put(&mut fp, "scene.front_axis".into(), s.front_axis);
    put(&mut fp, "scene.unit".into(), s.unit);
    put(&mut fp, "scene.roots".into(), &s.roots);
    put(
        &mut fp,
        "scene.material_variants".into(),
        &s.material_variants,
    );
    put_extras(&mut fp, "scene", &s.extras);

    for (i, n) in s.nodes.iter().enumerate() {
        let p = format!("node[{i}]");
        put(&mut fp, format!("{p}.name"), &n.name);
        put(
            &mut fp,
            format!("{p}.transform"),
            canon_transform(n.transform),
        );
        put(&mut fp, format!("{p}.children"), &n.children);
        put(&mut fp, format!("{p}.mesh"), n.mesh);
        put(&mut fp, format!("{p}.camera"), n.camera);
        put(&mut fp, format!("{p}.light"), n.light);
        put(&mut fp, format!("{p}.skin"), n.skin);
        put(&mut fp, format!("{p}.weights"), &n.weights);
        put(&mut fp, format!("{p}.audio_emitter"), n.audio_emitter);
        put_extras(&mut fp, &p, &n.extras);
    }
    for (i, m) in s.meshes.iter().enumerate() {
        let p = format!("mesh[{i}]");
        put(&mut fp, format!("{p}.name"), &m.name);
        put(&mut fp, format!("{p}.weights"), &m.weights);
        put(&mut fp, format!("{p}.target_names"), &m.target_names);
        for (j, pr) in m.primitives.iter().enumerate() {
            let p = format!("{p}.prim[{j}]");
            put(&mut fp, format!("{p}.topology"), pr.topology);
            put(&mut fp, format!("{p}.positions"), &pr.positions);
            put(&mut fp, format!("{p}.normals"), &pr.normals);
            put(&mut fp, format!("{p}.tangents"), &pr.tangents);
            for (k, uv) in pr.uvs.iter().enumerate() {
                put(&mut fp, format!("{p}.uvs[{k}]"), uv);
            }
            for (k, c) in pr.colors.iter().enumerate() {
                put(&mut fp, format!("{p}.colors[{k}]"), c);
            }
            put(&mut fp, format!("{p}.joints"), &pr.joints);
            put(&mut fp, format!("{p}.weights"), &pr.weights);
            put(&mut fp, format!("{p}.indices"), &pr.indices);
            put(&mut fp, format!("{p}.material"), pr.material);
            put(
                &mut fp,
                format!("{p}.variant_mappings"),
                &pr.variant_mappings,
            );
            for (t, tg) in pr.targets.iter().enumerate() {
                let p = format!("{p}.target[{t}]");
                put(&mut fp, format!("{p}.position"), &tg.position);
                put(&mut fp, format!("{p}.normal"), &tg.normal);
                put(&mut fp, format!("{p}.tangent"), &tg.tangent);
                put(&mut fp, format!("{p}.inbetweens"), &tg.inbetweens);
            }
            put_extras(&mut fp, &p, &pr.extras);
        }
    }
    for (i, m) in s.materials.iter().enumerate() {
        let p = format!("material[{i}]");
        put(&mut fp, format!("{p}.name"), &m.name);
        put(&mut fp, format!("{p}.base_color"), m.base_color);
        put(
            &mut fp,
            format!("{p}.base_color_texture"),
            m.base_color_texture,
        );
        put(&mut fp, format!("{p}.metallic"), m.metallic);
        put(&mut fp, format!("{p}.roughness"), m.roughness);
        put(
            &mut fp,
            format!("{p}.metallic_roughness_texture"),
            m.metallic_roughness_texture,
        );
        put(&mut fp, format!("{p}.normal_texture"), m.normal_texture);
        put(&mut fp, format!("{p}.normal_scale"), m.normal_scale);
        put(
            &mut fp,
            format!("{p}.occlusion_texture"),
            m.occlusion_texture,
        );
        put(
            &mut fp,
            format!("{p}.occlusion_strength"),
            m.occlusion_strength,
        );
        put(&mut fp, format!("{p}.emissive_factor"), m.emissive_factor);
        put(&mut fp, format!("{p}.emissive_texture"), m.emissive_texture);
        put(&mut fp, format!("{p}.alpha_mode"), m.alpha_mode);
        put(&mut fp, format!("{p}.double_sided"), m.double_sided);
        put(&mut fp, format!("{p}.ext"), &m.ext);
        put_extras(&mut fp, &p, &m.extras);
    }
    for (i, t) in s.textures.iter().enumerate() {
        let p = format!("texture[{i}]");
        put(&mut fp, format!("{p}.name"), &t.name);
        let image = match &t.image {
            oxideav_mesh3d::ImageData::External { uri, mime } => {
                format!("External uri={uri:?} mime={mime:?}")
            }
            oxideav_mesh3d::ImageData::Embedded(f) => format!("Embedded planes={}", f.planes.len()),
            oxideav_mesh3d::ImageData::Source(src) => {
                format!("Source mime={:?} size={:?}", src.mime(), src.size_hint())
            }
        };
        fp.insert(format!("{p}.image"), image);
        put(&mut fp, format!("{p}.sampler"), t.sampler);
    }
    for (i, sk) in s.skeletons.iter().enumerate() {
        let p = format!("skeleton[{i}]");
        put(&mut fp, format!("{p}.name"), &sk.name);
        put(&mut fp, format!("{p}.joints"), &sk.joints);
        put(
            &mut fp,
            format!("{p}.inverse_bind_matrices"),
            &sk.inverse_bind_matrices,
        );
    }
    for (i, sk) in s.skins.iter().enumerate() {
        put(&mut fp, format!("skin[{i}]"), sk);
    }
    for (i, a) in s.animations.iter().enumerate() {
        let p = format!("animation[{i}]");
        put(&mut fp, format!("{p}.name"), &a.name);
        for (j, ch) in a.channels.iter().enumerate() {
            let p = format!("{p}.channel[{j}]");
            put(&mut fp, format!("{p}.target"), ch.target);
            put(
                &mut fp,
                format!("{p}.interpolation"),
                ch.sampler.interpolation,
            );
            put(&mut fp, format!("{p}.keyframes"), &ch.sampler.keyframes);
            match &ch.sampler.values {
                oxideav_mesh3d::AnimationValues::Quat(qs) => {
                    let qs: Vec<[f32; 4]> = qs.iter().map(|q| canon_quat(*q)).collect();
                    put(&mut fp, format!("{p}.values"), qs);
                }
                other => put(&mut fp, format!("{p}.values"), other),
            }
        }
    }
    for (i, c) in s.cameras.iter().enumerate() {
        put(&mut fp, format!("camera[{i}]"), c);
    }
    for (i, l) in s.lights.iter().enumerate() {
        put(&mut fp, format!("light[{i}]"), l);
    }
    fp
}

// ---------------------------------------------------------------------
// Wire-record census
// ---------------------------------------------------------------------

fn census_walk(n: &FbxNode, path: &str, out: &mut BTreeMap<String, usize>) {
    let mut here = if path.is_empty() {
        n.name.clone()
    } else {
        format!("{path}/{}", n.name)
    };
    if n.name == "P" {
        if let Some(FbxProperty::String(name)) = n.properties.first() {
            here = format!("{here}:{}", String::from_utf8_lossy(name));
        }
    }
    // Object headers: `Objects/Geometry` → append the subtype string
    // (third property) so `Geometry{Mesh}` and `Geometry{Shape}` are
    // distinct census rows.
    if path == "Objects" || path == "Objects/" {
        if let Some(FbxProperty::String(sub)) = n.properties.get(2) {
            here = format!("{here}{{{}}}", String::from_utf8_lossy(sub));
        }
    }
    *out.entry(here.clone()).or_insert(0) += 1;
    for c in &n.children {
        census_walk(c, &here, out);
    }
}

fn census(doc: &FbxDocument) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for c in &doc.root.children {
        census_walk(c, "", &mut out);
    }
    out
}

fn parse_any(bytes: &[u8]) -> FbxDocument {
    if oxideav_fbx::is_ascii_fbx(bytes) {
        oxideav_fbx::ascii::parse(bytes).expect("ascii parse")
    } else {
        oxideav_fbx::binary::parse(bytes).expect("binary parse")
    }
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("scene decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new()
        .form(form)
        .encode(scene)
        .expect("scene encode")
}

// ---------------------------------------------------------------------
// Diff reporting
// ---------------------------------------------------------------------

struct Diff {
    dropped: Vec<(String, String)>,
    changed: Vec<(String, String, String)>,
    added: Vec<(String, String)>,
}

/// Tolerant value equality: the non-numeric skeleton must match
/// exactly and every numeric token must agree to 2e-4 (the
/// fingerprint strings are already rounded to 1e-4, so this absorbs
/// a value sitting on a rounding boundary).
fn values_equal(a: &str, b: &str) -> bool {
    if a == b {
        return true;
    }
    fn tokens(s: &str) -> Vec<Result<f64, String>> {
        let mut out = Vec::new();
        let mut cur = String::new();
        let bytes = s.as_bytes();
        let mut i = 0;
        while i < bytes.len() {
            let c = bytes[i] as char;
            let is_num_start = c.is_ascii_digit()
                || (c == '-'
                    && i + 1 < bytes.len()
                    && (bytes[i + 1] as char).is_ascii_digit()
                    && !(i > 0 && (bytes[i - 1] as char).is_ascii_alphanumeric()));
            if is_num_start && !(i > 0 && (bytes[i - 1] as char).is_ascii_alphanumeric()) {
                if !cur.is_empty() {
                    out.push(Err(std::mem::take(&mut cur)));
                }
                let start = i;
                i += 1;
                while i < bytes.len()
                    && matches!(bytes[i] as char, '0'..='9' | '.' | 'e' | 'E' | '-' | '+')
                {
                    i += 1;
                }
                match s[start..i].parse::<f64>() {
                    Ok(v) => out.push(Ok(v)),
                    Err(_) => out.push(Err(s[start..i].to_string())),
                }
            } else {
                cur.push(c);
                i += 1;
            }
        }
        if !cur.is_empty() {
            out.push(Err(cur));
        }
        out
    }
    let ta = tokens(a);
    let tb = tokens(b);
    ta.len() == tb.len()
        && ta.iter().zip(tb.iter()).all(|(x, y)| match (x, y) {
            (Ok(x), Ok(y)) => (x - y).abs() <= 2e-4,
            (Err(x), Err(y)) => x == y,
            _ => false,
        })
}

fn diff(a: &Fingerprint, b: &Fingerprint) -> Diff {
    let mut d = Diff {
        dropped: Vec::new(),
        changed: Vec::new(),
        added: Vec::new(),
    };
    for (k, va) in a {
        match b.get(k) {
            None => d.dropped.push((k.clone(), va.clone())),
            Some(vb) if !values_equal(va, vb) => {
                d.changed.push((k.clone(), va.clone(), vb.clone()))
            }
            _ => {}
        }
    }
    for (k, vb) in b {
        if !a.contains_key(k) {
            d.added.push((k.clone(), vb.clone()));
        }
    }
    d
}

fn short(s: &str) -> String {
    if s.len() > 96 {
        format!("{}…({} chars)", &s[..96], s.len())
    } else {
        s.to_string()
    }
}

/// Records the census deliberately does not require the writer to
/// reproduce: producer-provenance / thumbnail / SDK-internal blocks
/// that carry no scene semantics.
fn census_ignored(path: &str) -> bool {
    const PREFIXES: &[&str] = &[
        "FBXHeaderExtension/SceneInfo/Thumbnail",
        "FBXHeaderExtension/OtherFlags",
        "FBXHeaderExtension/EncryptionType",
    ];
    PREFIXES.iter().any(|p| path.starts_with(p))
}

struct Outcome {
    /// `fp(gen1)` vs `fp(gen2)`.
    parity: Diff,
    /// `fp(gen2)` vs `fp(gen3)`.
    fixed: Diff,
    /// `(path, count in fixture, count in re-encode)` for every
    /// semantic wire path the re-encode carries fewer of.
    wire: Vec<(String, usize, usize)>,
}

fn run_fixture(name: &str, form: FbxOutputForm) -> Option<Result<Outcome, String>> {
    let bytes = fixture(name)?;
    let gen1 = decode(&bytes);
    let enc1 = match FbxEncoder::new().form(form).encode(&gen1) {
        Ok(b) => b,
        Err(e) => return Some(Err(format!("{e:?}"))),
    };
    let gen2 = decode(&enc1);
    let enc2 = encode(&gen2, form);
    let gen3 = decode(&enc2);

    let f1 = fingerprint(&gen1);
    let f2 = fingerprint(&gen2);
    let f3 = fingerprint(&gen3);
    let parity = diff(&f1, &f2);
    let fixed = diff(&f2, &f3);

    let c1: BTreeMap<String, usize> = census(&parse_any(&bytes))
        .into_iter()
        .filter(|(k, _)| !census_ignored(k))
        .collect();
    let c2 = census(&parse_any(&enc1));
    let wire = c1
        .iter()
        .filter_map(|(k, &ca)| {
            let cb = c2.get(k).copied().unwrap_or(0);
            (cb < ca).then(|| (k.clone(), ca, cb))
        })
        .collect();
    Some(Ok(Outcome {
        parity,
        fixed,
        wire,
    }))
}

// ---------------------------------------------------------------------
// Known gaps — the round-455 burn-down list
// ---------------------------------------------------------------------

/// Parity / census gaps the harness has enumerated and that are
/// still open. Every entry names the feature it stands for; the
/// laws below skip exactly these and nothing else, so closing a
/// feature means deleting its line here (and the law then guards
/// it forever).
fn known_gap(form: FbxOutputForm, kind: &str, key: &str) -> bool {
    // The ASCII writer has no form for `R` blobs (binary `FileId`,
    // embedded `Video.Content`), so every fixture carrying one
    // cannot yet be re-encoded as ASCII at all.
    if kind == "encode" && form == FbxOutputForm::Ascii {
        return true;
    }
    let parity: &[&str] = &[
        // material property coverage
        "material[*].extras[fbx:shading_model]",
        "material[*].roughness",
        // geometry: shared vertices / polygons / edges / smoothing
        "mesh[*].prim[*].extras[fbx:edges]",
        "mesh[*].prim[*].extras[fbx:edge_smoothing]",
        "mesh[*].prim[*].extras[fbx:shared_positions]",
        "mesh[*].prim[*].extras[fbx:material_mapping]",
        // animation: uninterpreted key-attribute passthrough
        "scene.extras[fbx:key_attrs]",
        // textures referenced by no material are never emitted
        "texture[*].image",
        "texture[*].name",
        "texture[*].sampler",
        "scene.extras[fbx:texture_records]",
    ];
    let census: &[&str] = &[
        "FBXHeaderExtension/SceneInfo",
        "GlobalSettings/Properties70/P:TimeMarker",
        "Documents/Document/Properties70/P:ActiveAnimStackName",
        "Definitions/",
        "Objects/",
        "Connections/C",
    ];
    let wild = |pat: &str, key: &str| -> bool {
        // `*` matches a run of digits.
        let mut k = key;
        let mut parts = pat.split('*').peekable();
        let first = parts.next().unwrap_or("");
        if !k.starts_with(first) {
            return false;
        }
        k = &k[first.len()..];
        for part in parts {
            let digits = k.len() - k.trim_start_matches(|c: char| c.is_ascii_digit()).len();
            if digits == 0 {
                return false;
            }
            k = &k[digits..];
            if !k.starts_with(part) {
                return false;
            }
            k = &k[part.len()..];
        }
        k.is_empty()
    };
    match kind {
        "parity" => parity.iter().any(|p| wild(p, key)),
        "census" => census.iter().any(|p| key.starts_with(p)),
        _ => false,
    }
}

fn report(label: &str, d: &Diff) -> String {
    let mut s = String::new();
    for (k, v) in &d.dropped {
        s.push_str(&format!("  [{label}] DROPPED {k} = {}\n", short(v)));
    }
    for (k, a, b) in &d.changed {
        // Show the window around the first differing byte, so a
        // long array diff points at the offending element.
        let first = a
            .bytes()
            .zip(b.bytes())
            .position(|(x, y)| x != y)
            .unwrap_or(a.len().min(b.len()));
        let window = |t: &str| -> String {
            let lo = first.saturating_sub(48);
            let lo = (0..=lo).rev().find(|i| t.is_char_boundary(*i)).unwrap_or(0);
            let hi = (first + 48).min(t.len());
            let hi = (hi..=t.len())
                .find(|i| t.is_char_boundary(*i))
                .unwrap_or(t.len());
            format!("…{}…(@{first}, {} chars)", &t[lo..hi], t.len())
        };
        s.push_str(&format!(
            "  [{label}] CHANGED {k}\n      was {}\n      now {}\n",
            if a.len() > 96 { window(a) } else { a.clone() },
            if b.len() > 96 { window(b) } else { b.clone() },
        ));
    }
    for (k, v) in &d.added {
        s.push_str(&format!("  [{label}] ADDED   {k} = {}\n", short(v)));
    }
    s
}

/// Print the full report for every fixture × form. Never fails —
/// it is the round's enumeration tool; the laws below assert.
#[test]
fn report_all_fixtures() {
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        for name in ALL_FIXTURES {
            let Some(out) = run_fixture(name, form) else {
                continue;
            };
            println!("=== {name} via {form:?}");
            match out {
                Err(e) => println!("  ENCODE FAILED: {e}"),
                Ok(o) => {
                    print!("{}", report("parity", &o.parity));
                    print!("{}", report("fixed", &o.fixed));
                    for (k, a, b) in &o.wire {
                        println!("  [wire] WIRE-LOST {k}: {a} → {b}");
                    }
                }
            }
        }
    }
}

/// Law 1 — fixed point: after one writer pass the scene converges;
/// a second decode → encode → decode changes nothing.
#[test]
fn every_fixture_reaches_a_fixed_point_after_one_pass() {
    let mut failures = String::new();
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        for name in ALL_FIXTURES {
            let Some(out) = run_fixture(name, form) else {
                continue;
            };
            let o = match out {
                Ok(o) => o,
                Err(_) if known_gap(form, "encode", name) => continue,
                Err(e) => {
                    failures.push_str(&format!("=== {name} via {form:?}: encode failed: {e}\n"));
                    continue;
                }
            };
            let d = &o.fixed;
            if !d.dropped.is_empty() || !d.changed.is_empty() || !d.added.is_empty() {
                failures.push_str(&format!("=== {name} via {form:?}\n{}", report("fixed", d)));
            }
        }
    }
    assert!(failures.is_empty(), "fixed-point violations:\n{failures}");
}

/// Law 2 — parity: nothing the reader surfaced from the fixture is
/// dropped or degraded by one writer pass, in either output form
/// (modulo the enumerated open gaps in [`known_gap`]).
#[test]
fn every_fixture_round_trips_without_dropping_a_decoded_feature() {
    let mut failures = String::new();
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        for name in ALL_FIXTURES {
            let Some(out) = run_fixture(name, form) else {
                continue;
            };
            let o = match out {
                Ok(o) => o,
                Err(_) if known_gap(form, "encode", name) => continue,
                Err(e) => {
                    failures.push_str(&format!("=== {name} via {form:?}: encode failed: {e}\n"));
                    continue;
                }
            };
            let mut d = Diff {
                dropped: Vec::new(),
                changed: Vec::new(),
                added: Vec::new(),
            };
            for (k, v) in &o.parity.dropped {
                if !known_gap(form, "parity", k) {
                    d.dropped.push((k.clone(), v.clone()));
                }
            }
            for (k, a, b) in &o.parity.changed {
                if !known_gap(form, "parity", k) {
                    d.changed.push((k.clone(), a.clone(), b.clone()));
                }
            }
            if !d.dropped.is_empty() || !d.changed.is_empty() {
                failures.push_str(&format!(
                    "=== {name} via {form:?}\n{}",
                    report("parity", &d)
                ));
            }
        }
    }
    assert!(failures.is_empty(), "parity violations:\n{failures}");
}

/// Law 3 — wire census: every semantic record path present in the
/// fixture's own document is present (at least as often) in our
/// re-encode of its decode (modulo [`known_gap`]).
#[test]
fn every_fixture_keeps_its_wire_record_census() {
    let mut failures = String::new();
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        for name in ALL_FIXTURES {
            let Some(out) = run_fixture(name, form) else {
                continue;
            };
            let o = match out {
                Ok(o) => o,
                Err(_) if known_gap(form, "encode", name) => continue,
                Err(e) => {
                    failures.push_str(&format!("=== {name} via {form:?}: encode failed: {e}\n"));
                    continue;
                }
            };
            let mut lines = String::new();
            for (k, a, b) in &o.wire {
                if !known_gap(form, "census", k) {
                    lines.push_str(&format!("  WIRE-LOST {k}: {a} → {b}\n"));
                }
            }
            if !lines.is_empty() {
                failures.push_str(&format!("=== {name} via {form:?}\n{lines}"));
            }
        }
    }
    assert!(failures.is_empty(), "wire-census losses:\n{failures}");
}
