//! The vendor-exporter NURBS / line / trim sweep
//! (`docs/3d/fbx/fbx-sdk-export-sweeps-2026-11.md` §3,
//! `fixtures/fbxsdk-nurbs/notes.md`): every geometry of every container
//! flavour decodes into its typed record, evaluates, tessellates, and
//! writes back record for record.

mod common;

use common::{display, exact, flavours, is_ascii_name, loose, parse_any, read, render, subtype};
use oxideav_fbx::binary::{FbxDocument, FbxNode, FbxProperty};
use oxideav_fbx::geometry_tree::{
    GEOMETRY_TREE_KEY, line_of, nurbs_surface_of, tree_subtype, trim_surface_of,
};
use oxideav_fbx::nurbs_geometry::TrimRegion;
use oxideav_fbx::{
    FbxBoundary, FbxDecoder, FbxEncoder, FbxLine, FbxNurbsCurve, FbxNurbsSurface, FbxOutputForm,
    FbxTrimSurface, NurbsForm, NurbsTessellation, SceneDecodeOptions,
};
use oxideav_mesh3d::{Indices, Mesh3DDecoder, Mesh3DEncoder, Node, Scene3D, Topology};

const DIR: &str = "fbxsdk-nurbs";

fn geometries(doc: &FbxDocument) -> Vec<&FbxNode> {
    doc.root
        .child("Objects")
        .expect("Objects")
        .children_named("Geometry")
        .collect()
}

fn decode(bytes: &[u8]) -> Scene3D {
    FbxDecoder::new().decode(bytes).expect("decode")
}

fn encode(scene: &Scene3D, form: FbxOutputForm) -> Vec<u8> {
    FbxEncoder::new().form(form).encode(scene).expect("encode")
}

fn ramp(len: usize, first: f64) -> Vec<f64> {
    (0..len).map(|j| first + j as f64).collect()
}

/// §3.2 / notes.md §1: the thirteen curves.
#[test]
fn curves_decode_to_the_authored_records() {
    // (geometry, form, order, dimension, rational, points, knots)
    let table: &[(&str, NurbsForm, u32, u32, bool, usize, usize)] = &[
        ("c01", NurbsForm::Open, 4, 3, false, 6, 10),
        ("c02", NurbsForm::Closed, 4, 3, false, 6, 11),
        ("c03", NurbsForm::Periodic, 4, 3, false, 6, 13),
        ("c04", NurbsForm::Open, 2, 3, false, 3, 5),
        ("c05", NurbsForm::Open, 3, 3, true, 5, 8),
        ("c06", NurbsForm::Periodic, 3, 3, true, 5, 10),
        ("c07", NurbsForm::Closed, 3, 3, false, 5, 9),
        ("c08", NurbsForm::Open, 4, 2, false, 6, 10),
        ("c09", NurbsForm::Open, 4, 3, false, 6, 10),
        ("c10", NurbsForm::Open, 6, 3, false, 8, 14),
        ("c11", NurbsForm::Closed, 2, 3, false, 4, 7),
        ("c12", NurbsForm::Periodic, 2, 3, false, 4, 7),
        ("c13", NurbsForm::Open, 2, 2, false, 3, 5),
    ];
    for name in flavours("nurbs-curves") {
        let doc = parse_any(&read(DIR, &name));
        let geoms = geometries(&doc);
        assert_eq!(geoms.len(), 13, "{name}");
        for (g, row) in geoms.iter().zip(table) {
            let (gname, form, order, dim, rational, n, knots) = *row;
            assert_eq!(display(g), gname, "{name}");
            assert_eq!(subtype(g), "NurbsCurve");
            let c = FbxNurbsCurve::from_element(g).expect("curve");
            assert_eq!(
                (c.form, c.order, c.dimension, c.rational),
                (form, order, dim, rational)
            );
            assert_eq!(c.points.len(), n, "{name} {gname}");
            assert_eq!(c.knots, ramp(knots, 0.5), "{name} {gname}");
            assert_eq!(c.expected_knot_count(), knots, "{name} {gname}");
            for (i, p) in c.points.iter().enumerate() {
                let k = (i + 1) as f64;
                let z = match gname {
                    "c08" => 0.0,
                    "c13" => 5.5 * k,
                    _ => 100.0 * k,
                };
                let w = if rational { 1.0 + 0.25 * i as f64 } else { 1.0 };
                assert_eq!(*p, [k, 10.0 * k, z, w], "{name} {gname} point {i}");
            }
            // Record for record.
            let rebuilt = c.to_element(0, gname);
            let f = if is_ascii_name(&name) { loose } else { exact };
            let body = |n: &FbxNode| n.children.iter().map(|c| render(f, c)).collect::<Vec<_>>();
            assert_eq!(body(g), body(&rebuilt), "{name} {gname}");
            // Every curve evaluates: the engine accepts the unclamped
            // ramps as ordinary B-spline knots.
            let curve = c
                .to_curve()
                .unwrap_or_else(|e| panic!("{name} {gname}: {e}"));
            let (lo, hi) = curve.domain();
            let k = order as usize;
            match form {
                NurbsForm::Periodic => assert_eq!((lo, hi), (k as f64 - 0.5, (n + k) as f64 - 0.5)),
                NurbsForm::Closed => assert_eq!((lo, hi), (k as f64 - 0.5, (n + 1) as f64 + 0.5)),
                NurbsForm::Open => assert_eq!((lo, hi), (k as f64 - 0.5, n as f64 + 0.5)),
            }
            let mid = curve.evaluate(0.5 * (lo + hi));
            assert!(mid.iter().all(|v| v.is_finite()));
            // A 2-D curve lies in z = 0 whatever third value is stored.
            if dim == 2 {
                assert_eq!(mid[2], 0.0, "{name} {gname}");
            }
        }
    }
}

/// §3.2 "what the extra knots mean" and "weight convention": the three
/// small curves whose vendor-tessellator samples the document quotes.
#[test]
fn curve_evaluation_matches_the_quoted_samples() {
    let square = vec![
        [0.0, 0.0, 0.0, 1.0],
        [4.0, 0.0, 0.0, 1.0],
        [4.0, 4.0, 0.0, 1.0],
        [0.0, 4.0, 0.0, 1.0],
    ];
    // Closed: the open B-spline over n + 1 points, first one repeated.
    let closed = FbxNurbsCurve::new(
        3,
        NurbsForm::Closed,
        square.clone(),
        vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0],
    )
    .to_curve()
    .expect("closed");
    assert_eq!(closed.domain(), (0.0, 3.0));
    let start = closed.evaluate(0.0);
    let end = closed.evaluate(3.0);
    assert_eq!((start[0], start[1]), (0.0, 0.0));
    assert!((end[0]).abs() < 1e-12 && (end[1]).abs() < 1e-12);
    // Periodic: the open B-spline over n + k - 1 points over knot
    // indices k - 1 … n + k - 1; it starts and ends on one point.
    let periodic = FbxNurbsCurve::new(
        3,
        NurbsForm::Periodic,
        square,
        vec![-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )
    .to_curve()
    .expect("periodic");
    assert_eq!(periodic.domain(), (0.0, 4.0));
    let a = periodic.evaluate(0.0);
    let b = periodic.evaluate(4.0 - 1e-12);
    assert!((a[0] - 2.0).abs() < 1e-9 && a[1].abs() < 1e-9, "{a:?}");
    assert!((b[0] - 2.0).abs() < 1e-6 && b[1].abs() < 1e-6, "{b:?}");
    // Weights: (0,0) (1,1) (2,0) with fourth components 1, 2, 1, order
    // 3, knots 0,0,0,1,1,1 → (0.6667, 0.3333) at the middle and
    // (0.3636, 0.2727) at one quarter: Σ N·(x, y) / Σ N·w.
    let rational = FbxNurbsCurve::new(
        3,
        NurbsForm::Open,
        vec![
            [0.0, 0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 2.0],
            [2.0, 0.0, 0.0, 1.0],
        ],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    );
    assert!(rational.rational);
    let curve = rational.to_curve().expect("rational");
    let mid = curve.evaluate(0.5);
    assert!((mid[0] - 2.0 / 3.0).abs() < 1e-12 && (mid[1] - 1.0 / 3.0).abs() < 1e-12);
    let quarter = curve.evaluate(0.25);
    assert!((quarter[0] - 0.3636).abs() < 1e-4 && (quarter[1] - 0.2727).abs() < 1e-4);
}

/// §3.3 / notes.md §2: the sixteen surfaces.
#[test]
fn surfaces_decode_to_the_authored_records() {
    use NurbsForm::{Closed, Open, Periodic};
    // (forms, orders, dimensions, knots, mode, step, flip)
    type Row = (
        [NurbsForm; 2],
        [u32; 2],
        [u32; 2],
        [usize; 2],
        i32,
        [u32; 2],
        bool,
    );
    let small = |mode: i32, step: [u32; 2], flip: bool| -> Row {
        ([Open, Open], [2, 2], [2, 3], [4, 5], mode, step, flip)
    };
    let table: Vec<Row> = vec![
        ([Open, Open], [4, 3], [5, 4], [9, 7], 4, [0, 0], false),
        ([Periodic, Open], [4, 3], [5, 4], [12, 7], 4, [0, 0], false),
        ([Closed, Open], [4, 3], [5, 4], [10, 7], 4, [0, 0], false),
        ([Open, Periodic], [4, 3], [5, 4], [9, 9], 4, [0, 0], false),
        ([Closed, Closed], [4, 3], [5, 4], [10, 8], 4, [0, 0], false),
        (
            [Periodic, Periodic],
            [4, 3],
            [5, 4],
            [12, 9],
            4,
            [0, 0],
            false,
        ),
        small(4, [0, 0], false),
        small(4, [3, 5], false),
        // s09 "flip UV": transposed at write time, a plain 3 x 2 surface.
        ([Open, Open], [2, 2], [3, 2], [5, 4], 4, [0, 0], false),
        small(4, [0, 0], false),
        small(4, [0, 0], true),
        small(0, [0, 0], false),
        small(1, [0, 0], false),
        small(2, [0, 0], false),
        small(3, [0, 0], false),
        small(4, [0, 0], false),
    ];
    for name in flavours("nurbs-surfaces") {
        let doc = parse_any(&read(DIR, &name));
        let geoms = geometries(&doc);
        assert_eq!(geoms.len(), 16, "{name}");
        for (i, (g, row)) in geoms.iter().zip(&table).enumerate() {
            let (form, order, dims, knots, mode, step, flip) = *row;
            assert_eq!(subtype(g), "NurbsSurface");
            let s = FbxNurbsSurface::from_element(g).expect("surface");
            assert_eq!(
                (s.form, s.order, s.dimensions),
                (form, order, dims),
                "{name} #{i}"
            );
            assert_eq!(
                (s.surface_mode, s.step, s.flip_normals),
                (mode, step, flip),
                "{name} #{i}"
            );
            assert_eq!(s.expected_knot_counts(), knots, "{name} #{i}");
            // U knots j + 0.5, V knots 100.25 + j — exchanged on s09.
            let (ku, kv) = if i == 8 { (100.25, 0.5) } else { (0.5, 100.25) };
            assert_eq!(s.knots_u, ramp(knots[0], ku), "{name} #{i}");
            assert_eq!(s.knots_v, ramp(knots[1], kv), "{name} #{i}");
            // Flat point i = (i, 0, 0); s09 is the transposed order.
            let xs: Vec<f64> = s.points.iter().map(|p| p[0]).collect();
            if i == 8 {
                assert_eq!(xs, vec![0.0, 2.0, 4.0, 1.0, 3.0, 5.0]);
            } else {
                assert_eq!(xs, ramp(xs.len(), 0.0), "{name} #{i}");
            }
            if i == 6 {
                let ws: Vec<f64> = s.points.iter().map(|p| p[3]).collect();
                for (k, w) in ws.iter().enumerate() {
                    assert!((w - (1.0 + 0.01 * k as f64)).abs() < 1e-12);
                }
            }
            let rebuilt = s.to_element(0, "");
            let f = if is_ascii_name(&name) { loose } else { exact };
            let body = |n: &FbxNode| n.children.iter().map(|c| render(f, c)).collect::<Vec<_>>();
            assert_eq!(body(g), body(&rebuilt), "{name} #{i}");
            // Evaluates and tessellates.
            let surface = s
                .to_surface()
                .unwrap_or_else(|e| panic!("{name} #{i}: {e}"));
            let (ulo, uhi) = surface.domain_u();
            let (vlo, vhi) = surface.domain_v();
            let p = surface.evaluate(0.5 * (ulo + uhi), 0.5 * (vlo + vhi));
            assert!(p.iter().all(|v| v.is_finite()), "{name} #{i}");
            let prim = s
                .tessellate(&NurbsTessellation::default())
                .expect("tessellate");
            assert_eq!(prim.topology, Topology::Triangles);
            assert!(!prim.positions.is_empty());
        }
    }
}

/// §3.3 "control-point order: U varies fastest", confirmed on the
/// 2 × 3 order-2 surface whose flat point i is (i, i², 0): the row at
/// u = 0 runs through flat points 0, 2, 4 along V, and the next row
/// starts at the mid-point of flat points 0 and 1.
#[test]
fn surface_control_points_are_u_fastest() {
    let points: Vec<[f64; 4]> = (0..6)
        .map(|i| [f64::from(i), f64::from(i * i), 0.0, 1.0])
        .collect();
    let s = FbxNurbsSurface::new(
        [2, 2],
        [2, 3],
        [NurbsForm::Open, NurbsForm::Open],
        points,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 2.0, 2.0],
    );
    let surface = s.to_surface().expect("surface");
    let at = |u: f64, v: f64| {
        let p = surface.evaluate(u, v);
        (p[0], p[1])
    };
    assert_eq!(at(0.0, 0.0), (0.0, 0.0));
    assert_eq!(at(0.0, 1.0), (2.0, 4.0));
    assert_eq!(at(0.0, 2.0), (4.0, 16.0));
    assert_eq!(at(0.5, 0.0), (0.5, 0.5));
    // Steps drive the per-span resolution: U 1 span x 3, V 2 spans x 5.
    let prim = s
        .clone()
        .with_step([3, 5])
        .tessellate(&NurbsTessellation::default())
        .expect("tessellate");
    assert_eq!(prim.positions.len(), (3 + 1) * (10 + 1));
    // FlipNormals reverses winding and normals.
    let plain = s.tessellate(&NurbsTessellation::default()).expect("plain");
    let flipped = s
        .with_flip_normals(true)
        .tessellate(&NurbsTessellation::default())
        .expect("flipped");
    let (Some(Indices::U32(a)), Some(Indices::U32(b))) = (&plain.indices, &flipped.indices) else {
        panic!("indices");
    };
    assert_eq!((a[0], a[1], a[2]), (b[0], b[2], b[1]));
    let (na, nb) = (plain.normals.unwrap(), flipped.normals.unwrap());
    assert_eq!(na[0], [-nb[0][0], -nb[0][1], -nb[0][2]]);
}

/// §3.4 / notes.md §3: the three line sets.
#[test]
fn lines_decode_to_the_authored_polylines() {
    for name in flavours("nurbs-line") {
        let doc = parse_any(&read(DIR, &name));
        let geoms = geometries(&doc);
        assert_eq!(geoms.len(), 3, "{name}");
        let lines: Vec<FbxLine> = geoms
            .iter()
            .map(|g| {
                assert_eq!(subtype(g), "Line");
                let l = FbxLine::from_element(g).expect("line");
                let f = if is_ascii_name(&name) { loose } else { exact };
                let body =
                    |n: &FbxNode| n.children.iter().map(|c| render(f, c)).collect::<Vec<_>>();
                assert_eq!(body(g), body(&l.to_element(0, "")), "{name}");
                l
            })
            .collect();
        assert_eq!(lines[0].points_index, vec![0, 1, -3, 4, 3, -1]);
        assert_eq!(lines[0].polylines(), vec![vec![0, 1, 2], vec![4, 3, 0]]);
        assert_eq!(lines[0].points.len(), 5);
        assert_eq!(lines[0].points[4], [5.0, 50.0, 500.0]);
        assert!(!lines[0].has_unmarked_tail());
        assert_eq!(lines[1].points_index, vec![2, 1, -1]);
        assert_eq!(lines[1].polylines(), vec![vec![2, 1, 0]]);
        // line03: no end mark at all.
        assert_eq!(lines[2].points_index, vec![0, 1]);
        assert!(lines[2].has_unmarked_tail());
        assert_eq!(lines[2].polylines(), vec![vec![0, 1]]);
        // The constructor writes the same end marks.
        let rebuilt =
            FbxLine::from_polylines(lines[0].points.clone(), &[vec![0, 1, 2], vec![4, 3, 0]]);
        assert_eq!(rebuilt, lines[0]);
    }
}

/// §3.5 / notes.md §4: the trimmed surface, its regions and loops.
#[test]
fn trim_surface_decodes_with_regions_and_loops() {
    for name in flavours("nurbs-trim") {
        let scene = decode(&read(DIR, &name));
        // One Model; the untrimmed surface, the three boundaries and
        // the eleven edge curves belong to the trimmed surface.
        assert_eq!(scene.nodes.len(), 1, "{name}");
        assert_eq!(scene.meshes.len(), 1, "{name}");
        let node = &scene.nodes[0];
        assert_eq!(
            node.name.as_deref(),
            Some("TR01_two_regions_outer4_hole4_outer3")
        );
        let prim = &scene.meshes[node.mesh.expect("mesh").0 as usize].primitives[0];
        assert_eq!(tree_subtype(prim), Some("TrimNurbsSurface"));
        let trim = trim_surface_of(prim).expect("trim");
        assert!(trim.flip_normals);
        assert_eq!(trim.surface.dimensions, [2, 2]);
        assert_eq!(trim.surface.order, [2, 2]);
        assert_eq!(trim.surface.knots_u, ramp(4, 0.5));
        assert_eq!(trim.surface.knots_v, ramp(4, 100.25));
        let shape: Vec<(bool, usize)> = trim
            .boundaries
            .iter()
            .map(|b| (b.outer, b.edges.len()))
            .collect();
        assert_eq!(shape, vec![(true, 4), (false, 4), (true, 3)]);
        assert_eq!(
            trim.regions(),
            vec![
                TrimRegion {
                    outer: 0,
                    holes: vec![1]
                },
                TrimRegion {
                    outer: 2,
                    holes: vec![]
                },
            ]
        );
        for b in &trim.boundaries {
            for e in &b.edges {
                assert_eq!((e.dimension, e.order, e.points.len()), (2, 2, 2));
                assert_eq!(e.knots, vec![0.0, 0.0, 1.0, 1.0]);
            }
        }
        let how = NurbsTessellation::default().with_segments_per_span(1);
        let loops: Vec<Vec<[f64; 2]>> = trim
            .boundaries
            .iter()
            .map(|b| b.loop_points(&how))
            .collect();
        let near = |a: &[[f64; 2]], b: &[[f64; 2]]| {
            a.len() == b.len()
                && a.iter()
                    .zip(b)
                    .all(|(p, q)| (p[0] - q[0]).abs() < 1e-6 && (p[1] - q[1]).abs() < 1e-6)
        };
        assert!(
            near(&loops[0], &[[0.1, 0.1], [0.9, 0.1], [0.9, 0.9], [0.1, 0.9]]),
            "{:?}",
            loops[0]
        );
        assert!(
            near(&loops[1], &[[0.4, 0.4], [0.4, 0.6], [0.6, 0.6], [0.6, 0.4]]),
            "{:?}",
            loops[1]
        );
        assert!(
            near(&loops[2], &[[0.45, 0.45], [0.55, 0.45], [0.5, 0.55]]),
            "{:?}",
            loops[2]
        );
        // Region membership on the loops themselves: inside the outer
        // square, outside its hole — except where the second region
        // (the triangle inside the hole) covers it again.
        let regions = trim.regions();
        let inside = |u: f64, v: f64| FbxTrimSurface::contains(&loops, &regions, u, v);
        assert!(inside(0.2, 0.2));
        assert!(!inside(0.05, 0.5));
        assert!(!inside(0.42, 0.58));
        assert!(inside(0.5, 0.5));
    }
}

/// §3.6 / notes.md §5: the older families ride along verbatim.
#[test]
fn legacy_families_are_carried_verbatim() {
    for name in flavours("nurbs-legacy") {
        let scene = decode(&read(DIR, &name));
        assert_eq!(scene.meshes.len(), 2, "{name}");
        let kinds: Vec<&str> = scene
            .meshes
            .iter()
            .map(|m| tree_subtype(&m.primitives[0]).expect("tree"))
            .collect();
        assert_eq!(kinds, vec!["Nurb", "Patch"], "{name}");
        let body_names = |i: usize| -> Vec<String> {
            scene.meshes[i].primitives[0].extras[GEOMETRY_TREE_KEY]["body"]
                .as_array()
                .unwrap()
                .iter()
                .map(|l| l["name"].as_str().unwrap().to_owned())
                .collect()
        };
        assert_eq!(
            body_names(0),
            [
                "NurbVersion",
                "SurfaceDisplay",
                "NurbOrder",
                "Dimensions",
                "Step",
                "Form",
                "Points",
                "MultiplicityU",
                "MultiplicityV",
                "KnotVectorU",
                "KnotVectorV",
                "GeometryVersion"
            ]
        );
        assert_eq!(
            body_names(1),
            [
                "PatchVersion",
                "SurfaceDisplay",
                "PatchType",
                "Dimensions",
                "Step",
                "Closed",
                "UCapped",
                "VCapped",
                "Points",
                "GeometryVersion"
            ]
        );
    }
}

/// Value-level rendition of every `Geometry` object keyed by name, and
/// the `Geometry → …` connections as name pairs in file order.
/// `(name, sub-type, body leaves)` per geometry and `(child, parent)`
/// per connection.
type Census = (Vec<(String, String, Vec<String>)>, Vec<(String, String)>);

fn geometry_census(doc: &FbxDocument) -> Census {
    let objects = doc.root.child("Objects").expect("Objects");
    let mut names = std::collections::HashMap::new();
    for o in &objects.children {
        if let Some(id) = o.properties.first().and_then(FbxProperty::as_i64) {
            names.insert(id, format!("{}:{}", o.name, display(o)));
        }
    }
    let mut geoms: Vec<(String, String, Vec<String>)> = objects
        .children_named("Geometry")
        .map(|g| {
            (
                display(g),
                subtype(g),
                g.children.iter().map(|c| render(loose, c)).collect(),
            )
        })
        .collect();
    geoms.sort();
    let conns = doc
        .root
        .child("Connections")
        .expect("Connections")
        .children_named("C")
        .filter_map(|c| {
            let child = c.properties.get(1)?.as_i64()?;
            let parent = c.properties.get(2)?.as_i64()?;
            let child = names.get(&child)?;
            child.starts_with("Geometry:").then(|| {
                (
                    child.clone(),
                    names.get(&parent).cloned().unwrap_or_else(|| "root".into()),
                )
            })
        })
        .collect();
    (geoms, conns)
}

/// Scene → file → document: every geometry object comes back with the
/// same records (the outer boundaries' doubled `Properties70`
/// included) and the same `Geometry → …` connections in the same
/// order, in both output forms, from every container flavour of every
/// scene of the sweep.
#[test]
fn scene_re_encode_keeps_every_geometry_object() {
    for stem in [
        "nurbs-curves",
        "nurbs-surfaces",
        "nurbs-line",
        "nurbs-trim",
        "nurbs-legacy",
    ] {
        for name in flavours(stem) {
            let bytes = read(DIR, &name);
            let source = parse_any(&bytes);
            let (src_geoms, mut src_conns) = geometry_census(&source);
            src_conns.sort();
            let scene = decode(&bytes);
            for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
                let out = encode(&scene, form);
                let again = parse_any(&out);
                let (geoms, mut conns) = geometry_census(&again);
                assert_eq!(geoms.len(), src_geoms.len(), "{name} {form:?}");
                for (a, b) in src_geoms.iter().zip(&geoms) {
                    assert_eq!(a, b, "{name} {form:?}");
                }
                // The boundary → trim order defines the regions.
                let order = |c: &[(String, String)]| -> Vec<String> {
                    c.iter()
                        .filter(|(_, p)| p == "Geometry:trim01")
                        .map(|(c, _)| c.clone())
                        .collect()
                };
                assert_eq!(
                    order(&conns),
                    order(&geometry_census(&source).1),
                    "{name} {form:?}"
                );
                conns.sort();
                assert_eq!(conns, src_conns, "{name} {form:?}");
                // Model sub-types repeat the geometry's.
                let models = |d: &FbxDocument| -> Vec<(String, String)> {
                    d.root
                        .child("Objects")
                        .unwrap()
                        .children_named("Model")
                        .map(|m| (display(m), subtype(m)))
                        .collect()
                };
                assert_eq!(models(&again), models(&source), "{name} {form:?}");
                // And the second decode is the same scene.
                let scene2 = decode(&out);
                assert_eq!(scene2.meshes.len(), scene.meshes.len(), "{name} {form:?}");
                for (a, b) in scene.meshes.iter().zip(&scene2.meshes) {
                    assert_eq!(a.primitives.len(), b.primitives.len());
                    for (p, q) in a.primitives.iter().zip(&b.primitives) {
                        assert_eq!(p.positions, q.positions, "{name} {form:?}");
                        assert_eq!(p.topology, q.topology);
                    }
                }
            }
        }
    }
}

/// The sweep's surfaces and lines in the scene model.
#[test]
fn surfaces_and_lines_become_sampled_meshes() {
    for name in flavours("nurbs-surfaces") {
        let scene = decode(&read(DIR, &name));
        assert_eq!((scene.nodes.len(), scene.meshes.len()), (16, 16), "{name}");
        for node in &scene.nodes {
            let prim = &scene.meshes[node.mesh.expect("mesh").0 as usize].primitives[0];
            assert_eq!(prim.topology, Topology::Triangles);
            assert!(nurbs_surface_of(prim).is_some());
            assert!(!prim.positions.is_empty(), "{name} {:?}", node.name);
            assert_eq!(node.extras["fbx:model_subtype"], "NurbsSurface");
        }
    }
    for name in flavours("nurbs-line") {
        let scene = decode(&read(DIR, &name));
        assert_eq!((scene.nodes.len(), scene.meshes.len()), (3, 3), "{name}");
        let mesh = &scene.meshes[scene.nodes[0].mesh.unwrap().0 as usize];
        assert_eq!(mesh.primitives.len(), 2);
        assert_eq!(
            mesh.primitives[0].positions,
            vec![[1.0, 10.0, 100.0], [2.0, 20.0, 200.0], [3.0, 30.0, 300.0]]
        );
        assert_eq!(
            mesh.primitives[1].positions,
            vec![[5.0, 50.0, 500.0], [4.0, 40.0, 400.0], [1.0, 10.0, 100.0]]
        );
        assert!(line_of(&mesh.primitives[0]).is_some());
    }
    // The curve scene: thirteen sampled lines; a chord tolerance
    // decides the sampling instead of the fixed per-span count.
    let bytes = read(DIR, "nurbs-curves-binary-v7400.fbx");
    let fixed = decode(&bytes);
    let tol = 1.0e-3;
    let within = FbxDecoder::new()
        .with_scene_options(SceneDecodeOptions::new().with_nurbs_tolerance(tol))
        .decode(&bytes)
        .expect("decode");
    assert_eq!(fixed.meshes.len(), 13);
    let prim = |s: &'_ Scene3D, i: usize| s.meshes[i].primitives[0].clone();
    // c10 (order 6, 8 collinear points): a straight line needs only
    // its span ends, where the fixed count spends 16 segments a span.
    assert_eq!(prim(&fixed, 9).positions.len(), 3 * 16 + 1);
    assert_eq!(prim(&within, 9).positions.len(), 4);
    // c05 (rational, so curved): every chord stays within the
    // tolerance of the curve.
    let sampled = prim(&within, 4);
    let curve = FbxNurbsCurve::from_primitive(&sampled)
        .expect("record")
        .to_curve()
        .expect("curve");
    let (lo, hi) = curve.domain();
    assert!(sampled.positions.len() > 4);
    for (w, t) in sampled.positions.windows(2).zip(sampled.uvs[0].windows(2)) {
        let mid_t = lo + (hi - lo) * 0.5 * f64::from(t[0][0] + t[1][0]);
        let on_curve = curve.evaluate(mid_t);
        let d: f64 = (0..3)
            .map(|k| (0.5 * f64::from(w[0][k] + w[1][k]) - on_curve[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(d <= 2.0 * tol, "chord {d} off the curve");
    }
}

/// A trimmed surface authored through the typed records: a flat 4 × 4
/// patch with a square outer loop and a square hole. The tessellation
/// keeps the ring, the encoder writes surface, boundaries and edges
/// with their connections, and the decode reads the same records back.
#[test]
fn authored_trim_surface_tessellates_and_round_trips() {
    let surface = FbxNurbsSurface::new(
        [2, 2],
        [2, 2],
        [NurbsForm::Open, NurbsForm::Open],
        vec![
            [0.0, 0.0, 0.0, 1.0],
            [4.0, 0.0, 0.0, 1.0],
            [0.0, 4.0, 0.0, 1.0],
            [4.0, 4.0, 0.0, 1.0],
        ],
        vec![0.0, 0.0, 4.0, 4.0],
        vec![0.0, 0.0, 4.0, 4.0],
    );
    let edge = |a: [f64; 2], b: [f64; 2]| {
        FbxNurbsCurve::new(
            2,
            NurbsForm::Open,
            vec![[a[0], a[1], 0.0, 1.0], [b[0], b[1], 0.0, 1.0]],
            vec![0.0, 0.0, 1.0, 1.0],
        )
        .with_dimension(2)
    };
    let square = |lo: f64, hi: f64| {
        vec![
            edge([lo, lo], [hi, lo]),
            edge([hi, lo], [hi, hi]),
            edge([hi, hi], [lo, hi]),
            edge([lo, hi], [lo, lo]),
        ]
    };
    let trim = FbxTrimSurface::new(
        surface,
        vec![
            FbxBoundary::new(true, square(0.5, 3.5)),
            FbxBoundary::new(false, square(1.5, 2.5)),
        ],
    );
    let how = NurbsTessellation::default().with_segments_per_span(8);
    let prim = trim.tessellate(&how).expect("tessellate");
    let Some(Indices::U32(indices)) = &prim.indices else {
        panic!("indices");
    };
    // Area of the kept triangles: the 3 × 3 square minus the 1 × 1 hole.
    let area: f64 = indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|t| {
            let p = |i: u32| prim.positions[i as usize];
            let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
            0.5 * f64::from(((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])).abs())
        })
        .sum();
    assert!((area - 8.0).abs() < 0.05, "area {area}");
    // No kept triangle has its centre in the hole or outside the loop.
    for t in indices.as_chunks::<3>().0 {
        let c = |k: usize| -> f32 {
            t.iter()
                .map(|i| prim.positions[*i as usize][k])
                .sum::<f32>()
                / 3.0
        };
        let (x, y) = (c(0), c(1));
        assert!((0.5..=3.5).contains(&x) && (0.5..=3.5).contains(&y));
        assert!(
            !((1.5..=2.5).contains(&x) && (1.5..=2.5).contains(&y)),
            "({x}, {y})"
        );
    }

    let mut scene = Scene3D::new();
    let mesh = scene.add_mesh(trim.to_mesh(Some("ring".to_string()), &how));
    let node = scene.add_node(Node::new().with_name("Ring").with_mesh(mesh));
    scene.roots.push(node);
    for form in [FbxOutputForm::Binary, FbxOutputForm::Ascii] {
        let out = encode(&scene, form);
        let doc = parse_any(&out);
        let kinds: Vec<String> = geometries(&doc).iter().map(|g| subtype(g)).collect();
        assert_eq!(kinds.iter().filter(|k| *k == "TrimNurbsSurface").count(), 1);
        assert_eq!(kinds.iter().filter(|k| *k == "NurbsSurface").count(), 1);
        assert_eq!(kinds.iter().filter(|k| *k == "Boundary").count(), 2);
        assert_eq!(kinds.iter().filter(|k| *k == "NurbsCurve").count(), 8);
        let model = doc.root.child("Objects").unwrap().child("Model").unwrap();
        assert_eq!(subtype(model), "TrimNurbsSurface");
        let back = FbxDecoder::new()
            .with_scene_options(SceneDecodeOptions::new().with_nurbs(how))
            .decode(&out)
            .expect("decode");
        assert_eq!(back.meshes.len(), 1, "{form:?}");
        let again = trim_surface_of(&back.meshes[0].primitives[0]).expect("trim");
        assert_eq!(again, trim, "{form:?}");
        assert_eq!(back.meshes[0].primitives[0].positions, prim.positions);
    }
}
