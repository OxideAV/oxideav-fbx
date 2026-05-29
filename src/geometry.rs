//! Polygonal mesh extraction from a `Geometry` node.
//!
//! Per `docs/3d/fbx/ufbx/elements-meshes.md`, an FBX `Geometry`
//! element of subtype `Mesh` carries:
//!
//! - `Vertices` (`d` array, length `3 * V` where `V = unique vertex
//!   count`).
//! - `PolygonVertexIndex` (`i` array of indices into `Vertices`; the
//!   *last* index of each polygon is bitwise-NOT'd, i.e. negative,
//!   to mark the polygon end. Decoding: `if idx < 0 { vertex_idx =
//!   !idx } else { vertex_idx = idx }`).
//! - `LayerElementNormal[N]` — per-vertex normals with mapping mode
//!   metadata (`MappingInformationType` + `ReferenceInformationType`).
//! - `LayerElementUV[N]`, `LayerElementColor[N]`,
//!   `LayerElementMaterial[N]`, etc. — same shape, different
//!   payloads.
//!
//! The output is a [`Mesh`] with one [`Primitive`] of [`Topology::Triangles`]
//! (we triangulate ngons via fan triangulation, the convention every
//! mainline FBX loader uses for round-1 output) carrying:
//!
//! - `positions` — one `[f32; 3]` per polygon-vertex corner (since
//!   ngons get expanded, this is the per-corner buffer, not the
//!   shared-vertex one).
//! - `normals` — pulled from the first `LayerElementNormal` if its
//!   mapping mode is `ByPolygonVertex` or `ByVertex` (with optional
//!   `IndexToDirect` indirection); other mapping modes pass through
//!   unmodified for now.
//!
//! The original shared-vertex buffer is preserved on
//! `Mesh::extras["fbx:shared_positions"]` (length `3 * V`) so an
//! authoring-tool consumer can reconstruct the original FBX vertex
//! layout if needed.

use serde_json::Value;

use oxideav_mesh3d::{Error, Mesh, Primitive, Result, Topology};

use crate::binary::{FbxNode, FbxProperty};

/// Build an `oxideav-mesh3d` [`Mesh`] from an FBX `Geometry` node.
///
/// Convenience wrapper around [`extract_geometry_mesh_with_corners`]
/// that drops the per-corner shared-vertex index cache. New callers
/// (the deformer module) should prefer the `_with_corners` variant.
pub fn extract_geometry_mesh(geom: &FbxNode, name: Option<String>) -> Result<Mesh> {
    extract_geometry_mesh_with_corners(geom, name).map(|(m, _)| m)
}

/// Same as [`extract_geometry_mesh`] but also returns the per-corner
/// shared-vertex index buffer (`corner_indices[i]` is the index into
/// the original `Vertices` array for the i-th `Primitive::positions`
/// entry). The deformer module uses this to map per-shared-vertex
/// skin / morph payloads to the per-corner attribute layout.
pub fn extract_geometry_mesh_with_corners(
    geom: &FbxNode,
    name: Option<String>,
) -> Result<(Mesh, Vec<u32>)> {
    let positions = read_vertices(geom)?;
    let polygon_indices = read_polygon_vertex_index(geom)?;
    let triangles = triangulate(&polygon_indices)?;

    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions.reserve(triangles.corner_indices.len());
    for &shared_ix in &triangles.corner_indices {
        let shared_ix_usize = shared_ix as usize;
        if shared_ix_usize * 3 + 2 >= positions.len() {
            return Err(Error::invalid(format!(
                "FBX Geometry: PolygonVertexIndex value {shared_ix} out of range for {}-vertex Vertices array",
                positions.len() / 3
            )));
        }
        prim.positions.push([
            positions[shared_ix_usize * 3] as f32,
            positions[shared_ix_usize * 3 + 1] as f32,
            positions[shared_ix_usize * 3 + 2] as f32,
        ]);
    }

    // Normals — first LayerElementNormal only (ufbx-doc convention:
    // multi-layer normals are unusual in practice; surface the first
    // and stash the rest in `extras` for a follow-up round).
    if let Some(layer) = geom.children_named("LayerElementNormal").next() {
        if let Some(normals) = pull_layer_vec3(
            layer,
            "Normals",
            "NormalsIndex",
            triangles.corner_indices.len(),
            &triangles,
        )? {
            prim.normals = Some(normals);
        }
    }

    // UVs — first LayerElementUV only for round 1.
    if let Some(layer) = geom.children_named("LayerElementUV").next() {
        if let Some(uvs) = pull_layer_vec2(
            layer,
            "UV",
            "UVIndex",
            triangles.corner_indices.len(),
            &triangles,
        )? {
            prim.uvs.push(uvs);
        }
    }

    // LayerElementMaterial — surface per-polygon material slot
    // indices on `Primitive::extras` per
    // `docs/3d/fbx/ufbx/elements-meshes.md` §"Materials": the
    // `Materials` (`i` array) sub-record carries one slot index per
    // polygon (when `MappingInformationType=ByPolygon`) or one slot
    // index for the whole mesh (when `MappingInformationType=AllSame`,
    // also the FBX default per ufbx reference §`ufbx_mesh.face_material`).
    // The per-corner expanded form lands on `fbx:face_material_slots`
    // so a downstream consumer can split the primitive on material
    // boundaries without re-deriving the triangulation; the
    // `fbx:material_mapping` key captures the original mapping mode
    // for diagnostics.
    if let Some(layer) = geom.children_named("LayerElementMaterial").next() {
        if let Some(per_corner_slots) = pull_layer_material_slots(layer, &triangles)? {
            // Per-corner buffer (length == corner_indices.len()).
            prim.extras.insert(
                "fbx:face_material_slots".to_string(),
                Value::Array(
                    per_corner_slots
                        .iter()
                        .map(|&s| Value::Number(serde_json::Number::from(s)))
                        .collect(),
                ),
            );
            // Record the source mapping for downstream consumers.
            if let Some(mapping) = layer.child("MappingInformationType").and_then(|n| {
                n.properties
                    .first()
                    .and_then(FbxProperty::as_str)
                    .map(str::to_owned)
            }) {
                prim.extras
                    .insert("fbx:material_mapping".to_string(), Value::String(mapping));
            }
        }
    }

    // Attach the original shared-vertex buffer to `Primitive::extras`
    // so consumers can reconstruct the FBX vertex layout. (`Mesh`
    // itself has no extras slot — it's a thin name + primitives bag.)
    let shared_positions: Vec<f32> = positions.iter().map(|&v| v as f32).collect();
    prim.extras.insert(
        "fbx:shared_positions".to_string(),
        Value::Array(
            shared_positions
                .into_iter()
                .map(|f| {
                    Value::Number(serde_json::Number::from_f64(f as f64).unwrap_or_else(|| {
                        // Non-finite floats fall back to 0.0 in JSON
                        // (serde_json refuses to serialise NaN/Inf).
                        serde_json::Number::from_f64(0.0).unwrap()
                    }))
                })
                .collect(),
        ),
    );
    let mut mesh = Mesh::new(name);
    mesh.primitives.push(prim);
    Ok((mesh, triangles.corner_indices))
}

/// Pull the `Vertices` payload (a `d` array) out of the Geometry
/// node.
fn read_vertices(geom: &FbxNode) -> Result<Vec<f64>> {
    let v = geom
        .child("Vertices")
        .ok_or_else(|| Error::invalid("FBX Geometry: missing required `Vertices` sub-record"))?;
    match v.properties.first() {
        Some(FbxProperty::F64Array(arr)) => {
            if arr.len() % 3 != 0 {
                return Err(Error::invalid(format!(
                    "FBX Geometry: Vertices length {} not a multiple of 3",
                    arr.len()
                )));
            }
            Ok(arr.clone())
        }
        Some(FbxProperty::F32Array(arr)) => {
            // Tolerated: some exporters emit `f` instead of `d`.
            if arr.len() % 3 != 0 {
                return Err(Error::invalid(format!(
                    "FBX Geometry: Vertices length {} not a multiple of 3",
                    arr.len()
                )));
            }
            Ok(arr.iter().map(|&v| v as f64).collect())
        }
        _ => Err(Error::invalid(
            "FBX Geometry: Vertices property is not a d/f array",
        )),
    }
}

/// Pull the `PolygonVertexIndex` payload (an `i` array; per the
/// binary writeup, the *last* index of each polygon is the bitwise-NOT
/// of its actual vertex index).
fn read_polygon_vertex_index(geom: &FbxNode) -> Result<Vec<i32>> {
    let pvi = geom
        .child("PolygonVertexIndex")
        .ok_or_else(|| Error::invalid("FBX Geometry: missing `PolygonVertexIndex` sub-record"))?;
    match pvi.properties.first() {
        Some(FbxProperty::I32Array(arr)) => Ok(arr.clone()),
        _ => Err(Error::invalid(
            "FBX Geometry: PolygonVertexIndex property is not an i32 array",
        )),
    }
}

/// Per-polygon triangulation result.
pub(crate) struct Triangulation {
    /// Shared-vertex indices (one per triangle corner, three per
    /// triangle), in fan-triangulation order. Negatives from
    /// `PolygonVertexIndex` have already been bit-NOT'd back to
    /// positive shared-vertex indices.
    pub corner_indices: Vec<u32>,
    /// Per-corner index into the original `PolygonVertexIndex`
    /// (i.e. into the per-polygon-vertex arrays carried by
    /// `LayerElement*` records). Same length as
    /// [`Self::corner_indices`].
    pub corner_pvi_index: Vec<u32>,
    /// Per-triangle index into the original polygon array (i.e. for
    /// `tri_polygon_index[t]`, which polygon in the source mesh this
    /// triangle was fanned from). Length = `corner_indices.len() / 3`.
    /// Used to expand `LayerElementMaterial` (which is keyed
    /// `ByPolygon`) into per-corner material-slot indices.
    pub tri_polygon_index: Vec<u32>,
    /// Total polygon count in the source mesh — matches the number of
    /// negative end-of-polygon markers in `PolygonVertexIndex`. Used to
    /// validate `LayerElementMaterial` mapping-mode payloads.
    pub polygon_count: u32,
}

/// Fan-triangulate the `PolygonVertexIndex` array. Decodes the
/// negative end-of-polygon marker per Gessler's writeup:
/// `vertex_idx = !signed_idx` when the value is negative.
fn triangulate(pvi: &[i32]) -> Result<Triangulation> {
    let mut corner_indices = Vec::new();
    let mut corner_pvi_index = Vec::new();
    let mut tri_polygon_index = Vec::new();
    let mut polygon_start = 0;
    let mut polygon_count: u32 = 0;
    for (i, &raw) in pvi.iter().enumerate() {
        let is_end = raw < 0;
        if is_end {
            // Polygon spans pvi[polygon_start..=i]. Fan-triangulate
            // around index polygon_start.
            let n = i - polygon_start + 1;
            if n < 3 {
                return Err(Error::invalid(format!(
                    "FBX Geometry: polygon at PolygonVertexIndex[{polygon_start}..={i}] has {n} corners (< 3)"
                )));
            }
            for tri in 1..n - 1 {
                let a_pvi = polygon_start;
                let b_pvi = polygon_start + tri;
                let c_pvi = polygon_start + tri + 1;
                corner_pvi_index.push(a_pvi as u32);
                corner_pvi_index.push(b_pvi as u32);
                corner_pvi_index.push(c_pvi as u32);
                corner_indices.push(decode_pvi(pvi[a_pvi]));
                corner_indices.push(decode_pvi(pvi[b_pvi]));
                corner_indices.push(decode_pvi(pvi[c_pvi]));
                tri_polygon_index.push(polygon_count);
            }
            polygon_count += 1;
            polygon_start = i + 1;
        }
    }
    if polygon_start != pvi.len() {
        return Err(Error::invalid(
            "FBX Geometry: PolygonVertexIndex did not end on a polygon-end marker (negative)",
        ));
    }
    Ok(Triangulation {
        corner_indices,
        corner_pvi_index,
        tri_polygon_index,
        polygon_count,
    })
}

/// Bitwise-NOT decode for FBX `PolygonVertexIndex` values: per
/// Gessler's writeup the last index of each polygon is stored as
/// `~vertex_index`. Positive values pass through.
fn decode_pvi(raw: i32) -> u32 {
    if raw < 0 {
        (!raw) as u32
    } else {
        raw as u32
    }
}

/// Generic 3-component `LayerElement*` puller (Normals, Tangents,
/// Bitangents). Returns `Some(per_corner_buf)` when the mapping mode
/// is something we know how to flatten, `None` otherwise.
fn pull_layer_vec3(
    layer: &FbxNode,
    data_name: &str,
    index_name: &str,
    expected_corners: usize,
    triangles: &Triangulation,
) -> Result<Option<Vec<[f32; 3]>>> {
    let mapping = layer.child("MappingInformationType").and_then(|n| {
        n.properties
            .first()
            .and_then(FbxProperty::as_str)
            .map(str::to_owned)
    });
    let reference = layer.child("ReferenceInformationType").and_then(|n| {
        n.properties
            .first()
            .and_then(FbxProperty::as_str)
            .map(str::to_owned)
    });
    let data_node = match layer.child(data_name) {
        Some(n) => n,
        None => return Ok(None),
    };
    let raw = match data_node.properties.first() {
        Some(FbxProperty::F64Array(a)) => a.clone(),
        Some(FbxProperty::F32Array(a)) => a.iter().map(|&v| v as f64).collect(),
        _ => return Ok(None),
    };
    if raw.len() % 3 != 0 {
        return Err(Error::invalid(format!(
            "FBX LayerElement: `{data_name}` length {} not a multiple of 3",
            raw.len()
        )));
    }
    let triples: Vec<[f32; 3]> = raw
        .chunks_exact(3)
        .map(|c| [c[0] as f32, c[1] as f32, c[2] as f32])
        .collect();
    // Optional indirection.
    let index_arr: Option<Vec<i32>> = layer.child(index_name).and_then(|n| {
        n.properties.first().and_then(|p| match p {
            FbxProperty::I32Array(a) => Some(a.clone()),
            _ => None,
        })
    });
    flatten_layer_vec3(
        triples,
        index_arr.as_deref(),
        mapping.as_deref(),
        reference.as_deref(),
        expected_corners,
        triangles,
    )
}

fn flatten_layer_vec3(
    triples: Vec<[f32; 3]>,
    index_arr: Option<&[i32]>,
    mapping: Option<&str>,
    reference: Option<&str>,
    expected_corners: usize,
    triangles: &Triangulation,
) -> Result<Option<Vec<[f32; 3]>>> {
    let direct_only = matches!(reference, None | Some("Direct"));
    let by_polygon_vertex = matches!(mapping, Some("ByPolygonVertex"));
    let by_vertex = matches!(mapping, Some("ByVertex") | Some("ByVertice"));
    if !by_polygon_vertex && !by_vertex {
        // Other mapping modes (`ByPolygon`, `AllSame`) deferred —
        // surface no per-vertex data for round 1 rather than
        // mis-attribute.
        return Ok(None);
    }
    let mut out = Vec::with_capacity(expected_corners);
    for (corner_ix, &shared_ix) in triangles.corner_indices.iter().enumerate() {
        let lookup = if by_vertex {
            shared_ix as usize
        } else {
            triangles.corner_pvi_index[corner_ix] as usize
        };
        let triple_ix = if direct_only {
            lookup
        } else if let Some(ix_arr) = index_arr {
            let i = ix_arr.get(lookup).copied().unwrap_or(-1);
            if i < 0 {
                return Err(Error::invalid(format!(
                    "FBX LayerElement: IndexToDirect index {i} (negative)"
                )));
            }
            i as usize
        } else {
            return Err(Error::invalid(
                "FBX LayerElement: ReferenceInformationType==IndexToDirect but no Index sub-record",
            ));
        };
        let triple = triples.get(triple_ix).copied().ok_or_else(|| {
            Error::invalid(format!(
                "FBX LayerElement: triple index {triple_ix} out of range for {}-element array",
                triples.len()
            ))
        })?;
        out.push(triple);
    }
    Ok(Some(out))
}

fn pull_layer_vec2(
    layer: &FbxNode,
    data_name: &str,
    index_name: &str,
    expected_corners: usize,
    triangles: &Triangulation,
) -> Result<Option<Vec<[f32; 2]>>> {
    let mapping = layer.child("MappingInformationType").and_then(|n| {
        n.properties
            .first()
            .and_then(FbxProperty::as_str)
            .map(str::to_owned)
    });
    let reference = layer.child("ReferenceInformationType").and_then(|n| {
        n.properties
            .first()
            .and_then(FbxProperty::as_str)
            .map(str::to_owned)
    });
    let data_node = match layer.child(data_name) {
        Some(n) => n,
        None => return Ok(None),
    };
    let raw = match data_node.properties.first() {
        Some(FbxProperty::F64Array(a)) => a.clone(),
        Some(FbxProperty::F32Array(a)) => a.iter().map(|&v| v as f64).collect(),
        _ => return Ok(None),
    };
    if raw.len() % 2 != 0 {
        return Err(Error::invalid(format!(
            "FBX LayerElement: `{data_name}` length {} not a multiple of 2",
            raw.len()
        )));
    }
    let pairs: Vec<[f32; 2]> = raw
        .chunks_exact(2)
        .map(|c| [c[0] as f32, c[1] as f32])
        .collect();
    let index_arr: Option<Vec<i32>> = layer.child(index_name).and_then(|n| {
        n.properties.first().and_then(|p| match p {
            FbxProperty::I32Array(a) => Some(a.clone()),
            _ => None,
        })
    });
    let direct_only = matches!(reference.as_deref(), None | Some("Direct"));
    let by_polygon_vertex = matches!(mapping.as_deref(), Some("ByPolygonVertex"));
    let by_vertex = matches!(mapping.as_deref(), Some("ByVertex") | Some("ByVertice"));
    if !by_polygon_vertex && !by_vertex {
        return Ok(None);
    }
    let mut out = Vec::with_capacity(expected_corners);
    for (corner_ix, &shared_ix) in triangles.corner_indices.iter().enumerate() {
        let lookup = if by_vertex {
            shared_ix as usize
        } else {
            triangles.corner_pvi_index[corner_ix] as usize
        };
        let pair_ix = if direct_only {
            lookup
        } else if let Some(ix_arr) = index_arr.as_deref() {
            let i = ix_arr.get(lookup).copied().unwrap_or(-1);
            if i < 0 {
                return Err(Error::invalid(format!(
                    "FBX LayerElement: UVIndex {i} (negative)"
                )));
            }
            i as usize
        } else {
            return Err(Error::invalid(
                "FBX LayerElement: UV ReferenceInformationType==IndexToDirect but no UVIndex sub-record",
            ));
        };
        let pair = pairs.get(pair_ix).copied().ok_or_else(|| {
            Error::invalid(format!(
                "FBX LayerElement: UV pair index {pair_ix} out of range for {}-pair array",
                pairs.len()
            ))
        })?;
        out.push(pair);
    }
    Ok(Some(out))
}

/// Pull the per-corner material-slot index buffer from a
/// `LayerElementMaterial` node.
///
/// FBX `LayerElementMaterial` (per ufbx
/// `elements-meshes.md` §"Materials") carries:
///
/// - `Materials` — `i` array. Length depends on `MappingInformationType`:
///   - `AllSame` (the default): exactly one entry — that slot applies
///     to every polygon. The buffer often holds the single value `0`.
///   - `ByPolygon`: one entry per polygon (length ==
///     `Triangulation::polygon_count`). Slot indices key
///     `Material -> Model` OO connection slots in the order that ufbx
///     reference §`ufbx_mesh.materials` documents.
/// - `MappingInformationType` — string (the two values above are the
///   ones ufbx reference §`ufbx_mesh_part`/§`ufbx_mesh.materials`
///   documents for materials).
/// - `ReferenceInformationType` — string. For materials, the
///   `IndexToDirect` form is what every binary FBX exporter actually
///   emits (the slot indices themselves are the "direct" payload —
///   ufbx documents this with `index_type == UFBX_INDEX_TYPE_DIRECT`).
///   `Direct` and missing-reference accepted as synonyms.
///
/// Returned buffer is one `u32` slot index per triangle corner
/// (length == `triangles.corner_indices.len()`), expanded from the
/// per-polygon payload via `triangles.tri_polygon_index`.
fn pull_layer_material_slots(
    layer: &FbxNode,
    triangles: &Triangulation,
) -> Result<Option<Vec<u32>>> {
    let mapping = layer.child("MappingInformationType").and_then(|n| {
        n.properties
            .first()
            .and_then(FbxProperty::as_str)
            .map(str::to_owned)
    });
    let data_node = match layer.child("Materials") {
        Some(n) => n,
        None => return Ok(None),
    };
    let raw: Vec<i32> = match data_node.properties.first() {
        Some(FbxProperty::I32Array(a)) => a.clone(),
        Some(FbxProperty::I64Array(a)) => a.iter().map(|&v| v as i32).collect(),
        _ => return Ok(None),
    };
    if raw.is_empty() {
        return Ok(None);
    }
    let all_same = matches!(mapping.as_deref(), Some("AllSame")) || raw.len() == 1;
    let by_polygon = matches!(mapping.as_deref(), Some("ByPolygon"));
    let n_corners = triangles.corner_indices.len();
    let n_tris = triangles.tri_polygon_index.len();
    debug_assert_eq!(n_corners, n_tris * 3);
    if all_same {
        let slot = raw[0].max(0) as u32;
        return Ok(Some(vec![slot; n_corners]));
    }
    if by_polygon {
        if raw.len() != triangles.polygon_count as usize {
            return Err(Error::invalid(format!(
                "FBX LayerElementMaterial: Materials length {} but polygon count {} (ByPolygon mapping)",
                raw.len(),
                triangles.polygon_count
            )));
        }
        let mut out = Vec::with_capacity(n_corners);
        for &poly_ix in &triangles.tri_polygon_index {
            let s = raw.get(poly_ix as usize).copied().unwrap_or(0).max(0) as u32;
            // Three corners per triangle.
            out.push(s);
            out.push(s);
            out.push(s);
        }
        return Ok(Some(out));
    }
    // Unknown mapping mode (NoMappingInformation, ByVertex on
    // materials — ufbx documents these as exporter quirks). Skip,
    // matching ufbx's "fall back to all-same" tolerance per
    // `elements-meshes.md` §"Materials".
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pvi_decodes_end_marker() {
        // Standard FBX convention: ~i = -i - 1. A polygon ending on
        // the unsigned vertex index 4 stores the value -5 in the
        // PolygonVertexIndex array.
        assert_eq!(decode_pvi(-5), 4);
        assert_eq!(decode_pvi(0), 0);
        assert_eq!(decode_pvi(7), 7);
        assert_eq!(decode_pvi(-1), 0);
    }

    #[test]
    fn fan_triangulates_quad() {
        // Quad spanning shared-vertices [0, 1, 2, 3] is stored as
        // [0, 1, 2, ~3] = [0, 1, 2, -4].
        let pvi = vec![0, 1, 2, -4];
        let tris = triangulate(&pvi).unwrap();
        // One quad → two fan triangles → six corner indices.
        assert_eq!(tris.corner_indices, vec![0, 1, 2, 0, 2, 3]);
    }

    #[test]
    fn fan_triangulates_triangle() {
        let pvi = vec![5, 6, -8];
        let tris = triangulate(&pvi).unwrap();
        assert_eq!(tris.corner_indices, vec![5, 6, 7]);
    }

    #[test]
    fn rejects_polygon_with_too_few_vertices() {
        let pvi = vec![0, -2];
        assert!(triangulate(&pvi).is_err());
    }

    #[test]
    fn triangulation_tracks_polygon_index() {
        // Three triangles + one quad = 4 polygons.
        // Quad fans into two triangles => 5 triangles total.
        let pvi = vec![
            // Triangle 0
            0, 1, -3, // Triangle 1
            4, 5, -7, // Quad polygon 2 (fans into 2 tris)
            8, 9, 10, -12, // Triangle 3
            13, 14, -16,
        ];
        let tris = triangulate(&pvi).unwrap();
        assert_eq!(tris.polygon_count, 4);
        assert_eq!(tris.tri_polygon_index, vec![0, 1, 2, 2, 3]);
        assert_eq!(tris.corner_indices.len(), 5 * 3);
    }

    fn make_layer_material_node(materials_arr: Vec<i32>, mapping: Option<&str>) -> FbxNode {
        let mut layer = FbxNode {
            name: "LayerElementMaterial".to_string(),
            properties: vec![FbxProperty::I32(0)],
            children: Vec::new(),
        };
        if let Some(m) = mapping {
            layer.children.push(FbxNode {
                name: "MappingInformationType".to_string(),
                properties: vec![FbxProperty::String(m.as_bytes().to_vec())],
                children: Vec::new(),
            });
            layer.children.push(FbxNode {
                name: "ReferenceInformationType".to_string(),
                properties: vec![FbxProperty::String(b"IndexToDirect".to_vec())],
                children: Vec::new(),
            });
        }
        layer.children.push(FbxNode {
            name: "Materials".to_string(),
            properties: vec![FbxProperty::I32Array(materials_arr)],
            children: Vec::new(),
        });
        layer
    }

    #[test]
    fn layer_material_all_same_broadcasts_single_slot() {
        let pvi = vec![0, 1, -3, 4, 5, -7];
        let tris = triangulate(&pvi).unwrap();
        assert_eq!(tris.polygon_count, 2);
        let layer = make_layer_material_node(vec![3], Some("AllSame"));
        let slots = pull_layer_material_slots(&layer, &tris).unwrap().unwrap();
        // Two triangles, three corners each, every slot is 3.
        assert_eq!(slots, vec![3, 3, 3, 3, 3, 3]);
    }

    #[test]
    fn layer_material_by_polygon_per_polygon_payload() {
        // Polygon 0 (triangle) -> slot 0
        // Polygon 1 (quad)     -> slot 1 (two fan triangles)
        // Polygon 2 (triangle) -> slot 0
        let pvi = vec![0, 1, -3, 4, 5, 6, -8, 9, 10, -12];
        let tris = triangulate(&pvi).unwrap();
        assert_eq!(tris.polygon_count, 3);
        let layer = make_layer_material_node(vec![0, 1, 0], Some("ByPolygon"));
        let slots = pull_layer_material_slots(&layer, &tris).unwrap().unwrap();
        // Triangle 0 (poly 0, slot 0): 0,0,0
        // Triangle 1 (poly 1, slot 1): 1,1,1
        // Triangle 2 (poly 1, slot 1): 1,1,1
        // Triangle 3 (poly 2, slot 0): 0,0,0
        assert_eq!(slots, vec![0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0]);
    }

    #[test]
    fn layer_material_by_polygon_length_mismatch_errors() {
        // Two polygons but Materials carries three entries.
        let pvi = vec![0, 1, -3, 4, 5, -7];
        let tris = triangulate(&pvi).unwrap();
        let layer = make_layer_material_node(vec![0, 1, 0], Some("ByPolygon"));
        let err = pull_layer_material_slots(&layer, &tris).unwrap_err();
        let msg = format!("{err}");
        assert!(
            msg.contains("ByPolygon"),
            "expected ByPolygon length mismatch error, got: {msg}"
        );
    }

    #[test]
    fn layer_material_single_entry_treated_as_all_same() {
        // Materials with one entry and no mapping mode header is the
        // exporter shorthand for AllSame per ufbx-doc tolerance.
        let pvi = vec![0, 1, -3, 4, 5, -7];
        let tris = triangulate(&pvi).unwrap();
        let layer = make_layer_material_node(vec![2], None);
        let slots = pull_layer_material_slots(&layer, &tris).unwrap().unwrap();
        assert_eq!(slots, vec![2; 6]);
    }

    #[test]
    fn layer_material_unknown_mapping_returns_none() {
        // ByVertex on materials is exporter-quirk territory; per ufbx
        // we tolerate it as "no surfacing", letting the connection
        // table do its default first-material-only wiring.
        let pvi = vec![0, 1, -3];
        let tris = triangulate(&pvi).unwrap();
        let layer = make_layer_material_node(vec![0, 1, 2], Some("ByVertex"));
        assert!(pull_layer_material_slots(&layer, &tris).unwrap().is_none());
    }
}
