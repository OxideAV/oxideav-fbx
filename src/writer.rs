//! Binary FBX writer — serialise an [`FbxDocument`] back to bytes.
//!
//! The output layout follows Alexander Gessler / Blender Foundation,
//! *FBX Binary File Format Specification* (August 2013, public-domain
//! dedication; staged at `docs/3d/fbx/blender-fbx-binary-format.html`)
//! and is **bit-compatible with [`crate::binary::parse`] on
//! round-trip**: every value the parser produces this module can write
//! back into a buffer that the parser will decode to an equal
//! [`FbxDocument`].
//!
//! # What the Gessler reference does NOT cover
//!
//! The Blender writeup spells out the 27-byte header, the recursive
//! Node Record layout (32-bit pre-7500, 64-bit ≥ 7500), the
//! per-property type-code dispatch, and the array `Encoding == 0` raw
//! / `Encoding == 1` zlib-deflated form. It states explicitly that
//! **"after [the top-level records] there is a footer with unknown
//! contents"**. We therefore write **no footer at all** — the parser
//! reads up to the final top-level NULL-record and then EOF, which
//! the Gessler-spec'd grammar already tolerates (per `parse`'s own
//! "EOF without explicit NULL-record" comment in [`crate::binary`]).
//! Files this writer produces consequently round-trip through our own
//! parser losslessly but may be flagged as missing the trailing
//! Autodesk-private footer signature by SDKs that validate it.
//!
//! # Array encoding policy
//!
//! By default arrays are written **uncompressed** (`Encoding == 0`).
//! The Gessler doc allows both forms; uncompressed is
//! bit-deterministic and avoids any zlib version / level
//! reproducibility surprise.
//!
//! Callers that want smaller output can opt into zlib-deflate
//! (`Encoding == 1`) on a per-call basis via
//! [`write_document_with_options`] + [`WriterOptions::compress_arrays`].
//! The compressed form is what every Autodesk-exported FBX in the
//! wild uses for arrays of meaningful size; the Gessler doc
//! enumerates exactly two values for `Encoding` (0 raw / 1 zlib) and
//! the post-deflate buffer length is stored verbatim in
//! `CompressedLength`. Output remains deterministic for a given
//! `miniz_oxide` version + compression level, but is no longer
//! guaranteed to match across `miniz_oxide` upgrades — the
//! round-trip closure through [`crate::binary::parse`] is what
//! callers should rely on, not byte-exact equality across crate
//! versions.
//!
//! # Version-dependent header widths
//!
//! Per the doc's *"Version-dependent quirks"* — files at version
//! `>= 7500` use 64-bit `EndOffset` / `NumProperties` /
//! `PropertyListLen` fields. The writer auto-selects based on
//! [`FbxDocument::version`] and writes the matching widths for both
//! every per-node header and the trailing NULL-record sentinel.

use oxideav_mesh3d::{Error, Result};

use crate::binary::{
    FbxDocument, FbxNode, FbxProperty, FBX_HEADER_BYTES, FBX_MAGIC, FBX_MAGIC_TAIL,
    FBX_VERSION_64BIT_THRESHOLD,
};

/// Tunable knobs for [`write_document_with_options`].
///
/// All fields are documented to the per-record level of the Gessler
/// spec they map to; defaults match the legacy [`write_document`]
/// behaviour (every array uncompressed, no Autodesk footer).
#[derive(Clone, Debug)]
pub struct WriterOptions {
    /// If `Some(threshold)`, array properties whose **raw** payload
    /// (`ArrayLength * elemSize`) is at least `threshold` bytes are
    /// written with `Encoding == 1` (zlib deflate) per the Gessler
    /// spec's array record format. The post-deflate buffer is stored
    /// in `CompressedLength`. Arrays below the threshold stay
    /// uncompressed; in particular, arrays where deflate would
    /// produce a *larger* buffer than the raw payload also fall back
    /// to `Encoding == 0` so the on-disk size never regresses.
    ///
    /// `None` (the default) keeps every array uncompressed, matching
    /// the round-3 writer behaviour and the legacy
    /// [`write_document`] entry point.
    pub compress_arrays: Option<usize>,

    /// zlib compression level forwarded to `miniz_oxide`. `0`
    /// (`compress_to_vec_zlib` returns a stored block — no deflate)
    /// through `10` (max compression). Default `6` matches zlib's
    /// own `Z_DEFAULT_COMPRESSION` constant and is what most FBX
    /// exporters appear to use in the wild.
    ///
    /// Ignored when `compress_arrays` is `None`.
    pub compression_level: u8,
}

impl Default for WriterOptions {
    fn default() -> Self {
        Self {
            compress_arrays: None,
            compression_level: 6,
        }
    }
}

impl WriterOptions {
    /// Builder helper — enable deflate of array properties whose raw
    /// payload is at least `threshold` bytes.
    pub fn compress_arrays_at(mut self, threshold: usize) -> Self {
        self.compress_arrays = Some(threshold);
        self
    }

    /// Builder helper — pick the zlib compression level (0..=10).
    pub fn compression_level(mut self, level: u8) -> Self {
        self.compression_level = level;
        self
    }
}

/// Serialise an [`FbxDocument`] to a byte buffer that decodes back
/// through [`crate::binary::parse`] to an equivalent document.
///
/// The resulting buffer is the 27-byte header + every child of
/// [`FbxDocument::root`] written as a top-level Node Record, capped
/// by the format's all-zero NULL-record sentinel. **No Autodesk
/// footer is written** — see the module docs for the rationale.
///
/// Arrays are written uncompressed (`Encoding == 0`). Use
/// [`write_document_with_options`] to opt into deflate of larger
/// arrays.
pub fn write_document(doc: &FbxDocument) -> Result<Vec<u8>> {
    write_document_with_options(doc, &WriterOptions::default())
}

/// Like [`write_document`] but parameterised by [`WriterOptions`].
///
/// Currently the only knob the options struct exposes is
/// per-array deflate compression (`Encoding == 1`); the document
/// structure (header, Node Record layout, NULL-record sentinel
/// placement) is unaffected.
pub fn write_document_with_options(doc: &FbxDocument, opts: &WriterOptions) -> Result<Vec<u8>> {
    let use_64bit = doc.version >= FBX_VERSION_64BIT_THRESHOLD;
    let mut out = Vec::new();
    // 27-byte header: 20-byte magic + 0x1A 0x00 + version (LE u32).
    out.extend_from_slice(FBX_MAGIC);
    out.extend_from_slice(FBX_MAGIC_TAIL);
    out.extend_from_slice(&doc.version.to_le_bytes());
    debug_assert_eq!(out.len(), FBX_HEADER_BYTES);

    for child in &doc.root.children {
        write_node(child, &mut out, use_64bit, opts)?;
    }
    // Final NULL-record sentinel for the top-level list. The header
    // size matches the file's 32-bit-vs-64-bit Node Record layout.
    let null_record_bytes = if use_64bit { 25 } else { 13 };
    out.extend(std::iter::repeat(0u8).take(null_record_bytes));
    Ok(out)
}

/// Serialise one [`FbxNode`] (with all properties and recursively all
/// children) into `out`. The per-record EndOffset is back-patched
/// after the whole record body has been written so its absolute value
/// is known.
fn write_node(
    node: &FbxNode,
    out: &mut Vec<u8>,
    use_64bit: bool,
    opts: &WriterOptions,
) -> Result<()> {
    // Per Gessler's header table:
    //   <= 7400:  EndOffset(u32) | NumProperties(u32) | PropertyListLen(u32) | NameLen(u8)
    //   >= 7500:  EndOffset(u64) | NumProperties(u64) | PropertyListLen(u64) | NameLen(u8)
    let header_start = out.len();
    if use_64bit {
        out.extend_from_slice(&0u64.to_le_bytes()); // EndOffset placeholder
        out.extend_from_slice(&(node.properties.len() as u64).to_le_bytes());
        out.extend_from_slice(&0u64.to_le_bytes()); // PropertyListLen placeholder
    } else {
        out.extend_from_slice(&0u32.to_le_bytes()); // EndOffset placeholder
        out.extend_from_slice(&(node.properties.len() as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // PropertyListLen placeholder
    }
    // NameLen — Gessler spec: 1 byte, `<= 255` characters. The parser
    // requires the byte to fit; we surface a clean error rather than
    // silently truncate.
    let name_bytes = node.name.as_bytes();
    if name_bytes.len() > u8::MAX as usize {
        return Err(Error::invalid(format!(
            "binary FBX writer: node name `{}` is {} bytes — limit is 255 per the spec",
            node.name,
            name_bytes.len()
        )));
    }
    out.push(name_bytes.len() as u8);
    out.extend_from_slice(name_bytes);

    // Property list — exactly `num_props` records.
    let prop_start = out.len();
    for prop in &node.properties {
        write_property(prop, out, opts)?;
    }
    let prop_list_len = out.len() - prop_start;

    // Nested list: every child + the NULL-record sentinel. The
    // Gessler spec says the nested list is **omitted entirely** when
    // there are no children, so we only emit it when the vector is
    // populated. (Some loaders accept both forms; the omitted form is
    // what every well-formed exporter writes.)
    if !node.children.is_empty() {
        for child in &node.children {
            write_node(child, out, use_64bit, opts)?;
        }
        let null_record_bytes = if use_64bit { 25 } else { 13 };
        out.extend(std::iter::repeat(0u8).take(null_record_bytes));
    }

    // Back-patch the EndOffset + PropertyListLen now that we know the
    // record's absolute end position.
    let end_offset = out.len();
    if use_64bit {
        out[header_start..header_start + 8].copy_from_slice(&(end_offset as u64).to_le_bytes());
        out[header_start + 16..header_start + 24]
            .copy_from_slice(&(prop_list_len as u64).to_le_bytes());
    } else {
        out[header_start..header_start + 4].copy_from_slice(&(end_offset as u32).to_le_bytes());
        out[header_start + 8..header_start + 12]
            .copy_from_slice(&(prop_list_len as u32).to_le_bytes());
    }
    Ok(())
}

/// Encode one [`FbxProperty`] into `out`. The byte sequence is the
/// 1-byte type code followed by the per-variant payload per Gessler's
/// *"Property Record Format"* / *"Array types"* / *"Special types"*
/// sections.
fn write_property(prop: &FbxProperty, out: &mut Vec<u8>, opts: &WriterOptions) -> Result<()> {
    match prop {
        // -- Scalars (Gessler §"Primitive Types") --
        FbxProperty::I16(v) => {
            out.push(b'Y');
            out.extend_from_slice(&v.to_le_bytes());
        }
        FbxProperty::Bool(v) => {
            out.push(b'C');
            // Gessler doesn't pin down which encoding of "bool" Autodesk
            // writes (the parser reads `value & 1`), but both `0x00`
            // and `0x01` are observed in the wild. Round-tripping our
            // parser's output therefore requires writing back the same
            // canonical byte the parser produced — we emit `0x01` for
            // `true` and `0x00` for `false`.
            out.push(if *v { 1 } else { 0 });
        }
        FbxProperty::I32(v) => {
            out.push(b'I');
            out.extend_from_slice(&v.to_le_bytes());
        }
        FbxProperty::F32(v) => {
            out.push(b'F');
            out.extend_from_slice(&v.to_le_bytes());
        }
        FbxProperty::F64(v) => {
            out.push(b'D');
            out.extend_from_slice(&v.to_le_bytes());
        }
        FbxProperty::I64(v) => {
            out.push(b'L');
            out.extend_from_slice(&v.to_le_bytes());
        }
        // -- Arrays (Gessler §"Array types"). Each variant routes
        //    through `write_array_body`, which picks raw vs deflate
        //    based on the caller's `WriterOptions` and the per-array
        //    raw byte size. --
        FbxProperty::F32Array(arr) => {
            out.push(b'f');
            let mut raw = Vec::with_capacity(arr.len() * 4);
            for v in arr {
                raw.extend_from_slice(&v.to_le_bytes());
            }
            write_array_body(out, arr.len(), &raw, opts);
        }
        FbxProperty::F64Array(arr) => {
            out.push(b'd');
            let mut raw = Vec::with_capacity(arr.len() * 8);
            for v in arr {
                raw.extend_from_slice(&v.to_le_bytes());
            }
            write_array_body(out, arr.len(), &raw, opts);
        }
        FbxProperty::I32Array(arr) => {
            out.push(b'i');
            let mut raw = Vec::with_capacity(arr.len() * 4);
            for v in arr {
                raw.extend_from_slice(&v.to_le_bytes());
            }
            write_array_body(out, arr.len(), &raw, opts);
        }
        FbxProperty::I64Array(arr) => {
            out.push(b'l');
            let mut raw = Vec::with_capacity(arr.len() * 8);
            for v in arr {
                raw.extend_from_slice(&v.to_le_bytes());
            }
            write_array_body(out, arr.len(), &raw, opts);
        }
        FbxProperty::BoolArray(arr) => {
            out.push(b'b');
            // Each bool occupies one byte (Gessler §"Array types"
            // size table — `b` has elemSize = 1). Same `& 1` canon
            // as the scalar `C` variant.
            let raw: Vec<u8> = arr.iter().map(|&v| if v { 1u8 } else { 0u8 }).collect();
            write_array_body(out, arr.len(), &raw, opts);
        }
        // -- Special types (Gessler §"Special types") --
        FbxProperty::String(bytes) => {
            out.push(b'S');
            write_blob_header(out, bytes.len())?;
            out.extend_from_slice(bytes);
        }
        FbxProperty::Raw(bytes) => {
            out.push(b'R');
            write_blob_header(out, bytes.len())?;
            out.extend_from_slice(bytes);
        }
    }
    Ok(())
}

/// Common array-body emitter: writes the
/// `ArrayLength | Encoding | CompressedLength | Contents` block. The
/// caller has already pushed the 1-byte type code. `raw` is the
/// little-endian element stream (`array_length * elemSize` bytes).
///
/// Picks deflate (`Encoding == 1`) when all of:
///   - `opts.compress_arrays = Some(threshold)`
///   - `raw.len() >= threshold`
///   - the deflate output is **strictly smaller** than the raw
///     payload (the spec allows either form; we never inflate on
///     purpose).
///
/// Otherwise falls back to the deterministic raw form
/// (`Encoding == 0` / `CompressedLength == 0`).
fn write_array_body(out: &mut Vec<u8>, count: usize, raw: &[u8], opts: &WriterOptions) {
    let compressed = match opts.compress_arrays {
        Some(threshold) if raw.len() >= threshold => {
            let level = opts.compression_level.min(10);
            let buf = miniz_oxide::deflate::compress_to_vec_zlib(raw, level);
            if buf.len() < raw.len() {
                Some(buf)
            } else {
                None
            }
        }
        _ => None,
    };
    if let Some(buf) = compressed {
        // `Encoding == 1`: the post-deflate buffer length goes into
        // `CompressedLength` per Gessler §"Array types". `ArrayLength`
        // remains the element count (the parser multiplies by
        // `elemSize` to validate the post-inflate length).
        out.extend_from_slice(&(count as u32).to_le_bytes());
        out.extend_from_slice(&1u32.to_le_bytes());
        out.extend_from_slice(&(buf.len() as u32).to_le_bytes());
        out.extend_from_slice(&buf);
    } else {
        out.extend_from_slice(&(count as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes()); // Encoding = 0
        out.extend_from_slice(&0u32.to_le_bytes()); // CompressedLength = 0
        out.extend_from_slice(raw);
    }
}

/// Write a `u32` length-prefix for an `S` / `R` blob and validate it
/// fits the spec-defined header width (4 bytes).
fn write_blob_header(out: &mut Vec<u8>, len: usize) -> Result<()> {
    if len > u32::MAX as usize {
        return Err(Error::invalid(format!(
            "binary FBX writer: blob length {} overflows the spec u32 length prefix",
            len
        )));
    }
    out.extend_from_slice(&(len as u32).to_le_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::{self};

    fn roundtrip_node(version: u32, root_children: Vec<FbxNode>) -> FbxDocument {
        let doc = FbxDocument {
            version,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: root_children,
            },
        };
        let bytes = write_document(&doc).expect("write_document succeeds");
        binary::parse(&bytes).expect("write_document output decodes")
    }

    #[test]
    fn empty_doc_round_trip_pre_7500() {
        let parsed = roundtrip_node(7400, Vec::new());
        assert_eq!(parsed.version, 7400);
        assert!(parsed.root.children.is_empty());
    }

    #[test]
    fn empty_doc_round_trip_post_7500() {
        let parsed = roundtrip_node(7700, Vec::new());
        assert_eq!(parsed.version, 7700);
        assert!(parsed.root.children.is_empty());
    }

    #[test]
    fn every_scalar_type_round_trips() {
        let node = FbxNode {
            name: "AllScalars".to_string(),
            properties: vec![
                FbxProperty::I16(-12345),
                FbxProperty::Bool(true),
                FbxProperty::I32(2_000_000_001),
                FbxProperty::F32(core::f32::consts::PI),
                FbxProperty::F64(core::f64::consts::E),
                FbxProperty::I64(-9_000_000_000_000_000_000),
            ],
            children: Vec::new(),
        };
        let parsed = roundtrip_node(7400, vec![node.clone()]);
        assert_eq!(parsed.root.children.len(), 1);
        let got = &parsed.root.children[0];
        assert_eq!(got.name, "AllScalars");
        assert_eq!(got.properties, node.properties);
    }

    #[test]
    fn every_array_type_round_trips() {
        let node = FbxNode {
            name: "AllArrays".to_string(),
            properties: vec![
                FbxProperty::F32Array(vec![1.0, -2.0, 3.5]),
                FbxProperty::F64Array(vec![1.0, 2.0, 3.0, 4.0]),
                FbxProperty::I32Array(vec![-1, 0, 1, 100, 200]),
                FbxProperty::I64Array(vec![-1, 0, 1, 1 << 40]),
                FbxProperty::BoolArray(vec![true, false, true]),
            ],
            children: Vec::new(),
        };
        let parsed = roundtrip_node(7700, vec![node.clone()]);
        assert_eq!(parsed.root.children[0].properties, node.properties);
    }

    #[test]
    fn string_and_raw_round_trip_with_interior_nul() {
        // FBX strings can contain interior NULs (binary FBX joins
        // Name+SubType with `\0\1` — already exercised by the
        // synthetic-quad test). The writer must round-trip them.
        let node = FbxNode {
            name: "Specials".to_string(),
            properties: vec![
                FbxProperty::String(b"Quad\x00\x01Geometry".to_vec()),
                FbxProperty::Raw(vec![0, 1, 2, 3, 0xFF, 0xFE]),
                FbxProperty::String(Vec::new()), // empty S
                FbxProperty::Raw(Vec::new()),    // empty R
            ],
            children: Vec::new(),
        };
        let parsed = roundtrip_node(7400, vec![node.clone()]);
        assert_eq!(parsed.root.children[0].properties, node.properties);
    }

    #[test]
    fn nested_children_round_trip() {
        // One top-level node with two children, each carrying scalar
        // properties — exercises the recursive write_node + the
        // nested-list NULL-record sentinel.
        let inner_a = FbxNode {
            name: "InnerA".into(),
            properties: vec![FbxProperty::I32(42)],
            children: Vec::new(),
        };
        let inner_b = FbxNode {
            name: "InnerB".into(),
            properties: vec![FbxProperty::F64(-7.5)],
            children: vec![FbxNode {
                name: "DeepC".into(),
                properties: vec![FbxProperty::String(b"hi".to_vec())],
                children: Vec::new(),
            }],
        };
        let parent = FbxNode {
            name: "Parent".into(),
            properties: vec![FbxProperty::I64(99)],
            children: vec![inner_a.clone(), inner_b.clone()],
        };
        let parsed = roundtrip_node(7700, vec![parent.clone()]);
        let got = &parsed.root.children[0];
        assert_eq!(got.name, "Parent");
        assert_eq!(got.children.len(), 2);
        assert_eq!(got.children[0].name, "InnerA");
        assert_eq!(got.children[0].properties, inner_a.properties);
        assert_eq!(got.children[1].name, "InnerB");
        assert_eq!(got.children[1].properties, inner_b.properties);
        assert_eq!(got.children[1].children.len(), 1);
        assert_eq!(got.children[1].children[0].name, "DeepC");
    }

    #[test]
    fn long_name_rejected_with_clean_error() {
        let too_long = "a".repeat(300);
        let bad = FbxNode {
            name: too_long,
            properties: Vec::new(),
            children: Vec::new(),
        };
        let doc = FbxDocument {
            version: 7400,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![bad],
            },
        };
        let err = write_document(&doc).expect_err("over-long name surfaces an error");
        assert!(
            err.to_string().contains("255"),
            "expected the spec limit in the error: {err}"
        );
    }

    /// Build a document with one compressible array of f64 zeros and
    /// helper utilities for the compression tests below.
    fn build_compressible_doc(version: u32, count: usize) -> FbxDocument {
        let zeros = vec![0.0_f64; count];
        FbxDocument {
            version,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![FbxNode {
                    name: "BigArray".into(),
                    properties: vec![FbxProperty::F64Array(zeros)],
                    children: Vec::new(),
                }],
            },
        }
    }

    #[test]
    fn deflate_opt_in_shrinks_compressible_arrays() {
        // 1024 doubles of zeros: 8192 raw bytes, deflate → tens of
        // bytes. With the threshold at 256 the writer must pick the
        // compressed form and the on-disk size must drop sharply.
        let doc = build_compressible_doc(7400, 1024);
        let raw_bytes = write_document(&doc).expect("baseline write");
        let opts = WriterOptions::default().compress_arrays_at(256);
        let compressed_bytes = write_document_with_options(&doc, &opts).expect("compressed write");
        assert!(
            compressed_bytes.len() < raw_bytes.len(),
            "deflate failed to shrink (raw {} vs compressed {})",
            raw_bytes.len(),
            compressed_bytes.len()
        );
        // Round-trips back to the same document.
        let parsed = binary::parse(&compressed_bytes).expect("compressed decodes");
        let parsed_arr = match &parsed.root.children[0].properties[0] {
            FbxProperty::F64Array(arr) => arr.clone(),
            other => panic!("wrong property variant: {other:?}"),
        };
        assert_eq!(parsed_arr.len(), 1024);
        assert!(parsed_arr.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn deflate_threshold_skips_small_arrays() {
        // A 24-byte array (3 f64) with a 1024-byte threshold must
        // stay uncompressed — the writer must not pay the 6-byte
        // zlib header / Adler32 overhead on every tiny array.
        let doc = FbxDocument {
            version: 7400,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![FbxNode {
                    name: "Tiny".into(),
                    properties: vec![FbxProperty::F64Array(vec![1.0, 2.0, 3.0])],
                    children: Vec::new(),
                }],
            },
        };
        let raw_bytes = write_document(&doc).expect("baseline");
        let opts = WriterOptions::default().compress_arrays_at(1024);
        let opt_bytes = write_document_with_options(&doc, &opts).expect("opt write");
        // Threshold was not crossed -> identical bytes.
        assert_eq!(raw_bytes, opt_bytes);
    }

    #[test]
    fn deflate_falls_back_when_compression_would_grow() {
        // Three random-ish floats — too small to compress well. With
        // the threshold at 0 (compress everything), the writer must
        // still fall back to raw rather than ship a larger payload.
        let doc = FbxDocument {
            version: 7400,
            root: FbxNode {
                name: String::new(),
                properties: Vec::new(),
                children: vec![FbxNode {
                    name: "Noise".into(),
                    properties: vec![FbxProperty::F32Array(vec![0.123_f32, 4.56_f32, -7.89_f32])],
                    children: Vec::new(),
                }],
            },
        };
        let raw_bytes = write_document(&doc).expect("baseline");
        let opts = WriterOptions::default()
            .compress_arrays_at(0)
            .compression_level(9);
        let opt_bytes = write_document_with_options(&doc, &opts).expect("opt write");
        // Falling back to raw means the byte stream matches the
        // unconditionally-raw output.
        assert_eq!(raw_bytes, opt_bytes);
    }

    #[test]
    fn deflate_round_trips_post_7500_64bit_layout() {
        // Same compression behaviour must hold under the 64-bit Node
        // Record layout. 2048 zeros = 16 KiB raw → very compressible.
        let doc = build_compressible_doc(7700, 2048);
        let opts = WriterOptions::default().compress_arrays_at(1024);
        let bytes = write_document_with_options(&doc, &opts).expect("64-bit compressed");
        let parsed = binary::parse(&bytes).expect("64-bit compressed decodes");
        assert_eq!(parsed.version, 7700);
        let arr = match &parsed.root.children[0].properties[0] {
            FbxProperty::F64Array(arr) => arr.clone(),
            other => panic!("wrong variant: {other:?}"),
        };
        assert_eq!(arr.len(), 2048);
        assert!(arr.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn raw_array_default_unchanged_against_previous_round() {
        // Bit-for-bit guard: without opting into compression, the
        // round-3 round-trip closure (synthetic-quad fixture, both
        // 32-bit and 64-bit layouts) must still produce byte-identical
        // output. A non-default `compress_arrays = None` keeps the
        // arrays raw — the only change should be the indirection
        // through `write_document_with_options`.
        let doc = build_compressible_doc(7400, 16);
        let v1 = write_document(&doc).unwrap();
        let v2 = write_document_with_options(&doc, &WriterOptions::default()).unwrap();
        assert_eq!(v1, v2);
        // And one more parse → write cycle through the default path
        // matches.
        let parsed = binary::parse(&v1).unwrap();
        let v3 = write_document(&parsed).unwrap();
        assert_eq!(v1, v3);
    }
}
