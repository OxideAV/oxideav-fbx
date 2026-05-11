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
//! Arrays are written **uncompressed** (`Encoding == 0`). The Gessler
//! doc allows both forms; uncompressed is bit-deterministic and
//! avoids any zlib version / level reproducibility surprise. Readers
//! that handle zlib-deflated arrays (every conformant parser, per the
//! type-code dispatch table) will also accept the uncompressed form.
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

/// Serialise an [`FbxDocument`] to a byte buffer that decodes back
/// through [`crate::binary::parse`] to an equivalent document.
///
/// The resulting buffer is the 27-byte header + every child of
/// [`FbxDocument::root`] written as a top-level Node Record, capped
/// by the format's all-zero NULL-record sentinel. **No Autodesk
/// footer is written** — see the module docs for the rationale.
pub fn write_document(doc: &FbxDocument) -> Result<Vec<u8>> {
    let use_64bit = doc.version >= FBX_VERSION_64BIT_THRESHOLD;
    let mut out = Vec::new();
    // 27-byte header: 20-byte magic + 0x1A 0x00 + version (LE u32).
    out.extend_from_slice(FBX_MAGIC);
    out.extend_from_slice(FBX_MAGIC_TAIL);
    out.extend_from_slice(&doc.version.to_le_bytes());
    debug_assert_eq!(out.len(), FBX_HEADER_BYTES);

    for child in &doc.root.children {
        write_node(child, &mut out, use_64bit)?;
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
fn write_node(node: &FbxNode, out: &mut Vec<u8>, use_64bit: bool) -> Result<()> {
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
        write_property(prop, out)?;
    }
    let prop_list_len = out.len() - prop_start;

    // Nested list: every child + the NULL-record sentinel. The
    // Gessler spec says the nested list is **omitted entirely** when
    // there are no children, so we only emit it when the vector is
    // populated. (Some loaders accept both forms; the omitted form is
    // what every well-formed exporter writes.)
    if !node.children.is_empty() {
        for child in &node.children {
            write_node(child, out, use_64bit)?;
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
fn write_property(prop: &FbxProperty, out: &mut Vec<u8>) -> Result<()> {
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
        // -- Arrays (Gessler §"Array types"). Always written
        //    uncompressed: ArrayLength | Encoding=0 | CompressedLength=0
        //    | raw little-endian element stream. --
        FbxProperty::F32Array(arr) => {
            out.push(b'f');
            write_array_header(out, arr.len());
            for v in arr {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        FbxProperty::F64Array(arr) => {
            out.push(b'd');
            write_array_header(out, arr.len());
            for v in arr {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        FbxProperty::I32Array(arr) => {
            out.push(b'i');
            write_array_header(out, arr.len());
            for v in arr {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        FbxProperty::I64Array(arr) => {
            out.push(b'l');
            write_array_header(out, arr.len());
            for v in arr {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        FbxProperty::BoolArray(arr) => {
            out.push(b'b');
            write_array_header(out, arr.len());
            // Each bool occupies one byte (Gessler §"Array types"
            // size table — `b` has elemSize = 1). Same `& 1` canon
            // as the scalar `C` variant.
            for &v in arr {
                out.push(if v { 1 } else { 0 });
            }
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

/// Write the per-array preamble — `ArrayLength | Encoding | CompressedLength`
/// — for an uncompressed array of `count` elements. The Gessler doc
/// permits both the raw (`Encoding == 0`) and zlib-deflated
/// (`Encoding == 1`) forms; we pick raw for byte-determinism.
fn write_array_header(out: &mut Vec<u8>, count: usize) {
    out.extend_from_slice(&(count as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // Encoding = 0 (raw)
    out.extend_from_slice(&0u32.to_le_bytes()); // CompressedLength = 0 (unused)
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
}
