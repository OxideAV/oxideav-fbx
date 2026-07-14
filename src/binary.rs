//! Binary FBX container reader.
//!
//! Parses the 27-byte header + recursive Node Record tree defined in
//! Alexander Gessler / Blender Foundation, *FBX Binary File Format
//! Specification* (August 2013, public-domain dedication; staged at
//! `docs/3d/fbx/blender-fbx-binary-format.html`).
//!
//! The output is a typed [`FbxNode`] tree: every node has a UTF-8
//! [`FbxNode::name`], a flat list of [`FbxProperty`] values, and an
//! ordered list of nested [`FbxNode`] children. The reader is
//! intentionally agnostic about object-graph semantics — that's the
//! [`crate::scene`] module's job.
//!
//! # Version-dependent layout
//!
//! Headers carry a 32-bit `Version` at offset 23 (LE). For
//! `Version >= 7500` the per-record `EndOffset`, `NumProperties`, and
//! `PropertyListLen` widen from 32-bit to 64-bit; the `NameLen` byte
//! and the body layout are unchanged. The reader auto-selects based
//! on the parsed version.
//!
//! # Property type codes
//!
//! Per Gessler §"Property Record Format":
//!
//! | Code | Type |
//! |------|------|
//! | `Y`  | i16 |
//! | `C`  | bool (LSB of one byte) |
//! | `I`  | i32 |
//! | `F`  | f32 |
//! | `D`  | f64 |
//! | `L`  | i64 |
//! | `f`  | array of f32 |
//! | `i`  | array of i32 |
//! | `d`  | array of f64 |
//! | `l`  | array of i64 |
//! | `b`  | array of bool (1 byte each) |
//! | `S`  | length-prefixed bytes (UTF-8 strings, may contain `\0`) |
//! | `R`  | raw binary blob |
//!
//! Array contents may be zlib-deflated (Encoding == 1); the reader
//! transparently decompresses via the pure-Rust `compcol` zlib codec.

use oxideav_mesh3d::{Error, Result};

/// FBX binary file magic: `b"Kaydara FBX Binary  \x00"` (20 bytes
/// including the trailing NUL).
pub const FBX_MAGIC: &[u8] = b"Kaydara FBX Binary  \x00";

/// Two "unknown" bytes immediately after the magic, observed in every
/// well-formed binary FBX (`0x1A 0x00`).
pub const FBX_MAGIC_TAIL: &[u8] = &[0x1A, 0x00];

/// Total header length: 20-byte magic + 2-byte tail + 4-byte version.
pub const FBX_HEADER_BYTES: usize = 27;

/// Version threshold for the 64-bit Node Record layout (per Gessler
/// §"Version-dependent quirks").
pub const FBX_VERSION_64BIT_THRESHOLD: u32 = 7500;

/// One property of an FBX node, fully decoded.
///
/// Variants are 1:1 with the property type codes documented in
/// Gessler §"Property Record Format". Strings stay as `Vec<u8>` —
/// FBX strings are not zero-terminated and may contain interior `\0`,
/// so callers that want `&str` should validate with `from_utf8` at
/// the call site.
#[derive(Clone, Debug, PartialEq)]
pub enum FbxProperty {
    /// `Y` — 2-byte signed integer.
    I16(i16),
    /// `C` — 1-bit boolean (LSB of one byte).
    Bool(bool),
    /// `I` — 4-byte signed integer.
    I32(i32),
    /// `F` — 32-bit IEEE 754 single.
    F32(f32),
    /// `D` — 64-bit IEEE 754 double.
    F64(f64),
    /// `L` — 8-byte signed integer.
    I64(i64),
    /// `f` — array of f32.
    F32Array(Vec<f32>),
    /// `d` — array of f64.
    F64Array(Vec<f64>),
    /// `l` — array of i64.
    I64Array(Vec<i64>),
    /// `i` — array of i32.
    I32Array(Vec<i32>),
    /// `b` — array of bools (1 byte per element).
    BoolArray(Vec<bool>),
    /// `S` — length-prefixed string, raw bytes (NOT NUL-terminated).
    String(Vec<u8>),
    /// `R` — length-prefixed raw binary blob.
    Raw(Vec<u8>),
}

impl FbxProperty {
    /// Convert an `S` property to a borrowed `&str`. Returns `None`
    /// when the property is a different variant or the bytes are not
    /// valid UTF-8.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Self::String(bytes) => std::str::from_utf8(bytes).ok(),
            _ => None,
        }
    }

    /// Convert a numeric scalar property to `i64` for ergonomic
    /// access. `f32` / `f64` values truncate towards zero; non-numeric
    /// variants (`String`, `Raw`, arrays) return `None`.
    pub fn as_i64(&self) -> Option<i64> {
        match *self {
            Self::I16(v) => Some(v as i64),
            Self::I32(v) => Some(v as i64),
            Self::I64(v) => Some(v),
            Self::F32(v) => Some(v as i64),
            Self::F64(v) => Some(v as i64),
            Self::Bool(v) => Some(v as i64),
            _ => None,
        }
    }
}

/// One Node Record in the FBX binary tree.
///
/// `name` is the UTF-8-decoded node name (Gessler "Name" field); all
/// known FBX node names are pure ASCII so we surface them as `String`
/// rather than raw bytes.
#[derive(Clone, Debug, Default)]
pub struct FbxNode {
    pub name: String,
    pub properties: Vec<FbxProperty>,
    pub children: Vec<FbxNode>,
}

impl FbxNode {
    /// Find the first direct child with the given `name`.
    pub fn child(&self, name: &str) -> Option<&FbxNode> {
        self.children.iter().find(|c| c.name == name)
    }

    /// All direct children with the given `name`.
    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a FbxNode> + 'a {
        self.children.iter().filter(move |c| c.name == name)
    }
}

/// Result of parsing a binary FBX file: the root node (a synthetic
/// container with empty name whose children are the top-level
/// records like `FBXHeaderExtension`, `Objects`, `Connections`, ...)
/// and the file-format version.
#[derive(Clone, Debug)]
pub struct FbxDocument {
    pub version: u32,
    pub root: FbxNode,
}

/// Parse a binary-FBX byte buffer.
pub fn parse(bytes: &[u8]) -> Result<FbxDocument> {
    if bytes.len() < FBX_HEADER_BYTES {
        return Err(Error::invalid(format!(
            "binary FBX truncated: need {} header bytes, got {}",
            FBX_HEADER_BYTES,
            bytes.len()
        )));
    }
    if &bytes[..FBX_MAGIC.len()] != FBX_MAGIC {
        return Err(Error::invalid(
            "binary FBX magic mismatch: expected `Kaydara FBX Binary  \\0`",
        ));
    }
    let tail_off = FBX_MAGIC.len();
    if &bytes[tail_off..tail_off + FBX_MAGIC_TAIL.len()] != FBX_MAGIC_TAIL {
        return Err(Error::invalid(
            "binary FBX magic-tail mismatch: expected 0x1A 0x00 at offset 20",
        ));
    }
    let version = u32::from_le_bytes([bytes[23], bytes[24], bytes[25], bytes[26]]);
    let use_64bit = version >= FBX_VERSION_64BIT_THRESHOLD;

    let mut cur = FBX_HEADER_BYTES;
    let mut root = FbxNode::default();
    loop {
        if cur >= bytes.len() {
            // Some FBX files end without an explicit final NULL-record
            // (Blender's writer sometimes omits it past the last
            // top-level record). Tolerate gracefully.
            break;
        }
        // Peek the record header to detect the all-zero NULL-record
        // sentinel that terminates the top-level list.
        let header_bytes = if use_64bit { 25 } else { 13 };
        if cur + header_bytes > bytes.len() {
            break;
        }
        if bytes[cur..cur + header_bytes].iter().all(|&b| b == 0) {
            // End-of-list NULL record consumed; we're done with the
            // top-level sequence.
            break;
        }
        let (node, next) = read_node(bytes, cur, use_64bit, 0)?;
        root.children.push(node);
        cur = next;
    }
    Ok(FbxDocument { version, root })
}

/// Maximum Node Record nesting depth the reader accepts.
///
/// The record tree is parsed recursively, so an unbounded depth lets a
/// crafted file (every record's nested list holding exactly one more
/// record — ~14 bytes per level) overflow the parser's stack: an
/// uncatchable abort, not an `Err`. Real FBX documents nest single-digit
/// levels (the staged fixtures peak at 4 — `Objects / Geometry /
/// LayerElementNormal / Normals`), so 128 is far beyond any legitimate
/// file while keeping worst-case stack use trivially small.
pub const MAX_NODE_DEPTH: usize = 128;

/// Read one Node Record starting at `bytes[off]` and return the
/// parsed node plus the file offset of the byte immediately past the
/// record.
fn read_node(bytes: &[u8], off: usize, use_64bit: bool, depth: usize) -> Result<(FbxNode, usize)> {
    if depth >= MAX_NODE_DEPTH {
        return Err(Error::invalid(format!(
            "binary FBX: node nesting exceeds the {MAX_NODE_DEPTH}-level limit"
        )));
    }
    // Header layout per Gessler:
    //   <= 7400:  EndOffset(u32) | NumProperties(u32) | PropertyListLen(u32) | NameLen(u8)
    //   >= 7500:  EndOffset(u64) | NumProperties(u64) | PropertyListLen(u64) | NameLen(u8)
    let mut p = off;
    let (end_offset, num_props, prop_list_len) = if use_64bit {
        let eo = read_u64(bytes, p)?;
        let np = read_u64(bytes, p + 8)?;
        let pl = read_u64(bytes, p + 16)?;
        p += 24;
        (eo as usize, np as usize, pl as usize)
    } else {
        let eo = read_u32(bytes, p)?;
        let np = read_u32(bytes, p + 4)?;
        let pl = read_u32(bytes, p + 8)?;
        p += 12;
        (eo as usize, np as usize, pl as usize)
    };
    if end_offset == 0 {
        // NULL-record sentinel inside a nested list — terminator, not
        // a real node. Caller handles this via the alternative
        // `peek_null` path; reaching this branch from `read_node`
        // means the caller mis-routed.
        return Err(Error::invalid(
            "binary FBX: read_node entered on a NULL-record sentinel",
        ));
    }
    if end_offset > bytes.len() {
        return Err(Error::invalid(format!(
            "binary FBX: node EndOffset {} past file length {}",
            end_offset,
            bytes.len()
        )));
    }
    let name_len = read_u8(bytes, p)? as usize;
    p += 1;
    if p + name_len > bytes.len() {
        return Err(Error::invalid("binary FBX: node Name extends past EOF"));
    }
    let name = std::str::from_utf8(&bytes[p..p + name_len])
        .map_err(|e| Error::invalid(format!("binary FBX: node Name not UTF-8: {e}")))?
        .to_string();
    p += name_len;

    // Properties.
    //
    // `num_props` is header-controlled (u32 / u64), so a hostile value
    // must not drive `Vec::with_capacity` directly — pre-fix, a crafted
    // `NumProperties = u32::MAX` requested a multi-GiB allocation
    // before the first property read could fail. The smallest encoded
    // property is 2 bytes (`C` — one type code + one byte), so a valid
    // count can never exceed half the declared property-list length,
    // itself capped by the bytes actually remaining in the buffer; the
    // parse loop still errors cleanly if the count is a lie.
    let prop_start = p;
    let capacity = num_props
        .min(prop_list_len / 2)
        .min(bytes.len().saturating_sub(p) / 2);
    let mut properties = Vec::with_capacity(capacity);
    for _ in 0..num_props {
        let (prop, next) = read_property(bytes, p)?;
        properties.push(prop);
        p = next;
    }
    if p - prop_start != prop_list_len {
        return Err(Error::invalid(format!(
            "binary FBX: PropertyListLen mismatch on `{name}` — header said {prop_list_len}, parser consumed {}",
            p - prop_start
        )));
    }

    // Nested list (optional). Presence is signalled by there being
    // unconsumed bytes between `p` and `end_offset`. If present, the
    // list is a sequence of node records terminated by a NULL-record
    // sentinel (13 bytes pre-7500, 25 bytes post-7500).
    let mut children = Vec::new();
    if p < end_offset {
        let null_record_bytes = if use_64bit { 25 } else { 13 };
        loop {
            if p + null_record_bytes > end_offset {
                return Err(Error::invalid(format!(
                    "binary FBX: nested list on `{name}` ran past EndOffset before NULL-record"
                )));
            }
            // Check for the NULL-record sentinel at `p`.
            if bytes[p..p + null_record_bytes].iter().all(|&b| b == 0) {
                p += null_record_bytes;
                break;
            }
            let (child, next) = read_node(bytes, p, use_64bit, depth + 1)?;
            children.push(child);
            p = next;
        }
    }
    if p != end_offset {
        return Err(Error::invalid(format!(
            "binary FBX: node `{name}` consumed up to {p} but EndOffset is {end_offset}"
        )));
    }
    Ok((
        FbxNode {
            name,
            properties,
            children,
        },
        end_offset,
    ))
}

/// Read one [`FbxProperty`] starting at `bytes[off]`. Returns the
/// decoded property and the offset of the byte immediately past it.
fn read_property(bytes: &[u8], off: usize) -> Result<(FbxProperty, usize)> {
    let type_code = read_u8(bytes, off)?;
    let mut p = off + 1;
    let prop = match type_code {
        // -- Scalars (Gessler §"Primitive Types") --
        b'Y' => {
            let v = read_i16(bytes, p)?;
            p += 2;
            FbxProperty::I16(v)
        }
        b'C' => {
            let raw = read_u8(bytes, p)?;
            p += 1;
            FbxProperty::Bool((raw & 1) != 0)
        }
        b'I' => {
            let v = read_i32(bytes, p)?;
            p += 4;
            FbxProperty::I32(v)
        }
        b'F' => {
            let v = read_f32(bytes, p)?;
            p += 4;
            FbxProperty::F32(v)
        }
        b'D' => {
            let v = read_f64(bytes, p)?;
            p += 8;
            FbxProperty::F64(v)
        }
        b'L' => {
            let v = read_i64(bytes, p)?;
            p += 8;
            FbxProperty::I64(v)
        }
        // -- Arrays (Gessler §"Array types") --
        b'f' => {
            let (data, next) = read_array_payload(bytes, p, 4)?;
            p = next;
            let mut out = Vec::with_capacity(data.len() / 4);
            for chunk in data.chunks_exact(4) {
                out.push(f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
            }
            FbxProperty::F32Array(out)
        }
        b'd' => {
            let (data, next) = read_array_payload(bytes, p, 8)?;
            p = next;
            let mut out = Vec::with_capacity(data.len() / 8);
            for chunk in data.chunks_exact(8) {
                out.push(f64::from_le_bytes([
                    chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7],
                ]));
            }
            FbxProperty::F64Array(out)
        }
        b'l' => {
            let (data, next) = read_array_payload(bytes, p, 8)?;
            p = next;
            let mut out = Vec::with_capacity(data.len() / 8);
            for chunk in data.chunks_exact(8) {
                out.push(i64::from_le_bytes([
                    chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7],
                ]));
            }
            FbxProperty::I64Array(out)
        }
        b'i' => {
            let (data, next) = read_array_payload(bytes, p, 4)?;
            p = next;
            let mut out = Vec::with_capacity(data.len() / 4);
            for chunk in data.chunks_exact(4) {
                out.push(i32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]));
            }
            FbxProperty::I32Array(out)
        }
        b'b' => {
            let (data, next) = read_array_payload(bytes, p, 1)?;
            p = next;
            let out: Vec<bool> = data.iter().map(|&b| b != 0).collect();
            FbxProperty::BoolArray(out)
        }
        // -- Special types (Gessler §"Special types") --
        b'S' => {
            let len = read_u32(bytes, p)? as usize;
            p += 4;
            if p + len > bytes.len() {
                return Err(Error::invalid("binary FBX: S property runs past EOF"));
            }
            let bytes_out = bytes[p..p + len].to_vec();
            p += len;
            FbxProperty::String(bytes_out)
        }
        b'R' => {
            let len = read_u32(bytes, p)? as usize;
            p += 4;
            if p + len > bytes.len() {
                return Err(Error::invalid("binary FBX: R property runs past EOF"));
            }
            let bytes_out = bytes[p..p + len].to_vec();
            p += len;
            FbxProperty::Raw(bytes_out)
        }
        other => {
            return Err(Error::invalid(format!(
                "binary FBX: unknown property type code `{}` (0x{:02x}) at offset {}",
                other as char, other, off
            )));
        }
    };
    Ok((prop, p))
}

/// Read an array property payload (`ArrayLength` / `Encoding` /
/// `CompressedLength` / `Contents`). Returns the *uncompressed* byte
/// buffer plus the offset just past the entire array record.
fn read_array_payload(bytes: &[u8], off: usize, elem_bytes: usize) -> Result<(Vec<u8>, usize)> {
    let array_length = read_u32(bytes, off)? as usize;
    let encoding = read_u32(bytes, off + 4)?;
    let comp_length = read_u32(bytes, off + 8)? as usize;
    let payload_off = off + 12;
    let raw_size = array_length
        .checked_mul(elem_bytes)
        .ok_or_else(|| Error::invalid("binary FBX: array_length * elem_bytes overflow"))?;
    let data = match encoding {
        0 => {
            if payload_off + raw_size > bytes.len() {
                return Err(Error::invalid(
                    "binary FBX: uncompressed array runs past EOF",
                ));
            }
            let out = bytes[payload_off..payload_off + raw_size].to_vec();
            (out, payload_off + raw_size)
        }
        1 => {
            if payload_off + comp_length > bytes.len() {
                return Err(Error::invalid("binary FBX: compressed array runs past EOF"));
            }
            let comp = &bytes[payload_off..payload_off + comp_length];
            // The post-inflate length is known up-front (`raw_size`), so
            // cap the decoder at exactly that — a corrupt/malicious
            // CompressedLength cannot expand into a decompression bomb.
            let inflated = compcol::vec::decompress_to_vec_capped::<compcol::zlib::Zlib>(
                comp,
                raw_size as u64,
            )
            .map_err(|e| Error::invalid(format!("binary FBX: zlib inflate failed ({e:?})")))?;
            if inflated.len() != raw_size {
                return Err(Error::invalid(format!(
                    "binary FBX: inflated array length mismatch — header said {} elements ({} bytes), got {} bytes",
                    array_length, raw_size, inflated.len()
                )));
            }
            (inflated, payload_off + comp_length)
        }
        other => {
            return Err(Error::invalid(format!(
                "binary FBX: unknown array encoding {other} (only 0 / 1 are documented)"
            )));
        }
    };
    Ok(data)
}

// -- Little-endian primitive readers with bounds checks --

fn read_u8(bytes: &[u8], off: usize) -> Result<u8> {
    bytes
        .get(off)
        .copied()
        .ok_or_else(|| Error::invalid(format!("binary FBX: u8 read past EOF at {off}")))
}

fn read_i16(bytes: &[u8], off: usize) -> Result<i16> {
    if off + 2 > bytes.len() {
        return Err(Error::invalid(format!(
            "binary FBX: i16 read past EOF at {off}"
        )));
    }
    Ok(i16::from_le_bytes([bytes[off], bytes[off + 1]]))
}

fn read_u32(bytes: &[u8], off: usize) -> Result<u32> {
    if off + 4 > bytes.len() {
        return Err(Error::invalid(format!(
            "binary FBX: u32 read past EOF at {off}"
        )));
    }
    Ok(u32::from_le_bytes([
        bytes[off],
        bytes[off + 1],
        bytes[off + 2],
        bytes[off + 3],
    ]))
}

fn read_u64(bytes: &[u8], off: usize) -> Result<u64> {
    if off + 8 > bytes.len() {
        return Err(Error::invalid(format!(
            "binary FBX: u64 read past EOF at {off}"
        )));
    }
    Ok(u64::from_le_bytes([
        bytes[off],
        bytes[off + 1],
        bytes[off + 2],
        bytes[off + 3],
        bytes[off + 4],
        bytes[off + 5],
        bytes[off + 6],
        bytes[off + 7],
    ]))
}

fn read_i32(bytes: &[u8], off: usize) -> Result<i32> {
    read_u32(bytes, off).map(|v| v as i32)
}

fn read_i64(bytes: &[u8], off: usize) -> Result<i64> {
    read_u64(bytes, off).map(|v| v as i64)
}

fn read_f32(bytes: &[u8], off: usize) -> Result<f32> {
    read_u32(bytes, off).map(f32::from_bits)
}

fn read_f64(bytes: &[u8], off: usize) -> Result<f64> {
    read_u64(bytes, off).map(f64::from_bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid binary FBX file with a single empty root
    /// list (just the trailing NULL-record). This lets tests exercise
    /// the header path without depending on a particular node
    /// arrangement.
    fn build_empty_doc(version: u32) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(FBX_MAGIC);
        out.extend_from_slice(FBX_MAGIC_TAIL);
        out.extend_from_slice(&version.to_le_bytes());
        // Empty top-level list — the parser tolerates EOF here.
        out
    }

    #[test]
    fn header_round_trip_pre_7500() {
        let buf = build_empty_doc(7400);
        let doc = parse(&buf).expect("empty 7400 doc parses");
        assert_eq!(doc.version, 7400);
        assert!(doc.root.children.is_empty());
    }

    #[test]
    fn header_round_trip_post_7500() {
        let buf = build_empty_doc(7700);
        let doc = parse(&buf).expect("empty 7700 doc parses");
        assert_eq!(doc.version, 7700);
        assert!(doc.root.children.is_empty());
    }

    #[test]
    fn rejects_bad_magic() {
        let mut buf = build_empty_doc(7400);
        buf[0] = b'X';
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn rejects_truncated_header() {
        let buf = vec![0u8; 10];
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn rejects_bad_magic_tail() {
        let mut buf = build_empty_doc(7400);
        buf[21] = 0xFF;
        assert!(parse(&buf).is_err());
    }

    /// Append one 32-bit node-record header (+ name) to `out`.
    fn push_node_header_32(
        out: &mut Vec<u8>,
        end_offset: u32,
        num_props: u32,
        prop_list_len: u32,
        name: &str,
    ) {
        out.extend_from_slice(&end_offset.to_le_bytes());
        out.extend_from_slice(&num_props.to_le_bytes());
        out.extend_from_slice(&prop_list_len.to_le_bytes());
        out.push(name.len() as u8);
        out.extend_from_slice(name.as_bytes());
    }

    #[test]
    fn truncated_i16_property_errors_instead_of_panicking() {
        // Round 413 hardening — a `Y` property whose two payload bytes
        // are cut short by EOF previously indexed past the buffer
        // (panic). EndOffset is kept within the (truncated) file so
        // the property read itself is the first thing to fail.
        let mut buf = build_empty_doc(7400);
        // Record: 13-byte header + 1-byte name + 'Y' + ONE byte (the
        // second payload byte is missing). EndOffset = 27+16 = 43 ==
        // final file length, so the offset checks pass.
        push_node_header_32(&mut buf, 43, 1, 3, "A");
        buf.push(b'Y');
        buf.push(0x07);
        let err = parse(&buf).expect_err("truncated Y errors");
        assert!(
            err.to_string().contains("i16"),
            "expected the bounds-checked i16 read to fire, got: {err}"
        );
    }

    #[test]
    fn hostile_num_properties_does_not_preallocate() {
        // Round 413 hardening — `NumProperties` is header-controlled;
        // u32::MAX previously drove `Vec::with_capacity` into a
        // multi-GiB allocation request before the first property read
        // could fail. The clamped capacity keeps this an ordinary
        // parse error (and the test completes without an OOM abort).
        let mut buf = build_empty_doc(7400);
        push_node_header_32(&mut buf, 43, u32::MAX, 3, "A");
        buf.push(b'Y');
        buf.push(0x07);
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn nesting_depth_bomb_errors_instead_of_overflowing_the_stack() {
        // Round 413 hardening — each nesting level costs ~14 bytes, so
        // a small crafted file previously drove the recursive reader
        // thousands of frames deep (uncatchable stack-overflow abort).
        // Build 10_000 nested records: N headers front-to-back, then
        // the (N-1) NULL sentinels that close every outer body, with
        // absolute EndOffsets computed from the fixed record sizes.
        const N: u32 = 10_000;
        let mut buf = build_empty_doc(7400);
        // Innermost record body ends right after its name; each outer
        // record additionally holds its child + one 13-byte NULL.
        // end(k) for the k-th header (0-based, outermost first):
        //   end(N-1) = 27 + 14*N
        //   end(k)   = end(k+1) + 13
        let innermost_end = 27 + 14 * N;
        for k in 0..N {
            let end = innermost_end + 13 * (N - 1 - k);
            push_node_header_32(&mut buf, end, 0, 0, "A");
        }
        for _ in 0..N - 1 {
            buf.extend_from_slice(&[0u8; 13]);
        }
        let err = parse(&buf).expect_err("depth bomb rejected");
        assert!(
            err.to_string().contains("nesting"),
            "expected the depth limit to fire, got: {err}"
        );
    }

    #[test]
    fn fixture_depth_stays_well_under_the_limit() {
        // The staged fixtures parse fine under MAX_NODE_DEPTH — their
        // real nesting peaks at 4 levels (Objects / Geometry /
        // LayerElement* / data array).
        let bytes = include_bytes!("../tests/fixtures/cubes-ascii-v7500.fbx");
        // (ASCII fixture — depth applies to the binary reader, so
        // round-trip it through the binary writer first.)
        let doc = crate::ascii::parse(bytes).expect("fixture parses");
        let bin = crate::writer::write_document(&doc).expect("writes");
        let doc2 = parse(&bin).expect("re-parses under the depth limit");
        fn depth(n: &FbxNode) -> usize {
            1 + n.children.iter().map(depth).max().unwrap_or(0)
        }
        assert!(depth(&doc2.root) <= 8, "fixture depth sanity");
    }
}
