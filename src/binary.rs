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
//! | `C`  | bool (one byte: the `T` token byte is true, else the LSB decides) |
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

/// The constant 16-byte signature that terminates a binary FBX file.
///
/// The Gessler writeup stops at *"after that record ... there is a
/// footer with unknown contents"*; the layout here is observer-derived
/// from the staged `docs/3d/fbx/fixtures/box-binary-v7400.fbx` bytes
/// (17200-byte file, footer region = the final 176 bytes past the
/// walk-end at offset 17024). Observed structure, in file order:
///
/// ```text
/// [top-level NULL record]      13 bytes (v<7500) / 25 bytes (v>=7500)
/// footer id                    16 bytes  (per-file; derivation unknown)
/// zero padding                 0..15 bytes, until the offset is
///                              16-byte aligned
/// zeros                        4 bytes
/// version echo                 uint32 LE — same value as header offset 23
/// zeros                        120 bytes
/// trailer signature            16 bytes  (this constant)
/// ```
///
/// In the fixture: id = `fa bc af 0f d2 c0 d8 63 b2 78 f4 89 14 f3 26
/// 75` (ends at offset 17053), 3 padding zeros reach the 16-byte
/// boundary at 17056, the version echo reads 7400 (`e8 1c 00 00`),
/// and the file ends with this trailer at 17184..17200. The id block
/// varies per file (its derivation is not documented by any staged
/// source); the trailer bytes are position-independent and are the
/// signature SDK-side validators check for.
pub const FOOTER_TRAILER: [u8; 16] = [
    0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5, 0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3, 0x75, 0x8f, 0x29, 0x0b,
];

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
    /// `C` — one-byte boolean. SDK-written files store the ASCII
    /// `T` / `F` token bytes (observed in the staged v7400 fixture);
    /// plain `0x00` / `0x01` forms decode via the LSB.
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
    /// `c` — array of raw bytes (1 byte per element). Part of the
    /// documented type-code alphabet (`docs/3d/fbx/README.md`
    /// "Property type codes"); the element width is pinned from the
    /// staged `fixtures/box-binary-v7500.fbx` bytes, whose
    /// `ImageData` record carries `ArrayLength = 12288` deflating to
    /// exactly 12288 bytes (1 byte/element). Kept distinct from
    /// [`FbxProperty::BoolArray`]: the observed payload is `0xff`
    /// pixel bytes, not booleans.
    ByteArray(Vec<u8>),
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

/// The decoded trailing footer block of a binary FBX file.
///
/// See [`FOOTER_TRAILER`] for the observer-derived layout. Only the
/// 16-byte per-file id needs capturing — the padding is recomputed
/// from the write position, the version echo repeats
/// [`FbxDocument::version`], and the trailer signature is the
/// constant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FbxFooter {
    /// The per-file 16-byte id block that opens the footer. Its
    /// derivation is undocumented by every staged source; parsers
    /// treat it as opaque bytes and byte-faithful re-encodes carry it
    /// verbatim.
    pub id: [u8; 16],
}

impl FbxFooter {
    /// Render the 16-byte id as a 32-char lowercase hex string (the
    /// form the decoder stashes on `Scene3D::extras["fbx:footer_id"]`).
    pub fn id_hex(&self) -> String {
        let mut s = String::with_capacity(32);
        for b in self.id {
            use std::fmt::Write;
            let _ = write!(s, "{b:02x}");
        }
        s
    }

    /// Parse a 32-char hex string back into the 16-byte id form.
    /// Returns `None` on any length / digit deviation.
    pub fn id_from_hex(s: &str) -> Option<[u8; 16]> {
        let bytes = s.as_bytes();
        if bytes.len() != 32 {
            return None;
        }
        let mut id = [0u8; 16];
        for (i, chunk) in bytes.chunks_exact(2).enumerate() {
            let hi = (chunk[0] as char).to_digit(16)?;
            let lo = (chunk[1] as char).to_digit(16)?;
            id[i] = ((hi << 4) | lo) as u8;
        }
        Some(id)
    }
}

/// Decode the trailing footer block of a binary FBX byte buffer.
///
/// Returns `Some` only when the full observer-derived shape (see
/// [`FOOTER_TRAILER`]) is present and well-formed: the top-level NULL
/// record, the 16-byte id, all-zero padding up to the 16-byte
/// boundary, the 4 zero bytes + version echo (matching the header
/// version) + 120 zero bytes, and the constant trailer signature
/// ending exactly at EOF. Any deviation — including files our own
/// pre-footer writer produced (which end at the NULL record) and
/// hostile/garbage tails — yields `None`, never an error or a panic.
///
/// This is a separate entry point rather than a field of
/// [`FbxDocument`] so the record-tree surface (and every existing
/// construction site) is unchanged; callers that care about
/// byte-faithful re-encoding pair [`parse`] with this.
pub fn parse_footer(bytes: &[u8]) -> Option<FbxFooter> {
    if bytes.len() < FBX_HEADER_BYTES
        || &bytes[..FBX_MAGIC.len()] != FBX_MAGIC
        || &bytes[FBX_MAGIC.len()..FBX_MAGIC.len() + FBX_MAGIC_TAIL.len()] != FBX_MAGIC_TAIL
    {
        return None;
    }
    let version = u32::from_le_bytes([bytes[23], bytes[24], bytes[25], bytes[26]]);
    let use_64bit = version >= FBX_VERSION_64BIT_THRESHOLD;
    let header_bytes = if use_64bit { 25 } else { 13 };

    // Hop over the top-level records via their EndOffset fields (no
    // property decoding needed) until the all-zero NULL record.
    let mut cur = FBX_HEADER_BYTES;
    loop {
        if cur + header_bytes > bytes.len() {
            return None;
        }
        if bytes[cur..cur + header_bytes].iter().all(|&b| b == 0) {
            cur += header_bytes;
            break;
        }
        let end_offset = if use_64bit {
            read_u64(bytes, cur).ok()? as usize
        } else {
            read_u32(bytes, cur).ok()? as usize
        };
        // A non-advancing or out-of-range EndOffset is malformed.
        if end_offset <= cur || end_offset > bytes.len() {
            return None;
        }
        cur = end_offset;
    }

    // 16-byte per-file id.
    let id: [u8; 16] = bytes.get(cur..cur + 16)?.try_into().ok()?;
    cur += 16;
    // Zero padding to the next 16-byte boundary (0..15 bytes; the
    // fixture shows 3). A non-zero pad byte breaks the shape.
    while cur % 16 != 0 {
        if *bytes.get(cur)? != 0 {
            return None;
        }
        cur += 1;
    }
    // 128-byte aligned block: 4 zeros | version echo | 120 zeros.
    let block = bytes.get(cur..cur + 128)?;
    if block[..4].iter().any(|&b| b != 0) {
        return None;
    }
    let echo = u32::from_le_bytes([block[4], block[5], block[6], block[7]]);
    if echo != version {
        return None;
    }
    if block[8..].iter().any(|&b| b != 0) {
        return None;
    }
    cur += 128;
    // Constant trailer signature, ending exactly at EOF.
    if bytes.get(cur..cur + 16)? != FOOTER_TRAILER {
        return None;
    }
    if cur + 16 != bytes.len() {
        return None;
    }
    Some(FbxFooter { id })
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
            // Gessler describes `C` as a 1-bit boolean in the LSB, but
            // the staged box-binary-v7400.fbx fixture's only `C`
            // property (`Shading`, whose ASCII counterpart is the bare
            // `Shading: T` true token of fbx-ascii-grammar.md §5)
            // stores the ASCII token byte 0x54 (`T`) — which the LSB
            // rule would misread as false. Observed rule: the `T`
            // token byte is true; otherwise the LSB decides (covers
            // the plain 0x00 / 0x01 encodings and reads the `F` token
            // byte 0x46 as false).
            FbxProperty::Bool(raw == b'T' || (raw & 1) != 0)
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
        b'c' => {
            let (data, next) = read_array_payload(bytes, p, 1)?;
            p = next;
            FbxProperty::ByteArray(data)
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

    /// Append the observer-derived footer block (top-level NULL
    /// record + id + alignment pad + version echo + trailer) to a
    /// buffer that currently ends just past its last top-level record.
    fn append_footer(out: &mut Vec<u8>, version: u32, id: [u8; 16]) {
        let null_bytes = if version >= FBX_VERSION_64BIT_THRESHOLD {
            25
        } else {
            13
        };
        out.extend(std::iter::repeat(0u8).take(null_bytes));
        out.extend_from_slice(&id);
        while out.len() % 16 != 0 {
            out.push(0);
        }
        out.extend_from_slice(&[0u8; 4]);
        out.extend_from_slice(&version.to_le_bytes());
        out.extend(std::iter::repeat(0u8).take(120));
        out.extend_from_slice(&FOOTER_TRAILER);
    }

    /// One tiny top-level record so the footer walk has something to
    /// hop over: `A` with a single `I` property.
    fn tiny_doc_with_footer(version: u32, id: [u8; 16]) -> Vec<u8> {
        let mut buf = build_empty_doc(version);
        if version >= FBX_VERSION_64BIT_THRESHOLD {
            let start = buf.len() as u64;
            let end = start + 8 * 3 + 1 + 1 + 5; // header + namelen + name + 'I' prop
            buf.extend_from_slice(&end.to_le_bytes());
            buf.extend_from_slice(&1u64.to_le_bytes());
            buf.extend_from_slice(&5u64.to_le_bytes());
            buf.push(1);
            buf.push(b'A');
        } else {
            let start = buf.len() as u32;
            let end = start + 4 * 3 + 1 + 1 + 5;
            buf.extend_from_slice(&end.to_le_bytes());
            buf.extend_from_slice(&1u32.to_le_bytes());
            buf.extend_from_slice(&5u32.to_le_bytes());
            buf.push(1);
            buf.push(b'A');
        }
        buf.push(b'I');
        buf.extend_from_slice(&7i32.to_le_bytes());
        append_footer(&mut buf, version, id);
        buf
    }

    #[test]
    fn bool_token_bytes_decode_per_the_fixture_observation() {
        // The staged v7400 fixture stores `Shading` (true) as the
        // ASCII token byte `T` (0x54) — the LSB rule alone would have
        // misread it as false. `F` (0x46) and 0x00 are false; the
        // plain 0x01 legacy form stays true.
        for (byte, expect) in [(b'T', true), (b'F', false), (0u8, false), (1u8, true)] {
            let mut buf = build_empty_doc(7400);
            push_node_header_32(&mut buf, 27 + 13 + 1 + 2, 1, 2, "B");
            buf.push(b'C');
            buf.push(byte);
            let doc = parse(&buf).expect("C prop parses");
            assert_eq!(
                doc.root.children[0].properties[0],
                FbxProperty::Bool(expect),
                "byte 0x{byte:02x}"
            );
        }
    }

    #[test]
    fn footer_round_trips_pre_7500() {
        let id = [
            0xfa, 0xbc, 0xaf, 0x0f, 0xd2, 0xc0, 0xd8, 0x63, 0xb2, 0x78, 0xf4, 0x89, 0x14, 0xf3,
            0x26, 0x75,
        ];
        let buf = tiny_doc_with_footer(7400, id);
        // The record tree still parses (footer bytes are past the
        // NULL record the tree walk stops at).
        let doc = parse(&buf).expect("doc with footer parses");
        assert_eq!(doc.root.children.len(), 1);
        let footer = parse_footer(&buf).expect("footer decodes");
        assert_eq!(footer.id, id);
    }

    #[test]
    fn footer_round_trips_post_7500_64bit_null_record() {
        let id = [0x11; 16];
        let buf = tiny_doc_with_footer(7700, id);
        let footer = parse_footer(&buf).expect("64-bit footer decodes");
        assert_eq!(footer.id, id);
    }

    #[test]
    fn missing_footer_is_none_not_an_error() {
        // Our own pre-footer writer output shape: ends at the NULL
        // record.
        let mut buf = build_empty_doc(7400);
        buf.extend_from_slice(&[0u8; 13]);
        assert_eq!(parse_footer(&buf), None);
        // And a bare header with no NULL record at all.
        assert_eq!(parse_footer(&build_empty_doc(7400)), None);
    }

    #[test]
    fn footer_version_echo_mismatch_is_rejected() {
        let mut buf = tiny_doc_with_footer(7400, [0x22; 16]);
        // The version echo sits 120 + 16 - 4 ... locate it from the
        // end instead: trailer(16) + zeros(120) + echo(4) => echo at
        // len-140..len-136.
        let n = buf.len();
        buf[n - 140..n - 136].copy_from_slice(&7500u32.to_le_bytes());
        assert_eq!(parse_footer(&buf), None);
    }

    #[test]
    fn footer_bad_trailer_signature_is_rejected() {
        let mut buf = tiny_doc_with_footer(7400, [0x22; 16]);
        let n = buf.len();
        buf[n - 1] ^= 0xff;
        assert_eq!(parse_footer(&buf), None);
    }

    #[test]
    fn footer_trailing_garbage_after_signature_is_rejected() {
        let mut buf = tiny_doc_with_footer(7400, [0x22; 16]);
        buf.push(0);
        assert_eq!(parse_footer(&buf), None);
    }

    #[test]
    fn footer_nonzero_alignment_pad_is_rejected() {
        // tiny_doc_with_footer's id ends at 27 + 16 + 6 + 16 = 65,
        // so alignment padding is present (65 % 16 != 0). Corrupt the
        // first pad byte.
        let id = [0x33; 16];
        let mut buf = build_empty_doc(7400);
        buf.extend_from_slice(&[0u8; 13]); // NULL record at 27
        buf.extend_from_slice(&id); // id 40..56
        assert_ne!(buf.len() % 16, 0, "test premise: padding exists");
        let pad_at = buf.len();
        while buf.len() % 16 != 0 {
            buf.push(0);
        }
        buf.extend_from_slice(&[0u8; 4]);
        buf.extend_from_slice(&7400u32.to_le_bytes());
        buf.extend(std::iter::repeat(0u8).take(120));
        buf.extend_from_slice(&FOOTER_TRAILER);
        assert!(parse_footer(&buf).is_some(), "well-formed shape decodes");
        buf[pad_at] = 1;
        assert_eq!(parse_footer(&buf), None);
    }

    #[test]
    fn footer_id_hex_round_trips() {
        let id = [
            0xfa, 0xbc, 0xaf, 0x0f, 0xd2, 0xc0, 0xd8, 0x63, 0xb2, 0x78, 0xf4, 0x89, 0x14, 0xf3,
            0x26, 0x75,
        ];
        let footer = FbxFooter { id };
        let hex = footer.id_hex();
        assert_eq!(hex, "fabcaf0fd2c0d863b278f48914f32675");
        assert_eq!(FbxFooter::id_from_hex(&hex), Some(id));
        assert_eq!(FbxFooter::id_from_hex("zz"), None);
        assert_eq!(FbxFooter::id_from_hex(&hex[..30]), None);
        let nonhex = format!("g{}", &hex[1..]);
        assert_eq!(FbxFooter::id_from_hex(&nonhex), None);
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
