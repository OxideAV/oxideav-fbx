//! ASCII FBX container reader.
//!
//! Parses the human-readable text encoding of FBX into the **same**
//! typed [`FbxDocument`] / [`FbxNode`] / [`FbxProperty`] tree the
//! [`crate::binary`] reader produces, so every downstream consumer
//! ([`crate::scene`], [`crate::geometry`], [`crate::material`],
//! [`crate::animation`], [`crate::deformer`], [`crate::pose`],
//! [`crate::properties70`]) Just Works for an ASCII input.
//!
//! Grammar source: `docs/3d/fbx/fbx-ascii-grammar.md`
//! (observer-derived from `docs/3d/fbx/fixtures/cubes-ascii-v7500.fbx`;
//! no FBX-implementation source consulted). Highlights from that doc
//! that drive the implementation here:
//!
//! - **Encoding**: UTF-8. Cyrillic identifiers occur inside quoted
//!   strings (e.g. `"Model::Куб1"`); the tokenizer treats string
//!   bodies as opaque bytes.
//! - **Comments**: a `;` starts a comment that runs to end-of-line.
//!   Full-line and trailing-after-data forms both occur.
//! - **Node shape**: `Key: <value-list>? { <children> }` (body form)
//!   or `Key: <value-list>` (leaf form). The optional value-list is a
//!   comma-separated sequence of scalar values whose token form
//!   determines its [`FbxProperty`] variant.
//! - **Typed arrays**: `Key: *N { a: v1,v2,... }`. `N` is the
//!   element count. We map to [`FbxProperty::F64Array`] when any
//!   element token has a `.`/`e`/`E`. Otherwise we narrow to
//!   [`FbxProperty::I32Array`] when every element fits in `i32`
//!   (matches the binary `i` array variant the
//!   [`crate::geometry`] puller of `PolygonVertexIndex` / `UVIndex` /
//!   `NormalsW` / `Materials` requires verbatim), and fall back to
//!   [`FbxProperty::I64Array`] when any element overflows
//!   (matches the binary `l` array variant used by
//!   `AnimationCurve::KeyTime`'s KTime ticks).
//! - **Indentation**: TAB-per-depth, cosmetic only — structure is
//!   defined by `{` / `}`.
//! - **Bare booleans**: `Shading: T` (or `F`) — a lone uppercase
//!   letter parses as [`FbxProperty::Bool`].
//! - **Object opening lines** carry three values
//!   `UID, "ClassTag::Name", "SubType"` per §7c. We surface them as
//!   three properties (`I64`, `String`, `String`) so the
//!   [`crate::scene`] walker (which expects exactly that shape from
//!   the binary side) round-trips without changes.
//! - **`P:` records** are ordinary leaf nodes with named-property
//!   value-lists; the typing rules above produce the same property
//!   sequence [`crate::properties70`] reads out of the binary side.
//!
//! # Version detection
//!
//! Per grammar §1 / §7a, the FBX version is carried two ways in the
//! ASCII shell:
//!
//! 1. The banner comment `; FBX 7.5.0 project file` (informational).
//! 2. The `FBXHeaderExtension { FBXVersion: 7500 }` leaf.
//!
//! We parse (2) and set [`FbxDocument::version`] from it. If neither
//! is present we default to `7400` (the pre-7500 32-bit layout
//! version; the value is consulted only by binary-side machinery, so
//! the choice for ASCII inputs is informational).

use oxideav_mesh3d::{Error, Result};

use crate::binary::{FbxDocument, FbxNode, FbxProperty, FBX_VERSION_64BIT_THRESHOLD};

/// `true` when `bytes` *looks like* an ASCII FBX file (starts with the
/// `; FBX` banner comment, optionally after a UTF-8 BOM and
/// whitespace).
///
/// This is a fast, syntactic check — it does NOT validate the full
/// grammar. [`parse`] is authoritative.
pub fn is_ascii_fbx(bytes: &[u8]) -> bool {
    let s = strip_bom(bytes);
    let mut i = 0;
    while i < s.len() && matches!(s[i], b' ' | b'\t' | b'\r' | b'\n') {
        i += 1;
    }
    // Banner: `; FBX <version>` or just `;` followed by the literal
    // `FBX` on the same line (the comment grammar is permissive about
    // what follows the semicolon).
    if i >= s.len() || s[i] != b';' {
        return false;
    }
    // Skim the rest of the first line for the case-sensitive marker
    // "FBX" — required by every observed banner.
    let mut j = i + 1;
    while j < s.len() && s[j] != b'\n' {
        j += 1;
    }
    let line = &s[i..j];
    line.windows(3).any(|w| w == b"FBX")
}

/// Parse an ASCII-FBX byte buffer into an [`FbxDocument`] whose shape
/// is interchangeable with the output of [`crate::binary::parse`].
pub fn parse(bytes: &[u8]) -> Result<FbxDocument> {
    let src = strip_bom(bytes);
    let mut p = Parser::new(src);
    let mut root = FbxNode::default();
    loop {
        p.skip_trivia();
        if p.eof() {
            break;
        }
        let node = p.parse_node()?;
        root.children.push(node);
    }
    let version = extract_version(&root).unwrap_or(7400);
    Ok(FbxDocument { version, root })
}

fn strip_bom(bytes: &[u8]) -> &[u8] {
    if bytes.len() >= 3 && &bytes[..3] == b"\xEF\xBB\xBF" {
        &bytes[3..]
    } else {
        bytes
    }
}

fn extract_version(root: &FbxNode) -> Option<u32> {
    let ext = root.child("FBXHeaderExtension")?;
    let v = ext.child("FBXVersion")?;
    let prop = v.properties.first()?;
    let n = prop.as_i64()?;
    if n < 0 || n > u32::MAX as i64 {
        return None;
    }
    Some(n as u32)
}

/// Streaming character-level parser. Tracks a 1-based `line` and
/// `column` for diagnostic messages and exposes the helpers
/// [`Parser::peek`] / [`Parser::bump`] / [`Parser::skip_trivia`].
struct Parser<'a> {
    src: &'a [u8],
    pos: usize,
    line: usize,
    col: usize,
    /// Current `{ }` body nesting level. [`Parser::parse_node`] and
    /// [`Parser::parse_body`] are mutually recursive, so an unbounded
    /// depth lets a crafted input of repeated `A: {` lines (~5 bytes
    /// per level) overflow the parser's stack — an uncatchable abort,
    /// not an `Err`. Capped at [`crate::binary::MAX_NODE_DEPTH`], the
    /// same limit the binary reader enforces (both front-ends produce
    /// the identical tree, so the accepted shape stays identical too).
    depth: usize,
}

impl<'a> Parser<'a> {
    fn new(src: &'a [u8]) -> Self {
        Self {
            src,
            pos: 0,
            line: 1,
            col: 1,
            depth: 0,
        }
    }

    fn eof(&self) -> bool {
        self.pos >= self.src.len()
    }

    fn peek(&self) -> Option<u8> {
        self.src.get(self.pos).copied()
    }

    fn peek_at(&self, off: usize) -> Option<u8> {
        self.src.get(self.pos + off).copied()
    }

    fn bump(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.pos += 1;
        if b == b'\n' {
            self.line += 1;
            self.col = 1;
        } else {
            self.col += 1;
        }
        Some(b)
    }

    /// Skip whitespace (including newlines) and `;`-comments. A
    /// comment runs from `;` to end-of-line.
    fn skip_trivia(&mut self) {
        loop {
            match self.peek() {
                Some(b' ' | b'\t' | b'\r' | b'\n') => {
                    self.bump();
                }
                Some(b';') => {
                    // Consume the rest of the line.
                    while let Some(b) = self.peek() {
                        if b == b'\n' {
                            break;
                        }
                        self.bump();
                    }
                }
                _ => break,
            }
        }
    }

    /// Skip horizontal whitespace and `;`-comments **without** crossing
    /// a newline. Used to gate the "is there a `{` opening a body on
    /// THIS LOGICAL LINE?" decision in [`Parser::parse_node`].
    fn skip_inline_trivia(&mut self) {
        loop {
            match self.peek() {
                Some(b' ' | b'\t') => {
                    self.bump();
                }
                Some(b';') => {
                    while let Some(b) = self.peek() {
                        if b == b'\n' {
                            break;
                        }
                        self.bump();
                    }
                }
                _ => break,
            }
        }
    }

    fn err<T>(&self, msg: impl Into<String>) -> Result<T> {
        Err(Error::invalid(format!(
            "ascii FBX:{}:{}: {}",
            self.line,
            self.col,
            msg.into()
        )))
    }

    fn parse_node(&mut self) -> Result<FbxNode> {
        let name = self.parse_identifier()?;
        // Mandatory `:`.
        self.skip_inline_trivia();
        match self.peek() {
            Some(b':') => {
                self.bump();
            }
            _ => return self.err(format!("expected ':' after node key '{name}'")),
        }
        // Two paths off the colon:
        //  (a) "*N { a: ... }" — typed-array shorthand
        //  (b) value-list, then optionally `{ body }`
        self.skip_inline_trivia();
        if self.peek() == Some(b'*') {
            let prop = self.parse_array_payload()?;
            return Ok(FbxNode {
                name,
                properties: vec![prop],
                children: Vec::new(),
            });
        }
        let properties = self.parse_value_list()?;
        // Decide leaf vs body. The grammar uses `{` to open a body; if
        // we see `{` (possibly after horizontal-whitespace or a
        // trailing comment but on the same logical line per §3a's
        // "Key: ... {"), descend into children. Otherwise the colon
        // line ended and this is a leaf.
        self.skip_inline_trivia();
        if self.peek() == Some(b'{') {
            self.bump();
            let children = self.parse_body()?;
            Ok(FbxNode {
                name,
                properties,
                children,
            })
        } else {
            Ok(FbxNode {
                name,
                properties,
                children: Vec::new(),
            })
        }
    }

    fn parse_body(&mut self) -> Result<Vec<FbxNode>> {
        if self.depth >= crate::binary::MAX_NODE_DEPTH {
            return self.err(format!(
                "node nesting exceeds the {}-level limit",
                crate::binary::MAX_NODE_DEPTH
            ));
        }
        self.depth += 1;
        let result = self.parse_body_inner();
        self.depth -= 1;
        result
    }

    fn parse_body_inner(&mut self) -> Result<Vec<FbxNode>> {
        let mut out = Vec::new();
        loop {
            self.skip_trivia();
            match self.peek() {
                None => return self.err("unexpected EOF inside `{` body"),
                Some(b'}') => {
                    self.bump();
                    return Ok(out);
                }
                _ => {
                    let node = self.parse_node()?;
                    out.push(node);
                }
            }
        }
    }

    /// Parse `*N { a: v1,v2,... }` (the caller has already verified
    /// the leading `*`).
    fn parse_array_payload(&mut self) -> Result<FbxProperty> {
        // Consume `*`.
        if self.bump() != Some(b'*') {
            return self.err("internal: parse_array_payload entered without '*'");
        }
        let count = self.parse_unsigned_integer()? as usize;
        self.skip_inline_trivia();
        // `{`
        if self.peek() != Some(b'{') {
            return self.err("expected '{' after typed-array count");
        }
        self.bump();
        self.skip_trivia();
        // `a` identifier
        let a_name = self.parse_identifier()?;
        if a_name != "a" {
            return self.err(format!(
                "expected 'a' as the typed-array body name, got '{a_name}'"
            ));
        }
        self.skip_inline_trivia();
        if self.peek() != Some(b':') {
            return self.err("expected ':' after typed-array 'a'");
        }
        self.bump();
        // Collect element tokens.
        let mut floats = Vec::new();
        let mut ints = Vec::new();
        let mut any_float = false;
        // Empty-array case: "*0 { a: }" — no elements.
        self.skip_trivia();
        if self.peek() != Some(b'}') {
            loop {
                let elem = self.parse_numeric_token()?;
                match elem {
                    NumericToken::Int(n) => {
                        if any_float {
                            floats.push(n as f64);
                        } else {
                            ints.push(n);
                        }
                    }
                    NumericToken::Float(f) => {
                        if !any_float {
                            // Promote any already-collected ints to f64s.
                            floats = ints.iter().map(|&i| i as f64).collect();
                            ints.clear();
                            any_float = true;
                        }
                        floats.push(f);
                    }
                }
                self.skip_trivia();
                match self.peek() {
                    Some(b',') => {
                        self.bump();
                        self.skip_trivia();
                        // Tolerate a trailing comma before the `}` (not
                        // observed in the fixture but a reasonable
                        // sloppiness defense).
                        if self.peek() == Some(b'}') {
                            break;
                        }
                    }
                    Some(b'}') => break,
                    None => return self.err("unexpected EOF inside typed-array body"),
                    Some(b) => {
                        return self
                            .err(format!("expected ',' or '}}' in array, got byte 0x{b:02x}"));
                    }
                }
            }
        }
        // Consume `}`.
        self.skip_trivia();
        if self.peek() != Some(b'}') {
            return self.err("expected '}' closing typed-array body");
        }
        self.bump();
        let prop = if any_float {
            // The count is a hint, not a hard invariant — surface
            // mismatches loudly rather than letting downstream consumers
            // walk a short or long buffer.
            if count != floats.len() {
                return Err(Error::invalid(format!(
                    "ascii FBX: typed-array count {} != actual element count {}",
                    count,
                    floats.len()
                )));
            }
            FbxProperty::F64Array(floats)
        } else {
            if count != ints.len() {
                return Err(Error::invalid(format!(
                    "ascii FBX: typed-array count {} != actual element count {}",
                    count,
                    ints.len()
                )));
            }
            // Narrow to I32Array when every element fits — this is the
            // variant the geometry / pose / material pullers pattern-
            // match on. Fall back to I64Array (matching the binary
            // `l` array variant) when any element overflows; the
            // animation module's KeyTime puller accepts both shapes.
            let fits_i32 = ints
                .iter()
                .all(|&n| (i32::MIN as i64..=i32::MAX as i64).contains(&n));
            if fits_i32 {
                FbxProperty::I32Array(ints.iter().map(|&n| n as i32).collect())
            } else {
                FbxProperty::I64Array(ints)
            }
        };
        Ok(prop)
    }

    /// Parse zero-or-more comma-separated scalar values (the
    /// `value-list` non-terminal in §3 of the grammar).
    fn parse_value_list(&mut self) -> Result<Vec<FbxProperty>> {
        let mut out = Vec::new();
        // An empty value-list is fine: leaf with no values (e.g. node
        // openings like `FBXHeaderExtension:  {`).
        loop {
            self.skip_inline_trivia();
            match self.peek() {
                None | Some(b'\n' | b'\r' | b'{' | b'}') => return Ok(out),
                _ => {}
            }
            let v = self.parse_scalar_value()?;
            out.push(v);
            self.skip_inline_trivia();
            match self.peek() {
                Some(b',') => {
                    self.bump();
                    continue;
                }
                _ => return Ok(out),
            }
        }
    }

    /// Parse one scalar value.
    fn parse_scalar_value(&mut self) -> Result<FbxProperty> {
        match self.peek() {
            Some(b'"') => self.parse_string().map(FbxProperty::String),
            Some(b'-') | Some(b'.') | Some(b'0'..=b'9') => {
                let tok = self.parse_numeric_token()?;
                Ok(match tok {
                    NumericToken::Int(n) => FbxProperty::I64(n),
                    NumericToken::Float(f) => FbxProperty::F64(f),
                })
            }
            Some(b'T') if self.is_bare_letter() => {
                self.bump();
                Ok(FbxProperty::Bool(true))
            }
            Some(b'F') if self.is_bare_letter() => {
                self.bump();
                Ok(FbxProperty::Bool(false))
            }
            Some(b) => self.err(format!(
                "unexpected byte 0x{b:02x} where a scalar value was expected"
            )),
            None => self.err("unexpected EOF where a scalar value was expected"),
        }
    }

    /// `true` when the byte at `pos` is `T`/`F` AND the next byte is
    /// not part of a larger identifier (i.e. it's a delimiter or EOF).
    /// Without this guard `TimeMode` parses as `T` followed by a
    /// stray identifier.
    fn is_bare_letter(&self) -> bool {
        match self.peek_at(1) {
            None => true,
            Some(b) => !is_ident_continue(b),
        }
    }

    fn parse_identifier(&mut self) -> Result<String> {
        let start = self.pos;
        match self.peek() {
            Some(b) if is_ident_start(b) => {}
            Some(b) => return self.err(format!("expected identifier, got byte 0x{b:02x}")),
            None => return self.err("expected identifier, hit EOF"),
        }
        self.bump();
        while let Some(b) = self.peek() {
            if is_ident_continue(b) {
                self.bump();
            } else {
                break;
            }
        }
        let slice = &self.src[start..self.pos];
        std::str::from_utf8(slice)
            .map(|s| s.to_string())
            .map_err(|e| Error::invalid(format!("ascii FBX: identifier not UTF-8: {e}")))
    }

    /// Parse a `"..."` quoted string into raw bytes. Backslashes are
    /// preserved literally (grammar §5: *"backslashes appear literally
    /// un-escaped in path strings"*).
    fn parse_string(&mut self) -> Result<Vec<u8>> {
        if self.bump() != Some(b'"') {
            return self.err("internal: parse_string entered without '\"'");
        }
        let start = self.pos;
        while let Some(b) = self.peek() {
            if b == b'"' {
                let bytes = self.src[start..self.pos].to_vec();
                self.bump();
                return Ok(bytes);
            }
            if b == b'\n' {
                return self.err("ascii FBX: unterminated string at newline");
            }
            self.bump();
        }
        self.err("ascii FBX: unterminated string at EOF")
    }

    fn parse_unsigned_integer(&mut self) -> Result<u64> {
        let start = self.pos;
        while let Some(b) = self.peek() {
            if b.is_ascii_digit() {
                self.bump();
            } else {
                break;
            }
        }
        if self.pos == start {
            return self.err("expected unsigned integer");
        }
        let s = std::str::from_utf8(&self.src[start..self.pos])
            .map_err(|e| Error::invalid(format!("ascii FBX: integer not UTF-8: {e}")))?;
        s.parse::<u64>()
            .map_err(|e| Error::invalid(format!("ascii FBX: bad unsigned integer '{s}': {e}")))
    }

    /// Parse a numeric token and classify it as Int (i64) or Float
    /// (f64) based on lexical shape (presence of `.` / `e` / `E`).
    /// Signed zero `-0` is observed in normal/tangent arrays per
    /// grammar §5; we surface it as `Int(0)` (which then promotes to
    /// `Float(0.0)` if the array carries any float element). The
    /// integer-vs-float split is otherwise driven purely by token
    /// form, not magnitude.
    fn parse_numeric_token(&mut self) -> Result<NumericToken> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.bump();
        }
        let mut has_int_digit = false;
        let mut has_dot = false;
        let mut has_exp = false;
        while let Some(b) = self.peek() {
            if b.is_ascii_digit() {
                has_int_digit = true;
                self.bump();
            } else {
                break;
            }
        }
        if self.peek() == Some(b'.') {
            has_dot = true;
            self.bump();
            while let Some(b) = self.peek() {
                if b.is_ascii_digit() {
                    self.bump();
                } else {
                    break;
                }
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            has_exp = true;
            self.bump();
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.bump();
            }
            while let Some(b) = self.peek() {
                if b.is_ascii_digit() {
                    self.bump();
                } else {
                    break;
                }
            }
        }
        if !has_int_digit && !has_dot {
            return self.err("expected numeric token");
        }
        let s = std::str::from_utf8(&self.src[start..self.pos])
            .map_err(|e| Error::invalid(format!("ascii FBX: numeric not UTF-8: {e}")))?;
        if has_dot || has_exp {
            let v: f64 = s
                .parse()
                .map_err(|e| Error::invalid(format!("ascii FBX: bad float '{s}': {e}")))?;
            Ok(NumericToken::Float(v))
        } else {
            // Try i64 first, fall through to f64 if it's a magnitude
            // bigger than i64::MAX (some KTime values get there).
            if let Ok(n) = s.parse::<i64>() {
                Ok(NumericToken::Int(n))
            } else {
                let v: f64 = s
                    .parse()
                    .map_err(|e| Error::invalid(format!("ascii FBX: bad numeric '{s}': {e}")))?;
                Ok(NumericToken::Float(v))
            }
        }
    }
}

#[derive(Debug)]
enum NumericToken {
    Int(i64),
    Float(f64),
}

fn is_ident_start(b: u8) -> bool {
    b.is_ascii_alphabetic() || b == b'_'
}

fn is_ident_continue(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// Convenience wrapper: returns the FBX version embedded in a parsed
/// document, or the `7400` default that [`parse`] applies when the
/// `FBXHeaderExtension { FBXVersion }` leaf is missing.
pub fn document_layout_is_64bit(doc: &FbxDocument) -> bool {
    doc.version >= FBX_VERSION_64BIT_THRESHOLD
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_ascii_banner() {
        assert!(is_ascii_fbx(b"; FBX 7.5.0 project file\n"));
        assert!(is_ascii_fbx(
            b"\xEF\xBB\xBF; FBX 7.4.0 project file\nFBXHeaderExtension:  {\n"
        ));
        assert!(is_ascii_fbx(b"\n\t; FBX 7.5.0\n"));
        assert!(!is_ascii_fbx(b"Kaydara FBX Binary  \x00"));
        assert!(!is_ascii_fbx(b""));
        assert!(!is_ascii_fbx(b";just a generic comment\n"));
    }

    #[test]
    fn parses_minimal_shell() {
        let src = b"; FBX 7.5.0 project file\n\
                    FBXHeaderExtension:  {\n\
                    \tFBXHeaderVersion: 1003\n\
                    \tFBXVersion: 7500\n\
                    \tCreator: \"oxideav-fbx test\"\n\
                    }\n\
                    GlobalSettings:  {\n\
                    \tVersion: 1000\n\
                    }\n";
        let doc = parse(src).unwrap();
        assert_eq!(doc.version, 7500);
        let ext = doc.root.child("FBXHeaderExtension").unwrap();
        assert_eq!(
            ext.child("FBXHeaderVersion").unwrap().properties[0].as_i64(),
            Some(1003)
        );
        assert_eq!(
            ext.child("Creator").unwrap().properties[0].as_str(),
            Some("oxideav-fbx test")
        );
        let gs = doc.root.child("GlobalSettings").unwrap();
        assert_eq!(
            gs.child("Version").unwrap().properties[0].as_i64(),
            Some(1000)
        );
    }

    #[test]
    fn parses_object_opening_line_three_values() {
        let src = b"; FBX 7.5.0\n\
                    Objects:  {\n\
                    \tMaterial: 2359823919504, \"Material::Mat_Green\", \"\" {\n\
                    \t\tVersion: 102\n\
                    \t}\n\
                    }\n";
        let doc = parse(src).unwrap();
        let objs = doc.root.child("Objects").unwrap();
        let mat = objs.child("Material").unwrap();
        assert_eq!(mat.properties.len(), 3);
        assert_eq!(mat.properties[0].as_i64(), Some(2359823919504));
        assert_eq!(mat.properties[1].as_str(), Some("Material::Mat_Green"));
        assert_eq!(mat.properties[2].as_str(), Some(""));
        assert_eq!(
            mat.child("Version").unwrap().properties[0].as_i64(),
            Some(102)
        );
    }

    #[test]
    fn parses_typed_array_floats_and_ints() {
        let src = b"; FBX 7.5.0\n\
                    Geometry: 1, \"Geometry::\", \"Mesh\" {\n\
                    \tVertices: *6 {\n\
                    \t\ta: -0.5,-0.5,0.5,0.5,-0.5,0.5\n\
                    \t}\n\
                    \tPolygonVertexIndex: *4 {\n\
                    \t\ta: 0,1,3,-3\n\
                    \t}\n\
                    }\n";
        let doc = parse(src).unwrap();
        let geom = doc.root.child("Geometry").unwrap();
        let vs = &geom.child("Vertices").unwrap().properties[0];
        match vs {
            FbxProperty::F64Array(v) => {
                assert_eq!(v.len(), 6);
                assert!((v[0] - -0.5).abs() < 1e-12);
            }
            other => panic!("expected F64Array, got {other:?}"),
        }
        let pvi = &geom.child("PolygonVertexIndex").unwrap().properties[0];
        match pvi {
            FbxProperty::I32Array(v) => assert_eq!(v, &[0, 1, 3, -3]),
            other => panic!("expected I32Array, got {other:?}"),
        }
    }

    #[test]
    fn typed_array_overflowing_i32_falls_back_to_i64() {
        // KeyTime arrays carry KTime ticks that easily exceed i32::MAX
        // (e.g. 1924423250 fits, 230930790000 doesn't). The fall-back
        // branch produces I64Array which `crate::animation` accepts.
        let src = b"; FBX 7.5.0\nKeyTime: *3 {\n\ta: 1,2,230930790000\n}\n";
        let doc = parse(src).unwrap();
        let arr = &doc.root.child("KeyTime").unwrap().properties[0];
        match arr {
            FbxProperty::I64Array(v) => assert_eq!(v, &[1, 2, 230930790000]),
            other => panic!("expected I64Array, got {other:?}"),
        }
    }

    #[test]
    fn parses_bare_booleans_and_strings() {
        let src = b"; FBX 7.5.0\n\
                    Model: 1, \"Model::M\", \"Mesh\" {\n\
                    \tShading: T\n\
                    \tInverted: F\n\
                    \tCulling: \"CullingOff\"\n\
                    }\n";
        let doc = parse(src).unwrap();
        let m = doc.root.child("Model").unwrap();
        match &m.child("Shading").unwrap().properties[0] {
            FbxProperty::Bool(true) => {}
            other => panic!("expected Bool(true), got {other:?}"),
        }
        match &m.child("Inverted").unwrap().properties[0] {
            FbxProperty::Bool(false) => {}
            other => panic!("expected Bool(false), got {other:?}"),
        }
        assert_eq!(
            m.child("Culling").unwrap().properties[0].as_str(),
            Some("CullingOff")
        );
    }

    #[test]
    fn parses_p_records_with_backslash_paths() {
        // §8 — `P:` records: name, type, label, flags, then 0..N values.
        // Backslashes in paths stay literal per §5.
        let src = b"; FBX 7.5.0\n\
                    Properties70:  {\n\
                    \tP: \"DocumentUrl\", \"KString\", \"Url\", \"\", \"U:\\path\\file.fbx\"\n\
                    \tP: \"UpAxis\", \"int\", \"Integer\", \"\",1\n\
                    \tP: \"Color\", \"ColorRGB\", \"Color\", \"\",0.8,0.8,0.8\n\
                    \tP: \"Original\", \"Compound\", \"\", \"\"\n\
                    }\n";
        let doc = parse(src).unwrap();
        let p70 = doc.root.child("Properties70").unwrap();
        let ps: Vec<&FbxNode> = p70.children_named("P").collect();
        assert_eq!(ps.len(), 4);
        // DocumentUrl: name, type, label, flags, value-string
        assert_eq!(ps[0].properties.len(), 5);
        assert_eq!(ps[0].properties[0].as_str(), Some("DocumentUrl"));
        assert_eq!(ps[0].properties[1].as_str(), Some("KString"));
        assert_eq!(ps[0].properties[4].as_str(), Some("U:\\path\\file.fbx"));
        // UpAxis: 5th property is integer 1.
        assert_eq!(ps[1].properties[4].as_i64(), Some(1));
        // Color: name, type, label, flags, r, g, b.
        assert_eq!(ps[2].properties.len(), 7);
        assert!(matches!(ps[2].properties[6], FbxProperty::F64(_)));
        // Compound: stops right after flags (4 properties total).
        assert_eq!(ps[3].properties.len(), 4);
    }

    #[test]
    fn parses_comments_inline_and_full_line() {
        // §2 — `;` starts a comment to end-of-line. The
        // Connections-style annotation lines and the trailing-blank
        // patterns from the fixture must round-trip without confusing
        // the parser.
        let src = b"; FBX 7.5.0 project file\n\
                    ; ----------------------------------------------------\n\
                    Connections:  {\n\
                    \t\n\
                    \t;Model::Cube2, Model::RootNode\n\
                    \tC: \"OO\",2359439406816,0\n\
                    \t;Geometry::, Model::Cube2\n\
                    \tC: \"OO\",2358377979296,2359439406816\n\
                    }\n";
        let doc = parse(src).unwrap();
        let cs = doc.root.child("Connections").unwrap();
        let recs: Vec<&FbxNode> = cs.children_named("C").collect();
        assert_eq!(recs.len(), 2);
        assert_eq!(recs[0].properties[0].as_str(), Some("OO"));
        assert_eq!(recs[0].properties[1].as_i64(), Some(2359439406816));
        assert_eq!(recs[0].properties[2].as_i64(), Some(0));
    }

    #[test]
    fn parses_value_then_body_node_form() {
        // §3a — a value-list may precede the `{`. Many LayerElement*
        // openings use this: `LayerElementNormal: 0 { ... }`.
        let src = b"; FBX 7.5.0\n\
                    Geometry: 1, \"Geometry::\", \"Mesh\" {\n\
                    \tLayerElementNormal: 0 {\n\
                    \t\tVersion: 102\n\
                    \t\tMappingInformationType: \"ByPolygonVertex\"\n\
                    \t\tReferenceInformationType: \"Direct\"\n\
                    \t\tNormals: *3 {\n\
                    \t\t\ta: 0,0,1\n\
                    \t\t}\n\
                    \t}\n\
                    }\n";
        let doc = parse(src).unwrap();
        let geom = doc.root.child("Geometry").unwrap();
        let len = geom.child("LayerElementNormal").unwrap();
        assert_eq!(len.properties.len(), 1);
        assert_eq!(len.properties[0].as_i64(), Some(0));
        assert_eq!(
            len.child("MappingInformationType").unwrap().properties[0].as_str(),
            Some("ByPolygonVertex")
        );
        let normals = &len.child("Normals").unwrap().properties[0];
        // All three elements are integer-shaped in this synthetic
        // (`0,0,1`) so the narrowing rule picks I32Array; real-world
        // normals arrays are always float-shaped (`0.0,0.0,1.0`) and
        // pick F64Array.
        match normals {
            FbxProperty::I32Array(v) => assert_eq!(v, &[0, 0, 1]),
            other => panic!("expected I32Array, got {other:?}"),
        }
    }

    #[test]
    fn rejects_unterminated_string() {
        let src = b"; FBX 7.5.0\nCreator: \"oops\n";
        assert!(parse(src).is_err());
    }

    #[test]
    fn rejects_bad_array_count() {
        let src = b"; FBX 7.5.0\nVertices: *3 {\n\ta: 0,0\n}\n";
        assert!(parse(src).is_err());
    }

    #[test]
    fn t_in_identifier_is_not_bare_boolean() {
        // Regression: `TimeMode` is an identifier, NOT a bare-T
        // boolean followed by `imeMode`. The is_bare_letter() guard
        // must reject the keyword case.
        let src = b"; FBX 7.5.0\nNode:  {\n\tInner: TimeMode\n}\n";
        // `TimeMode` as a bare identifier is not a valid scalar value
        // — we expect a parse error rather than a spurious Bool(true).
        let err = parse(src).unwrap_err();
        let msg = err.to_string();
        assert!(
            !msg.contains("Bool"),
            "should not have interpreted T as bare boolean"
        );
    }

    #[test]
    fn parses_typed_array_with_trailing_brace_space() {
        // §4 mentions some closing braces have a trailing space; the
        // fixture also wraps array contents across many lines.
        let src = b"; FBX 7.5.0\nA: *4 {\n\ta: 1,2,\n3,4\n} \n";
        let doc = parse(src).unwrap();
        let a = doc.root.child("A").unwrap();
        match &a.properties[0] {
            FbxProperty::I32Array(v) => assert_eq!(v, &[1, 2, 3, 4]),
            other => panic!("expected I32Array, got {other:?}"),
        }
    }

    #[test]
    fn nesting_depth_bomb_errors_instead_of_overflowing_the_stack() {
        // Round 413 hardening — repeated `A: {` lines (~5 bytes per
        // level) previously drove the mutually-recursive parse_node /
        // parse_body thousands of frames deep (uncatchable stack
        // overflow). The reader now enforces the same
        // MAX_NODE_DEPTH limit as the binary front-end.
        let mut src = b"; FBX 7.5.0\n".to_vec();
        const N: usize = 10_000;
        for _ in 0..N {
            src.extend_from_slice(b"A: {\n");
        }
        for _ in 0..N {
            src.extend_from_slice(b"}\n");
        }
        let err = parse(&src).expect_err("depth bomb rejected");
        assert!(
            err.to_string().contains("nesting"),
            "expected the depth limit to fire, got: {err}"
        );
    }

    #[test]
    fn nesting_below_the_limit_still_parses() {
        // Sanity companion to the depth-bomb test: a tree nested to
        // half the limit parses fine and yields the full chain.
        let mut src = b"; FBX 7.5.0\n".to_vec();
        let n = crate::binary::MAX_NODE_DEPTH / 2;
        for _ in 0..n {
            src.extend_from_slice(b"A: {\n");
        }
        src.extend_from_slice(b"Leaf: 1\n");
        for _ in 0..n {
            src.extend_from_slice(b"}\n");
        }
        let doc = parse(&src).expect("half-limit nesting parses");
        let mut node = doc.root.child("A").expect("outermost A");
        for _ in 1..n {
            node = node.child("A").expect("nested A");
        }
        assert!(node.child("Leaf").is_some());
    }

    #[test]
    fn fixture_cubes_ascii_v7500_round_trips_structure() {
        // Parse the staged ASCII fixture and assert top-level
        // structure matches §7 of the grammar doc.
        let bytes = include_bytes!("../tests/fixtures/cubes-ascii-v7500.fbx");
        let doc = parse(bytes).unwrap();
        assert_eq!(doc.version, 7500);
        // §7 sections expected.
        for s in &[
            "FBXHeaderExtension",
            "GlobalSettings",
            "Documents",
            "References",
            "Definitions",
            "Objects",
            "Connections",
            "Takes",
        ] {
            assert!(
                doc.root.child(s).is_some(),
                "missing top-level section: {s}"
            );
        }
        // Header / Creator.
        let ext = doc.root.child("FBXHeaderExtension").unwrap();
        assert_eq!(
            ext.child("FBXHeaderVersion").unwrap().properties[0].as_i64(),
            Some(1003)
        );
        assert_eq!(
            ext.child("Creator").unwrap().properties[0]
                .as_str()
                .unwrap_or(""),
            "FBX SDK/FBX Plugins version 2018.1.1"
        );
        // Objects: at least 4 Geometry + 4 Model + 2 Material + 1 AnimationStack + 1 AnimationLayer
        let objs = doc.root.child("Objects").unwrap();
        assert_eq!(objs.children_named("Geometry").count(), 4);
        assert_eq!(objs.children_named("Model").count(), 4);
        assert_eq!(objs.children_named("Material").count(), 2);
        assert_eq!(objs.children_named("AnimationStack").count(), 1);
        assert_eq!(objs.children_named("AnimationLayer").count(), 1);
        // First Geometry has a *24 Vertices array — must come back as
        // F64Array of length 24.
        let g0 = objs.children_named("Geometry").next().unwrap();
        let verts = &g0.child("Vertices").unwrap().properties[0];
        match verts {
            FbxProperty::F64Array(v) => assert_eq!(v.len(), 24),
            other => panic!("expected F64Array(24), got {other:?}"),
        }
        // Cyrillic name preserved in second Model.
        let models: Vec<&FbxNode> = objs.children_named("Model").collect();
        let kuf1 = models
            .iter()
            .find(|m| {
                m.properties
                    .get(1)
                    .and_then(|p| p.as_str())
                    .map(|s| s.contains("Куб1"))
                    .unwrap_or(false)
            })
            .expect("expected Model::Куб1 in fixture");
        assert!(kuf1.child("Properties70").is_some());
        // Connections OO records.
        let cs = doc.root.child("Connections").unwrap();
        let oo: Vec<&FbxNode> = cs.children_named("C").collect();
        assert!(oo.len() >= 12);
        for c in &oo {
            assert_eq!(c.properties[0].as_str(), Some("OO"));
        }
        // 64-bit layout flag matches version threshold.
        assert!(document_layout_is_64bit(&doc));
    }

    #[test]
    fn fixture_geometry_decodes_through_scene_builder() {
        // End-to-end: parse(ASCII) -> scene::build_scene gives a
        // populated Scene3D with at least one mesh.
        let bytes = include_bytes!("../tests/fixtures/cubes-ascii-v7500.fbx");
        let doc = parse(bytes).unwrap();
        let scene = crate::scene::build_scene(&doc).expect("scene builder");
        assert!(!scene.meshes.is_empty(), "expected at least one mesh");
    }
}
