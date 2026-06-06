//! `Properties70` `P`-record decoder.
//!
//! Per the observer-derived `docs/3d/fbx/fbx-binary-properties70.md`
//! §4 grammar (and its ASCII counterpart in
//! `docs/3d/fbx/fbx-ascii-grammar.md` §8), a `Properties70` node is a
//! container whose children are all named `P` and follow the
//! five-field shape:
//!
//! ```text
//! P [NumProperties = 4 + valueCount]
//!    prop0 : S  name      e.g. "DiffuseColor"
//!    prop1 : S  typeName  e.g. "ColorRGB"
//!    prop2 : S  label     e.g. "Color"
//!    prop3 : S  flags     e.g. ""   ("A" animatable, "U" user)
//!    prop4.. : typed value(s)       — 0 (Compound), 1 (scalar), or 3 (vector/color)
//! ```
//!
//! The number of trailing value props is `NumProperties − 4` per the
//! docs §4: 0 for `Compound` (and any value-less property), 1 for
//! scalars (`int` / `enum` → `I`, `double` / `Number` → `D`,
//! `KTime` / `ULongLong` → `L`, `KString` / `DateTime` → `S`,
//! `bool` → `C`), and 3 for triples (`ColorRGB` / `Color` /
//! `Vector3D` / `Vector` / `Lcl Translation` / `Lcl Scaling`).
//!
//! ASCII and binary FBX render the same record using different
//! type-tag conventions; the typed [`PValue`] decoded here is
//! identical to what the ASCII grammar §8 documents.

use std::collections::HashMap;

use crate::binary::{FbxNode, FbxProperty};

/// Decoded typed value of one `P` record.
///
/// Variants mirror the value-count branches documented in
/// `docs/3d/fbx/fbx-binary-properties70.md` §4 (`Compound` / scalar /
/// triple) plus the wire type codes documented in the same file's §3
/// (binary) and `fbx-ascii-grammar.md` §5 (ASCII).
#[derive(Clone, Debug, PartialEq)]
pub enum PValue {
    /// No value field — `type == "Compound"` (or any value-less P).
    Compound,
    /// Single `I` (`int` / `enum`) value.
    Int(i32),
    /// Single `L` (`KTime` / `ULongLong`) value.
    Long(i64),
    /// Single `D` / `F` (`double` / `Number` / `float`) value.
    Double(f64),
    /// Single `C` (`bool`) value.
    Bool(bool),
    /// Single `S` (`KString` / `DateTime` / `object`) value.
    Str(String),
    /// Triple of doubles (`Vector3D` / `Vector` / `Lcl Translation`
    /// / `Lcl Scaling` / `ColorRGB` / `Color`). The value-count branch
    /// docs §4 calls *"3 for vectors/colours"*.
    Vec3([f64; 3]),
    /// Catch-all for trailing-value records whose shape we don't
    /// recognise: keeps the raw `FbxProperty` list so callers can
    /// fall back to ad-hoc decoding without re-walking the document.
    Other(Vec<FbxProperty>),
}

/// One `P` record after typed decoding.
///
/// `type_name` is the wire string from prop1 (`"int"`, `"double"`,
/// `"ColorRGB"`, `"Lcl Translation"`, `"Compound"`, …). `label` and
/// `flags` are the docs §4 prop2 + prop3 strings. `value` is the
/// typed payload assembled from the trailing `(NumProperties − 4)`
/// value props.
#[derive(Clone, Debug, PartialEq)]
pub struct PRecord {
    pub type_name: String,
    pub label: String,
    pub flags: String,
    pub value: PValue,
}

/// A `Properties70` block decoded into a name → record map.
///
/// Names are unique within a single `Properties70` block by FBX
/// convention (the docs §4 sample shows each `P` record's prop0 as a
/// distinct property identifier); when an FBX file does repeat a
/// name (rare; observed only for some exporter-emitted compound
/// substructures), this decoder keeps the **last** occurrence — the
/// same last-wins shape exporters use when they want to override a
/// template default.
#[derive(Clone, Debug, Default)]
pub struct PropertyMap {
    inner: HashMap<String, PRecord>,
}

impl PropertyMap {
    /// Decode a `Properties70` node into a typed property map.
    ///
    /// `parent` is the owning element node — *not* the `Properties70`
    /// node itself. The function finds the first `Properties70`
    /// direct child and returns an empty map if the parent has none.
    /// This matches the docs §4 placement — `Properties70` sits as a
    /// direct child of the element (e.g. `Material`, `Model`,
    /// `GlobalSettings`).
    pub fn from_element(parent: &FbxNode) -> Self {
        let Some(props70) = parent.child("Properties70") else {
            return Self::default();
        };
        Self::from_properties70(props70)
    }

    /// Decode a `Properties70` node directly.
    ///
    /// Same shape as [`Self::from_element`] but takes the
    /// `Properties70` node itself, useful when the caller already
    /// holds a reference to it.
    pub fn from_properties70(props70: &FbxNode) -> Self {
        let mut inner = HashMap::new();
        for p in props70.children_named("P") {
            if let Some((name, record)) = decode_p_record(p) {
                inner.insert(name, record);
            }
        }
        Self { inner }
    }

    /// Look up a `P` record by name.
    pub fn get(&self, name: &str) -> Option<&PRecord> {
        self.inner.get(name)
    }

    /// Number of `P` records decoded.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// True when no records were decoded.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    // --- typed scalar / vector helpers ---

    /// Pull a `double` / `Number` / `float` factor by name. Accepts
    /// `Int` / `Long` / `Bool` too (so `Opacity: int 1` reads as
    /// `1.0`) — exporters mix these freely.
    pub fn as_f64(&self, name: &str) -> Option<f64> {
        match &self.inner.get(name)?.value {
            PValue::Double(v) => Some(*v),
            PValue::Int(v) => Some(*v as f64),
            PValue::Long(v) => Some(*v as f64),
            PValue::Bool(v) => Some(if *v { 1.0 } else { 0.0 }),
            _ => None,
        }
    }

    /// Pull an `int` / `enum` value by name.
    pub fn as_i32(&self, name: &str) -> Option<i32> {
        match &self.inner.get(name)?.value {
            PValue::Int(v) => Some(*v),
            PValue::Long(v) => Some(*v as i32),
            PValue::Bool(v) => Some(*v as i32),
            _ => None,
        }
    }

    /// Pull a `KTime` / `ULongLong` / `Long` value by name without
    /// loss of precision.
    ///
    /// Per `docs/3d/fbx/fbx-binary-properties70.md` §4, the wire
    /// codes for these typeNames are `L` (int64), so a `KTime`
    /// payload may exceed the i32 range (the doc's sample
    /// `TimeSpanStop = 46_186_158_000` already does) and the
    /// floating-point [`Self::as_f64`] path would lose precision
    /// near the 2^53 boundary. This accessor returns the underlying
    /// [`PValue::Long`] verbatim, widening [`PValue::Int`] /
    /// [`PValue::Bool`] losslessly so an exporter that wires an
    /// otherwise-`KTime` value as `I` (the docs §4 note about older
    /// exporters mixing the integer wire codes) still reads back
    /// correctly.
    ///
    /// Returns `None` for non-numeric records (`Str` / `Vec3` /
    /// `Compound` / `Double` / `Other`).
    pub fn as_i64(&self, name: &str) -> Option<i64> {
        match &self.inner.get(name)?.value {
            PValue::Long(v) => Some(*v),
            PValue::Int(v) => Some(*v as i64),
            PValue::Bool(v) => Some(if *v { 1 } else { 0 }),
            _ => None,
        }
    }

    /// Pull a `bool` value by name. `Int` / `Long` values are
    /// coerced via `!= 0` (FBX `bool` is wire-encoded as `int` in
    /// many older exporters per docs §4).
    pub fn as_bool(&self, name: &str) -> Option<bool> {
        match &self.inner.get(name)?.value {
            PValue::Bool(v) => Some(*v),
            PValue::Int(v) => Some(*v != 0),
            PValue::Long(v) => Some(*v != 0),
            _ => None,
        }
    }

    /// Pull a `KString` / `object` / `DateTime` string by name.
    pub fn as_str(&self, name: &str) -> Option<&str> {
        match &self.inner.get(name)?.value {
            PValue::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pull a `ColorRGB` / `Color` / `Vector3D` / `Vector` triple by
    /// name.
    pub fn as_vec3(&self, name: &str) -> Option<[f64; 3]> {
        match &self.inner.get(name)?.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    // --- typeName-discriminating accessors ---
    //
    // The base [`Self::as_vec3`] / [`Self::as_str`] flatten every
    // trailing-value typeName the docs §4 trailing-value table
    // enumerates. The following accessors honour
    // [`PRecord::type_name`] — the typeName string parsed from
    // prop1 — so a caller asking for a `ColorRGB` triple does not
    // accidentally pick up a `Lcl Translation` triple sitting on
    // the same name. Per `docs/3d/fbx/fbx-binary-properties70.md`
    // §4 *"The typeName/label/flags strings carry the semantic
    // type"*, this is the typeName's documented role.

    /// Pull a `ColorRGB` / `Color` triple by name.
    ///
    /// Per `docs/3d/fbx/fbx-binary-properties70.md` §4 trailing-value
    /// rule *"3 for vectors/colours (`ColorRGB`/`Color`/`Vector3D`)"*,
    /// the docs §4 worked sample (`AmbientColor S"ColorRGB" S"Color"
    /// S"" D=0 D=0 D=0`) and the ASCII grammar §8 enumerated typeName
    /// list, `"ColorRGB"` and `"Color"` are the colour-bearing
    /// typeNames in the same family. Returns `None` for records whose
    /// `type_name` falls outside that pair (e.g. a `Vector3D` triple
    /// — those go through [`Self::as_vector3d`]).
    pub fn as_color_rgb(&self, name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if !matches!(rec.type_name.as_str(), "ColorRGB" | "Color") {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `Vector3D` / `Vector` triple by name.
    ///
    /// Per the docs §4 trailing-value rule and the ASCII grammar §8
    /// typeName list, `"Vector3D"` and `"Vector"` carry plain
    /// geometric triples (positions / directions / Euler angles) as
    /// distinct from colour triples (`ColorRGB`/`Color` — see
    /// [`Self::as_color_rgb`]) and transform triples (`Lcl …` — see
    /// [`Self::as_lcl_translation`] et al). The cubes fixture's
    /// `PreRotation` / `PostRotation` / `GeometricTranslation` /
    /// `GeometricRotation` / `GeometricScaling` all wire as
    /// `"Vector3D"`; `"Vector"` is observed as the label string but
    /// is also documented as a typeName variant.
    pub fn as_vector3d(&self, name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if !matches!(rec.type_name.as_str(), "Vector3D" | "Vector") {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `Lcl Translation` triple by name.
    ///
    /// `"Lcl Translation"` is the typeName the docs §4 trailing-value
    /// table calls out explicitly (alongside `"Lcl Scaling"`) as a
    /// triple typeName; the cubes-ascii-v7500.fbx fixture's `Model`
    /// node carries it as `P: "Lcl Translation", "Lcl Translation",
    /// "", "A", -1.04…, 0.998…, -1.043…`. The accessor validates the
    /// typeName so a caller cannot accidentally pick up a `Vector3D`
    /// triple sitting under the same `"Lcl Translation"` name from a
    /// non-standard exporter.
    pub fn as_lcl_translation(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Translation")
    }

    /// Pull a `Lcl Rotation` triple (XYZ Euler degrees) by name.
    ///
    /// `"Lcl Rotation"` is listed in the ASCII grammar §8 typeName
    /// enumeration and the binary-doc §4 P-record family; the
    /// cubes-ascii-v7500.fbx fixture has `P: "Lcl Rotation", "Lcl
    /// Rotation", "", "A", 0, 0, 0`. The triple is XYZ Euler in
    /// degrees per the ufbx-doc convention the [`crate::animation`]
    /// module already follows when converting `Lcl Rotation` curves
    /// to quaternions.
    pub fn as_lcl_rotation(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Rotation")
    }

    /// Pull a `Lcl Scaling` triple by name.
    ///
    /// `"Lcl Scaling"` is the second `"Lcl …"` typeName the docs §4
    /// trailing-value table calls out explicitly; the cubes fixture's
    /// `Model` node carries `P: "Lcl Scaling", "Lcl Scaling", "", "A",
    /// 10, 10, 10`.
    pub fn as_lcl_scaling(&self, name: &str) -> Option<[f64; 3]> {
        self.as_typed_vec3(name, "Lcl Scaling")
    }

    /// Internal helper for the `as_lcl_*` family — match a
    /// triple-valued record with the requested typeName.
    fn as_typed_vec3(&self, name: &str, expected_type_name: &str) -> Option<[f64; 3]> {
        let rec = self.inner.get(name)?;
        if rec.type_name != expected_type_name {
            return None;
        }
        match &rec.value {
            PValue::Vec3(v) => Some(*v),
            _ => None,
        }
    }

    /// Pull a `DateTime` value by name.
    ///
    /// Per the docs §4 *"`KString`/`DateTime` → `S`"* row, a
    /// `"DateTime"` typeName is wire-encoded as an `S` string; the
    /// cubes-ascii-v7500.fbx fixture's `FBXHeaderExtension` block
    /// shows the documented sample form `P:
    /// "Original|DateTime_GMT", "DateTime", "", "", "07/01/2019
    /// 16:17:31.730"` (the doc's own §3 / §5 enumerates
    /// `"DateTime"` as one of the typeName values that wires as a
    /// quoted string). The accessor returns the raw string body
    /// (the docs do not specify a parsed `chrono`-style breakdown,
    /// so the bytes are surfaced verbatim for caller-side parsing);
    /// it validates the typeName so a `KString` payload doesn't
    /// surface here unintentionally.
    pub fn as_datetime(&self, name: &str) -> Option<&str> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "DateTime" {
            return None;
        }
        match &rec.value {
            PValue::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// Pull an `"object"` reference value by name.
    ///
    /// The ASCII grammar §8 typeName enumeration lists `"object"`
    /// as a distinct typeName variant (separate from `"KString"`).
    /// In the cubes-ascii-v7500.fbx fixture the `"object"` records
    /// (`P: "SourceObject", "object", "", ""`, `P:
    /// "LookAtProperty", "object", "", ""`, `P:
    /// "UpVectorProperty", "object", "", ""`) all carry an empty
    /// string body — the object reference itself is recorded
    /// elsewhere (in `Connections` `OP` records that wire the
    /// owning element to the referenced object). The accessor
    /// returns the (typically empty) string body for callers that
    /// want to detect the presence of an `"object"` slot
    /// independently of its `Connections` resolution. typeName
    /// validation prevents a `KString` body sneaking in.
    pub fn as_object_ref(&self, name: &str) -> Option<&str> {
        let rec = self.inner.get(name)?;
        if rec.type_name != "object" {
            return None;
        }
        match &rec.value {
            PValue::Str(s) => Some(s.as_str()),
            // The fixture's `"object"` records have zero trailing
            // values (the doc-§4 zero-value `Compound` shape applies
            // when an exporter omits the body entirely). Surface that
            // case as an empty string so the caller can still detect
            // the slot's presence without re-walking.
            PValue::Compound => Some(""),
            _ => None,
        }
    }

    /// Iterate every record name. Order is HashMap-defined (no
    /// particular file order).
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.inner.keys().map(String::as_str)
    }
}

/// Decode one `P` node into a `(name, PRecord)` pair. Returns `None`
/// when the node does not match the docs §4 shape (fewer than 4
/// string-typed leading props, etc.).
fn decode_p_record(node: &FbxNode) -> Option<(String, PRecord)> {
    if node.properties.len() < 4 {
        return None;
    }
    let name = string_prop(&node.properties[0])?;
    let type_name = string_prop(&node.properties[1])?;
    let label = string_prop(&node.properties[2])?;
    let flags = string_prop(&node.properties[3])?;
    let trailing = &node.properties[4..];
    let value = decode_value(&type_name, trailing);
    Some((
        name,
        PRecord {
            type_name,
            label,
            flags,
            value,
        },
    ))
}

/// Build a typed [`PValue`] from the trailing value-prop slice. The
/// shape branches mirror the value-count rules in docs §4:
///
/// - Empty trailing slice → `Compound` (`type == "Compound"` or any
///   value-less P).
/// - One element → typed scalar based on `type_name`.
/// - Three elements → `Vec3` (used for `ColorRGB` / `Vector3D` /
///   `Lcl Translation` / etc.).
/// - Anything else → `Other` (round-trips through the FbxDocument
///   without losing data).
fn decode_value(type_name: &str, trailing: &[FbxProperty]) -> PValue {
    match trailing.len() {
        0 => PValue::Compound,
        1 => scalar_value(type_name, &trailing[0]),
        3 => {
            // The docs §4 explicitly lists three-double triples for
            // ColorRGB / Color / Vector3D / Vector / Lcl Translation /
            // Lcl Scaling. The wire encoding is three D records.
            let a = trailing[0].as_f64_loose();
            let b = trailing[1].as_f64_loose();
            let c = trailing[2].as_f64_loose();
            if let (Some(a), Some(b), Some(c)) = (a, b, c) {
                PValue::Vec3([a, b, c])
            } else {
                PValue::Other(trailing.to_vec())
            }
        }
        _ => PValue::Other(trailing.to_vec()),
    }
}

/// Decode one scalar value. The wire encoding code in
/// `FbxProperty` is authoritative for what the bytes really mean;
/// the `type_name` string is used to disambiguate ambiguous cases
/// (e.g. an `Int` payload whose typeName is `"bool"`).
fn scalar_value(type_name: &str, prop: &FbxProperty) -> PValue {
    match prop {
        FbxProperty::Bool(b) => PValue::Bool(*b),
        FbxProperty::F32(v) => PValue::Double(*v as f64),
        FbxProperty::F64(v) => PValue::Double(*v),
        FbxProperty::I16(v) => PValue::Int(*v as i32),
        FbxProperty::I32(v) => match type_name {
            // Some FBX `P` records carry "bool" payloads as `I` wire
            // codes (the docs §4 sample for `BlendModeBypass` /
            // `Mute` shows ULongLong / bool with integer wire). We
            // honour the typeName for unambiguous decoding.
            "bool" => PValue::Bool(*v != 0),
            _ => PValue::Int(*v),
        },
        FbxProperty::I64(v) => match type_name {
            "bool" => PValue::Bool(*v != 0),
            _ => PValue::Long(*v),
        },
        FbxProperty::String(bytes) => match std::str::from_utf8(bytes) {
            Ok(s) => PValue::Str(s.to_owned()),
            Err(_) => PValue::Other(vec![prop.clone()]),
        },
        _ => PValue::Other(vec![prop.clone()]),
    }
}

fn string_prop(p: &FbxProperty) -> Option<String> {
    match p {
        FbxProperty::String(bytes) => std::str::from_utf8(bytes).ok().map(str::to_owned),
        _ => None,
    }
}

impl FbxProperty {
    /// Coerce numeric scalars to `f64`. Returns `None` for non-numeric
    /// variants (strings, raw blobs, arrays).
    ///
    /// Defined here rather than on the main `binary` module to keep
    /// the `Properties70` triple-value decode self-contained (a few
    /// callers in `material` / `light` / `camera` will reuse it).
    pub(crate) fn as_f64_loose(&self) -> Option<f64> {
        match *self {
            FbxProperty::F32(v) => Some(v as f64),
            FbxProperty::F64(v) => Some(v),
            FbxProperty::I16(v) => Some(v as f64),
            FbxProperty::I32(v) => Some(v as f64),
            FbxProperty::I64(v) => Some(v as f64),
            FbxProperty::Bool(v) => Some(if v { 1.0 } else { 0.0 }),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(b: &[u8]) -> FbxProperty {
        FbxProperty::String(b.to_vec())
    }

    fn p(name: &str, type_name: &str, label: &str, flags: &str, vals: Vec<FbxProperty>) -> FbxNode {
        let mut props = vec![
            s(name.as_bytes()),
            s(type_name.as_bytes()),
            s(label.as_bytes()),
            s(flags.as_bytes()),
        ];
        props.extend(vals);
        FbxNode {
            name: "P".into(),
            properties: props,
            children: Vec::new(),
        }
    }

    fn props70(records: Vec<FbxNode>) -> FbxNode {
        FbxNode {
            name: "Properties70".into(),
            properties: Vec::new(),
            children: records,
        }
    }

    #[test]
    fn decode_scalar_double() {
        // docs §4 sample: `UnitScaleFactor S"double" S"Number" S"" D=100.0`.
        let block = props70(vec![p(
            "UnitScaleFactor",
            "double",
            "Number",
            "",
            vec![FbxProperty::F64(100.0)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.len(), 1);
        assert_eq!(pm.as_f64("UnitScaleFactor"), Some(100.0));
        let rec = pm.get("UnitScaleFactor").expect("decoded UnitScaleFactor");
        assert_eq!(rec.type_name, "double");
        assert_eq!(rec.label, "Number");
        assert_eq!(rec.flags, "");
        assert_eq!(rec.value, PValue::Double(100.0));
    }

    #[test]
    fn decode_scalar_int_and_enum() {
        // docs §4: `UpAxis S"int" S"Integer" S"" I=1`.
        let block = props70(vec![
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
            p("TimeMode", "enum", "", "", vec![FbxProperty::I32(0)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i32("UpAxis"), Some(1));
        assert_eq!(pm.as_i32("TimeMode"), Some(0));
    }

    #[test]
    fn decode_vec3_color() {
        // docs §4 sample: `AmbientColor S"ColorRGB" S"Color" S"" D=0 D=0 D=0`.
        let block = props70(vec![p(
            "AmbientColor",
            "ColorRGB",
            "Color",
            "",
            vec![
                FbxProperty::F64(0.4),
                FbxProperty::F64(0.5),
                FbxProperty::F64(0.6),
            ],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vec3("AmbientColor"), Some([0.4, 0.5, 0.6]));
    }

    #[test]
    fn decode_kstring() {
        // docs §4 sample: `DefaultCamera S"KString" S"" S"" S"Producer Perspective"`.
        let block = props70(vec![p(
            "DefaultCamera",
            "KString",
            "",
            "",
            vec![s(b"Producer Perspective")],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_str("DefaultCamera"), Some("Producer Perspective"));
    }

    #[test]
    fn decode_ktime_long() {
        // docs §4 sample: `TimeSpanStop S"KTime" S"Time" S"" L=46186158000`.
        let block = props70(vec![p(
            "TimeSpanStop",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(46_186_158_000)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.get("TimeSpanStop").map(|r| r.value.clone()),
            Some(PValue::Long(46_186_158_000))
        );
        assert_eq!(pm.as_f64("TimeSpanStop"), Some(46_186_158_000.0));
        // Lossless int64 accessor (the docs §4 sample value exceeds
        // the i32 range): `46_186_158_000` cleanly survives the
        // round trip, where the f64 path quietly drops precision for
        // values approaching 2^53.
        assert_eq!(pm.as_i64("TimeSpanStop"), Some(46_186_158_000));
    }

    #[test]
    fn as_i64_preserves_int64_past_f64_safe_range() {
        // 2^53 + 1 is the smallest positive integer not exactly
        // representable by f64, so `as_f64 -> i64 round trip` would
        // lose precision; `as_i64` returns the wire value verbatim.
        let big: i64 = (1_i64 << 53) + 1;
        let block = props70(vec![p(
            "TimeSpanStop",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(big)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("TimeSpanStop"), Some(big));
        // The f64 accessor still works but quietly truncates the
        // low-order bit — this assertion documents the precision
        // ceiling that motivates the typed `as_i64` path.
        let lossy = pm.as_f64("TimeSpanStop").unwrap() as i64;
        assert_eq!(lossy, big - 1);
    }

    #[test]
    fn as_i64_widens_int_and_bool_wire_codes() {
        // Per docs §4, older exporters wire some `KTime` / `ULongLong`
        // payloads as `I` (int32); the accessor widens losslessly.
        let block = props70(vec![
            p(
                "BlendModeBypass",
                "ULongLong",
                "",
                "",
                vec![FbxProperty::I32(7)],
            ),
            p("Mute", "bool", "", "", vec![FbxProperty::Bool(true)]),
            p("ZeroBool", "bool", "", "", vec![FbxProperty::I32(0)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("BlendModeBypass"), Some(7));
        assert_eq!(pm.as_i64("Mute"), Some(1));
        assert_eq!(pm.as_i64("ZeroBool"), Some(0));
    }

    #[test]
    fn as_i64_rejects_non_numeric_records() {
        // String / triple / Compound records all return `None` so the
        // caller can fall back without ambiguity.
        let block = props70(vec![
            p(
                "DefaultCamera",
                "KString",
                "",
                "",
                vec![s(b"Producer Perspective")],
            ),
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p("TimeMarker", "Compound", "", "", vec![]),
            p(
                "UnitScaleFactor",
                "double",
                "Number",
                "",
                vec![FbxProperty::F64(100.0)],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("DefaultCamera"), None);
        assert_eq!(pm.as_i64("AmbientColor"), None);
        assert_eq!(pm.as_i64("TimeMarker"), None);
        // Double payload is also rejected — callers wanting numeric
        // coercion across f64/int should call `as_f64`.
        assert_eq!(pm.as_i64("UnitScaleFactor"), None);
        // Missing record: also `None`.
        assert_eq!(pm.as_i64("DoesNotExist"), None);
    }

    #[test]
    fn as_i64_handles_negative_ktime() {
        // KTime values can be negative (e.g. `TimeSpanStart` before
        // time 0 in animations that loop). The wire is signed int64.
        let block = props70(vec![p(
            "TimeSpanStart",
            "KTime",
            "Time",
            "",
            vec![FbxProperty::I64(-1_924_423_250)],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_i64("TimeSpanStart"), Some(-1_924_423_250));
    }

    #[test]
    fn decode_compound_no_value() {
        // docs §4 sample: `TimeMarker S"Compound" S"" S""` — NO value.
        let block = props70(vec![p("TimeMarker", "Compound", "", "", vec![])]);
        let pm = PropertyMap::from_properties70(&block);
        let rec = pm.get("TimeMarker").expect("Compound P decoded");
        assert_eq!(rec.value, PValue::Compound);
    }

    #[test]
    fn from_element_finds_properties70_child() {
        // `Material` parent with a Properties70 child.
        let mat = FbxNode {
            name: "Material".into(),
            properties: vec![],
            children: vec![props70(vec![p(
                "DiffuseColor",
                "Color",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.8),
                    FbxProperty::F64(0.4),
                    FbxProperty::F64(0.2),
                ],
            )])],
        };
        let pm = PropertyMap::from_element(&mat);
        assert_eq!(pm.as_vec3("DiffuseColor"), Some([0.8, 0.4, 0.2]));
    }

    #[test]
    fn missing_properties70_returns_empty_map() {
        let bare = FbxNode {
            name: "Material".into(),
            properties: vec![],
            children: vec![],
        };
        let pm = PropertyMap::from_element(&bare);
        assert!(pm.is_empty());
    }

    #[test]
    fn bool_typed_payload_with_int_wire() {
        // Older exporters wire `bool` as `I` — typeName disambiguates.
        let block = props70(vec![p("Mute", "bool", "", "", vec![FbxProperty::I32(1)])]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_bool("Mute"), Some(true));
    }

    #[test]
    fn lcl_translation_triple() {
        // docs §4 / ASCII §8 sample:
        // `P "Lcl Translation","Lcl Translation","","A",-1.04,0.99,-1.04`.
        let block = props70(vec![p(
            "Lcl Translation",
            "Lcl Translation",
            "",
            "A",
            vec![
                FbxProperty::F64(-1.04),
                FbxProperty::F64(0.998),
                FbxProperty::F64(-1.043),
            ],
        )]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vec3("Lcl Translation"), Some([-1.04, 0.998, -1.043]));
        let rec = pm.get("Lcl Translation").expect("decoded translation");
        assert_eq!(rec.flags, "A");
    }

    // --- typeName-discriminating accessor tests ---

    #[test]
    fn as_color_rgb_accepts_colorrgb_and_color_typenames() {
        // Docs §4 sample uses "ColorRGB"; cubes-ascii-v7500.fbx Material
        // records use "Color" — both belong to the same triple-typeName
        // family and the accessor accepts either.
        let block = props70(vec![
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.1),
                    FbxProperty::F64(0.2),
                    FbxProperty::F64(0.3),
                ],
            ),
            p(
                "DiffuseColor",
                "Color",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.8),
                    FbxProperty::F64(0.4),
                    FbxProperty::F64(0.2),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_color_rgb("AmbientColor"), Some([0.1, 0.2, 0.3]));
        assert_eq!(pm.as_color_rgb("DiffuseColor"), Some([0.8, 0.4, 0.2]));
    }

    #[test]
    fn as_color_rgb_rejects_non_color_typenames() {
        // A `Vector3D` triple is structurally identical (three doubles)
        // but semantically a direction/Euler/etc. — the typeName guard
        // keeps the surfaces disjoint.
        let block = props70(vec![
            p(
                "PreRotation",
                "Vector3D",
                "Vector",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(90.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Translation",
                "Lcl Translation",
                "",
                "A",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(3.0),
                ],
            ),
            p("UpAxis", "int", "Integer", "", vec![FbxProperty::I32(1)]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_color_rgb("PreRotation"), None);
        assert_eq!(pm.as_color_rgb("Lcl Translation"), None);
        assert_eq!(pm.as_color_rgb("UpAxis"), None);
        // Plain `as_vec3` still surfaces the Vector3D/Lcl triples —
        // the typeName-discriminating accessor narrows on top of the
        // generic one.
        assert_eq!(pm.as_vec3("PreRotation"), Some([0.0, 90.0, 0.0]));
        assert_eq!(pm.as_vec3("Lcl Translation"), Some([1.0, 2.0, 3.0]));
    }

    #[test]
    fn as_vector3d_accepts_vector3d_and_vector_typenames() {
        // Cubes fixture's `PreRotation` / `PostRotation` /
        // `GeometricTranslation` / `GeometricRotation` /
        // `GeometricScaling` all wire as `"Vector3D"`. `"Vector"`
        // is also enumerated in ASCII grammar §8.
        let block = props70(vec![
            p(
                "PreRotation",
                "Vector3D",
                "Vector",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "GeometricScaling",
                "Vector",
                "",
                "",
                vec![
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                    FbxProperty::F64(1.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vector3d("PreRotation"), Some([0.0, 0.0, 0.0]));
        assert_eq!(pm.as_vector3d("GeometricScaling"), Some([1.0, 1.0, 1.0]));
    }

    #[test]
    fn as_vector3d_rejects_color_and_lcl_typenames() {
        let block = props70(vec![
            p(
                "AmbientColor",
                "ColorRGB",
                "Color",
                "",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Scaling",
                "Lcl Scaling",
                "",
                "A",
                vec![
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(2.0),
                    FbxProperty::F64(2.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_vector3d("AmbientColor"), None);
        assert_eq!(pm.as_vector3d("Lcl Scaling"), None);
    }

    #[test]
    fn as_lcl_translation_rotation_scaling_split_by_typename() {
        // The cubes-ascii-v7500.fbx fixture's `Model` block carries
        // all three `Lcl …` records on the same element; each
        // accessor must surface only its own typeName.
        let block = props70(vec![
            p(
                "Lcl Translation",
                "Lcl Translation",
                "",
                "A",
                vec![
                    FbxProperty::F64(-1.04023893373156),
                    FbxProperty::F64(0.998288783259251),
                    FbxProperty::F64(-1.04375962988677),
                ],
            ),
            p(
                "Lcl Rotation",
                "Lcl Rotation",
                "",
                "A",
                vec![
                    FbxProperty::F64(0.0),
                    FbxProperty::F64(45.0),
                    FbxProperty::F64(0.0),
                ],
            ),
            p(
                "Lcl Scaling",
                "Lcl Scaling",
                "",
                "A",
                vec![
                    FbxProperty::F64(10.0),
                    FbxProperty::F64(10.0),
                    FbxProperty::F64(10.0),
                ],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.as_lcl_translation("Lcl Translation"),
            Some([-1.04023893373156, 0.998288783259251, -1.04375962988677]),
        );
        assert_eq!(pm.as_lcl_rotation("Lcl Rotation"), Some([0.0, 45.0, 0.0]));
        assert_eq!(pm.as_lcl_scaling("Lcl Scaling"), Some([10.0, 10.0, 10.0]));
        // Cross-name rejection — `Lcl Translation` payload does not
        // surface under `as_lcl_rotation`, etc.
        assert_eq!(pm.as_lcl_rotation("Lcl Translation"), None);
        assert_eq!(pm.as_lcl_scaling("Lcl Translation"), None);
        assert_eq!(pm.as_lcl_translation("Lcl Rotation"), None);
        // And a plain `Vector3D` record (e.g. PreRotation) is not
        // promoted to `Lcl Rotation` just because its triple shape
        // matches.
        let pre = props70(vec![p(
            "PreRotation",
            "Vector3D",
            "Vector",
            "",
            vec![
                FbxProperty::F64(0.0),
                FbxProperty::F64(0.0),
                FbxProperty::F64(0.0),
            ],
        )]);
        let pm2 = PropertyMap::from_properties70(&pre);
        assert_eq!(pm2.as_lcl_rotation("PreRotation"), None);
    }

    #[test]
    fn as_datetime_accepts_datetime_typename_only() {
        // ASCII grammar §8 worked sample:
        // `P: "Original|DateTime_GMT", "DateTime", "", "", "07/01/2019 16:17:31.730"`
        let block = props70(vec![
            p(
                "Original|DateTime_GMT",
                "DateTime",
                "",
                "",
                vec![FbxProperty::String(b"07/01/2019 16:17:31.730".to_vec())],
            ),
            // A KString record on a similarly-named slot must NOT
            // surface through as_datetime; the typeName is the only
            // signal that disambiguates the two.
            p(
                "DocumentUrl",
                "KString",
                "Url",
                "",
                vec![s(b"U:\\path\\file.fbx")],
            ),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(
            pm.as_datetime("Original|DateTime_GMT"),
            Some("07/01/2019 16:17:31.730"),
        );
        assert_eq!(pm.as_datetime("DocumentUrl"), None);
        // Plain as_str still surfaces both — the typeName accessor
        // narrows on top.
        assert_eq!(
            pm.as_str("Original|DateTime_GMT"),
            Some("07/01/2019 16:17:31.730"),
        );
        assert_eq!(pm.as_str("DocumentUrl"), Some("U:\\path\\file.fbx"));
        // Missing record → None.
        assert_eq!(pm.as_datetime("MissingRecord"), None);
    }

    #[test]
    fn as_object_ref_accepts_object_typename_with_str_or_compound_body() {
        // Cubes fixture's `SourceObject`, `LookAtProperty`,
        // `UpVectorProperty` records carry an empty body which the
        // decoder lands as `PValue::Compound` (zero trailing values).
        // `as_object_ref` surfaces "" so the caller can still detect
        // the slot's presence; a typed body (someone wires the slot
        // with an inline name) also surfaces.
        let block = props70(vec![
            p("SourceObject", "object", "", "", vec![]),
            p("LookAtProperty", "object", "", "", vec![]),
            p(
                "InlineRef",
                "object",
                "",
                "",
                vec![FbxProperty::String(b"Model::SomeNode".to_vec())],
            ),
            // A KString sitting under a slot name common in the
            // fixture (`currentUVSet`) must NOT surface here.
            p("currentUVSet", "KString", "", "U", vec![s(b"map1")]),
        ]);
        let pm = PropertyMap::from_properties70(&block);
        assert_eq!(pm.as_object_ref("SourceObject"), Some(""));
        assert_eq!(pm.as_object_ref("LookAtProperty"), Some(""));
        assert_eq!(pm.as_object_ref("InlineRef"), Some("Model::SomeNode"));
        assert_eq!(pm.as_object_ref("currentUVSet"), None);
        assert_eq!(pm.as_object_ref("MissingRecord"), None);
    }
}
