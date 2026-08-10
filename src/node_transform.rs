//! `Model` node local-transform decode — the **full FBX
//! node-transform chain** per `docs/3d/fbx/fbx-node-transform-chain.md`.
//!
//! Every FBX `Model` element carries its local-to-parent placement in
//! its `Properties70` block as the three `Lcl …` transform P-records
//! (per `docs/3d/fbx/fbx-ascii-grammar.md` §8 typeName enumeration and
//! the cubes-ascii-v7500.fbx fixture's `Model` blocks):
//!
//! ```text
//! P: "Lcl Translation", "Lcl Translation", "", "A", tx, ty, tz
//! P: "Lcl Rotation",    "Lcl Rotation",    "", "A", rx, ry, rz   (Euler degrees)
//! P: "Lcl Scaling",     "Lcl Scaling",     "", "A", sx, sy, sz
//! ```
//!
//! plus the chain-extension records (`RotationOffset` / `RotationPivot`
//! / `PreRotation` / `PostRotation` / `ScalingOffset` / `ScalingPivot`
//! `Vector3D` triples, and the `RotationOrder` `enum`). The local
//! matrix is the §1 product from
//! `docs/3d/fbx/fbx-node-transform-chain.md`:
//!
//! ```text
//! Local = T · Roff · Rp · Rpre · R · Rpost⁻¹ · Rp⁻¹ · Soff · Sp · S · Sp⁻¹
//! ```
//!
//! (column-vector convention, `out = M · v`; the parent's world matrix
//! multiplies from the left). This module resolves each `Model`'s
//! `Properties70` against the `ObjectType: "Model"` `PropertyTemplate`
//! defaults and composes the **entire** product into the node's local
//! [`oxideav_mesh3d::Transform::Trs`] — see
//! [`TransformChain::compose`] for the closed form. When any
//! chain-extension record is non-trivial, the raw authored components
//! are additionally surfaced on `Node::extras` (`fbx:lcl_*`,
//! `fbx:rotation_pivot`, `fbx:pre_rotation`, ... — see the constants
//! below) so the encode side can re-emit the authored chain verbatim
//! instead of the composed reduction.
//!
//! ## Rotation order
//!
//! The `RotationOrder` enum follows the doc §3 table (declaration
//! order; `0` = `XYZ` is fixture-confirmed, positions 1–6 are the
//! doc's inferred numbering): an order named `ABC` applies the
//! `A`-axis rotation first, so the matrix product is
//! `R_C · R_B · R_A`. Value `6` (`SphericXYZ`) is not a Euler order —
//! per the doc it selects spherical interpolation and its rest matrix
//! is constructed as `XYZ`; the raw enum value is preserved on
//! `extras["fbx:rotation_order"]` so a re-export does not silently
//! remap it. An enum value **outside** the documented `0..=6` table
//! leaves the node at identity with the raw components and an
//! `extras["fbx:transform_incomplete"] = "rotation_order_unrecognized"`
//! marker — the honest surface for input the staged docs don't cover.
//!
//! ## Geometric transform
//!
//! `GeometricTranslation` / `GeometricRotation` / `GeometricScaling`
//! form a **post-multiplied, non-inheriting** offset applied to the
//! node's own geometry only (doc §2: *"`ParentWorldTransform` does not
//! contain the `OT`, `OR`, and `OS` of `WorldTransform`'s parent
//! node"*). They are deliberately **not** composed into
//! `Node::transform` — that would leak them into every child's parent
//! chain — and instead surface on `extras["fbx:geometric_*"]`; the
//! [`geometric_transform`] helper rebuilds the `OT · OR · OS` product
//! a consumer must post-multiply onto the node's world matrix when
//! transforming that node's own mesh.
//!
//! ## InheritType
//!
//! `InheritType` selects how the **parent's** rotation and scale
//! propagate — a world-transform concern, not part of this module's
//! local composition. A non-zero value surfaces raw on
//! `extras["fbx:inherit_type"]` (and re-emits on encode); the doc §4
//! per-type composition products are implemented by
//! [`crate::inherit`] (`inherit::world_transforms` walks a decoded
//! scene honouring them).

use std::collections::HashMap;

use oxideav_mesh3d::{Node, NodeId, Scene3D, Transform};

use crate::binary::{FbxDocument, FbxNode, FbxProperty};
use crate::definitions::Definitions;
use crate::properties70::PropertyMap;

/// Euler rotation order — `docs/3d/fbx/fbx-node-transform-chain.md` §3
/// table (declaration order; integer values are the doc's inferred
/// `0..=6` numbering, with `0` = `XYZ` fixture-confirmed).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RotationOrder {
    /// `0` — apply X, then Y, then Z (`R = Rz · Ry · Rx`). The FBX
    /// default.
    Xyz,
    /// `1` — apply X, then Z, then Y (`R = Ry · Rz · Rx`).
    Xzy,
    /// `2` — apply Y, then Z, then X (`R = Rx · Rz · Ry`).
    Yzx,
    /// `3` — apply Y, then X, then Z (`R = Rz · Rx · Ry`).
    Yxz,
    /// `4` — apply Z, then X, then Y (`R = Ry · Rx · Rz`).
    Zxy,
    /// `5` — apply Z, then Y, then X (`R = Rx · Ry · Rz`).
    Zyx,
    /// `6` — spherical interpolation mode; **not** a Euler order. The
    /// rest matrix is constructed as `XYZ` per the doc, but the raw
    /// enum value must survive re-export.
    SphericXyz,
}

impl RotationOrder {
    /// Map the stored `"enum"` integer to an order. Returns `None`
    /// outside the documented `0..=6` table.
    pub fn from_enum_int(v: i64) -> Option<Self> {
        Some(match v {
            0 => Self::Xyz,
            1 => Self::Xzy,
            2 => Self::Yzx,
            3 => Self::Yxz,
            4 => Self::Zxy,
            5 => Self::Zyx,
            6 => Self::SphericXyz,
            _ => return None,
        })
    }

    /// The stored `"enum"` integer for this order.
    pub fn to_enum_int(self) -> i64 {
        match self {
            Self::Xyz => 0,
            Self::Xzy => 1,
            Self::Yzx => 2,
            Self::Yxz => 3,
            Self::Zxy => 4,
            Self::Zyx => 5,
            Self::SphericXyz => 6,
        }
    }

    /// Application-order axis indices (first, second, third; `0` = X,
    /// `1` = Y, `2` = Z). Doc §3: an order named `ABC` rotates about
    /// `A` first and the product is `R_C · R_B · R_A`. `SphericXYZ`
    /// constructs its rest matrix as `XYZ`.
    pub fn application_axes(self) -> [usize; 3] {
        match self {
            Self::Xyz | Self::SphericXyz => [0, 1, 2],
            Self::Xzy => [0, 2, 1],
            Self::Yzx => [1, 2, 0],
            Self::Yxz => [1, 0, 2],
            Self::Zxy => [2, 0, 1],
            Self::Zyx => [2, 1, 0],
        }
    }
}

/// The complete set of `Properties70` records feeding the doc §1 local
/// transform product for one `Model`.
///
/// Angles are Euler **degrees** (the FBX wire form). Defaults mirror
/// the fixture's `FbxNode` `PropertyTemplate`: identity everywhere,
/// scaling `1,1,1`, rotation order `XYZ`.
#[derive(Debug, Clone, PartialEq)]
pub struct TransformChain {
    /// `Lcl Translation` (`T`).
    pub lcl_translation: [f64; 3],
    /// `Lcl Rotation` (`R`), Euler degrees in `rotation_order`.
    pub lcl_rotation: [f64; 3],
    /// `Lcl Scaling` (`S`).
    pub lcl_scaling: [f64; 3],
    /// `RotationOffset` (`Roff`).
    pub rotation_offset: [f64; 3],
    /// `RotationPivot` (`Rp`).
    pub rotation_pivot: [f64; 3],
    /// `PreRotation` (`Rpre`), Euler degrees, XYZ construction.
    pub pre_rotation: [f64; 3],
    /// `PostRotation` (`Rpost`) — the chain applies its **inverse**
    /// (doc §1 "watch the `Rpost` sign": the current documented form
    /// is `Rpost⁻¹`).
    pub post_rotation: [f64; 3],
    /// `ScalingOffset` (`Soff`) — sits *after* `Rp⁻¹`, not paired
    /// with `Sp` the way `Roff` pairs with `Rp` (doc §1).
    pub scaling_offset: [f64; 3],
    /// `ScalingPivot` (`Sp`).
    pub scaling_pivot: [f64; 3],
    /// `RotationOrder` for `Lcl Rotation`.
    pub rotation_order: RotationOrder,
}

impl Default for TransformChain {
    fn default() -> Self {
        Self {
            lcl_translation: [0.0; 3],
            lcl_rotation: [0.0; 3],
            lcl_scaling: [1.0, 1.0, 1.0],
            rotation_offset: [0.0; 3],
            rotation_pivot: [0.0; 3],
            pre_rotation: [0.0; 3],
            post_rotation: [0.0; 3],
            scaling_offset: [0.0; 3],
            scaling_pivot: [0.0; 3],
            rotation_order: RotationOrder::Xyz,
        }
    }
}

impl TransformChain {
    /// Compose the doc §1 product into an exact `(translation,
    /// rotation-quaternion xyzw, scale)` triple.
    ///
    /// The full chain
    ///
    /// ```text
    /// Local = T · Roff · Rp · Rpre · R · Rpost⁻¹ · Rp⁻¹ · Soff · Sp · S · Sp⁻¹
    /// ```
    ///
    /// always reduces to a single `T' · R' · S'`: with the rotation
    /// block `Q = Rpre · R · Rpost⁻¹` and the (diagonal) scale `S`,
    /// pushing the translations through `Q` (`Q · Trans(w) =
    /// Trans(Q·w) · Q`) and through `S` (`S · Trans(v) =
    /// Trans(S∘v) · S`, `∘` componentwise) collapses the product to
    ///
    /// ```text
    /// Local = Trans(T + Roff + Rp + Q·(Soff + Sp − Rp − S∘Sp)) · Q · S
    /// ```
    ///
    /// which is exactly the mesh3d `Trs` build order (`T * R * S`).
    /// No matrix decomposition — the reduction is algebraically exact.
    pub fn compose(&self) -> ([f64; 3], [f64; 4], [f64; 3]) {
        let q_r = euler_to_quat(self.lcl_rotation, self.rotation_order);
        // Pre/Post rotation triples are constructed with the default
        // XYZ order (doc §3 ties `RotationOrder` to the `R` term).
        let q_pre = euler_to_quat(self.pre_rotation, RotationOrder::Xyz);
        let q_post = euler_to_quat(self.post_rotation, RotationOrder::Xyz);
        let q = quat_mul(q_pre, quat_mul(q_r, quat_conjugate(q_post)));

        let s = self.lcl_scaling;
        let sp = self.scaling_pivot;
        // w = Soff + Sp − Rp − S∘Sp
        let w = [
            self.scaling_offset[0] + sp[0] - self.rotation_pivot[0] - s[0] * sp[0],
            self.scaling_offset[1] + sp[1] - self.rotation_pivot[1] - s[1] * sp[1],
            self.scaling_offset[2] + sp[2] - self.rotation_pivot[2] - s[2] * sp[2],
        ];
        let qw = rotate_vec(q, w);
        let t = [
            self.lcl_translation[0] + self.rotation_offset[0] + self.rotation_pivot[0] + qw[0],
            self.lcl_translation[1] + self.rotation_offset[1] + self.rotation_pivot[1] + qw[1],
            self.lcl_translation[2] + self.rotation_offset[2] + self.rotation_pivot[2] + qw[2],
        ];
        (t, q, s)
    }

    /// Inverse of [`TransformChain::compose`] for one composed
    /// sample: given a composed `(translation, rotation-quat, scale)`
    /// triple, recover the authored `(Lcl Translation, Lcl Rotation
    /// Euler-degrees, Lcl Scaling)` under this chain's static pivots
    /// / offsets / Pre-/PostRotation / rotation order:
    ///
    /// ```text
    /// R    = Rpre⁻¹ · Q · Rpost          (then Euler-extracted in `rotation_order`)
    /// T    = t − Roff − Rp − Q·(Soff + Sp − Rp − S∘Sp)
    /// S    = s
    /// ```
    ///
    /// Used by the encode side to write **authored** `Lcl` animation
    /// curves for a chain-bearing node — emitting the composed values
    /// verbatim would double-apply the chain on the next decode.
    pub fn decompose_sample(
        &self,
        translation: [f64; 3],
        rotation: [f64; 4],
        scale: [f64; 3],
    ) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let q_pre = euler_to_quat(self.pre_rotation, RotationOrder::Xyz);
        let q_post = euler_to_quat(self.post_rotation, RotationOrder::Xyz);
        let q_r = quat_mul(quat_conjugate(q_pre), quat_mul(rotation, q_post));
        let lcl_rotation = quat_to_euler(q_r, self.rotation_order);

        let sp = self.scaling_pivot;
        let w = [
            self.scaling_offset[0] + sp[0] - self.rotation_pivot[0] - scale[0] * sp[0],
            self.scaling_offset[1] + sp[1] - self.rotation_pivot[1] - scale[1] * sp[1],
            self.scaling_offset[2] + sp[2] - self.rotation_pivot[2] - scale[2] * sp[2],
        ];
        let qw = rotate_vec(rotation, w);
        let lcl_translation = [
            translation[0] - self.rotation_offset[0] - self.rotation_pivot[0] - qw[0],
            translation[1] - self.rotation_offset[1] - self.rotation_pivot[1] - qw[1],
            translation[2] - self.rotation_offset[2] - self.rotation_pivot[2] - qw[2],
        ];
        (lcl_translation, lcl_rotation, scale)
    }

    /// `true` when any record beyond the plain `Lcl` triple is
    /// non-trivial, i.e. the composed `Trs` is not simply
    /// `T · R(XYZ) · S` of the raw `Lcl` values.
    pub fn has_extensions(&self) -> bool {
        self.rotation_order != RotationOrder::Xyz
            || nonzero(self.rotation_offset)
            || nonzero(self.rotation_pivot)
            || nonzero(self.pre_rotation)
            || nonzero(self.post_rotation)
            || nonzero(self.scaling_offset)
            || nonzero(self.scaling_pivot)
    }
}

/// Convert a Euler-degree triple (`deg[0]` about X, `deg[1]` about Y,
/// `deg[2]` about Z) to an xyzw quaternion under the given
/// [`RotationOrder`]: for application axes `[a, b, c]` the product is
/// `q_c * q_b * q_a` (Hamilton, right-to-left application).
pub fn euler_to_quat(deg: [f64; 3], order: RotationOrder) -> [f64; 4] {
    let [a, b, c] = order.application_axes();
    let qa = axis_quat(a, deg[a]);
    let qb = axis_quat(b, deg[b]);
    let qc = axis_quat(c, deg[c]);
    quat_mul(qc, quat_mul(qb, qa))
}

/// Recover a Euler-degree triple from an xyzw rotation quaternion
/// under the given [`RotationOrder`] — the inverse of
/// [`euler_to_quat`] (`euler_to_quat(quat_to_euler(q, o), o)` is the
/// same rotation as `q`, up to the usual `q ≡ −q` ambiguity).
///
/// Extraction works on the rotation-matrix entries: with application
/// axes `[a, b, c]` (so `R = R_c · R_b · R_a`) and `ε` the parity of
/// the permutation `(a, b, c)`,
///
/// ```text
/// sin β = −ε·R[c][a]
/// α = atan2(ε·R[c][b], R[c][c])      (about axis a)
/// γ = atan2(ε·R[b][a], R[a][a])      (about axis c)
/// ```
///
/// At the gimbal singularity (`|sin β| = 1`) the `a`/`c` rotations
/// share an axis; the conventional `γ = 0` representative is
/// returned.
pub fn quat_to_euler(q: [f64; 4], order: RotationOrder) -> [f64; 3] {
    let [a, b, c] = order.application_axes();
    // Permutation parity of (a, b, c): +1 for the cyclic (even)
    // permutations of (0, 1, 2).
    let eps = if (a + 1) % 3 == b { 1.0 } else { -1.0 };
    let m = quat_to_mat3(q);

    let sin_b = (-eps * m[c][a]).clamp(-1.0, 1.0);
    let mut deg = [0.0; 3];
    if sin_b.abs() < 1.0 - 1e-9 {
        deg[a] = (eps * m[c][b]).atan2(m[c][c]).to_degrees();
        deg[b] = sin_b.asin().to_degrees();
        deg[c] = (eps * m[b][a]).atan2(m[a][a]).to_degrees();
    } else {
        // Gimbal lock: β = ±90°, γ pinned to 0.
        let sigma = sin_b.signum();
        deg[a] = (sigma * m[a][b]).atan2(sigma * eps * m[a][c]).to_degrees();
        deg[b] = sigma * 90.0;
        deg[c] = 0.0;
    }
    deg
}

/// 3×3 rotation matrix (column-vector convention) from an xyzw unit
/// quaternion.
fn quat_to_mat3(q: [f64; 4]) -> [[f64; 3]; 3] {
    let [x, y, z, w] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Unit quaternion for a rotation of `deg` degrees about axis `axis`
/// (`0` = X, `1` = Y, `2` = Z), xyzw layout.
fn axis_quat(axis: usize, deg: f64) -> [f64; 4] {
    let half = deg.to_radians() * 0.5;
    let (s, w) = (half.sin(), half.cos());
    let mut q = [0.0, 0.0, 0.0, w];
    q[axis] = s;
    q
}

/// Hamilton quaternion product, xyzw layout.
pub(crate) fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// Conjugate == inverse for unit quaternions.
pub(crate) fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    [-q[0], -q[1], -q[2], q[3]]
}

/// Rotate a vector by a unit quaternion (`q · v · q⁻¹`).
pub(crate) fn rotate_vec(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let p = [v[0], v[1], v[2], 0.0];
    let r = quat_mul(quat_mul(q, p), quat_conjugate(q));
    [r[0], r[1], r[2]]
}

fn nonzero(v: [f64; 3]) -> bool {
    v.iter().any(|c| *c != 0.0)
}

/// Decode each `Model` element's transform-chain P-records into the
/// owning scene-graph node's local [`Transform`] (full doc §1
/// composition), surfacing raw chain / geometric-TRS / `InheritType`
/// components on `Node::extras` where applicable.
///
/// `model_nodes` maps each `Model` FBX id to the `NodeId`
/// `crate::scene::build_scene` created for it. Returns the number of
/// nodes whose transform was set to a non-identity `Trs`.
pub fn extract_node_transforms(
    doc: &FbxDocument,
    scene: &mut Scene3D,
    model_nodes: &HashMap<i64, NodeId>,
) -> usize {
    let definitions = Definitions::from_root(&doc.root);
    let model_template = definitions.template_for("Model");

    let mut applied = 0usize;
    let Some(objects) = doc.root.child("Objects") else {
        return 0;
    };
    for child in objects.children_named("Model") {
        let Some(id) = element_id(child) else {
            continue;
        };
        let Some(&nid) = model_nodes.get(&id) else {
            continue;
        };

        // Resolve own records over the `ObjectType: "Model"` template
        // defaults, mirroring the material decoder's resolution path.
        let own = PropertyMap::from_element(child);
        let resolved = match model_template {
            Some(t) => own.with_template_defaults(t),
            None => own,
        };

        let decoded = decode_local_transform(&resolved);
        let Some(node) = scene.nodes.get_mut(nid.0 as usize) else {
            continue;
        };

        match decoded.local {
            LocalTransform::Trs {
                translation,
                rotation,
                scale,
            } => {
                node.transform = Transform::Trs {
                    translation,
                    rotation,
                    scale,
                };
                if translation != [0.0; 3] || rotation != [0.0, 0.0, 0.0, 1.0] || scale != [1.0; 3]
                {
                    applied += 1;
                }
            }
            LocalTransform::Incomplete { reason } => {
                // Input outside the documented tables — leave the node
                // at identity but mark it so nothing is silently wrong.
                node.extras.insert(
                    "fbx:transform_incomplete".to_string(),
                    serde_json::Value::String(reason.to_string()),
                );
            }
        }
        for (key, value) in decoded.extras {
            node.extras.insert(key, value);
        }
    }
    applied
}

/// One decoded `Model` transform: the composed local form plus the
/// extras to surface alongside it.
struct DecodedTransform {
    local: LocalTransform,
    extras: Vec<(String, serde_json::Value)>,
}

/// The composed local transform of one `Model`.
enum LocalTransform {
    /// The full doc §1 chain, composed exactly.
    Trs {
        translation: [f32; 3],
        rotation: [f32; 4],
        scale: [f32; 3],
    },
    /// Input outside the documented tables (a `RotationOrder` enum
    /// int beyond `0..=6`) — the node stays at identity; the raw
    /// components are surfaced on `extras`.
    Incomplete { reason: &'static str },
}

/// Read the chain fields from a resolved `Properties70` map. The
/// returned chain's `rotation_order` stays at the XYZ default; the
/// raw `RotationOrder` int (when present) rides alongside so callers
/// apply their own out-of-table policy.
pub(crate) fn chain_from_props(props: &PropertyMap) -> (TransformChain, Option<i64>) {
    let chain = TransformChain {
        lcl_translation: props
            .as_lcl_translation("Lcl Translation")
            .unwrap_or([0.0; 3]),
        lcl_rotation: props.as_lcl_rotation("Lcl Rotation").unwrap_or([0.0; 3]),
        lcl_scaling: props
            .as_lcl_scaling("Lcl Scaling")
            .unwrap_or([1.0, 1.0, 1.0]),
        rotation_offset: vec3_or_zero(props, "RotationOffset"),
        rotation_pivot: vec3_or_zero(props, "RotationPivot"),
        pre_rotation: vec3_or_zero(props, "PreRotation"),
        post_rotation: vec3_or_zero(props, "PostRotation"),
        scaling_offset: vec3_or_zero(props, "ScalingOffset"),
        scaling_pivot: vec3_or_zero(props, "ScalingPivot"),
        rotation_order: RotationOrder::Xyz,
    };
    (chain, props.as_enum("RotationOrder").map(i64::from))
}

/// Resolve every `Model`'s effective (template-resolved)
/// [`TransformChain`], keyed by FBX element id. Models whose
/// `RotationOrder` int falls outside the documented `0..=6` table are
/// omitted (the scene decode marks those `fbx:transform_incomplete`).
/// Used by the animation module to slot animated `Lcl` values into
/// the middle of the doc §1 chain.
pub fn model_chains(doc: &FbxDocument) -> HashMap<i64, TransformChain> {
    let definitions = Definitions::from_root(&doc.root);
    let template = definitions.template_for("Model");
    let mut out = HashMap::new();
    let Some(objects) = doc.root.child("Objects") else {
        return out;
    };
    for child in objects.children_named("Model") {
        let Some(id) = element_id(child) else {
            continue;
        };
        let own = PropertyMap::from_element(child);
        let resolved = match template {
            Some(t) => own.with_template_defaults(t),
            None => own,
        };
        let (mut chain, order_int) = chain_from_props(&resolved);
        match order_int.map(RotationOrder::from_enum_int) {
            Some(None) => continue,
            Some(Some(order)) => chain.rotation_order = order,
            None => {}
        }
        out.insert(id, chain);
    }
    out
}

/// Resolve a `Model`'s effective `Properties70` into its composed
/// local transform + companion extras.
fn decode_local_transform(props: &PropertyMap) -> DecodedTransform {
    let (mut chain, order_int) = chain_from_props(props);

    let mut extras: Vec<(String, serde_json::Value)> = Vec::new();

    // Geometric TRS (doc §2) — never composed into the node transform
    // (non-inheriting, geometry-only); surfaced raw when non-trivial.
    let geo_t = vec3_or_zero(props, "GeometricTranslation");
    let geo_r = vec3_or_zero(props, "GeometricRotation");
    let geo_s = props
        .as_vector3d("GeometricScaling")
        .unwrap_or([1.0, 1.0, 1.0]);
    if nonzero(geo_t) {
        extras.push(("fbx:geometric_translation".to_string(), json_vec3(geo_t)));
    }
    if nonzero(geo_r) {
        extras.push(("fbx:geometric_rotation".to_string(), json_vec3(geo_r)));
    }
    if geo_s != [1.0, 1.0, 1.0] {
        extras.push(("fbx:geometric_scaling".to_string(), json_vec3(geo_s)));
    }

    // InheritType (doc §4) — a world-composition selector, surfaced
    // raw when non-default; `crate::inherit` consumes it.
    if let Some(inherit) = props.as_enum("InheritType") {
        if inherit != 0 {
            extras.push((
                "fbx:inherit_type".to_string(),
                serde_json::Value::from(i64::from(inherit)),
            ));
        }
    }

    let order = match order_int {
        None => Some(RotationOrder::Xyz),
        Some(v) => RotationOrder::from_enum_int(v),
    };

    let Some(order) = order else {
        // Enum int outside the doc §3 table: surface everything raw
        // and mark the node.
        push_raw_chain_extras(&mut extras, &chain);
        if let Some(v) = order_int {
            extras.push(("fbx:rotation_order".to_string(), serde_json::Value::from(v)));
        }
        return DecodedTransform {
            local: LocalTransform::Incomplete {
                reason: "rotation_order_unrecognized",
            },
            extras,
        };
    };
    chain.rotation_order = order;

    if chain.has_extensions() {
        // Surface the authored chain so the encode side re-emits it
        // verbatim rather than the composed reduction.
        push_raw_chain_extras(&mut extras, &chain);
        if order != RotationOrder::Xyz {
            extras.push((
                "fbx:rotation_order".to_string(),
                serde_json::Value::from(order.to_enum_int()),
            ));
        }
    }

    let (t, q, s) = chain.compose();
    DecodedTransform {
        local: LocalTransform::Trs {
            translation: vec3_f32(t),
            rotation: [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32],
            scale: vec3_f32(s),
        },
        extras,
    }
}

/// Surface the raw authored chain components (`Lcl` triple + every
/// non-zero extension) for lossless re-encode.
fn push_raw_chain_extras(extras: &mut Vec<(String, serde_json::Value)>, chain: &TransformChain) {
    extras.push((
        "fbx:lcl_translation".to_string(),
        json_vec3(chain.lcl_translation),
    ));
    extras.push((
        "fbx:lcl_rotation".to_string(),
        json_vec3(chain.lcl_rotation),
    ));
    extras.push(("fbx:lcl_scaling".to_string(), json_vec3(chain.lcl_scaling)));
    for (key, v) in [
        ("fbx:rotation_offset", chain.rotation_offset),
        ("fbx:rotation_pivot", chain.rotation_pivot),
        ("fbx:pre_rotation", chain.pre_rotation),
        ("fbx:post_rotation", chain.post_rotation),
        ("fbx:scaling_offset", chain.scaling_offset),
        ("fbx:scaling_pivot", chain.scaling_pivot),
    ] {
        if nonzero(v) {
            extras.push((key.to_string(), json_vec3(v)));
        }
    }
}

/// Rebuild the non-inheriting geometric transform (`OT · OR · OS`,
/// doc §2) a consumer must post-multiply onto this node's **world**
/// matrix when transforming the node's own mesh — and only the mesh:
/// children never inherit it. Returns `None` when the node carries no
/// geometric-TRS extras (the common case; the offset is then
/// identity).
///
/// `GeometricRotation` is constructed with the default XYZ order.
pub fn geometric_transform(node: &Node) -> Option<Transform> {
    let t = extras_vec3(node, "fbx:geometric_translation");
    let r = extras_vec3(node, "fbx:geometric_rotation");
    let s = extras_vec3(node, "fbx:geometric_scaling");
    if t.is_none() && r.is_none() && s.is_none() {
        return None;
    }
    let q = euler_to_quat(r.unwrap_or([0.0; 3]), RotationOrder::Xyz);
    Some(Transform::Trs {
        translation: vec3_f32(t.unwrap_or([0.0; 3])),
        rotation: [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32],
        scale: vec3_f32(s.unwrap_or([1.0, 1.0, 1.0])),
    })
}

/// Rebuild the authored [`TransformChain`] from a node's `fbx:*`
/// chain extras — the encode-side inverse of the raw-chain surfacing
/// in [`extract_node_transforms`]. Returns `None` when the node
/// carries no `fbx:lcl_*` chain extras (plain node — its
/// `Node::transform` is authoritative) or when `fbx:rotation_order`
/// falls outside the documented `0..=6` table.
pub fn chain_from_extras(node: &Node) -> Option<TransformChain> {
    let has = |k: &str| node.extras.contains_key(k);
    if !(has("fbx:lcl_translation") || has("fbx:lcl_rotation") || has("fbx:lcl_scaling")) {
        return None;
    }
    let rotation_order = match node
        .extras
        .get("fbx:rotation_order")
        .and_then(|v| v.as_i64())
    {
        None => RotationOrder::Xyz,
        Some(v) => RotationOrder::from_enum_int(v)?,
    };
    Some(TransformChain {
        lcl_translation: extras_vec3(node, "fbx:lcl_translation").unwrap_or([0.0; 3]),
        lcl_rotation: extras_vec3(node, "fbx:lcl_rotation").unwrap_or([0.0; 3]),
        lcl_scaling: extras_vec3(node, "fbx:lcl_scaling").unwrap_or([1.0, 1.0, 1.0]),
        rotation_offset: extras_vec3(node, "fbx:rotation_offset").unwrap_or([0.0; 3]),
        rotation_pivot: extras_vec3(node, "fbx:rotation_pivot").unwrap_or([0.0; 3]),
        pre_rotation: extras_vec3(node, "fbx:pre_rotation").unwrap_or([0.0; 3]),
        post_rotation: extras_vec3(node, "fbx:post_rotation").unwrap_or([0.0; 3]),
        scaling_offset: extras_vec3(node, "fbx:scaling_offset").unwrap_or([0.0; 3]),
        scaling_pivot: extras_vec3(node, "fbx:scaling_pivot").unwrap_or([0.0; 3]),
        rotation_order,
    })
}

/// Read a `[f64; 3]` JSON array off `Node::extras`. Shared with the
/// encode side ([`crate::scene_writer`]), which re-emits the chain
/// records from the same extras keys.
pub(crate) fn extras_vec3(node: &Node, key: &str) -> Option<[f64; 3]> {
    let arr = node.extras.get(key)?.as_array()?;
    if arr.len() != 3 {
        return None;
    }
    Some([arr[0].as_f64()?, arr[1].as_f64()?, arr[2].as_f64()?])
}

/// A `Vector3D` record's value, defaulting to zero when absent.
fn vec3_or_zero(props: &PropertyMap, name: &str) -> [f64; 3] {
    props.as_vector3d(name).unwrap_or([0.0; 3])
}

fn vec3_f32(v: [f64; 3]) -> [f32; 3] {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

fn json_vec3(v: [f64; 3]) -> serde_json::Value {
    serde_json::json!([v[0], v[1], v[2]])
}

/// Read property[0] (the FBX element id) of an `Objects`-child record.
fn element_id(n: &FbxNode) -> Option<i64> {
    n.properties.first().and_then(FbxProperty::as_i64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary::FbxProperty;

    /// Build a `P` record node: `P` with `[name, type, label, flags,
    /// values...]` string + numeric properties.
    fn p_vec3(name: &str, type_name: &str, v: [f64; 3]) -> FbxNode {
        FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(name.as_bytes().to_vec()),
                FbxProperty::String(type_name.as_bytes().to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"A".to_vec()),
                FbxProperty::F64(v[0]),
                FbxProperty::F64(v[1]),
                FbxProperty::F64(v[2]),
            ],
            children: vec![],
        }
    }

    fn p_enum(name: &str, value: i32) -> FbxNode {
        FbxNode {
            name: "P".to_string(),
            properties: vec![
                FbxProperty::String(name.as_bytes().to_vec()),
                FbxProperty::String(b"enum".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::String(b"".to_vec()),
                FbxProperty::I32(value),
            ],
            children: vec![],
        }
    }

    fn props70(records: Vec<FbxNode>) -> PropertyMap {
        let node = FbxNode {
            name: "Properties70".to_string(),
            properties: vec![],
            children: records,
        };
        PropertyMap::from_properties70(&node)
    }

    fn expect_trs(d: &DecodedTransform) -> ([f32; 3], [f32; 4], [f32; 3]) {
        match d.local {
            LocalTransform::Trs {
                translation,
                rotation,
                scale,
            } => (translation, rotation, scale),
            LocalTransform::Incomplete { reason } => {
                panic!("expected composed Trs, got Incomplete({reason})")
            }
        }
    }

    fn extras_map(d: &DecodedTransform) -> HashMap<&str, &serde_json::Value> {
        d.extras.iter().map(|(k, v)| (k.as_str(), v)).collect()
    }

    // ---- matrix oracle -------------------------------------------

    type Mat4 = [[f64; 4]; 4];

    fn mat_identity() -> Mat4 {
        let mut m = [[0.0; 4]; 4];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        m
    }

    fn mat_mul(a: Mat4, b: Mat4) -> Mat4 {
        let mut out = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                out[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
            }
        }
        out
    }

    fn mat_translate(v: [f64; 3]) -> Mat4 {
        let mut m = mat_identity();
        m[0][3] = v[0];
        m[1][3] = v[1];
        m[2][3] = v[2];
        m
    }

    fn mat_scale(v: [f64; 3]) -> Mat4 {
        let mut m = mat_identity();
        m[0][0] = v[0];
        m[1][1] = v[1];
        m[2][2] = v[2];
        m
    }

    /// Elementary rotation about one axis, degrees, column-vector
    /// convention.
    fn mat_rot_axis(axis: usize, deg: f64) -> Mat4 {
        let r = deg.to_radians();
        let (s, c) = (r.sin(), r.cos());
        let mut m = mat_identity();
        match axis {
            0 => {
                m[1][1] = c;
                m[1][2] = -s;
                m[2][1] = s;
                m[2][2] = c;
            }
            1 => {
                m[0][0] = c;
                m[0][2] = s;
                m[2][0] = -s;
                m[2][2] = c;
            }
            _ => {
                m[0][0] = c;
                m[0][1] = -s;
                m[1][0] = s;
                m[1][1] = c;
            }
        }
        m
    }

    /// Euler triple → rotation matrix under an order: axes `[a,b,c]`
    /// applied first-to-last means the product `R_c · R_b · R_a`.
    fn mat_euler(deg: [f64; 3], order: RotationOrder) -> Mat4 {
        let [a, b, c] = order.application_axes();
        mat_mul(
            mat_rot_axis(c, deg[c]),
            mat_mul(mat_rot_axis(b, deg[b]), mat_rot_axis(a, deg[a])),
        )
    }

    fn mat_transpose(m: Mat4) -> Mat4 {
        let mut out = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                out[i][j] = m[j][i];
            }
        }
        out
    }

    fn quat_to_mat(q: [f64; 4]) -> Mat4 {
        let [x, y, z, w] = q;
        let mut m = mat_identity();
        m[0][0] = 1.0 - 2.0 * (y * y + z * z);
        m[0][1] = 2.0 * (x * y - z * w);
        m[0][2] = 2.0 * (x * z + y * w);
        m[1][0] = 2.0 * (x * y + z * w);
        m[1][1] = 1.0 - 2.0 * (x * x + z * z);
        m[1][2] = 2.0 * (y * z - x * w);
        m[2][0] = 2.0 * (x * z - y * w);
        m[2][1] = 2.0 * (y * z + x * w);
        m[2][2] = 1.0 - 2.0 * (x * x + y * y);
        m
    }

    /// Literal doc §1 product:
    /// `T · Roff · Rp · Rpre · R · Rpost⁻¹ · Rp⁻¹ · Soff · Sp · S · Sp⁻¹`.
    fn chain_matrix_literal(c: &TransformChain) -> Mat4 {
        let neg = |v: [f64; 3]| [-v[0], -v[1], -v[2]];
        let factors = [
            mat_translate(c.lcl_translation),
            mat_translate(c.rotation_offset),
            mat_translate(c.rotation_pivot),
            mat_euler(c.pre_rotation, RotationOrder::Xyz),
            mat_euler(c.lcl_rotation, c.rotation_order),
            // Rpost⁻¹ — orthonormal, so inverse == transpose.
            mat_transpose(mat_euler(c.post_rotation, RotationOrder::Xyz)),
            mat_translate(neg(c.rotation_pivot)),
            mat_translate(c.scaling_offset),
            mat_translate(c.scaling_pivot),
            mat_scale(c.lcl_scaling),
            mat_translate(neg(c.scaling_pivot)),
        ];
        factors.into_iter().fold(mat_identity(), mat_mul)
    }

    fn assert_mat_close(a: Mat4, b: Mat4, tol: f64) {
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (a[i][j] - b[i][j]).abs() < tol,
                    "matrix mismatch at [{i}][{j}]: {} vs {}\n{a:?}\n{b:?}",
                    a[i][j],
                    b[i][j],
                );
            }
        }
    }

    /// The composed closed form must equal the literal doc §1 matrix
    /// product for a fully non-trivial chain, in every rotation order.
    #[test]
    fn compose_matches_literal_chain_product_all_orders() {
        for order_int in 0..=6 {
            let order = RotationOrder::from_enum_int(order_int).unwrap();
            let chain = TransformChain {
                lcl_translation: [1.5, -2.0, 3.25],
                lcl_rotation: [30.0, -45.0, 60.0],
                lcl_scaling: [2.0, 0.5, 1.25],
                rotation_offset: [0.25, 0.5, -0.75],
                rotation_pivot: [1.0, 2.0, -1.0],
                pre_rotation: [10.0, 20.0, -30.0],
                post_rotation: [-15.0, 5.0, 25.0],
                scaling_offset: [-0.5, 0.25, 0.125],
                scaling_pivot: [0.5, -1.5, 2.5],
                rotation_order: order,
            };
            let (t, q, s) = chain.compose();
            let composed = mat_mul(mat_translate(t), mat_mul(quat_to_mat(q), mat_scale(s)));
            assert_mat_close(composed, chain_matrix_literal(&chain), 1e-9);
        }
    }

    /// A rotation about a pivot leaves the pivot point fixed.
    #[test]
    fn rotation_pivot_point_is_fixed() {
        let chain = TransformChain {
            lcl_rotation: [0.0, 0.0, 90.0],
            rotation_pivot: [1.0, 2.0, 3.0],
            ..TransformChain::default()
        };
        let (t, q, _s) = chain.compose();
        let p = chain.rotation_pivot;
        let rotated = rotate_vec(q, p);
        let moved = [rotated[0] + t[0], rotated[1] + t[1], rotated[2] + t[2]];
        for i in 0..3 {
            assert!((moved[i] - p[i]).abs() < 1e-12, "pivot moved: {moved:?}");
        }
        // And a probe one unit +X of the pivot swings to one unit +Y.
        let probe = [2.0, 2.0, 3.0];
        let rp = rotate_vec(q, probe);
        let mp = [rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]];
        assert!((mp[0] - 1.0).abs() < 1e-12 && (mp[1] - 3.0).abs() < 1e-12);
    }

    /// A scale about a pivot leaves the pivot point fixed.
    #[test]
    fn scaling_pivot_point_is_fixed() {
        let chain = TransformChain {
            lcl_scaling: [2.0, 3.0, 4.0],
            scaling_pivot: [1.0, -1.0, 2.0],
            ..TransformChain::default()
        };
        let (t, _q, s) = chain.compose();
        let p = chain.scaling_pivot;
        let moved = [s[0] * p[0] + t[0], s[1] * p[1] + t[1], s[2] * p[2] + t[2]];
        for i in 0..3 {
            assert!((moved[i] - p[i]).abs() < 1e-12, "pivot moved: {moved:?}");
        }
    }

    /// `RotationOffset` translates in parent space (outside `Q`);
    /// `ScalingOffset` sits inside the rotation block (doc §1 "Soff
    /// placement").
    #[test]
    fn offset_placement_straddles_rotation() {
        // Roff with a 90° Z rotation: offset unrotated.
        let roff = TransformChain {
            lcl_rotation: [0.0, 0.0, 90.0],
            rotation_offset: [0.0, 1.0, 0.0],
            ..TransformChain::default()
        };
        let (t, _, _) = roff.compose();
        assert!((t[0]).abs() < 1e-12 && (t[1] - 1.0).abs() < 1e-12);

        // Soff with the same rotation: offset rides through Q, so
        // +Y swings to -X.
        let soff = TransformChain {
            lcl_rotation: [0.0, 0.0, 90.0],
            scaling_offset: [0.0, 1.0, 0.0],
            ..TransformChain::default()
        };
        let (t, _, _) = soff.compose();
        assert!((t[0] + 1.0).abs() < 1e-12 && (t[1]).abs() < 1e-12);
    }

    /// `PostRotation` equal to the local rotation (with no
    /// pre-rotation) cancels it: `Q = R · R⁻¹ = I`.
    #[test]
    fn post_rotation_applies_inverse() {
        let chain = TransformChain {
            lcl_rotation: [25.0, -40.0, 65.0],
            post_rotation: [25.0, -40.0, 65.0],
            ..TransformChain::default()
        };
        let (_, q, _) = chain.compose();
        assert!(
            (q[3].abs() - 1.0).abs() < 1e-12 && q[0].abs() < 1e-12,
            "expected identity rotation, got {q:?}"
        );
    }

    /// `PreRotation` multiplies from the left: with `R` = 90° about Z
    /// and `Rpre` = 90° about X, +X → (rotate Z) +Y → (pre X) +Z.
    #[test]
    fn pre_rotation_composes_left_of_r() {
        let chain = TransformChain {
            lcl_rotation: [0.0, 0.0, 90.0],
            pre_rotation: [90.0, 0.0, 0.0],
            ..TransformChain::default()
        };
        let (_, q, _) = chain.compose();
        let v = rotate_vec(q, [1.0, 0.0, 0.0]);
        assert!(
            v[0].abs() < 1e-12 && v[1].abs() < 1e-12 && (v[2] - 1.0).abs() < 1e-12,
            "got {v:?}"
        );
    }

    /// Rotation orders apply the named axes first-to-last: (90, 0, 90)
    /// under XYZ takes +Y to +Z (X first), under ZYX takes +Y to -X
    /// (Z first).
    #[test]
    fn rotation_order_discriminates() {
        let xyz = euler_to_quat([90.0, 0.0, 90.0], RotationOrder::Xyz);
        let v = rotate_vec(xyz, [0.0, 1.0, 0.0]);
        assert!(
            v[2] > 0.999,
            "XYZ: expected +Y → +Z (X applied first), got {v:?}"
        );

        let zyx = euler_to_quat([90.0, 0.0, 90.0], RotationOrder::Zyx);
        let v = rotate_vec(zyx, [0.0, 1.0, 0.0]);
        assert!(
            v[0] < -0.999,
            "ZYX: expected +Y → -X (Z applied first), got {v:?}"
        );
    }

    // ---- decode surface ------------------------------------------

    #[test]
    fn pure_trs_decodes_translation_scale() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 2.0, 3.0]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [10.0, 10.0, 10.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, rotation, scale) = expect_trs(&d);
        assert_eq!(translation, [1.0, 2.0, 3.0]);
        assert_eq!(scale, [10.0, 10.0, 10.0]);
        // No Lcl Rotation → identity quaternion.
        assert!((rotation[3] - 1.0).abs() < 1e-6);
        assert!(rotation[0].abs() < 1e-6);
        // No extension records → no chain extras.
        assert!(d.extras.is_empty());
    }

    #[test]
    fn missing_records_default_to_identity_trs() {
        let map = props70(vec![]);
        let d = decode_local_transform(&map);
        let (translation, rotation, scale) = expect_trs(&d);
        assert_eq!(translation, [0.0, 0.0, 0.0]);
        assert_eq!(scale, [1.0, 1.0, 1.0]);
        assert!((rotation[3] - 1.0).abs() < 1e-6);
        assert!(d.extras.is_empty());
    }

    #[test]
    fn lcl_rotation_90_about_x_becomes_quat() {
        let map = props70(vec![p_vec3(
            "Lcl Rotation",
            "Lcl Rotation",
            [90.0, 0.0, 0.0],
        )]);
        let d = decode_local_transform(&map);
        let (_, rotation, _) = expect_trs(&d);
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert!((rotation[0] - s).abs() < 1e-5);
        assert!((rotation[3] - s).abs() < 1e-5);
    }

    /// Non-zero `PreRotation` now composes (doc §1) instead of
    /// bailing to identity, and surfaces the raw chain.
    #[test]
    fn nonzero_pre_rotation_composes_and_surfaces_raw() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 0.0, 0.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 90.0, 0.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, rotation, _) = expect_trs(&d);
        assert_eq!(translation, [1.0, 0.0, 0.0]);
        // Q = Rpre: 90° about Y.
        let h = std::f32::consts::FRAC_1_SQRT_2;
        assert!((rotation[1] - h).abs() < 1e-5 && (rotation[3] - h).abs() < 1e-5);
        let ex = extras_map(&d);
        assert!(ex.contains_key("fbx:pre_rotation"));
        assert!(ex.contains_key("fbx:lcl_translation"));
        assert!(!ex.contains_key("fbx:transform_incomplete"));
    }

    /// A rotation pivot composes into the exact closed form:
    /// Rp = (1,0,0), R = 90° about Z → t = Rp + Q·(−Rp) = (1,−1,0).
    #[test]
    fn rotation_pivot_composes() {
        let map = props70(vec![
            p_vec3("RotationPivot", "Vector3D", [1.0, 0.0, 0.0]),
            p_vec3("Lcl Rotation", "Lcl Rotation", [0.0, 0.0, 90.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, _, _) = expect_trs(&d);
        assert!((translation[0] - 1.0).abs() < 1e-6);
        assert!((translation[1] + 1.0).abs() < 1e-6);
        assert!(extras_map(&d).contains_key("fbx:rotation_pivot"));
    }

    /// Non-XYZ rotation orders compose per the doc §3 table and
    /// surface the raw enum.
    #[test]
    fn non_xyz_rotation_order_composes() {
        let map = props70(vec![
            p_enum("RotationOrder", 5),
            p_vec3("Lcl Rotation", "Lcl Rotation", [90.0, 0.0, 90.0]),
        ]);
        let d = decode_local_transform(&map);
        let (_, rotation, _) = expect_trs(&d);
        // ZYX: Z first — +Y → -X, then X: -X stays.
        let q = [
            f64::from(rotation[0]),
            f64::from(rotation[1]),
            f64::from(rotation[2]),
            f64::from(rotation[3]),
        ];
        let v = rotate_vec(q, [0.0, 1.0, 0.0]);
        assert!(v[0] < -0.999, "got {v:?}");
        assert_eq!(
            extras_map(&d)
                .get("fbx:rotation_order")
                .and_then(|v| v.as_i64()),
            Some(5)
        );
    }

    /// `SphericXYZ` (6) constructs its rest matrix as XYZ but keeps
    /// the raw enum recoverable.
    #[test]
    fn spheric_order_rest_pose_is_xyz() {
        let map = props70(vec![
            p_enum("RotationOrder", 6),
            p_vec3("Lcl Rotation", "Lcl Rotation", [30.0, 40.0, 50.0]),
        ]);
        let d = decode_local_transform(&map);
        let (_, rotation, _) = expect_trs(&d);
        let expect = euler_to_quat([30.0, 40.0, 50.0], RotationOrder::Xyz);
        for i in 0..4 {
            assert!((f64::from(rotation[i]) - expect[i]).abs() < 1e-6);
        }
        assert_eq!(
            extras_map(&d)
                .get("fbx:rotation_order")
                .and_then(|v| v.as_i64()),
            Some(6)
        );
    }

    /// An enum int outside the documented table stays honest:
    /// identity + marker.
    #[test]
    fn out_of_table_rotation_order_is_incomplete() {
        let map = props70(vec![
            p_enum("RotationOrder", 9),
            p_vec3("Lcl Translation", "Lcl Translation", [4.0, 5.0, 6.0]),
        ]);
        let d = decode_local_transform(&map);
        assert!(matches!(
            d.local,
            LocalTransform::Incomplete {
                reason: "rotation_order_unrecognized"
            }
        ));
        let ex = extras_map(&d);
        assert_eq!(
            ex.get("fbx:rotation_order").and_then(|v| v.as_i64()),
            Some(9)
        );
        assert!(ex.contains_key("fbx:lcl_translation"));
    }

    #[test]
    fn xyz_rotation_order_zero_stays_plain_trs() {
        let map = props70(vec![
            p_enum("RotationOrder", 0),
            p_vec3("Lcl Translation", "Lcl Translation", [5.0, 0.0, 0.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, _, _) = expect_trs(&d);
        assert_eq!(translation, [5.0, 0.0, 0.0]);
        assert!(d.extras.is_empty());
    }

    #[test]
    fn zero_extension_records_stay_plain_trs() {
        // All the template's zero-valued extension records present but
        // trivial — no chain extras (the fixture's case).
        let map = props70(vec![
            p_vec3("RotationOffset", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("RotationPivot", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("ScalingOffset", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("ScalingPivot", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 0.0, 0.0]),
            p_vec3("PostRotation", "Vector3D", [0.0, 0.0, 0.0]),
            p_enum("RotationOrder", 0),
            p_vec3("Lcl Translation", "Lcl Translation", [-1.04, 0.99, -1.04]),
            p_vec3("Lcl Scaling", "Lcl Scaling", [10.0, 10.0, 10.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, _, scale) = expect_trs(&d);
        assert!((translation[0] - (-1.04)).abs() < 1e-5);
        assert_eq!(scale, [10.0, 10.0, 10.0]);
        assert!(d.extras.is_empty());
    }

    /// Geometric TRS never touches the composed transform; it
    /// surfaces raw and rebuilds via [`geometric_transform`].
    #[test]
    fn geometric_trs_surfaces_without_composing() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 0.0, 0.0]),
            p_vec3("GeometricTranslation", "Vector3D", [0.0, 5.0, 0.0]),
            p_vec3("GeometricRotation", "Vector3D", [0.0, 0.0, 90.0]),
            p_vec3("GeometricScaling", "Vector3D", [2.0, 2.0, 2.0]),
        ]);
        let d = decode_local_transform(&map);
        let (translation, rotation, scale) = expect_trs(&d);
        // Node transform is the plain Lcl chain — geometric TRS
        // excluded (doc §2 non-inheritance).
        assert_eq!(translation, [1.0, 0.0, 0.0]);
        assert!((rotation[3] - 1.0).abs() < 1e-6);
        assert_eq!(scale, [1.0, 1.0, 1.0]);

        let mut node = Node::new();
        for (k, v) in &d.extras {
            node.extras.insert(k.clone(), v.clone());
        }
        let geo = geometric_transform(&node).expect("geometric transform present");
        match geo {
            Transform::Trs {
                translation,
                rotation,
                scale,
            } => {
                assert_eq!(translation, [0.0, 5.0, 0.0]);
                assert_eq!(scale, [2.0, 2.0, 2.0]);
                let h = std::f32::consts::FRAC_1_SQRT_2;
                assert!((rotation[2] - h).abs() < 1e-5 && (rotation[3] - h).abs() < 1e-5);
            }
            Transform::Matrix(_) => panic!("expected Trs"),
        }
    }

    #[test]
    fn geometric_transform_absent_is_none() {
        assert!(geometric_transform(&Node::new()).is_none());
    }

    /// Non-default `InheritType` surfaces raw (doc §4 leaves the
    /// formula open); default `0` stays silent.
    #[test]
    fn inherit_type_surfaces_when_nonzero() {
        let map = props70(vec![p_enum("InheritType", 1)]);
        let d = decode_local_transform(&map);
        assert_eq!(
            extras_map(&d)
                .get("fbx:inherit_type")
                .and_then(|v| v.as_i64()),
            Some(1)
        );

        let map = props70(vec![p_enum("InheritType", 0)]);
        let d = decode_local_transform(&map);
        assert!(d.extras.is_empty());
    }

    /// `quat_to_euler` inverts `euler_to_quat` (up to `q ≡ −q`) for
    /// every order, including gimbal-lock poses.
    #[test]
    fn quat_to_euler_inverts_euler_to_quat_all_orders() {
        let triples = [
            [30.0, -45.0, 60.0],
            [10.0, 20.0, -30.0],
            [-170.0, 15.0, 100.0],
            [0.0, 90.0, 0.0],
            [90.0, -90.0, 0.0],
            [45.0, 0.0, -90.0],
        ];
        for order_int in 0..=6 {
            let order = RotationOrder::from_enum_int(order_int).unwrap();
            for deg in triples {
                let q = euler_to_quat(deg, order);
                let back = quat_to_euler(q, order);
                let q2 = euler_to_quat(back, order);
                let dot: f64 = (0..4).map(|i| q[i] * q2[i]).sum();
                assert!(
                    dot.abs() > 1.0 - 1e-9,
                    "order {order:?}, deg {deg:?}: {q:?} vs {q2:?} (via {back:?})"
                );
            }
        }
    }

    /// `decompose_sample` inverts `compose` for a fully non-trivial
    /// chain in every rotation order.
    #[test]
    fn decompose_sample_inverts_compose() {
        for order_int in 0..=6 {
            let order = RotationOrder::from_enum_int(order_int).unwrap();
            let chain = TransformChain {
                lcl_translation: [1.5, -2.0, 3.25],
                lcl_rotation: [30.0, -45.0, 60.0],
                lcl_scaling: [2.0, 0.5, 1.25],
                rotation_offset: [0.25, 0.5, -0.75],
                rotation_pivot: [1.0, 2.0, -1.0],
                pre_rotation: [10.0, 20.0, -30.0],
                post_rotation: [-15.0, 5.0, 25.0],
                scaling_offset: [-0.5, 0.25, 0.125],
                scaling_pivot: [0.5, -1.5, 2.5],
                rotation_order: order,
            };
            let (t, q, s) = chain.compose();
            let (lt, lr, ls) = chain.decompose_sample(t, q, s);
            for i in 0..3 {
                assert!(
                    (lt[i] - chain.lcl_translation[i]).abs() < 1e-9,
                    "order {order:?}: T {lt:?}"
                );
                assert!((ls[i] - chain.lcl_scaling[i]).abs() < 1e-12);
            }
            // Euler angles may come back as an equivalent triple —
            // compare as rotations.
            let qa = euler_to_quat(lr, order);
            let qb = euler_to_quat(chain.lcl_rotation, order);
            let dot: f64 = (0..4).map(|i| qa[i] * qb[i]).sum();
            assert!(dot.abs() > 1.0 - 1e-9, "order {order:?}: R {lr:?}");
        }
    }

    /// `chain_from_extras` rebuilds exactly what the decode side
    /// surfaced.
    #[test]
    fn chain_from_extras_round_trips_surfaced_chain() {
        let map = props70(vec![
            p_vec3("Lcl Translation", "Lcl Translation", [1.0, 2.0, 3.0]),
            p_vec3("RotationPivot", "Vector3D", [4.0, 5.0, 6.0]),
            p_vec3("PreRotation", "Vector3D", [0.0, 30.0, 0.0]),
            p_enum("RotationOrder", 5),
        ]);
        let d = decode_local_transform(&map);
        let mut node = Node::new();
        for (k, v) in &d.extras {
            node.extras.insert(k.clone(), v.clone());
        }
        let chain = chain_from_extras(&node).expect("chain extras present");
        assert_eq!(chain.lcl_translation, [1.0, 2.0, 3.0]);
        assert_eq!(chain.rotation_pivot, [4.0, 5.0, 6.0]);
        assert_eq!(chain.pre_rotation, [0.0, 30.0, 0.0]);
        assert_eq!(chain.rotation_order, RotationOrder::Zyx);
        // Plain node → None.
        assert!(chain_from_extras(&Node::new()).is_none());
    }

    #[test]
    fn rotation_order_round_trips_enum_ints() {
        for v in 0..=6 {
            let o = RotationOrder::from_enum_int(v).unwrap();
            assert_eq!(o.to_enum_int(), v);
        }
        assert!(RotationOrder::from_enum_int(7).is_none());
        assert!(RotationOrder::from_enum_int(-1).is_none());
    }
}
