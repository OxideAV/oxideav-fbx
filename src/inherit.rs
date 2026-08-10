//! `InheritType` world-transform composition — how a parent's rotation
//! and scale propagate to its children, per
//! `docs/3d/fbx/fbx-node-transform-chain.md` §4.
//!
//! FBX nodes carry an `InheritType` `"enum"` property selecting one of
//! three propagation rules. The doc pins the wire integers (declaration
//! order, `0` = the default `RrSs`) and the three rotation-and-scale
//! products. Writing `P_R` for the parent's accumulated world rotation,
//! `P_S` for the parent's global scale-and-shear block (obtained by
//! stripping translation and then rotation from the parent's world
//! matrix — `P_S = P_R⁻¹ · P_T⁻¹ · P_world`), `L_R` / `L_S` for the
//! node's local rotation / scaling, and `p_s` for the parent's own
//! **local** `Lcl Scaling`:
//!
//! | wire | rule | rotation-and-scale product |
//! |------|------|----------------------------|
//! | `0` (`RrSs`) | parent scale applied in the child world, after the child's local rotation | `P_R · L_R · P_S · L_S` |
//! | `1` (`RSrs`) | parent scale applied in the parent world | `P_R · P_S · L_R · L_S` |
//! | `2` (`Rrs`) | parent scale not propagated to children | `P_R · L_R · (P_S · p_s⁻¹) · L_S` |
//!
//! Translation sits outside the product in all three cases (doc §4
//! "Composition"): the node's global translation is the parent's
//! **full** world matrix applied to the local translation the §1 pivot
//! chain produces, and the world matrix recombines as
//! `World = T_global · (rotation-and-scale product)`.
//!
//! Mode `1` is exactly the naive `World = ParentWorld · Local` matrix
//! concatenation (the composition every plain scene-graph consumer
//! applies), which is why ordinary Maya exports — whose `Model` nodes
//! carry `InheritType = 1` on the wire even though the template
//! defaults to `0` — render correctly under naive composition. Modes
//! `0` and `2` diverge from it exactly when the parent's global scale
//! block is non-uniform (mode `0`) or the parent authored a local
//! scale at all (mode `2`).
//!
//! # What this module deliberately does not do
//!
//! - The doc §2 **geometric transform** stays excluded: it is
//!   non-inheriting and applies to a node's own mesh only. The matrix
//!   to draw a node's mesh by is
//!   `world_transforms(...)[node] · geometric_transform(node)` (see
//!   [`crate::node_transform::geometric_transform`]).
//! - An `InheritType` int outside the documented `0..=2` table falls
//!   back to the default `RrSs` for composition (the raw value is
//!   already surfaced on `extras["fbx:inherit_type"]` by the decode
//!   side, so nothing is lost).
//! - A [`Transform::Matrix`] node (never produced by this crate's FBX
//!   decode, which always composes to `Trs`) has no rotation / scale
//!   split, so it composes naively and contributes its linear part to
//!   its children's inherited `P_S` block with an identity rotation.

use std::collections::HashMap;

use oxideav_mesh3d::{Node, NodeId, Scene3D, Transform};

use crate::node_transform::{chain_from_extras, quat_conjugate, quat_mul};

/// 4×4 matrix, column-vector convention (`out = M · v`), row-major
/// storage — `m[row][col]`, translation in `m[0..3][3]`.
pub type Mat4 = [[f64; 4]; 4];

/// The three documented `InheritType` propagation rules
/// (`docs/3d/fbx/fbx-node-transform-chain.md` §4 wire-value table).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InheritType {
    /// `0` — the default. Parent scale is applied in the child world,
    /// after the child's local rotation: `P_R · L_R · P_S · L_S`.
    #[default]
    RrSs,
    /// `1` — parent scale applied in the parent world:
    /// `P_R · P_S · L_R · L_S` (naive matrix concatenation). The value
    /// ordinary Maya exports carry on the wire.
    RSrs,
    /// `2` — parent scale not propagated:
    /// `P_R · L_R · (P_S · p_s⁻¹) · L_S`, i.e. the parent's own local
    /// scale is divided back out of the inherited global block.
    Rrs,
}

impl InheritType {
    /// Map the stored `"enum"` integer to a rule. `None` outside the
    /// documented `0..=2` table.
    pub fn from_enum_int(v: i64) -> Option<Self> {
        Some(match v {
            0 => Self::RrSs,
            1 => Self::RSrs,
            2 => Self::Rrs,
            _ => return None,
        })
    }

    /// The stored `"enum"` integer for this rule.
    pub fn to_enum_int(self) -> i64 {
        match self {
            Self::RrSs => 0,
            Self::RSrs => 1,
            Self::Rrs => 2,
        }
    }
}

/// Read a node's effective [`InheritType`] from the
/// `extras["fbx:inherit_type"]` surface the decode side populates
/// (absent — the template default `0` — or out-of-table both resolve
/// to [`InheritType::RrSs`]).
pub fn inherit_type_of(node: &Node) -> InheritType {
    node.extras
        .get("fbx:inherit_type")
        .and_then(|v| v.as_i64())
        .and_then(InheritType::from_enum_int)
        .unwrap_or_default()
}

/// The propagation state a parent hands each child: its full world
/// matrix, its accumulated world rotation, and its own local scaling
/// (the `p_s` of the doc §4 mode-2 product).
#[derive(Debug, Clone)]
pub struct ParentFrame {
    /// Parent's full world matrix (`P_world`).
    pub world: Mat4,
    /// Parent's accumulated world rotation (`P_R`), xyzw quaternion —
    /// the product of the local rotations down the chain (each mode's
    /// product starts `P_R · …`, so the accumulation is
    /// mode-independent).
    pub rotation: [f64; 4],
    /// Parent's own local scaling (`p_s`).
    pub local_scaling: [f64; 3],
}

impl ParentFrame {
    /// The scene-root frame: identity everywhere.
    pub fn root() -> Self {
        Self {
            world: mat_identity(),
            rotation: [0.0, 0.0, 0.0, 1.0],
            local_scaling: [1.0, 1.0, 1.0],
        }
    }

    /// The parent's global scale-and-shear block
    /// `P_S = P_R⁻¹ · P_T⁻¹ · P_world` (doc §4: strip translation,
    /// then rotation; FBX has no shear type, so shear rides inside
    /// this block).
    fn global_scale_block(&self) -> [[f64; 3]; 3] {
        let r_inv = quat_to_mat3(quat_conjugate(self.rotation));
        let linear = [
            [self.world[0][0], self.world[0][1], self.world[0][2]],
            [self.world[1][0], self.world[1][1], self.world[1][2]],
            [self.world[2][0], self.world[2][1], self.world[2][2]],
        ];
        mat3_mul(r_inv, linear)
    }
}

/// Compose one node's world matrix from its parent frame, its local
/// `(translation, rotation, scale)` triple (the §1 chain composition
/// — [`crate::node_transform::TransformChain::compose`] output or the
/// node's plain `Trs`), and its [`InheritType`]. Returns the node's
/// world matrix plus the [`ParentFrame`] its own children inherit.
pub fn compose_world(
    frame: &ParentFrame,
    local_t: [f64; 3],
    local_q: [f64; 4],
    local_s: [f64; 3],
    inherit: InheritType,
) -> (Mat4, ParentFrame) {
    let p_r = quat_to_mat3(frame.rotation);
    let p_s = frame.global_scale_block();
    let l_r = quat_to_mat3(local_q);
    let l_s = [
        [local_s[0], 0.0, 0.0],
        [0.0, local_s[1], 0.0],
        [0.0, 0.0, local_s[2]],
    ];

    // Doc §4 rotation-and-scale product table.
    let rs = match inherit {
        InheritType::RrSs => mat3_mul(mat3_mul(p_r, l_r), mat3_mul(p_s, l_s)),
        InheritType::RSrs => mat3_mul(mat3_mul(p_r, p_s), mat3_mul(l_r, l_s)),
        InheritType::Rrs => {
            // `P_S · p_s⁻¹` — divide the parent's own local scale
            // back out of the inherited global block. A zero parent
            // scale component is degenerate (the parent world is
            // singular); leave that component undivided rather than
            // poison the product with infinities.
            let inv = frame
                .local_scaling
                .map(|c| if c != 0.0 { 1.0 / c } else { 1.0 });
            let mut ps_adj = p_s;
            for row in &mut ps_adj {
                for (j, cell) in row.iter_mut().enumerate() {
                    *cell *= inv[j];
                }
            }
            mat3_mul(mat3_mul(p_r, l_r), mat3_mul(ps_adj, l_s))
        }
    };

    // Translation is outside the product in all three modes: the
    // parent's FULL world matrix applied to the chain-local
    // translation.
    let t_global = mat4_apply_point(&frame.world, local_t);

    let mut world = mat_identity();
    for i in 0..3 {
        for j in 0..3 {
            world[i][j] = rs[i][j];
        }
        world[i][3] = t_global[i];
    }

    let child_frame = ParentFrame {
        world,
        rotation: quat_mul(frame.rotation, local_q),
        local_scaling: local_s,
    };
    (world, child_frame)
}

/// Compute every node's world matrix for a decoded scene, honouring
/// each node's `extras["fbx:inherit_type"]` per the doc §4 products.
///
/// Local `(t, q, s)` triples come from the authored chain when the
/// node carries `fbx:lcl_*` chain extras (recomposed exactly via
/// [`crate::node_transform::TransformChain::compose`]) and from
/// [`Transform::Trs`] otherwise. [`Transform::Matrix`] nodes compose
/// naively (see the module docs). Nodes unreachable from
/// [`Scene3D::roots`] are absent from the result.
pub fn world_transforms(scene: &Scene3D) -> HashMap<NodeId, Mat4> {
    let mut out = HashMap::new();
    let root = ParentFrame::root();
    for &nid in &scene.roots {
        walk(scene, nid, &root, &mut out);
    }
    out
}

fn walk(scene: &Scene3D, nid: NodeId, frame: &ParentFrame, out: &mut HashMap<NodeId, Mat4>) {
    let Some(node) = scene.nodes.get(nid.0 as usize) else {
        return;
    };
    if out.contains_key(&nid) {
        // Cycle / duplicate-parent guard.
        return;
    }
    let (world, child_frame) = match local_trs(node) {
        Some((t, q, s)) => compose_world(frame, t, q, s, inherit_type_of(node)),
        None => {
            // Transform::Matrix — no R/S split available: naive
            // composition, linear part folded into the children's
            // inherited P_S (rotation contribution identity).
            let local = transform_to_mat4(node.transform);
            let world = mat4_mul(&frame.world, &local);
            let child = ParentFrame {
                world,
                rotation: frame.rotation,
                local_scaling: [1.0, 1.0, 1.0],
            };
            (world, child)
        }
    };
    out.insert(nid, world);
    for &child in &node.children {
        walk(scene, child, &child_frame, out);
    }
}

/// A node's local `(t, q, s)`: the authored chain when present
/// (exact f64 recomposition), else the plain `Trs`. `None` for
/// `Transform::Matrix`.
fn local_trs(node: &Node) -> Option<([f64; 3], [f64; 4], [f64; 3])> {
    if let Some(chain) = chain_from_extras(node) {
        return Some(chain.compose());
    }
    match node.transform {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => Some((
            translation.map(f64::from),
            rotation.map(f64::from),
            scale.map(f64::from),
        )),
        Transform::Matrix(_) => None,
    }
}

// ---- small matrix kit -------------------------------------------------

fn mat_identity() -> Mat4 {
    let mut m = [[0.0; 4]; 4];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = (0..4).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn mat4_apply_point(m: &Mat4, p: [f64; 3]) -> [f64; 3] {
    let mut out = [0.0; 3];
    for (i, o) in out.iter_mut().enumerate() {
        *o = m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3];
    }
    out
}

fn mat3_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
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

/// A [`Transform`] as a plain 4×4 (naive-composition helper for the
/// `Matrix` fallback; `Trs` builds `T · R · S`).
fn transform_to_mat4(t: Transform) -> Mat4 {
    match t {
        Transform::Trs {
            translation,
            rotation,
            scale,
        } => {
            let r = quat_to_mat3(rotation.map(f64::from));
            let s = scale.map(f64::from);
            let mut m = mat_identity();
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] = r[i][j] * s[j];
                }
                m[i][3] = f64::from(translation[i]);
            }
            m
        }
        Transform::Matrix(m32) => {
            // mesh3d's `Matrix` uses the same row-major-storage
            // column-vector convention as `Mat4` (translation in
            // `m[0..3][3]`) — widen componentwise.
            let mut m = [[0.0; 4]; 4];
            for i in 0..4 {
                for j in 0..4 {
                    m[i][j] = f64::from(m32[i][j]);
                }
            }
            m
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node_transform::euler_to_quat;
    use crate::node_transform::RotationOrder;

    fn assert_mat_close(a: Mat4, b: Mat4, tol: f64) {
        for i in 0..4 {
            for j in 0..4 {
                assert!(
                    (a[i][j] - b[i][j]).abs() < tol,
                    "mismatch at [{i}][{j}]: {} vs {}\n{a:?}\n{b:?}",
                    a[i][j],
                    b[i][j],
                );
            }
        }
    }

    fn trs_mat(t: [f64; 3], q: [f64; 4], s: [f64; 3]) -> Mat4 {
        let r = quat_to_mat3(q);
        let mut m = mat_identity();
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = r[i][j] * s[j];
            }
            m[i][3] = t[i];
        }
        m
    }

    fn frame_for(t: [f64; 3], q: [f64; 4], s: [f64; 3]) -> ParentFrame {
        let (world, frame) = compose_world(&ParentFrame::root(), t, q, s, InheritType::RrSs);
        // A root node's world is mode-independent (P_S = I).
        assert_mat_close(world, trs_mat(t, q, s), 1e-12);
        frame
    }

    #[test]
    fn enum_ints_round_trip() {
        for v in 0..=2 {
            assert_eq!(InheritType::from_enum_int(v).unwrap().to_enum_int(), v);
        }
        assert!(InheritType::from_enum_int(3).is_none());
        assert!(InheritType::from_enum_int(-1).is_none());
        assert_eq!(InheritType::default(), InheritType::RrSs);
    }

    /// Mode 1 (`RSrs`) is exactly the naive `ParentWorld · Local`
    /// matrix concatenation.
    #[test]
    fn mode_rsrs_equals_naive_concatenation() {
        let pq = euler_to_quat([20.0, -35.0, 50.0], RotationOrder::Xyz);
        let frame = frame_for([1.0, 2.0, 3.0], pq, [2.0, 0.5, 3.0]);
        let cq = euler_to_quat([10.0, 70.0, -15.0], RotationOrder::Xyz);
        let (ct, cs) = ([0.5, -1.0, 2.0], [1.5, 2.5, 0.75]);

        let (world, _) = compose_world(&frame, ct, cq, cs, InheritType::RSrs);
        let naive = mat4_mul(&frame.world, &trs_mat(ct, cq, cs));
        assert_mat_close(world, naive, 1e-12);
    }

    /// Mode 0 (`RrSs`) matches the literal doc §4 product
    /// `P_R · L_R · P_S · L_S` with the translation carried by the
    /// parent's full world matrix.
    #[test]
    fn mode_rrss_matches_literal_product() {
        let pq = euler_to_quat([20.0, -35.0, 50.0], RotationOrder::Xyz);
        let frame = frame_for([1.0, 2.0, 3.0], pq, [2.0, 0.5, 3.0]);
        let cq = euler_to_quat([10.0, 70.0, -15.0], RotationOrder::Xyz);
        let (ct, cs) = ([0.5, -1.0, 2.0], [1.5, 2.5, 0.75]);

        let (world, _) = compose_world(&frame, ct, cq, cs, InheritType::RrSs);

        // Literal product built directly from the frame definition.
        let p_r = quat_to_mat3(frame.rotation);
        let p_s = frame.global_scale_block();
        let l_r = quat_to_mat3(cq);
        let l_s = [[cs[0], 0.0, 0.0], [0.0, cs[1], 0.0], [0.0, 0.0, cs[2]]];
        let rs = mat3_mul(mat3_mul(p_r, l_r), mat3_mul(p_s, l_s));
        let t = mat4_apply_point(&frame.world, ct);
        let mut expect = mat_identity();
        for i in 0..3 {
            for j in 0..3 {
                expect[i][j] = rs[i][j];
            }
            expect[i][3] = t[i];
        }
        assert_mat_close(world, expect, 1e-12);

        // And with a non-uniform parent scale + rotated child, mode 0
        // genuinely differs from the naive mode-1 composition.
        let (naive, _) = compose_world(&frame, ct, cq, cs, InheritType::RSrs);
        let max_delta = (0..3)
            .flat_map(|i| (0..3).map(move |j| (i, j)))
            .map(|(i, j)| (world[i][j] - naive[i][j]).abs())
            .fold(0.0f64, f64::max);
        assert!(max_delta > 1e-3, "expected divergence, got {max_delta}");
    }

    /// Under a *uniform* parent scale, modes 0 and 1 agree (the
    /// scalar block commutes with the child rotation).
    #[test]
    fn uniform_parent_scale_collapses_modes_0_and_1() {
        let pq = euler_to_quat([20.0, -35.0, 50.0], RotationOrder::Xyz);
        let frame = frame_for([1.0, 2.0, 3.0], pq, [2.0, 2.0, 2.0]);
        let cq = euler_to_quat([10.0, 70.0, -15.0], RotationOrder::Xyz);
        let (ct, cs) = ([0.5, -1.0, 2.0], [1.5, 2.5, 0.75]);
        let (w0, _) = compose_world(&frame, ct, cq, cs, InheritType::RrSs);
        let (w1, _) = compose_world(&frame, ct, cq, cs, InheritType::RSrs);
        assert_mat_close(w0, w1, 1e-12);
    }

    /// Mode 2 (`Rrs`): the parent's own local scale does not reach
    /// the child's rotation-and-scale block — two parents differing
    /// only in local scale produce children with the identical linear
    /// block.
    #[test]
    fn mode_rrs_strips_parent_local_scale() {
        let pq = euler_to_quat([20.0, -35.0, 50.0], RotationOrder::Xyz);
        let scaled = frame_for([1.0, 2.0, 3.0], pq, [2.0, 5.0, 0.25]);
        let unscaled = frame_for([1.0, 2.0, 3.0], pq, [1.0, 1.0, 1.0]);
        let cq = euler_to_quat([10.0, 70.0, -15.0], RotationOrder::Xyz);
        let (ct, cs) = ([0.5, -1.0, 2.0], [1.5, 2.5, 0.75]);

        let (w_scaled, _) = compose_world(&scaled, ct, cq, cs, InheritType::Rrs);
        let (w_unscaled, _) = compose_world(&unscaled, ct, cq, cs, InheritType::Rrs);
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (w_scaled[i][j] - w_unscaled[i][j]).abs() < 1e-12,
                    "linear block leaked parent scale at [{i}][{j}]"
                );
            }
        }
        // The translation DOES still feel the parent scale (it rides
        // the parent's full world matrix).
        let dt = (0..3)
            .map(|i| (w_scaled[i][3] - w_unscaled[i][3]).abs())
            .fold(0.0f64, f64::max);
        assert!(dt > 1e-6, "translation should differ, got {dt}");
    }

    /// Three-level scene walk: a grandchild under a mode-2 middle
    /// node sees the middle node's scale stripped but the root's
    /// scale intact — checked against hand-built frame algebra.
    #[test]
    fn scene_walk_honours_per_node_inherit_extras() {
        use oxideav_mesh3d::Node;

        let mut scene = Scene3D::new();
        let mut root = Node::new().with_name("root");
        root.transform = Transform::Trs {
            translation: [0.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [3.0, 3.0, 3.0],
        };
        let mut mid = Node::new().with_name("mid");
        mid.transform = Transform::Trs {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [2.0, 2.0, 2.0],
        };
        mid.extras.insert(
            "fbx:inherit_type".to_string(),
            serde_json::Value::from(1i64),
        );
        let mut leaf = Node::new().with_name("leaf");
        leaf.transform = Transform::Trs {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0, 1.0, 1.0],
        };
        leaf.extras.insert(
            "fbx:inherit_type".to_string(),
            serde_json::Value::from(2i64),
        );

        let rid = scene.add_node(root);
        let mid_id = scene.add_node(mid);
        let leaf_id = scene.add_node(leaf);
        scene.nodes[rid.0 as usize].children.push(mid_id);
        scene.nodes[mid_id.0 as usize].children.push(leaf_id);
        scene.roots.push(rid);

        let worlds = world_transforms(&scene);
        assert_eq!(worlds.len(), 3);

        // mid (mode 1, all-uniform): world = T(3,0,0) · S(6).
        let w_mid = worlds[&mid_id];
        assert!((w_mid[0][3] - 3.0).abs() < 1e-12);
        assert!((w_mid[0][0] - 6.0).abs() < 1e-12);

        // leaf (mode 2): the mid node's own local scale (2) is
        // divided out of the inherited block, leaving the root's 3;
        // translation rides mid's full world: 3 + 6·1 = 9.
        let w_leaf = worlds[&leaf_id];
        assert!((w_leaf[0][3] - 9.0).abs() < 1e-12, "{w_leaf:?}");
        assert!((w_leaf[0][0] - 3.0).abs() < 1e-12, "{w_leaf:?}");
        assert!((w_leaf[1][1] - 3.0).abs() < 1e-12);
    }

    /// Chain-bearing nodes recompose their authored chain (exact f64)
    /// inside the walk, and the geometric TRS never leaks in.
    #[test]
    fn scene_walk_uses_authored_chain_extras() {
        use oxideav_mesh3d::Node;

        let mut scene = Scene3D::new();
        let mut n = Node::new().with_name("chained");
        // Node::transform deliberately stale — the chain extras are
        // authoritative when present.
        n.transform = Transform::Trs {
            translation: [99.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0, 1.0, 1.0],
        };
        n.extras.insert(
            "fbx:lcl_translation".to_string(),
            serde_json::json!([1.0, 2.0, 3.0]),
        );
        n.extras.insert(
            "fbx:lcl_rotation".to_string(),
            serde_json::json!([0.0, 0.0, 90.0]),
        );
        n.extras.insert(
            "fbx:lcl_scaling".to_string(),
            serde_json::json!([1.0, 1.0, 1.0]),
        );
        n.extras.insert(
            "fbx:geometric_translation".to_string(),
            serde_json::json!([50.0, 0.0, 0.0]),
        );
        let nid = scene.add_node(n);
        scene.roots.push(nid);

        let worlds = world_transforms(&scene);
        let w = worlds[&nid];
        // Chain translation (1,2,3), not the stale (99,0,0), and no
        // geometric offset.
        assert!((w[0][3] - 1.0).abs() < 1e-12);
        assert!((w[1][3] - 2.0).abs() < 1e-12);
        // 90° about Z: column 0 maps +X → +Y.
        assert!(w[1][0] > 0.999);
    }

    /// `Transform::Matrix` nodes compose naively.
    #[test]
    fn matrix_nodes_compose_naively() {
        use oxideav_mesh3d::Node;

        let mut scene = Scene3D::new();
        let mut parent = Node::new();
        parent.transform = Transform::Trs {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            scale: [2.0, 2.0, 2.0],
        };
        let mut child = Node::new();
        // Identity with translation (0, 5, 0) in column 3.
        let mut m = [[0.0f32; 4]; 4];
        for (i, row) in m.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        m[1][3] = 5.0;
        child.transform = Transform::Matrix(m);
        let pid = scene.add_node(parent);
        let cid = scene.add_node(child);
        scene.nodes[pid.0 as usize].children.push(cid);
        scene.roots.push(pid);

        let worlds = world_transforms(&scene);
        let w = worlds[&cid];
        assert!((w[0][3] - 1.0).abs() < 1e-12);
        assert!((w[1][3] - 10.0).abs() < 1e-12); // 2 · 5
    }
}
