"""(c) カリキュラム用: 重複量を制御できる合成部分ビュー生成器。

各「インスタンス」をランダム姿勢 T_i に置き、トップ視点で見える表面を
**法線キャップ**で近似する。cap_angle_deg（可視キャップの半角）が重複量のダイヤル:
  - 180° : 全表面（両面）= 完全ビュー → ペア重複 最大（A1 が成功するはずの易条件）
  -  90° : 上向き法線の半球 = 単一深度カメラ相当（バラ積みに近い難条件）
  -  60° : 小さなキャップ = 重複僅少（極難）

bin_sim（物理＋相互遮蔽の忠実版）とは別物の「制御用」生成器。A1 の登録能力が
どの重複量で破綻するかを単調なダイヤルで測るためのもの。出力は mock_bin.Scene 互換。
"""
from __future__ import annotations

from typing import List

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .. import geometry as G
from .mock_bin import Scene


def make_view_scene(mesh: trimesh.Trimesh, n: int, seed: int,
                    cap_angle_deg: float, dense_n: int = 40000,
                    gt_n: int = 50000, spread: float = 2.0) -> Scene:
    rng = np.random.default_rng(seed)

    m = G.trimesh_to_o3d(mesh)
    m.compute_triangle_normals()
    m.compute_vertex_normals()
    dense = m.sample_points_uniformly(number_of_points=dense_n, use_triangle_normal=True)
    canon = np.asarray(dense.points)
    nrm = np.asarray(dense.normals)

    s_gt = G.sample_surface(m, gt_n, poisson=False, seed=seed + 1)
    L = G.bbox_diagonal(s_gt)
    cos_thr = float(np.cos(np.deg2rad(cap_angle_deg)))

    rots = Rotation.random(num=n, random_state=int(rng.integers(0, 2**31 - 1))).as_matrix()
    partials: List[np.ndarray] = []
    poses: List[np.ndarray] = []
    for i in range(n):
        R = rots[i]
        rp = canon @ R.T
        rn = nrm @ R.T
        mask = rn[:, 2] >= cos_thr            # +z(カメラ方向)を向く法線 = 可視
        t = np.array([rng.uniform(-spread * L, spread * L),
                      rng.uniform(-spread * L, spread * L), 0.0])
        partials.append(rp[mask] + t)
        poses.append(G.make_pose(R, t))

    return Scene(mesh=mesh, s_gt=s_gt, partials_world=partials, poses=poses, scale_L=L)


def make_covering_scene(mesh: trimesh.Trimesh, seed: int,
                        cap_angle_deg: float = 120.0,
                        coverage_target: float = 0.99,
                        coverage_tau_frac: float = 0.01,
                        min_seg_points: int = 1500,
                        n_min: int = 10, n_max: int = 12,
                        dense_n: int = 40000, gt_n: int = 50000,
                        spread: float = 2.0):
    """**前提保証つき**シーン生成（[data-generation-design.md] の前提を満たす）。

    保証する性質:
      (1) 各セグメントは「マッチングに有効な大きさ」 — cap_angle_deg は半球超(>=110推奨)で
          十分大きく、かつ min_seg_points 未満の断片は捨てる。
      (2) セグメント和集合が全周囲を被覆 — oracle 被覆率を逐次検証し、coverage_target に
          達するまでビューを追加（n_max で打ち切り、達成被覆率を info に記録）。
      (3) cap>半球なので任意の2ビューが重複 → 登録グラフが連結（組み合わせ可能）。

    返り値: (Scene, info)。info に achieved coverage / n / cap を記録。
    """
    rng = np.random.default_rng(seed)
    m = G.trimesh_to_o3d(mesh)
    m.compute_triangle_normals()
    m.compute_vertex_normals()
    dense = m.sample_points_uniformly(number_of_points=dense_n, use_triangle_normal=True)
    canon = np.asarray(dense.points)
    nrm = np.asarray(dense.normals)
    s_gt = G.sample_surface(m, gt_n, poisson=False, seed=seed + 1)
    L = G.bbox_diagonal(s_gt)
    cos_thr = float(np.cos(np.deg2rad(cap_angle_deg)))
    tau = coverage_tau_frac * L

    def _coverage(seg_list):
        # 被覆 = s_gt 各点が和集合内の点から tau 以内にある割合（oracle と同じ向き）
        union = np.concatenate(seg_list, axis=0)
        return float((cKDTree(union).query(s_gt, k=1)[0] < tau).mean())

    partials: List[np.ndarray] = []
    poses: List[np.ndarray] = []
    seg_list: List[np.ndarray] = []
    cov = 0.0
    attempts = 0
    while attempts < n_max * 5 and len(partials) < n_max:
        attempts += 1
        R = Rotation.random(random_state=int(rng.integers(0, 2**31 - 1))).as_matrix()
        rn = nrm @ R.T
        mask = rn[:, 2] >= cos_thr
        if int(mask.sum()) < min_seg_points:          # マッチング不可な小断片は捨てる
            continue
        seg_list.append(canon[mask])
        rp = canon @ R.T
        t = np.array([rng.uniform(-spread * L, spread * L),
                      rng.uniform(-spread * L, spread * L), 0.0])
        partials.append(rp[mask] + t)
        poses.append(G.make_pose(R, t))
        if len(partials) >= n_min:
            cov = _coverage(seg_list)
            if cov >= coverage_target:                # 全周囲を被覆したら停止
                break
    if seg_list:
        cov = _coverage(seg_list)

    info = {"n": len(partials), "coverage": cov,
            "cap_angle_deg": cap_angle_deg, "coverage_target": coverage_target,
            "attempts": attempts}
    return Scene(mesh=mesh, s_gt=s_gt, partials_world=partials, poses=poses, scale_L=L), info
