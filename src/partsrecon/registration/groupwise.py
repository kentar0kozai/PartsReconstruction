"""A1: 参照モデル無し groupwise registration（古典幾何）。

baseline-classical-geometry.md §3.1。各部分点群（世界座標）を共通正準系へ揃える:
  前処理(法線+FPFH) → 全ペア RANSAC+ICP 粗位置合わせ → 信頼エッジでポーズグラフ
  → spanning-tree で絶対姿勢を初期化 → グローバル最適化(LM+line process) → 絶対姿勢。

「姿勢なし SfM」相当。重複が乏しい個体ペアは繋がらず、重複グラフの連結成分のみ
登録される（F1: 連結性問題）。返り値の est[i] は world→canonical(global) 変換。
"""
from __future__ import annotations

from collections import deque
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d

from .. import geometry as G

_reg = o3d.pipelines.registration


def _rot_angle_deg(Rm: np.ndarray) -> float:
    c = (np.trace(Rm[:3, :3]) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _cycle_filter(pair: dict, voxel: float, rot_tol_deg: float = 15.0) -> dict:
    """(a) 3-cycle 整合性で誤エッジを除去。

    真の相対姿勢は三角形 i->j->k と i->k が一致する。FPFH+RANSAC の誤対応は
    一致しないので、整合する三角形に一度も入らないエッジを落とす（precision 優先）。
    """
    nodes = sorted({i for e in pair for i in e})
    trans_tol = voxel * 12.0
    supported = set()
    for ai in range(len(nodes)):
        for bi in range(ai + 1, len(nodes)):
            for ci in range(bi + 1, len(nodes)):
                i, j, k = nodes[ai], nodes[bi], nodes[ci]
                if (i, j) not in pair or (j, k) not in pair or (i, k) not in pair:
                    continue
                comp = pair[(j, k)][0] @ pair[(i, j)][0]        # i->j->k
                err = np.linalg.inv(pair[(i, k)][0]) @ comp      # vs i->k（理想 I）
                if _rot_angle_deg(err) < rot_tol_deg and np.linalg.norm(err[:3, 3]) < trans_tol:
                    supported |= {(i, j), (j, k), (i, k)}
    return {e: v for e, v in pair.items() if e in supported}


def _preprocess(pts: np.ndarray, voxel: float):
    pcd = G.to_o3d_pcd(pts).voxel_down_sample(voxel)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    fpfh = _reg.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return pcd, fpfh


def _pair_register(src, src_f, dst, dst_f, voxel, method="ransac"):
    """align src->dst。返り値: (T, fitness, inlier_rmse, info)。

    method="fgr": Fast Global Registration（高速）/ "ransac": RANSAC（堅牢だが重い）。
    いずれも FPFH 特徴ベース。続けて point-to-plane ICP で精緻化。
    """
    dist = voxel * 1.5
    if method == "ransac":
        res = _reg.registration_ransac_based_on_feature_matching(
            src, dst, src_f, dst_f, True, dist,
            _reg.TransformationEstimationPointToPoint(False), 3,
            [_reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
             _reg.CorrespondenceCheckerBasedOnDistance(dist)],
            _reg.RANSACConvergenceCriteria(50000, 0.999))
    else:
        res = _reg.registration_fgr_based_on_feature_matching(
            src, dst, src_f, dst_f,
            _reg.FastGlobalRegistrationOption(maximum_correspondence_distance=dist))
    icp = _reg.registration_icp(
        src, dst, voxel * 2.0, res.transformation,
        _reg.TransformationEstimationPointToPlane())
    info = _reg.get_information_matrix_from_point_clouds(src, dst, voxel * 2.0, icp.transformation)
    return np.array(icp.transformation), icp.fitness, icp.inlier_rmse, info


def register_groupwise(partials: List[np.ndarray], voxel: float,
                       fit_thresh: float = 0.25,
                       cycle_consistency: bool = True,
                       method: str = "ransac",
                       max_pairs: int = 150) -> Tuple[Dict[int, np.ndarray], dict]:
    n = len(partials)
    pcds, feats = [], []
    for pts in partials:
        pc, f = _preprocess(pts, voxel)
        pcds.append(pc); feats.append(f)

    # --- 候補ペア選択（律速回避: O(N^2) を max_pairs で上限）---
    cand = [(i, j) for i in range(n) for j in range(i + 1, n)
            if len(pcds[i].points) >= 10 and len(pcds[j].points) >= 10]
    if max_pairs and len(cand) > max_pairs:
        prng = np.random.default_rng(0)
        cset = set(cand)
        backbone = [(i, i + 1) for i in range(n - 1) if (i, i + 1) in cset]  # 連結性を担保
        bset = set(backbone)
        extras = [p for p in cand if p not in bset]
        prng.shuffle(extras)
        cand = backbone + extras[:max(0, max_pairs - len(backbone))]

    # --- ペア粗位置合わせ（逐次。並列は物体レベルで ProcessPool 化）---
    pair: Dict[Tuple[int, int], Tuple[np.ndarray, float, np.ndarray]] = {}
    adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(n)}
    for (i, j) in cand:
        T, fit, rmse, info = _pair_register(pcds[i], feats[i], pcds[j], feats[j], voxel, method)
        if fit >= fit_thresh:
            pair[(i, j)] = (T, fit, info)
            adj[i].append((j, fit)); adj[j].append((i, fit))

    n_edges_raw = len(pair)
    # --- (a) サイクル整合フィルタで誤エッジ除去 ---
    if cycle_consistency and len(pair) >= 3:
        pair = _cycle_filter(pair, voxel)
        adj = {i: [] for i in range(n)}
        for (i, j), (T, fit, info_m) in pair.items():
            adj[i].append((j, fit)); adj[j].append((i, fit))

    # --- 最大次数ノードを根に spanning-tree で絶対姿勢を初期化 ---
    root = max(range(n), key=lambda k: len(adj[k]))
    poses: Dict[int, np.ndarray] = {root: np.eye(4)}
    tree_edges = set()
    visited = {root}
    q = deque([root])
    while q:
        u = q.popleft()
        for v, _ in sorted(adj[u], key=lambda e: -e[1]):
            if v in visited:
                continue
            a, b = (u, v) if u < v else (v, u)
            T_ab = pair[(a, b)][0]                    # align a->b
            if u == a:                                 # u=a known, v=b: Tn_b = Tn_a @ inv(T_ab)
                poses[v] = poses[u] @ np.linalg.inv(T_ab)
            else:                                      # u=b known, v=a: Tn_a = Tn_b @ T_ab
                poses[v] = poses[u] @ T_ab
            visited.add(v); tree_edges.add((a, b)); q.append(v)

    comp = sorted(visited)
    idx = {old: k for k, old in enumerate(comp)}

    # --- ポーズグラフ構築 ---
    pg = _reg.PoseGraph()
    for old in comp:
        pg.nodes.append(_reg.PoseGraphNode(poses[old]))
    for (i, j), (T, fit, info) in pair.items():
        if i in visited and j in visited:
            pg.edges.append(_reg.PoseGraphEdge(
                idx[i], idx[j], T, info, uncertain=((i, j) not in tree_edges)))

    # --- グローバル最適化 ---
    if len(pg.nodes) >= 2 and len(pg.edges) >= 1:
        option = _reg.GlobalOptimizationOption(
            max_correspondence_distance=voxel * 2.0,
            edge_prune_threshold=0.25, reference_node=idx[root])
        _reg.global_optimization(
            pg, _reg.GlobalOptimizationLevenbergMarquardt(),
            _reg.GlobalOptimizationConvergenceCriteria(), option)

    est = {old: np.array(pg.nodes[idx[old]].pose) for old in comp}
    info = {"n": n, "n_registered": len(comp), "registered_ids": comp,
            "n_edges": len(pair), "n_edges_raw": n_edges_raw,
            "n_tree_edges": len(tree_edges), "root": root}
    return est, info
