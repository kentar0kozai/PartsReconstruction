"""評価ハーネス — 全手法ファミリ共通の点群再構成指標。

docs/evaluation-harness.md の実装:
- §4 再構成品質 (Accuracy / Completeness / Chamfer-L1 / Precision・Recall・F-score@τ)
- §6.1 観測被覆 (coverage)
- §3 グローバル整列 (gauge 自由度吸収; A1+ 用。A0 は GT 系で一致するため不要)

距離は入力単位（本 PoC では m）。τ・voxel は呼び出し側で L 比から決める。
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .. import geometry as G


def voxel_downsample(points: np.ndarray, voxel: Optional[float]) -> np.ndarray:
    if not voxel or voxel <= 0:
        return points
    return G.from_o3d_pcd(G.to_o3d_pcd(points).voxel_down_sample(voxel))


def nn_dist(query: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """query 各点から ref への最近傍ユークリッド距離。"""
    d, _ = cKDTree(ref).query(query, k=1)
    return d


def evaluate(s_hat: np.ndarray, s_gt: np.ndarray,
             taus: List[float], voxel: Optional[float] = None) -> Dict[str, float]:
    """§4 の指標一式。整列は呼び出し側責務（A0 は不要）。"""
    a = voxel_downsample(s_hat, voxel)   # 復元点群
    b = voxel_downsample(s_gt, voxel)    # GT 点群

    d_ab = nn_dist(a, b)                 # accuracy 方向（精度）
    d_ba = nn_dist(b, a)                 # completeness 方向（再現）
    acc = float(d_ab.mean())
    comp = float(d_ba.mean())

    res: Dict[str, float] = {
        "accuracy": acc,
        "completeness": comp,
        "chamfer_l1": acc + comp,
        "n_hat": int(len(a)),
        "n_gt": int(len(b)),
    }
    for tau in taus:
        P = float((d_ab < tau).mean())
        R = float((d_ba < tau).mean())
        F = 0.0 if (P + R) == 0 else 2 * P * R / (P + R)
        key = f"{tau:.6g}"
        res[f"P@{key}"] = P
        res[f"R@{key}"] = R
        res[f"F@{key}"] = F
    return res


def coverage(s_gt: np.ndarray, source: np.ndarray, tau: float,
             voxel: Optional[float] = None) -> float:
    """§6.1 観測被覆: source が GT 表面を τ 以内でどれだけ覆うか（recall 方向）。"""
    b = voxel_downsample(s_gt, voxel)
    a = voxel_downsample(source, voxel)
    return float((nn_dist(b, a) < tau).mean())


# --------------------------------------------------------------------------- #
# §3 グローバル整列（A1+ 用。A0 では GT 正準系と一致するため呼ばない）
# --------------------------------------------------------------------------- #
def global_align(s_hat: np.ndarray, s_gt: np.ndarray, voxel: float) -> np.ndarray:
    """FPFH+RANSAC で粗整列し point-to-plane ICP で精緻化。4x4 変換を返す。"""
    src = G.to_o3d_pcd(s_hat).voxel_down_sample(voxel)
    dst = G.to_o3d_pcd(s_gt).voxel_down_sample(voxel)
    for p in (src, dst):
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))

    def fpfh(p):
        return o3d.pipelines.registration.compute_fpfh_feature(
            p, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, dst, fpfh(src), fpfh(dst), True, voxel * 1.5,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel * 1.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    icp = o3d.pipelines.registration.registration_icp(
        src, dst, voxel * 2, result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return np.asarray(icp.transformation)


def align_to_gt(s_hat: np.ndarray, s_gt: np.ndarray, voxel: float,
                taus: List[float], n_starts: int = 30, seed: int = 0):
    """最終モデル s_hat を GT に「初期位置合わせ(多重)+ICP」で重ね、重なり率を測る。

    評価方針（ユーザー指定）: 最終の全周囲モデルが元モデルとどれだけ一致するか。
    姿勢は自由（任意の剛体変換を許容）。多重初期回転から ICP し、最良 CD の整列を採用
    することで対称等価・部分モデルにロバストにする。返り値: (best_T, evaluate結果)。
    """
    src = G.to_o3d_pcd(s_hat).voxel_down_sample(voxel)
    dst = G.to_o3d_pcd(s_gt).voxel_down_sample(voxel)
    dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    csrc = src.get_center()
    cdst = dst.get_center()
    src_pts = np.asarray(src.points)
    dst_pts = np.asarray(dst.points)

    rots = [np.eye(3)] + list(Rotation.random(num=n_starts, random_state=seed).as_matrix())
    best_T, best_cd = np.eye(4), np.inf
    for R in rots:
        T0 = np.eye(4)
        T0[:3, :3] = R
        T0[:3, 3] = cdst - R @ csrc                         # 回転後に重心を合わせる初期化
        reg = o3d.pipelines.registration.registration_icp(
            src, dst, voxel * 2.5, T0,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        T = np.asarray(reg.transformation)
        a = src_pts @ T[:3, :3].T + T[:3, 3]
        cd = float(nn_dist(a, dst_pts).mean() + nn_dist(dst_pts, a).mean())
        if cd < best_cd:
            best_cd, best_T = cd, T
    return best_T, evaluate(G.apply_pose(s_hat, best_T), s_gt, taus=taus, voxel=voxel)
