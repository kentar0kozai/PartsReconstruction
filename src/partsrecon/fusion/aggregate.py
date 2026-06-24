"""stage④ 融合・集約。

A0 (oracle): GT 姿勢 T_i の逆を各部分点群に適用して正準系へ戻し、連結→ボクセル
統合→外れ値除去。GT 姿勢を使うため出力は GT 正準系と一致する（整列不要）。
これは同時に「観測被覆の上界 U = ∪_i T_i^{-1}(P_i)」を与える。
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from .. import geometry as G


def fuse_oracle(partials_world: List[np.ndarray],
                poses: List[np.ndarray],
                voxel: Optional[float] = None,
                denoise: bool = True) -> np.ndarray:
    canon = [G.apply_pose(P, G.invert_pose(T)) for P, T in zip(partials_world, poses)]
    pts = np.concatenate(canon, axis=0)
    pcd = G.to_o3d_pcd(pts)
    if voxel and voxel > 0:
        pcd = pcd.voxel_down_sample(voxel)
    if denoise:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return G.from_o3d_pcd(pcd)
