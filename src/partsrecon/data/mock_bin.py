"""モックのバラ積みシーン生成器（BlenderProc 非依存）。

1物体メッシュから、N 個のインスタンスを姿勢付きで配置し、各個体のトップ視点
2.5D 部分点群を生成する。GT 姿勢・GT 個体セグ（=リスト要素）・GT 正準点群を
構成上そのまま得られるため、A0（oracle 融合）と評価ハーネスの検証に使える。

実データ化（M2）は本生成器を BlenderProc 物理ドロップ＋深度レンダに差し替える。
簡略化: 個体間の相互遮蔽は無視（各個体を上方から独立に観測）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np
import trimesh

from .. import geometry as G


@dataclass
class Scene:
    mesh: trimesh.Trimesh
    s_gt: np.ndarray                  # 正準 GT 点群
    partials_world: List[np.ndarray]  # 各個体の世界座標 2.5D 部分点群
    poses: List[np.ndarray]           # 各個体の GT 姿勢 T_i (4x4, canonical->world)
    scale_L: float                    # 物体スケール (bbox 対角長)


def generate_scene(mesh: trimesh.Trimesh,
                   n_instances: int = 20,
                   seed: int = 0,
                   orient: str = "random",
                   dense_n: int = 30000,
                   gt_n: int = 50000,
                   bin_radius_factor: float = 2.0) -> Scene:
    rng = np.random.default_rng(seed)
    mesh_o3d = G.trimesh_to_o3d(mesh)

    canon_dense = G.sample_surface(mesh_o3d, dense_n, poisson=False, seed=seed)
    s_gt = G.sample_surface(mesh_o3d, gt_n, poisson=False, seed=seed + 1)
    L = G.bbox_diagonal(s_gt)

    if orient == "stable":
        Rs = G.stable_pose_rotations(mesh, n_instances, rng)
    else:
        Rs = G.random_rotations(n_instances, rng)

    bin_r = bin_radius_factor * L
    partials: List[np.ndarray] = []
    poses: List[np.ndarray] = []

    for i in range(n_instances):
        R = Rs[i]
        rot = canon_dense @ R.T                      # 回転後（原点まわり）
        tz = -float(rot[:, 2].min())                 # 最下点を床(z=0)に接地
        ang = rng.uniform(0.0, 2 * np.pi)
        rad = bin_r * np.sqrt(rng.uniform(0.0, 1.0))  # 円板内一様
        t = np.array([rad * np.cos(ang), rad * np.sin(ang), tz])
        world = rot + t
        idx = G.top_view_visible_indices(world)
        partials.append(world[idx])
        poses.append(G.make_pose(R, t))

    return Scene(mesh=mesh, s_gt=s_gt, partials_world=partials, poses=poses, scale_L=L)
