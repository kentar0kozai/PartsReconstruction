"""忠実なバラ積みシーン生成器: PyBullet 物理山積み + Open3D 単一トップダウン raycast。

docs/data-generation-design.md §5 の実装。出力は mock_bin.Scene と互換なので、
fusion.fuse_oracle と eval.metrics をそのまま再利用できる。

- 物理: 箱(床+4壁)に同一メッシュ N 個を重力落下→衝突→静止し、GT 姿勢 T_i を得る。
  （動的剛体の衝突は凸包近似。深度は真メッシュで raycast するため形状は正確。）
- 計測: 箱上方の単一ピンホールカメラから全シーンへ raycast。相互遮蔽・斜め視・自己
  遮蔽が自動的に正しく入る。ヒット三角形の geometry_id から個体セグ GT を付与。
"""
from __future__ import annotations

import os
import tempfile
from typing import List

import numpy as np
import open3d as o3d
import pybullet as p
import trimesh
from scipy.spatial.transform import Rotation

from .. import geometry as G
from .mock_bin import Scene


# --------------------------------------------------------------------------- #
# 物理シミュ（PyBullet, DIRECT/headless）
# --------------------------------------------------------------------------- #
def _simulate_pile(mesh: trimesh.Trimesh, n: int, rng: np.random.Generator,
                   obj_extent: float, bin_half: float, wall_half_z: float,
                   steps: int):
    obj_path = os.path.join(tempfile.gettempdir(), f"prx_obj_{os.getpid()}_{n}.obj")
    mesh.export(obj_path)

    cid = p.connect(p.DIRECT)
    try:
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)

        floor = p.createCollisionShape(p.GEOM_PLANE)
        fid = p.createMultiBody(0, floor)
        p.changeDynamics(fid, -1, lateralFriction=0.8)

        t = 0.005
        walls = [(bin_half + t, 0.0, t, bin_half + t),
                 (-bin_half - t, 0.0, t, bin_half + t),
                 (0.0, bin_half + t, bin_half + t, t),
                 (0.0, -bin_half - t, bin_half + t, t)]
        for (cx, cy, hx, hy) in walls:
            wcol = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, wall_half_z])
            p.createMultiBody(0, wcol, basePosition=[cx, cy, wall_half_z])

        col = p.createCollisionShape(p.GEOM_MESH, fileName=obj_path, meshScale=[1, 1, 1])

        rots = Rotation.random(num=n, random_state=int(rng.integers(0, 2**31 - 1)))
        body_ids = []
        for i in range(n):
            ang = rng.uniform(0, 2 * np.pi)
            r = bin_half * 0.6 * np.sqrt(rng.uniform(0, 1))
            x, y = r * np.cos(ang), r * np.sin(ang)
            z = obj_extent * (1.0 + 1.3 * i)              # 段差スポーンで初期貫通回避
            quat = rots[i].as_quat()                       # [x,y,z,w]
            bid = p.createMultiBody(
                baseMass=0.1, baseCollisionShapeIndex=col,
                basePosition=[x, y, z], baseOrientation=quat.tolist(),
                baseInertialFramePosition=[0, 0, 0])       # link frame = mesh canonical 原点
            p.changeDynamics(bid, -1, lateralFriction=0.7, restitution=0.0)
            body_ids.append(bid)

        for _ in range(steps):
            p.stepSimulation()

        poses = []
        for bid in body_ids:
            pos, orn = p.getBasePositionAndOrientation(bid)
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            poses.append(G.make_pose(R, np.array(pos)))
        return poses
    finally:
        p.disconnect(cid)
        try:
            os.remove(obj_path)
        except OSError:
            pass


# --------------------------------------------------------------------------- #
# 計測（Open3D RaycastingScene, 単一トップダウンピンホール）
# --------------------------------------------------------------------------- #
def _topdown_capture(mesh: trimesh.Trimesh, poses: List[np.ndarray],
                     bin_half: float, wall_half_z: float, obj_extent: float,
                     width: int = 640, margin: float = 1.15):
    scene = o3d.t.geometry.RaycastingScene()
    for T in poses:
        m = G.trimesh_to_o3d(mesh)            # canonical
        m.transform(T)                        # → world
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(m))

    # カメラ: 箱中心の真上から鉛直下向き（透視 → 周辺個体は斜め視）
    cam_h = 2.0 * wall_half_z + 3.0 * bin_half + obj_extent
    f = (width / 2.0) * cam_h / (bin_half * margin)
    K = np.array([[f, 0, width / 2.0], [0, f, width / 2.0], [0, 0, 1]], dtype=np.float64)
    R_c2w = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float64)  # +z_cam → -z_world
    T_c2w = np.eye(4); T_c2w[:3, :3] = R_c2w; T_c2w[:3, 3] = [0, 0, cam_h]
    extr = np.linalg.inv(T_c2w)               # world→camera

    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        o3d.core.Tensor(K), o3d.core.Tensor(extr), width, width)
    ans = scene.cast_rays(rays)

    t_hit = ans["t_hit"].numpy()
    gids = ans["geometry_ids"].numpy()
    rays_np = rays.numpy()
    hit = np.isfinite(t_hit)
    pts = rays_np[..., :3] + t_hit[..., None] * rays_np[..., 3:6]
    return pts[hit], gids[hit].astype(int)


# --------------------------------------------------------------------------- #
# 公開API
# --------------------------------------------------------------------------- #
def generate_bin_scene(mesh: trimesh.Trimesh,
                       n_instances: int = 15,
                       seed: int = 0,
                       gt_n: int = 50000,
                       width: int = 640,
                       steps: int | None = None) -> Scene:
    rng = np.random.default_rng(seed)
    mesh_o3d = G.trimesh_to_o3d(mesh)
    s_gt = G.sample_surface(mesh_o3d, gt_n, poisson=False, seed=seed + 1)
    L = G.bbox_diagonal(s_gt)
    obj_extent = float(np.max(mesh.extents))

    bin_half = 0.5 * obj_extent * max(2.0, 0.9 * np.sqrt(n_instances))
    wall_half_z = 0.5 * obj_extent * (2.0 + 1.3 * n_instances)   # スポーン柱を囲える高さ
    if steps is None:
        steps = 2500 + 180 * n_instances

    poses = _simulate_pile(mesh, n_instances, rng, obj_extent, bin_half, wall_half_z, steps)
    points, labels = _topdown_capture(mesh, poses, bin_half, wall_half_z, obj_extent, width)

    partials: List[np.ndarray] = []
    for i in range(n_instances):
        partials.append(points[labels == i])

    return Scene(mesh=mesh, s_gt=s_gt, partials_world=partials, poses=poses, scale_L=L)
