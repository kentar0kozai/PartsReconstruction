"""Geometry helpers: mesh<->o3d 変換, 表面サンプリング, 姿勢演算, トップ視点可視判定。"""
from __future__ import annotations

import numpy as np
import open3d as o3d
import trimesh
from scipy.spatial.transform import Rotation


# --------------------------------------------------------------------------- #
# o3d <-> numpy / trimesh
# --------------------------------------------------------------------------- #
def to_o3d_pcd(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    return pcd


def from_o3d_pcd(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    return np.asarray(pcd.points)


def trimesh_to_o3d(mesh: trimesh.Trimesh) -> o3d.geometry.TriangleMesh:
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices, dtype=np.float64))
    m.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.faces, dtype=np.int32))
    m.compute_vertex_normals()
    return m


# --------------------------------------------------------------------------- #
# 物体生成（PoC 用の手続き生成形状。--mesh で実データに差し替え可）
# --------------------------------------------------------------------------- #
def make_default_object(kind: str = "cuboid") -> trimesh.Trimesh:
    """正準フレーム = 重心原点の単一メッシュ（単位: m）。"""
    if kind == "cuboid":                                   # 3辺が異なる直方体
        mesh = trimesh.creation.box(extents=(0.10, 0.06, 0.03))
    elif kind == "washer":                                 # FA らしいワッシャ（軸対称・内孔あり）
        mesh = trimesh.creation.annulus(r_min=0.02, r_max=0.05, height=0.02)
    elif kind == "lblock":                                 # 段付きブロック（非対称・要 watertight 注意）
        a = trimesh.creation.box(extents=(0.10, 0.04, 0.03))
        b = trimesh.creation.box(extents=(0.04, 0.04, 0.07))
        b.apply_translation((0.03, 0.0, 0.02))
        mesh = trimesh.util.concatenate([a, b])
    elif kind == "ellipsoid":                              # 平滑・特徴乏しい（FPFH が効きにくい）
        mesh = trimesh.creation.icosphere(subdivisions=3, radius=0.05)
        mesh.apply_scale([1.0, 0.7, 0.45])                 # 3 軸異なる滑らかな楕円体
    elif kind == "cylinder":                               # 回転対称（軸まわり連続対称）
        mesh = trimesh.creation.cylinder(radius=0.03, height=0.10, sections=64)
    elif kind == "cone":                                   # 回転対称＋頂点で識別可
        mesh = trimesh.creation.cone(radius=0.04, height=0.10, sections=64)
    elif kind == "plate":                                  # 薄板・平面・特徴乏しい（最難クラス）
        mesh = trimesh.creation.box(extents=(0.12, 0.08, 0.008))
    else:
        raise ValueError(f"unknown object kind: {kind}")
    mesh.apply_translation(-mesh.centroid)
    return mesh


def sample_surface(mesh_o3d: o3d.geometry.TriangleMesh, n: int,
                   poisson: bool = False, seed: int = 0) -> np.ndarray:
    if poisson:
        pcd = mesh_o3d.sample_points_poisson_disk(number_of_points=n)
    else:
        try:
            pcd = mesh_o3d.sample_points_uniformly(number_of_points=n, seed=seed)
        except TypeError:                                  # 古い open3d は seed 引数なし
            pcd = mesh_o3d.sample_points_uniformly(number_of_points=n)
    return from_o3d_pcd(pcd)


def bbox_diagonal(points: np.ndarray) -> float:
    mn = points.min(axis=0)
    mx = points.max(axis=0)
    return float(np.linalg.norm(mx - mn))


# --------------------------------------------------------------------------- #
# 姿勢演算（SE(3) を 4x4 同次行列で表現）
# --------------------------------------------------------------------------- #
def make_pose(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_pose(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def apply_pose(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def random_rotations(n: int, rng: np.random.Generator) -> np.ndarray:
    seed = int(rng.integers(0, 2**31 - 1))
    return Rotation.random(num=n, random_state=seed).as_matrix()


def _yaw(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def stable_pose_rotations(mesh: trimesh.Trimesh, n: int,
                          rng: np.random.Generator) -> np.ndarray:
    """安定接地姿勢（確率重み付き）+ ランダムヨー。A2 被覆性の接地バイアスを模す。
    計算が失敗したら一様ランダム回転にフォールバック。"""
    try:
        Ts, probs = mesh.compute_stable_poses(n_samples=1)
        if Ts is None or len(Ts) == 0:
            raise ValueError("no stable poses")
        probs = np.asarray(probs, dtype=float)
        probs = probs / probs.sum()
        idx = rng.choice(len(Ts), size=n, p=probs)
        Rs = []
        for i in idx:
            Rbase = np.asarray(Ts[i])[:3, :3]
            Rs.append(_yaw(rng.uniform(0.0, 2 * np.pi)) @ Rbase)
        return np.stack(Rs, axis=0)
    except Exception:
        return random_rotations(n, rng)


# --------------------------------------------------------------------------- #
# 2.5D 取得: トップ視点からの可視点抽出（Hidden Point Removal）
# --------------------------------------------------------------------------- #
def top_view_visible_indices(points_world: np.ndarray) -> np.ndarray:
    """物体上方の単一視点から見える点のインデックス（自己遮蔽を反映した 2.5D 近似）。"""
    pcd = to_o3d_pcd(points_world)
    mn = points_world.min(axis=0)
    mx = points_world.max(axis=0)
    diameter = float(np.linalg.norm(mx - mn))
    centroid = points_world.mean(axis=0)
    cam = [float(centroid[0]), float(centroid[1]), float(mx[2] + 3.0 * diameter)]
    radius = diameter * 100.0
    try:
        _, idx = pcd.hidden_point_removal(cam, radius)
        idx = np.asarray(idx, dtype=int)
        if idx.size == 0:
            raise ValueError("empty visible set")
        return idx
    except Exception:
        # フォールバック: 上半分（+z 側）を可視とみなす粗近似
        return np.where(points_world[:, 2] >= np.median(points_world[:, 2]))[0]
