"""(A) 物体クラス傾向: 公開DBメッシュ＋FAプリミティブで A1(古典)の限界を見る。

    python scripts/run_a1_objects.py

評価方針（ユーザー指定）: 最終の全周囲モデルが**元モデルとどれだけ一致するか**。
姿勢は自由 — 初期位置合わせ(多重)+ICP で最良に重ねた**重なり率(F-score)**で判定。

高速化: 登録は FGR(Fast Global Registration)＋粗voxel、評価ICP多重開始は削減、
さらに**物体レベルで ProcessPool 並列**（各物体は独立プロセスで同時実行）。
"""
from __future__ import annotations

import os
os.environ.setdefault("OMP_NUM_THREADS", "2")    # 物体並列時の過剰サブスク抑制（best-effort）

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np                                   # noqa: E402
import open3d as o3d                                 # noqa: E402

from partsrecon import geometry as G                 # noqa: E402
from partsrecon.data.synthetic_views import make_covering_scene  # noqa: E402
from partsrecon.eval import metrics as M             # noqa: E402
from partsrecon.registration.groupwise import register_groupwise  # noqa: E402

O3D_MESH = {"bunny": "BunnyMesh", "armadillo": "ArmadilloMesh", "knot": "KnotMesh"}
CASES = [
    ("bunny", "bunny", "非対称・特徴豊富"),
    ("armadillo", "armadillo", "非対称・特徴豊富(公開)"),
    ("knot", "knot", "滑らか・自己相似(公開)"),
    ("lblock", "lblock", "非対称・角あり"),
    ("ellipsoid", "ellipsoid", "平滑・特徴乏"),
    ("cylinder", "cylinder", "回転対称"),
    ("cone", "cone", "回転対称+頂点"),
    ("cuboid", "cuboid", "離散対称・平面"),
    ("washer", "washer", "回転対称・穴"),
    ("plate", "plate", "薄板・平面(最難)"),
]


def _fuse_node(partials, est, voxel):
    canon = [G.apply_pose(partials[i], est[i]) for i in est if len(partials[i])]
    if not canon:
        return np.zeros((0, 3))
    pcd = G.to_o3d_pcd(np.concatenate(canon, 0)).voxel_down_sample(voxel)
    pcd, _ = pcd.remove_statistical_outlier(20, 2.0)
    return G.from_o3d_pcd(pcd)


def _load(obj, target=0.12):
    import trimesh
    if obj in O3D_MESH:
        try:
            mesh = trimesh.load(getattr(o3d.data, O3D_MESH[obj])().path, force="mesh")
        except Exception:
            mesh = G.make_default_object("lblock")
    else:
        mesh = G.make_default_object(obj)
    mesh.apply_translation(-mesh.centroid)
    mesh.apply_scale(target / float(np.max(mesh.extents)))
    return mesh


def _run_one(obj, seed):
    scene, sinfo = make_covering_scene(_load(obj), seed=seed, dense_n=25000)
    L = scene.scale_L
    ev, rv = 0.005 * L, 0.015 * L                    # 評価は細かく、登録は粗く（高速化）
    est, info = register_groupwise(scene.partials_world, rv)
    s_a1 = _fuse_node(scene.partials_world, est, ev)
    cd = f1 = f2 = float("nan")
    best_T = np.eye(4)
    if len(s_a1) > 100:
        taus = [0.01 * L, 0.02 * L]
        best_T, res = M.align_to_gt(s_a1, scene.s_gt, ev, taus=taus, n_starts=16, seed=seed)
        cd = res["chamfer_l1"] / L * 100
        f1 = res[f"F@{taus[0]:.6g}"]
        f2 = res[f"F@{taus[1]:.6g}"]
    aligned = G.apply_pose(s_a1, best_T) if len(s_a1) else s_a1
    rec = {"n": sinfo["n"], "coverage": sinfo["coverage"],
           "edges_raw": info["n_edges_raw"], "edges": info["n_edges"],
           "n_registered": info["n_registered"], "CD_pctL": cd, "F1": f1, "F2": f2}
    return rec, s_a1, aligned, scene.s_gt


def _object_worker(args):
    """1物体を独立プロセスで処理し、PLYを保存して結果dictを返す（picklable）。"""
    label, obj, note, seed, out_str = args
    try:
        rec, s_raw, s_aln, s_gt = _run_one(obj, seed)
    except Exception as e:
        return {"object": label, "note": note, "error": str(e)}
    d = Path(out_str) / label
    d.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(d / "recon_a1.ply"), G.to_o3d_pcd(s_raw))
    o3d.io.write_point_cloud(str(d / "recon_a1_aligned_to_gt.ply"), G.to_o3d_pcd(s_aln))
    o3d.io.write_point_cloud(str(d / "gt.ply"), G.to_o3d_pcd(s_gt))
    rec.update({"object": label, "note": note})
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(ROOT / "outputs" / "a1_objects"))
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    tasks = [(label, obj, note, args.seed, str(out)) for label, obj, note in CASES]
    workers = max(1, min(os.cpu_count() or 4, len(tasks)))
    print(f"\n=== (A) 物体クラス傾向（FGR+粗voxel, 物体プロセス並列 x{workers}）===")
    print(f"{'object':<12}{'note':<20}{'N':>3}{'cov%':>5}{'edges':>9}{'reg/N':>7}{'CD%L':>8}{'F@1':>7}{'F@2':>7}  verdict")
    print("-" * 88)

    rows = []
    import time
    t0 = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers) as ex:
        results = list(ex.map(_object_worker, tasks))
    for r in results:
        rows.append(r)
        if "error" in r:
            print(f"{r['object']:<12}{r['note']:<20}  ERROR: {r['error']}")
            continue
        v = "OK" if r["F1"] > 0.85 else ("PARTIAL" if r["F1"] > 0.6 else "FAIL")
        print(f"{r['object']:<12}{r['note']:<20}{r['n']:>3}{r['coverage']*100:>5.0f}"
              f"{r['edges_raw']:>4}->{r['edges']:<3}{r['n_registered']:>3}/{r['n']:<2}"
              f"{r['CD_pctL']:>8.2f}{r['F1']:>7.3f}{r['F2']:>7.3f}  {v}")
    (out / "report.json").write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
    print("-" * 88)
    print(f"total {time.perf_counter()-t0:.1f}s   saved -> {out/'report.json'} (+ per-object PLY)\n")


if __name__ == "__main__":
    main()
