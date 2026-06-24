"""(A) ブリットルネス＆速度検証: 物体別 × 複数シードで F@1 と per-task 時間を測る。

    python scripts/run_a1_brittleness.py --method ransac --seeds 0 1 2
    python scripts/run_a1_brittleness.py --method fgr    --seeds 0 1 2 3 4

方針（ユーザー指示反映）:
  - 並列化(ProcessPool)は Open3D 内部マルチコアと競合して逆効果だったため **廃止 → 逐次実行**。
    各 RANSAC/FGR タスクは Open3D が全コアを使う。
  - 各タスクの実時間を計測し、per-task 秒数・合計・平均を報告（速度を可視化）。
  - resume 対応（method 別の出力ディレクトリ。完了済み (object,seed) はスキップ）。
評価＝姿勢自由・多重ICP整列の重なり率 F@1。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
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
    ("knot", "knot", "滑らか曲面(公開)"),
    ("lblock", "lblock", "非対称・平面・角"),
    ("ellipsoid", "ellipsoid", "平滑・特徴乏"),
    ("cylinder", "cylinder", "回転対称"),
    ("cone", "cone", "回転対称+頂点"),
    ("cuboid", "cuboid", "離散対称・平面"),
    ("washer", "washer", "回転対称・穴"),
    ("plate", "plate", "薄板・平面"),
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


def _run_task(label, obj, note, seed, method, rv):
    scene, sinfo = make_covering_scene(_load(obj), seed=seed, dense_n=25000)
    L = scene.scale_L
    ev = 0.005 * L
    est, info = register_groupwise(scene.partials_world, rv * L, method=method)
    s_a1 = _fuse_node(scene.partials_world, est, ev)
    f1 = cd = float("nan")
    if len(s_a1) > 100:
        taus = [0.01 * L, 0.02 * L]
        _, res = M.align_to_gt(s_a1, scene.s_gt, ev, taus=taus, n_starts=16, seed=seed)
        cd = res["chamfer_l1"] / L * 100
        f1 = res[f"F@{taus[0]:.6g}"]
    return {"object": label, "note": note, "seed": seed,
            "F1": f1, "CD": cd, "n": sinfo["n"], "edges": info["n_edges"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["ransac", "fgr"], default="ransac")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--rv", type=float, default=0.015, help="登録voxel (×L)")
    args = ap.parse_args()
    out = ROOT / "outputs" / f"a1_brittleness_{args.method}"
    out.mkdir(parents=True, exist_ok=True)
    rawpath = out / "raw.json"

    tasks = [(l, o, nt, s) for (l, o, nt) in CASES for s in args.seeds]
    raw = []
    if rawpath.exists():                              # resume
        try:
            raw = json.loads(rawpath.read_text(encoding="utf-8"))
        except Exception:
            raw = []
    done = {(r.get("object"), r.get("seed")) for r in raw if "error" not in r}
    pending = [t for t in tasks if (t[0], t[3]) not in done]

    print(f"\n=== (A) {args.method.upper()} 検証（逐次・rv={args.rv}×L） "
          f"done={len(done)} pending={len(pending)}/{len(tasks)} ===", flush=True)
    times = []
    for (l, o, nt, s) in pending:
        t0 = time.perf_counter()
        try:
            r = _run_task(l, o, nt, s, args.method, args.rv)
        except Exception as e:
            r = {"object": l, "note": nt, "seed": s, "error": str(e)}
        dt = time.perf_counter() - t0
        r["sec"] = round(dt, 1)
        times.append(dt)
        raw.append(r)
        rawpath.write_text(json.dumps(raw, indent=2, ensure_ascii=False), encoding="utf-8")
        f1s = f"{r.get('F1', float('nan')):.3f}" if "error" not in r else "ERR"
        avg = sum(times) / len(times)
        eta = avg * (len(pending) - len(times))
        print(f"  {l}/s{s}: F@1={f1s}  {dt:.1f}s  (avg {avg:.1f}s/task, ETA {eta/60:.1f}min)", flush=True)

    # 集計
    print("\n--- 集計（F@1 mean±std, 速度）---", flush=True)
    print(f"{'object':<11}{'note':<18}{'F@1 mean±std':>15}{'CD%L mean':>10}{'sec/avg':>9}  分類", flush=True)
    print("-" * 72)
    agg = []
    for (label, obj, note) in CASES:
        rs = [r for r in raw if r.get("object") == label and "error" not in r and r["F1"] == r["F1"]]
        if not rs:
            continue
        f1 = np.array([r["F1"] for r in rs]); cd = np.array([r["CD"] for r in rs])
        secs = np.array([r.get("sec", float("nan")) for r in rs])
        f1m, f1s, cdm = f1.mean(), f1.std(), cd.mean()
        cls = "安定OK" if (f1m > 0.85 and f1s < 0.1) else \
              ("安定NG" if (f1m < 0.6 and f1s < 0.15) else
               ("脆い(高分散)" if f1s >= 0.15 else "中間"))
        agg.append({"object": label, "note": note, "F1_mean": float(f1m), "F1_std": float(f1s),
                    "CD_mean": float(cdm), "sec_mean": float(np.nanmean(secs)),
                    "n_runs": len(rs), "class": cls})
        print(f"{label:<11}{note:<18}{f1m:>7.3f}±{f1s:<5.3f}{cdm:>10.2f}{np.nanmean(secs):>8.1f}s  {cls}", flush=True)
    (out / "summary.json").write_text(json.dumps(agg, indent=2, ensure_ascii=False), encoding="utf-8")
    tot = sum(times)
    print("-" * 72)
    print(f"{args.method}: this-run {len(times)} tasks in {tot:.0f}s "
          f"(avg {tot/max(1,len(times)):.1f}s/task)  saved -> {out}\n", flush=True)


if __name__ == "__main__":
    main()
