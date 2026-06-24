"""(A) 難条件で A1(古典) の限界を測る — 「baseline はできない＋遅い → 学習へ」の根拠。

    python scripts/run_a1_hardcases.py

2軸で測定（いずれも前提保証シーン make_covering_scene 上＝被覆は満たした状態で
登録能力だけを問う）:
  1. 品質: 物体クラス {bunny(非対称・特徴豊富/対照), ellipsoid(平滑・特徴乏), cuboid(対称・平面),
     washer(回転対称)} ＋ ノイズ で CD/F がどこで崩れるか。
  2. 速度: bunny で N を増やし、全ペア RANSAC O(N^2) の実行時間悪化を実測。

注: 対称物体は姿勢が非一意なので「pose正解率」は sym 非考慮で参考値。再構成品質は
対称不変な CD/F で判断する。
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


def _rot_deg(Rm):
    c = (np.trace(Rm[:3, :3]) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _fuse_node(partials, est, voxel):
    canon = [G.apply_pose(partials[i], est[i]) for i in est if len(partials[i])]
    if not canon:
        return np.zeros((0, 3))
    pcd = G.to_o3d_pcd(np.concatenate(canon, 0)).voxel_down_sample(voxel)
    pcd, _ = pcd.remove_statistical_outlier(20, 2.0)
    return G.from_o3d_pcd(pcd)


def _load(obj, target=0.12):
    import trimesh
    if obj == "bunny":
        try:
            mesh = trimesh.load(o3d.data.BunnyMesh().path, force="mesh")
        except Exception:
            mesh = G.make_default_object("lblock")
    else:
        mesh = G.make_default_object(obj)
    mesh.apply_translation(-mesh.centroid)
    mesh.apply_scale(target / float(np.max(mesh.extents)))
    return mesh


def _run_one(mesh, seed, force_n=None, noise_frac=0.0):
    if force_n:
        scene, sinfo = make_covering_scene(mesh, seed=seed, n_min=force_n,
                                           n_max=force_n, coverage_target=1.1)
    else:
        scene, sinfo = make_covering_scene(mesh, seed=seed)
    L = scene.scale_L
    ev, rv = 0.005 * L, 0.01 * L
    parts = scene.partials_world
    if noise_frac > 0:
        rng = np.random.default_rng(seed + 7)
        parts = [p + rng.normal(0, noise_frac * L, p.shape) for p in parts]

    t0 = time.perf_counter()
    est, info = register_groupwise(parts, rv)
    reg_t = time.perf_counter() - t0

    s_a1 = _fuse_node(parts, est, ev)
    cd = f1 = float("nan")
    tau = 0.01 * L
    if len(s_a1) > 100 and info["root"] in est:
        s_al = G.apply_pose(s_a1, np.linalg.inv(scene.poses[info["root"]]))
        res = M.evaluate(s_al, scene.s_gt, taus=[tau], voxel=ev)
        cd = res["chamfer_l1"] / L * 100
        f1 = res[f"F@{tau:.6g}"]
    Ms = {i: est[i] @ scene.poses[i] for i in est}
    ids = sorted(est)
    rot = [_rot_deg(np.linalg.inv(Ms[info["root"]]) @ Ms[i]) for i in ids] if ids else []
    correct = sum(1 for r in rot if r < 10.0)
    return {"n": sinfo["n"], "coverage": sinfo["coverage"], "reg_time_s": reg_t,
            "edges_raw": info["n_edges_raw"], "edges": info["n_edges"],
            "n_registered": info["n_registered"], "correct_pose": correct,
            "CD_pctL": cd, "F1": f1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--skip-speed", action="store_true", help="速度掃引をスキップ（品質のみ）")
    ap.add_argument("--out", default=str(ROOT / "outputs" / "a1_hardcases"))
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    report = {"quality": [], "speed": []}

    # --- 1. 品質: 物体クラス + ノイズ ---
    print("\n=== (A) 品質: 物体クラス（前提保証シーン, 被覆は満たした状態）===")
    print(f"{'case':<18}{'N':>4}{'cov%':>6}{'edges':>10}{'reg/N':>8}{'poseOK':>7}{'CD%L':>8}{'F@1':>7}  note")
    print("-" * 78)
    cases = [("bunny", "bunny", 0.0), ("ellipsoid", "ellipsoid", 0.0),
             ("cuboid(sym)", "cuboid", 0.0), ("washer(sym)", "washer", 0.0),
             ("bunny+noise2%", "bunny", 0.02)]
    for label, obj, noise in cases:
        r = _run_one(_load(obj), args.seed, noise_frac=noise)
        r["case"] = label
        report["quality"].append(r)
        sym = "sym→pose非一意" if "sym" in label else ("noise" if noise else "")
        print(f"{label:<18}{r['n']:>4}{r['coverage']*100:>6.0f}"
              f"{r['edges_raw']:>5}->{r['edges']:<4}{r['n_registered']:>3}/{r['n']:<3}"
              f"{r['correct_pose']:>4}/{r['n']:<2}{r['CD_pctL']:>8.2f}{r['F1']:>7.3f}  {sym}")

    # --- 2. 速度: O(N^2) 全ペア RANSAC ---
    speed_ns = [] if args.skip_speed else [8, 16, 24]
    if speed_ns:
        print("\n=== (A) 速度: bunny で N を増やす（全ペア RANSAC = O(N^2)）===")
        print(f"{'N':>5}{'pairs=N(N-1)/2':>16}{'reg_time_s':>12}{'s/pair':>10}")
        print("-" * 46)
    for nn in speed_ns:
        r = _run_one(_load("bunny"), args.seed, force_n=nn)
        r["case"] = f"bunny_N{nn}"
        report["speed"].append(r)
        pairs = nn * (nn - 1) // 2
        print(f"{nn:>5}{pairs:>16}{r['reg_time_s']:>12.1f}{r['reg_time_s']/pairs*1000:>9.1f}ms")

    (out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nsaved -> {out/'report.json'}\n")


if __name__ == "__main__":
    main()
