"""(c) A1 カリキュラム: 重複量(cap_angle)を掃引し、参照無し登録の破綻境界を測る。

    python scripts/run_a1_curriculum.py --n 10 --seed 0

cap_angle=180(全周/易) → 90(半球/バラ積み相当) → 80(難) と下げ、各点で
A1 の「正しく登録された個体率」「CD(整列後)」を測定。パイプライン健全性
（重複大なら成功するか）と、どの重複量で崩れるかの境界を出す。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np                                   # noqa: E402
import open3d as o3d                                 # noqa: E402

from partsrecon import geometry as G                 # noqa: E402
from partsrecon.data.synthetic_views import make_view_scene  # noqa: E402
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


def _load_bunny(target=0.12):
    import trimesh
    try:
        mesh = trimesh.load(o3d.data.BunnyMesh().path, force="mesh")
    except Exception:
        mesh = G.make_default_object("lblock")
    mesh.apply_translation(-mesh.centroid)
    mesh.apply_scale(target / float(np.max(mesh.extents)))
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--caps", type=float, nargs="+", default=[180, 150, 130, 110, 95, 80])
    ap.add_argument("--out", default=str(ROOT / "outputs" / "a1_curriculum"))
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    mesh = _load_bunny()
    rows = []
    print(f"\n=== A1 curriculum (Bunny, N={args.n}, seed={args.seed}) ===")
    print(f"{'cap':>5}{'reg/N':>6}{'corr':>10}{'rotMed':>8}{'CD%L':>8}{'F@1':>7}{'cov%':>7}  verdict")
    print("-" * 64)
    for cap in args.caps:
        scene = make_view_scene(mesh, args.n, args.seed, cap_angle_deg=cap)
        L = scene.scale_L
        ev, rv = 0.005 * L, 0.01 * L
        taus = [0.01 * L]
        est, info = register_groupwise(scene.partials_world, rv)

        Ms = {i: est[i] @ scene.poses[i] for i in est}
        ids = sorted(est)
        rot = []
        if ids:
            Ginv = np.linalg.inv(Ms[ids[0]])
            rot = [_rot_deg(Ginv @ Ms[i]) for i in ids]
        correct = sum(1 for r in rot if r < 10.0)
        rot_med = float(np.median(rot)) if rot else float("nan")

        s_a1 = _fuse_node(scene.partials_world, est, ev)
        cd = f1 = cov = float("nan")
        if len(s_a1) > 100:                          # GT既知ゲージ(root)で整列 → 真のモデル品質
            Galign = np.linalg.inv(scene.poses[info["root"]])
            s_al = G.apply_pose(s_a1, Galign)
            res = M.evaluate(s_al, scene.s_gt, taus=taus, voxel=ev)
            cd = res["chamfer_l1"] / L * 100
            f1 = res[f"F@{taus[0]:.6g}"]
            cov = M.coverage(scene.s_gt, s_al, taus[0], voxel=ev) * 100
        verdict = "OK" if (correct >= 0.8 * info["n"] and f1 > 0.7) else (
            "PARTIAL" if correct >= 0.4 * info["n"] else "FAIL")
        rows.append({"cap_deg": cap, "n": info["n"], "n_registered": info["n_registered"],
                     "n_edges": info["n_edges"], "correct_lt10deg": correct,
                     "rot_median_deg": rot_med, "CD_pctL": cd, "F1": f1, "cov_pct": cov,
                     "verdict": verdict})
        print(f"{cap:>5.0f}{info['n_registered']:>4}/{info['n']:<2}{correct:>8}/{info['n']:<2}"
              f"{rot_med:>8.1f}{cd:>8.2f}{f1:>7.3f}{cov:>7.1f}  {verdict}")

    (out / "report.json").write_text(json.dumps({"config": {"n": args.n, "seed": args.seed},
                                                 "sweep": rows}, indent=2, ensure_ascii=False),
                                     encoding="utf-8")
    print("-" * 64)
    print(f"saved -> {out/'report.json'}\n")


if __name__ == "__main__":
    main()
