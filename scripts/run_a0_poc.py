"""A0 PoC ランナー — oracle 融合 + 評価ハーネス + 被覆 vs N。

使い方:
    python scripts/run_a0_poc.py --n 20 --orient random
    python scripts/run_a0_poc.py --n 30 --orient stable --object washer

検証する仮説:
    1. 基盤+評価ハーネスが要件 R1-R5 を満たして動く。
    2. A2 被覆性の上界（GT姿勢で融合すれば全周囲が埋まるか）が N とともに伸びる。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import open3d as o3d  # noqa: E402

from partsrecon import geometry as G          # noqa: E402
from partsrecon.data.mock_bin import generate_scene  # noqa: E402
from partsrecon.eval import metrics as M       # noqa: E402
from partsrecon.fusion.aggregate import fuse_oracle  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20, help="インスタンス数")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--orient", choices=["random", "stable"], default="random")
    ap.add_argument("--object", default="cuboid", choices=["cuboid", "washer", "lblock"])
    ap.add_argument("--mesh", default=None, help="メッシュファイル（--object を上書き）")
    ap.add_argument("--out", default=str(ROOT / "outputs" / "a0_poc"))
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    if args.mesh:
        import trimesh
        mesh = trimesh.load(args.mesh, force="mesh")
        mesh.apply_translation(-mesh.centroid)
        obj_name = args.mesh
    else:
        mesh = G.make_default_object(args.object)
        obj_name = args.object

    scene = generate_scene(mesh, n_instances=args.n, seed=args.seed, orient=args.orient)
    L = scene.scale_L
    voxel = 0.005 * L
    tau_pcts = [0.005, 0.01, 0.02]
    taus = [p * L for p in tau_pcts]
    tau_cov = 0.01 * L

    # --- A0: oracle 融合 + 評価 ---
    s_hat = fuse_oracle(scene.partials_world, scene.poses, voxel=voxel, denoise=True)
    res = M.evaluate(s_hat, scene.s_gt, taus=taus, voxel=voxel)

    # --- 被覆 vs N（oracle 観測和集合） ---
    candidate_ns = sorted({n for n in [1, 2, 3, 5, 8, 12, 20, 30, 50] if n <= args.n} | {args.n})
    cov_curve = {}
    for n in candidate_ns:
        union = fuse_oracle(scene.partials_world[:n], scene.poses[:n], voxel=voxel, denoise=False)
        cov_curve[n] = M.coverage(scene.s_gt, union, tau_cov, voxel=voxel)

    report = {
        "config": {
            "object": obj_name, "n": args.n, "seed": args.seed, "orient": args.orient,
            "scale_L_m": L, "voxel_m": voxel,
            "tau_pcts_of_L": tau_pcts, "tau_coverage_pct_of_L": 1.0,
            "total_partial_points": int(sum(len(p) for p in scene.partials_world)),
        },
        "A0_metrics": res,
        "coverage_vs_N": cov_curve,
    }
    (out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    o3d.io.write_point_cloud(str(out / "s_hat_A0.ply"), G.to_o3d_pcd(s_hat))
    o3d.io.write_point_cloud(str(out / "s_gt.ply"), G.to_o3d_pcd(scene.s_gt))

    # --- 表示 ---
    print(f"\n=== A0 PoC ===  object={obj_name}  N={args.n}  orient={args.orient}  seed={args.seed}")
    print(f"scale L = {L*1000:.1f} mm   voxel = {voxel*1000:.2f} mm   "
          f"partial pts(total) = {report['config']['total_partial_points']:,}   "
          f"|S_hat|={res['n_hat']:,}  |S_gt|={res['n_gt']:,}")
    print("-" * 60)
    print(f"Accuracy     : {res['accuracy']*1000:7.3f} mm  ({res['accuracy']/L*100:5.2f}% L)")
    print(f"Completeness : {res['completeness']*1000:7.3f} mm  ({res['completeness']/L*100:5.2f}% L)")
    print(f"Chamfer-L1   : {res['chamfer_l1']*1000:7.3f} mm  ({res['chamfer_l1']/L*100:5.2f}% L)")
    print("-" * 60)
    for p, tau in zip(tau_pcts, taus):
        k = f"{tau:.6g}"
        print(f"  tau={p*100:.1f}% L : P={res[f'P@{k}']:.3f}  R={res[f'R@{k}']:.3f}  F={res[f'F@{k}']:.3f}")
    print("-" * 60)
    print(f"Coverage vs N (oracle union, tau=1.0% L):")
    for n in candidate_ns:
        bar = "#" * int(round(cov_curve[n] * 40))
        print(f"  N={n:3d} : {cov_curve[n]*100:6.2f}%  {bar}")
    print(f"\nsaved -> {out/'report.json'} , s_hat_A0.ply , s_gt.ply\n")


if __name__ == "__main__":
    main()
