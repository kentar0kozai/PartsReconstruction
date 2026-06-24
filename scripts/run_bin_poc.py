"""バラ積み生成器 PoC ランナー（PyBullet 物理 + Open3D 単一トップダウン raycast）。

    python scripts/run_bin_poc.py --n 15 --object cuboid
    python scripts/run_bin_poc.py --n 15 --object washer --mesh path/to.ply

検証点:
  1. 物理山積み＋単一視点 raycast で「相互遮蔽のある 2.5D 集団点群」が生成できる。
  2. 個体セグ GT・GT 姿勢が取れ、A0 融合＋評価ハーネスがそのまま動く。
  3. 独立観測モック比で、被覆が遮蔽の影響でどう変わるか。
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
from partsrecon.data.bin_sim import generate_bin_scene  # noqa: E402
from partsrecon.eval import metrics as M             # noqa: E402
from partsrecon.fusion.aggregate import fuse_oracle  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--object", default="cuboid", choices=["cuboid", "washer", "lblock"])
    ap.add_argument("--mesh", default=None)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--out", default=str(ROOT / "outputs" / "bin_poc"))
    args = ap.parse_args()

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    if args.mesh:
        import trimesh
        mesh = trimesh.load(args.mesh, force="mesh")
        mesh.apply_translation(-mesh.centroid)
        obj_name = args.mesh
    else:
        mesh = G.make_default_object(args.object)
        obj_name = args.object

    scene = generate_bin_scene(mesh, n_instances=args.n, seed=args.seed, width=args.width)
    L = scene.scale_L
    voxel = 0.005 * L
    tau_pcts = [0.005, 0.01, 0.02]
    taus = [p * L for p in tau_pcts]

    counts = np.array([len(pw) for pw in scene.partials_world])
    occluded = int((counts == 0).sum())
    total_pts = int(counts.sum())

    # A0: GT姿勢で融合（=観測和集合）+ 評価
    s_hat = fuse_oracle(scene.partials_world, scene.poses, voxel=voxel, denoise=True)
    res = M.evaluate(s_hat, scene.s_gt, taus=taus, voxel=voxel)

    # 被覆 vs N（観測和集合, tau=1%L）
    ns = sorted({k for k in [1, 2, 3, 5, 8, 12, 15, 20, 30] if k <= args.n} | {args.n})
    cov = {}
    for k in ns:
        union = fuse_oracle(scene.partials_world[:k], scene.poses[:k], voxel=voxel, denoise=False)
        cov[k] = M.coverage(scene.s_gt, union, 0.01 * L, voxel=voxel) if len(union) else 0.0

    report = {
        "config": {"object": obj_name, "n": args.n, "seed": args.seed,
                   "width": args.width, "scale_L_m": L, "voxel_m": voxel},
        "occlusion": {"captured_points": total_pts,
                      "per_instance_min": int(counts.min()), "per_instance_median": int(np.median(counts)),
                      "per_instance_max": int(counts.max()), "fully_occluded_instances": occluded},
        "A0_metrics": res, "coverage_vs_N": cov,
    }
    (out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    o3d.io.write_point_cloud(str(out / "s_hat_A0.ply"), G.to_o3d_pcd(s_hat))
    o3d.io.write_point_cloud(str(out / "s_gt.ply"), G.to_o3d_pcd(scene.s_gt))
    # シーン全点（個体セグ色は省略、形状確認用）
    allpts = np.concatenate([pw for pw in scene.partials_world if len(pw)], axis=0) if total_pts else np.zeros((0, 3))
    o3d.io.write_point_cloud(str(out / "scene_topdown.ply"), G.to_o3d_pcd(allpts))

    print(f"\n=== BIN PoC ===  object={obj_name}  N={args.n}  seed={args.seed}  width={args.width}")
    print(f"scale L = {L*1000:.1f} mm   voxel = {voxel*1000:.2f} mm")
    print(f"captured pts = {total_pts:,}  | per-instance min/med/max = "
          f"{counts.min()}/{int(np.median(counts))}/{counts.max()}  | fully-occluded = {occluded}/{args.n}")
    print("-" * 60)
    print(f"Accuracy     : {res['accuracy']*1000:7.3f} mm  ({res['accuracy']/L*100:5.2f}% L)")
    print(f"Completeness : {res['completeness']*1000:7.3f} mm  ({res['completeness']/L*100:5.2f}% L)")
    print(f"Chamfer-L1   : {res['chamfer_l1']*1000:7.3f} mm  ({res['chamfer_l1']/L*100:5.2f}% L)")
    print("-" * 60)
    for pc, tau in zip(tau_pcts, taus):
        k = f"{tau:.6g}"
        print(f"  tau={pc*100:.1f}% L : P={res[f'P@{k}']:.3f}  R={res[f'R@{k}']:.3f}  F={res[f'F@{k}']:.3f}")
    print("-" * 60)
    print("Coverage vs N (oracle union, tau=1.0% L):")
    for k in ns:
        bar = "#" * int(round(cov[k] * 40))
        print(f"  N={k:3d} : {cov[k]*100:6.2f}%  {bar}")
    print(f"\nsaved -> {out/'report.json'} , s_hat_A0.ply , s_gt.ply , scene_topdown.ply\n")


if __name__ == "__main__":
    main()
