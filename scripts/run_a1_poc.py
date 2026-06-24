"""A1 PoC: 参照無し groupwise registration の feasibility 検証（Stanford Bunny）。

    python scripts/run_a1_poc.py --n 12 --seed 0

「姿勢なし SfM」相当の難問。複雑・非対称な Bunny 1種類で「本当に通るか」を、
A0(GT姿勢=上界) と比較しつつ正直に測る。GT姿勢は評価専用（復元には未使用）。
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
from partsrecon.registration.groupwise import register_groupwise  # noqa: E402


def _rot_angle_deg(Rm: np.ndarray) -> float:
    c = (np.trace(Rm[:3, :3]) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _fuse_node(partials, est, voxel):
    canon = [G.apply_pose(partials[i], est[i]) for i in est if len(partials[i])]
    if not canon:
        return np.zeros((0, 3))
    pcd = G.to_o3d_pcd(np.concatenate(canon, axis=0)).voxel_down_sample(voxel)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return G.from_o3d_pcd(pcd)


def _load_bunny(target_extent=0.12):
    import trimesh
    try:
        mesh = trimesh.load(o3d.data.BunnyMesh().path, force="mesh")
        name = "stanford_bunny"
    except Exception as e:                            # ネット不通等
        print(f"[warn] bunny load failed ({e}); fallback to lblock")
        mesh = G.make_default_object("lblock"); name = "lblock"
    mesh.apply_translation(-mesh.centroid)
    mesh.apply_scale(target_extent / float(np.max(mesh.extents)))
    return mesh, name


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--scene", choices=["covering", "bin"], default="covering",
                    help="covering=前提保証(マッチング有効＋全周囲被覆), bin=物理バラ積み(遮蔽あり)")
    ap.add_argument("--out", default=str(ROOT / "outputs" / "a1_poc"))
    args = ap.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

    mesh, name = _load_bunny()
    if args.scene == "covering":
        from partsrecon.data.synthetic_views import make_covering_scene
        scene, sinfo = make_covering_scene(mesh, seed=args.seed)
        print(f"[scene] premise-conforming: n={sinfo['n']}  coverage={sinfo['coverage']*100:.1f}%  "
              f"cap={sinfo['cap_angle_deg']:.0f}deg  (matchable segments exhaustively covering)")
    else:
        scene = generate_bin_scene(mesh, n_instances=args.n, seed=args.seed)
    L = scene.scale_L
    n_act = len(scene.partials_world)
    eval_voxel = 0.005 * L
    reg_voxel = 0.01 * L
    taus = [0.005 * L, 0.01 * L, 0.02 * L]
    tk = [f"{t:.6g}" for t in taus]

    # --- A0 oracle（上界, GT姿勢） ---
    s_a0 = fuse_oracle(scene.partials_world, scene.poses, voxel=eval_voxel, denoise=True)
    a0 = M.evaluate(s_a0, scene.s_gt, taus=taus, voxel=eval_voxel)

    # --- A1（参照無し groupwise registration） ---
    est, info = register_groupwise(scene.partials_world, reg_voxel)
    s_a1 = _fuse_node(scene.partials_world, est, eval_voxel)

    # 姿勢一貫性（GTゲージで評価）: M_i = est_i @ T_gt_i は理想的に定数 G
    Ms = {i: est[i] @ scene.poses[i] for i in est}
    reg_ids = sorted(est.keys())
    rot_err, tr_err = [], []
    if reg_ids:
        Gref = Ms[info["root"]]            # root が canonical-est をアンカー（est[root]=I）
        Ginv = np.linalg.inv(Gref)
        for i in reg_ids:
            E = Ginv @ Ms[i]
            rot_err.append(_rot_angle_deg(E))
            tr_err.append(float(np.linalg.norm(E[:3, 3]) / L * 100))

    # A1 を GT 既知ゲージ（root の世界姿勢の逆）で正準系へ整列して品質を測る。
    # RANSAC 整列は部分モデルで誤収束するため、評価には使わない。
    a1 = None
    if len(s_a1) > 100 and info["root"] in est:
        Galign = np.linalg.inv(scene.poses[info["root"]])
        a1 = M.evaluate(G.apply_pose(s_a1, Galign), scene.s_gt, taus=taus, voxel=eval_voxel)

    # inlier-by-gauge: 各個体の M_i が Gref に近い割合（正しく登録された個体率）
    inlier = sum(1 for r in rot_err if r < 10.0) if rot_err else 0

    report = {
        "config": {"object": name, "n": args.n, "seed": args.seed, "scale_L_m": L,
                   "reg_voxel_m": reg_voxel, "eval_voxel_m": eval_voxel},
        "registration": {**info, "gauge_inlier_(<10deg)": inlier,
                         "rot_err_deg_median": float(np.median(rot_err)) if rot_err else None,
                         "rot_err_deg_max": float(np.max(rot_err)) if rot_err else None,
                         "trans_err_pctL_median": float(np.median(tr_err)) if tr_err else None},
        "A0_oracle": a0, "A1": a1,
    }
    (out / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    o3d.io.write_point_cloud(str(out / "s_a1.ply"), G.to_o3d_pcd(s_a1))
    o3d.io.write_point_cloud(str(out / "s_a0_oracle.ply"), G.to_o3d_pcd(s_a0))
    o3d.io.write_point_cloud(str(out / "s_gt.ply"), G.to_o3d_pcd(scene.s_gt))

    print(f"\n=== A1 PoC ===  object={name}  scene={args.scene}  N={n_act}  seed={args.seed}  L={L*1000:.1f}mm")
    print(f"registration: {info['n_registered']}/{info['n']} nodes in main component  "
          f"(edges={info['n_edges']}, tree={info['n_tree_edges']}, root={info['root']})")
    if rot_err:
        print(f"pose consistency (GT gauge): rot median={np.median(rot_err):.2f}deg max={np.max(rot_err):.2f}deg, "
              f"trans median={np.median(tr_err):.2f}%L  | correctly-registered(<10deg)={inlier}/{info['n_registered']}")
    print("-" * 64)
    print(f"{'metric':<14}{'A0 oracle':>14}{'A1 (reg-free)':>16}")
    def row(label, key, scale=1.0, pct=False):
        av = a0.get(key); bv = a1.get(key) if a1 else None
        fa = f"{av*scale:.3f}" if av is not None else "-"
        fb = f"{bv*scale:.3f}" if bv is not None else "-"
        print(f"{label:<14}{fa:>14}{fb:>16}")
    row("CD (%L)", "chamfer_l1", scale=100.0 / L)
    row("Accuracy(%L)", "accuracy", scale=100.0 / L)
    row("Complete(%L)", "completeness", scale=100.0 / L)
    row(f"F@1%L", f"F@{tk[1]}")
    row(f"F@2%L", f"F@{tk[2]}")
    print("-" * 64)
    verdict = "FAILED" if a1 is None else ("OK" if a1.get(f"F@{tk[1]}", 0) > 0.7 and inlier >= 0.6 * info["n_registered"] else "PARTIAL")
    print(f"verdict: {verdict}")
    print(f"saved -> {out/'report.json'} , s_a1.ply , s_a0_oracle.ply , s_gt.ply\n")


if __name__ == "__main__":
    main()
