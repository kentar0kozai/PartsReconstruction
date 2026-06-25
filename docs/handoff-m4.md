# Handoff — M4（学習系）を GPU 環境で再開するために

> CPU専用 torch では学習手法が回せないため GPU 環境へ移行する。本ドキュメントは新環境での**再構築手順**と**M4計画**をまとめた引き継ぎメモ。意思決定・知見の実体は `docs/` 各ファイル（origin/main に push 済み）。

- 作成: 2026-06-25
- 現在地: `origin/main @ aa21b7f` に M0–M3 一式（[research-overview](research-overview.md) ロードマップ参照）

## 現在地サマリ（M0–M3 完了）
- **M0–M2**: 研究spec・文献調査（新規性ギャップ確認・4ファミリ）・忠実バラ積み生成器（PyBullet物理＋Open3D単一トップダウンraycast）＋前提保証シーン生成器＋評価ハーネス（姿勢自由・多重ICP整列の重なり率 F）。
- **M3**: 古典ベースライン。A0(oracle)で前提成立を実証。A1(参照無し groupwise registration)を実装し、物体クラス×手法(RANSAC/FGR)×シードで限界を確定：
  - **特徴豊富な曲面のみ RANSAC で堅牢（F≈0.998）だが遅い（~60s/task）**。
  - **FA頻出の対称・平面は RANSAC/FGR どちらも脆い or 失敗、勝者も一定せず**（[poc-a1-results](poc-a1-results.md) §9）。
  - → **学習移行の動機が確立**（特に対称・平面部品）。

## GPU 環境での再開手順
1. `git clone https://github.com/kentar0kozai/PartsReconstruction.git`
2. `uv venv --python 3.11` ; `uv pip install -r requirements.txt`
3. **torch を CUDA 版に入れ替え**（requirements の torch は注記のみ。例: CUDA 12.1）:
   `uv pip install torch --index-url https://download.pytorch.org/whl/cu121`
4. スモークテスト（古典が動くこと）: `python scripts/run_a1_poc.py --scene covering`
5. GPU確認: `python -c "import torch; print(torch.cuda.is_available())"` → True

## M4 計画（GPUで現実的になるもの）
CPUでは不可だった学習手法が着手可能になる。いずれも**既存の評価ハーネス＋シーン生成器で古典(RANSAC/FGR)と同条件比較**する（`run_a1_brittleness.py` の method切替・seed・逐次の構造を学習手法にも踏襲）。

| 候補 | 内容 | 期待 |
|------|------|------|
| **学習記述子で登録改善**（推奨初手） | `registration/groupwise.py` の FPFH を GeoTransformer/Predator 等の**学習特徴**に置換。最小変更。 | 対称・平面の誤対応を減らせるか（古典の最大の弱点に直撃） |
| **B 正準化**（ConDor / SE(3)-equivariant） | 部分点群→正準姿勢を学習。要カテゴリ学習データ。 | 対称物体の姿勢曖昧性を学習で吸収 |
| **C 陰関数同時最適化**（DeepSDF系） | 潜在形状＋姿勢を同時最適化（事前学習prior or per-scene）。 | 欠損補完＋滑らかな全周囲復元 |

### 学習データ生成（既存資産で可能）
- `data/synthetic_views.make_covering_scene`（前提保証・可制御）＋ `data/bin_sim`（忠実バラ積み）で物体クラス×姿勢を大量生成。GT姿勢・正準点群つき。
- 物体ソース: 手続き生成（`geometry.make_default_object`）＋公開メッシュ（Open3D Bunny/Armadillo/Knot）＋将来 BOP（T-LESS/XYZ-IBD/IPD, [synthetic-data-foundation](synthetic-data-foundation.md) §3.2）。

### 評価
- `eval/metrics.align_to_gt`（姿勢自由・重なり率 F@τ）を全手法共通で使用。古典の確定比較は [poc-a1-results](poc-a1-results.md) §9（RANSAC/FGR × 3シード）。学習手法も同じ表に並べる。

## 未決（M4着手時に決める）
- 最初に実装する学習手法（推奨: 学習記述子でのFPFH置換 → 次に B 正準化）。
- 学習データの規模・物体カテゴリ構成。
- 学習特徴モデルの選定とライセンス／導入容易性（Windows/Linux差）。

## チャット履歴について
本セッションの意思決定・知見・設計は全て `docs/` に記録済み（これが移行に必要な実体）。生のチャット履歴(jsonl)はリポジトリに含めない（大量・ノイズ・ローカルパス含むため）。新環境では clone ＋ docs ＋ 本handoffで文脈を復元する。
