# PartsReconstruction — バラ積み部品のワンショット計測からの全周囲モデル生成

FA分野で同一種類の剛体部品が段ボールにバラ積みされたシーンを**上方から1ショット深度計測**し、得られる **2.5D 片面部分点群の集団**を、参照モデル無しで共通正準座標系へ統合して **1物体の全周囲モデル（点群）** を復元する研究。

**核心アイデア**: 物理的に別個体だが幾何的に同一なインスタンス群を、1物体のマルチビュー観測の代用とみなす（＝「姿勢なし SfM」的な参照無し groupwise registration）。

> スペック駆動開発。設計・調査・実験はすべて `docs/` に記録。個人研究用途。

## ステータス（M0–M3 完了）

- **M0 全体像** / **M1 文献調査**（新規性ギャップ確認・4手法ファミリ系統化）/ **M1.5 補足**（対称性処理・接地姿勢分布の古典的基礎 = Goldberg）。
- **M2 合成データ基盤＋評価ハーネス**: PyBullet 物理山積み＋Open3D 単一トップダウン raycast の忠実なバラ積み生成器、前提保証シーン生成器（マッチング有効＋全周囲被覆を保証）、**姿勢自由・多重ICP整列の重なり率(F-score)** 評価。
- **M3 古典ベースライン（ファミリA）**:
  - **A0 (oracle)**: GT姿勢で融合すれば全周囲を復元可（前提成立、CD ≒ 0.4–0.5% L）。
  - **A1 (参照無し)**: 前提保証シーンで中核仮説を実証（10/10登録, CD ≒ oracle）。物体クラス × 手法(RANSAC/FGR) × シードで限界を確定：
    - 特徴豊富な曲面 → RANSAC で堅牢 (F≈0.998) だが**遅い** (~60s/task)。
    - **FA頻出の対称・平面 → RANSAC/FGR どちらも脆い or 失敗、勝者も一定せず** → 学習移行の動機。
- **次（M4）**: 学習ファミリ（B 正準化 / C 陰関数同時最適化）を同じ評価ハーネス＆前提保証シーンで比較。

## セットアップ

```bash
uv venv --python 3.11
uv pip install -r requirements.txt
```

## 実行（PoC / 実験）

```bash
# A0: oracle融合＋評価ハーネス（モックデータ）
.venv/Scripts/python scripts/run_a0_poc.py --n 20 --object cuboid
# 忠実バラ積み生成（PyBullet物理＋Open3D単一トップダウンraycast）
.venv/Scripts/python scripts/run_bin_poc.py --n 15 --object washer
# A1: 参照無し groupwise registration（前提保証シーン）
.venv/Scripts/python scripts/run_a1_poc.py --scene covering
# 物体別ブリットルネス＋速度（逐次・method切替・resume）
.venv/Scripts/python scripts/run_a1_brittleness.py --method ransac --seeds 0 1 2
```

## 構成

- `docs/` — 設計・調査・実験記録（下記インデックス）
- `src/partsrecon/` — `geometry` / `data`（mock_bin・bin_sim・synthetic_views）/ `eval`（metrics）/ `fusion`（aggregate）/ `registration`（groupwise）
- `scripts/` — PoC・実験ランナー
- `outputs/` — 実験結果（git管理外）

## ドキュメント索引

| doc | 内容 |
|-----|------|
| [research-overview](docs/research-overview.md) | 研究全体像（問題定義・新規性・共通パイプライン・ロードマップ） |
| [literature-survey](docs/literature-survey.md) | 文献調査（新規性ギャップ・4ファミリ・対称性/被覆性） |
| [synthetic-data-foundation](docs/synthetic-data-foundation.md) | 合成データ基盤の技術選定 |
| [data-generation-design](docs/data-generation-design.md) | バラ積みシーン生成設計（PyBullet＋Open3D・前提保証） |
| [evaluation-harness](docs/evaluation-harness.md) | 評価指標定義（CD / F-score / 被覆性 / 誤差3層分解） |
| [baseline-classical-geometry](docs/baseline-classical-geometry.md) | ファミリA（古典幾何）設計 |
| [poc-a0-results](docs/poc-a0-results.md) | A0・被覆性・忠実生成器の結果 |
| [poc-a1-results](docs/poc-a1-results.md) | A1の実証と限界（RANSAC vs FGR 確定比較） |
| [headroom-setup](docs/headroom-setup.md) | （補助）入力トークン圧縮ツール headroom のメモ |

## 環境

Python 3.11 / Open3D / trimesh / scipy / PyBullet（uv 管理の `.venv`）。
