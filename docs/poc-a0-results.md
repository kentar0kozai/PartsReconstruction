# A0 PoC 結果 — oracle 融合 + 評価ハーネス検証

> [baseline-classical-geometry.md](baseline-classical-geometry.md) §8/§10 の A0 段階、[evaluation-harness.md](evaluation-harness.md) §6 被覆性、[synthetic-data-foundation.md](synthetic-data-foundation.md) §7 PoC の最初の vertical slice。
>
> - ステータス: A0 PoC 完了（モックデータ）
> - 実施日: 2026-06-23
> - コード: `src/partsrecon/`（eval / data / fusion / geometry）、`scripts/run_a0_poc.py`
> - 再現: `python scripts/run_a0_poc.py --n 20 --orient random --object cuboid`

---

## 1. 検証目的

BlenderProc 非依存のモック生成器（メッシュ→N個を姿勢付き配置→トップ視点 HPR で 2.5D 化、GT姿勢/セグ/正準点群は構成上既知）で、以下を end-to-end 検証：
1. 基盤要件 R1–R5（[synthetic-data-foundation.md](synthetic-data-foundation.md) §1）が動くか。
2. 評価ハーネス（[evaluation-harness.md](evaluation-harness.md) §4/§6）が正しく動くか。
3. **A2 被覆性の上界**（GT姿勢で融合すれば全周囲が埋まるか）が N とともにどう伸びるか。

---

## 2. 結果

| 物体 | 姿勢 | N | Accuracy(%L) | Completeness(%L) | CD-L1(%L) | F@1%L | 被覆飽和（τ=1%L） |
|------|------|---|:---:|:---:|:---:|:---:|---|
| cuboid | random | 20 | 0.30 | 0.39 | 0.69 | 0.987 | **99.8%**（N=5 で飽和） |
| cuboid | stable | 30 | 0.30 | 0.40 | 0.69 | 0.987 | 99.7%（N=30、緩やかに上昇） |
| washer | random | 30 | 0.26 | 0.72 | 0.99 | 0.938 | **~89.9% で頭打ち** |
| washer | stable | 30 | 0.26 | 0.74 | 1.00 | 0.934 | ~89.4% で頭打ち |

被覆 vs N（代表例）:
- cuboid/random: 51.7%(N=1) → 79.4%(2) → 92.2%(3) → **99.8%(5+)**
- cuboid/stable: 29.5%(1) → 58.9%(2) → 71.9%(5) → 92.4%(8) → **99.7%(30)**
- washer/random: 45.8%(1) → 76.2%(2) → 89.6%(5) → **89.9%(頭打ち)**

---

## 3. 主要観察

1. **A0 oracle が GT 表面にほぼ一致（Accuracy 0.26–0.30% L）** → 融合・正準系・評価ハーネスの実装が正しいことを確認。偽点が無い（高 Precision）。
2. **接地姿勢バイアスが被覆の伸びを鈍化** — cuboid は random だと N=5 で 99.8% に達するが、stable では N=30 まで要する。**A2 リスク（[research-overview.md](research-overview.md) §5.2）の機序を定量再現**。
3. **形状によっては被覆に天井がある** — washer は N を増やしても ~90% で頭打ち。残り ~10% は単一トップ視点では原理的に観測しにくい領域（内孔壁・下向き面・接地面）。**A2 が成立しない幾何が存在する**ことの実証。
4. 評価ハーネスは欠損を **Completeness 側**に正しく帰属（washer は Completeness 悪化＝0.72–0.74% L、Accuracy は良好のまま）。[evaluation-harness.md](evaluation-harness.md) §6 の誤差分解の妥当性を支持。

---

## 4. 重要な注意（モックの簡略化と数値の解釈）

本 PoC の被覆数値は**モックの単純化に依存**するため、研究主張としての確定値ではない。BlenderProc 実データ（M2）で変化する：

| 簡略化 | 影響 | M2 で変わる点 |
|--------|------|----------------|
| 各個体を**真上から**観測（厳密な鉛直視） | 斜め視で見える面を取りこぼす → 被覆を過小評価しうる | 実機は単一トップセンサで周辺個体を**斜めから**観測 → 被覆増 |
| **接地面は常に下向き**で不可視 | 接地領域が観測されない | 物理的に妥当（実機も同様）。傾き接地で緩和 |
| **個体間の相互遮蔽を無視** | 被覆を過大評価しうる | 実機は山積みで相互遮蔽 → 被覆減 |
| 一様サンプリング（Poisson-disk でない） | 密度ムラ | spec 既定は Poisson-disk |
| cuboid は対称形状 | 位置合わせ評価（A1）には不適 | 非対称/対称を分けて評価 |

→ **PoC の役割は「ハーネスと機序の検証」**であり、~90% という具体値は mock 依存。被覆の絶対評価は実データで行う。

---

## 5. 含意

- 評価ハーネス・A0 融合・被覆解析が**全ファミリ共通基盤として機能**することを確認。A1（実位置合わせ）はこの上に載せられる。
- **被覆性 A2 の定量解析が研究貢献になりうる**（[literature-survey.md](literature-survey.md) §8）方針を、PoC レベルで裏付け：接地バイアス・形状依存の天井という**具体的な失敗様式**を測れる。
- 「ワンショット・トップダウン」前提の限界（内孔・下向き面）は、対象形状クラスの**適用可能範囲**として論文で正面から扱う論点。

---

## 6. 次アクション候補

| 候補 | 内容 | 使う既存資産 |
|------|------|--------------|
| **A1 実装** | GT姿勢を外し、FPFH+RANSAC→ポーズグラフで groupwise registration（[baseline-classical-geometry.md](baseline-classical-geometry.md) §3.1） | `eval.global_align` 雛形・評価ハーネス・モック生成器 |
| M2 実データ化 | モック生成器を **BlenderProc 物理ドロップ＋深度レンダ**に差し替え（T-LESS 等） | 評価ハーネス全体 |
| モック高度化 | 斜め視・個体間遮蔽・Poisson-disk・センサノイズ | 生成器の拡張 |

> 推奨: **A1 実装**（本研究の核 = 参照無し groupwise registration を初めて実問題として解く。既存資産でそのまま着手可、誤差3層分解の②層を測れる）。

---

## 7. 忠実版バラ積み生成器（PyBullet＋Open3D）の結果

`src/partsrecon/data/bin_sim.py` + `scripts/run_bin_poc.py`（[data-generation-design.md](data-generation-design.md) §5）。物理山積み→単一トップダウン raycast→個体セグGT。N=15:

| 物体 | captured pts | per-inst min/med/max | 完全埋没 | CD(%L) | F@1%L | 被覆飽和 |
|------|---|---|---|---|---|---|
| cuboid | 170k | 5047/10719/18229 | 0/15 | 0.48 | 1.000 | 100%(N=12) |
| washer | 201k | 5439/13451/18968 | 0/15 | 0.57 | 0.974 | 100%(N=12) |

被覆 vs N（cuboid）: 26.8%(1)→41.8%(2)→54.5%(3)→62.1%(5)→91.8%(8)→**100%(12)**

**主要観察:**
1. **相互遮蔽が正しく入った** — per-instance 点数が大きくばらつく（遮蔽量の差）。物理 piling＋単一視点 raycast の妥当性を確認。
2. **被覆は独立モックより緩やかに増えて収束** — モック(random)は N=5 で 99.8%、忠実版は N=8 で ~92%・N=12 で 100%。安定接地＋遮蔽で必要 N は増えるが、**15個のワンショットで全周囲を被覆（A2 成立）**。
3. **§4 のモック過小評価を是正** — モック washer は ~90% 頭打ちだったが、忠実版 washer は 100% 到達。理由は「物理 piling の傾き接地＋透視カメラの斜め視で内孔・側面が露出」。→ **被覆の絶対評価は忠実版で行うべき**との §4 の見立てが裏付けられた。
4. A0 融合＋評価ハーネスは**生成器を差し替えただけ**でそのまま動作（CD 0.48–0.57% L、偽点なし）。

→ M2「合成データ生成基盤」の中核が完成。stage① セグは GT 利用（スコープ外）。次は **A1**（GT姿勢を外した参照無し groupwise registration）。
