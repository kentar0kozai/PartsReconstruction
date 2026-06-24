# 文献調査レポート（M1）— 系統サーベイと新規性確認

> [research-overview.md](research-overview.md) §3/§4 の根拠資料。deep-research ハーネス（24ソース取得→115主張→上位25を3票敵対的検証→23確証/2棄却）の結果を durable に記録。
>
> - ステータス: M1 一次サーベイ完了（補足調査の残課題あり、§7）
> - 実施日: 2026-06-23
> - 検証強度: 各主張を独立3エージェントで反証試行（2/3反証で棄却）。下記は high confidence のみ。

---

## 1. 結論サマリ

- **新規性ギャップは保たれている**。「参照モデル無し・CAD未知の同一剛体部品が多様姿勢でバラ積み → 上方1ショット深度 → 幾何同一な多数インスタンスを未知物体のマルチビュー観測の代用 → 共通正準系へ groupwise registration → 全周囲復元」を**そのまま提案した先行研究は確認されなかった**。
- ただし「**繰り返し出現する同一インスタンスを補完的観測として活用し joint reconstruction**」という**中核アイデアには明確な先行概念がある**（最近接 = FurnSet）。新規性は「アイデアが未踏」ではなく「**モダリティ・ドメイン・参照依存性・手法の組み合わせが未踏**」という形で主張すべき（§2）。
- 比較対象の系統は4ファミリに整理できた（§3）。本研究の手法ファミリ A〜E へのマッピングは §5。
- **データ・基盤候補と、被覆性/対称性/評価指標の系統は本バッチで未充足**。補足調査が必須（§7）。

---

## 2. 最近接先行研究と新規性ギャップ

### FurnSet（arXiv:2604.20093, 2026-04）— 最近接
- 「繰り返し同一インスタンスを明示的に識別・活用して復元を改善」「per-object CLS token と set-aware self-attention で同一インスタンスをグループ化し**補完的観測を集約**して joint reconstruction」を明示提案。中核アイデアの概念連鎖が本研究と一致。
- **だが本研究と異なる4軸（=新規性ギャップの正体）**:

| 軸 | FurnSet | 本研究 |
|----|---------|--------|
| モダリティ | 単一 RGB | 深度 / 点群 |
| ドメイン | 屋内家具（3D-Future/3D-Front） | 産業バラ積み剛体部品 |
| 参照依存性 | カテゴリ学習済み生成事前分布 ＋ GT 同一性ラベル | 参照モデル無し・CAD未知・GTラベル無し |
| 手法 | set-aware self-attention の特徴集約 | 共通正準系への groupwise registration |

### 周辺
- **Lookalike3D（arXiv:2603.24713, 2026-03）**: 同一/類似インスタンスの繰り返し手掛かりを扱うが、**マルチビュー画像からのペア分類タスク**であり全周囲復元ではない。
- **Splat-and-Replace（SIGGRAPH 2025, arXiv:2506.06462）**: 繰り返し 3DGS インスタンスを登録し被覆不良/遮蔽領域を補完。関連系統が活発であることの傍証。

> 留意: FurnSet・Lookalike3D は極めて新しいプレプリント。新規性主張はこの2件への近接性に依存するため、**投稿先・査読状況を継続監視**する（§8）。

---

## 3. 比較対象の系統（taxonomy）

```
参照モデル無し 点群整合
├─ 古典ペアワイズ堅牢整合（ビルディングブロック / ベースライン）
│   ├─ TEASER++ (arXiv:2001.07715, T-RO2020)  …99%超外れ値耐性・certifiable・誤差理論限界
│   └─ KISS-Matcher (arXiv:2409.15615)         …学習不要・初期推定不要・FPFH系(Faster-PFH)
│        ※両者ともペアワイズのみ。groupwise/multiway は扱わない
└─ 参照モデル無し 多視点/multiway joint registration（本研究 stage③ の主軸）
    ├─ SGHR (arXiv:2304.00467, CVPR2023)         …pose-graph + IRLS、信頼sparse graph初期化＋history reweighting
    ├─ ODIN + global optimization (arXiv:2404.00429, CVPR2024) …学習拡散ペアワイズ→回転平均(L1+IRLS)→consensus並進再推定
    └─ correspondence-free depth-guided joint opt (arXiv:2506.18922, IROS2025)
                                                  …多フレームを単一global depth mapへ姿勢で関連付け、姿勢とmapを同時NLS最適化（特徴/対応付け不要）

部分観測 → 完全形状（融合・補完）
├─ 陰関数: DeepSDF (arXiv:1901.05103, CVPR2019) → MV-DeepSDF (arXiv:2309.16715, ICCV2023, 複数部分観測をset-level latentへ集約)
└─ 拡散生成: PVD (arXiv:2104.03670, ICCV2021)   …無条件生成と条件付き補完を単一モデルで、別個参照モデル不要

教師なし / カテゴリレベル 正準化
├─ ConDor (arXiv:2201.07788, CVPR2022)          …完全/部分点群の向き＋位置を自己教師で正準化、推論時は任意姿勢の部分入力に対応
└─ SE(3)-equivariant self-supervised pose (arXiv:2111.00190, NeurIPS2021)
                                                  …単一点群からGT姿勢/CAD/多視点教師なしでカテゴリレベル6D姿勢、注釈無しで共通正準系
```

**古典→学習の流れ**: 手作り特徴(FPFH)+RANSAC/TEASER → 学習特徴・拡散(ODIN) ／ pose-graph+IRLS(SGHR) ／ 陰関数(DeepSDF)→集約(MV-DeepSDF)→拡散補完(PVD) ／ 正準化学習(ConDor, SE(3)-equiv)。

---

## 4. ★重要な注意：安易な等置を避ける（敵対的検証で棄却された関連付け）

検証で**反証された**2つの「構造的類似」主張。本研究と混同しないこと：

- **MV-DeepSDF は本研究と構造が異なる（vote 0-3 で棄却）**: MV-DeepSDF の multi-sweeps は「**同一の追跡車両の経時的反復観測**」であり、**物理的に別個体の同一形状群ではない**。かつ ShapeNet 車両のカテゴリ事前分布に強く条件づけられる。
- **ConDor のカテゴリ集団学習も本研究と構造が異なる（vote 1-2 で棄却）**: ConDor は「**カテゴリ集団から正準化を学習**」であり、「多数の同一インスタンス観測をマルチビュー代用にする」本研究とは構造が違う。
- 加えて ConDor・SE(3)-equiv は**カテゴリレベル**（同一カテゴリ多数で学習）であり、**完全未知カテゴリへの無学習汎化や対称物体での頑健性は限定的**（両論文とも対称物体で性能劣化を報告）。

→ これらは「直接の解」ではなく「**関連パラダイム**」。本研究の差分（物理的別個体・参照無し・無学習）を明確に保つ。

---

## 5. 本研究の手法ファミリ A〜E へのマッピング

[research-overview.md](research-overview.md) §4 の候補に、特定済みの具体手法を割り当てる：

| 本研究ファミリ | 具体手法（比較/土台） | 位置づけ |
|----------------|----------------------|----------|
| **A 古典幾何（baseline）** | ペアワイズ: TEASER++ / KISS-Matcher(FPFH) ＋ 多視点: **SGHR (pose-graph+IRLS)** | ベースラインの中核。SGHR が参照無し多視点 joint の直接の比較・土台 |
| **B 学習正準化** | ConDor / SE(3)-equivariant self-supervised | カテゴリレベルの制約に留意（未知カテゴリ・対称で弱い）。差別化の論点 |
| **C 陰関数場** | DeepSDF / MV-DeepSDF | カテゴリ事前依存に留意。参照無し化が本研究の工夫所 |
| **D 微分可能/joint最適化** | correspondence-free depth-guided joint opt (2506.18922) / ODIN | 深度ベース joint 最適化。NeRF-深度特化は要追加調査(§7) |
| **E 生成事前分布** | PVD（拡散補完） | 未観測面の補完。参照モデル不要で本研究に親和的 |

**推奨初期比較**: A（TEASER++/KISS-Matcher + SGHR系の多視点最適化）を baseline に据え、最初の学習系比較として **SGHR/ODIN（同系の最適化）** と **PVD（補完）** を並べるのが系統的。B/C のカテゴリレベル手法は「未知カテゴリ・参照無し」制約下での適用可否を論点化。

---

## 6. データ・基盤候補（部分的・要追加調査）

本バッチで触れたソース（一次資料の確認は限定的、§7 で補強）：
- **MVTec ITODD**（産業金属部品・6D姿勢、研究用途）: https://www.mvtec.com/research-teaching/datasets/mvtec-itodd
- **Fraunhofer IPA bin-picking dataset**（大規模・産業バラ積み 6D姿勢、arXiv:1912.12125）
- **bin-picking.ai dataset**: https://www.bin-picking.ai/en/dataset.html
- **BlenderProc4BOP**（物理ドロップ＋深度＋BOP形式GT 生成基盤）: https://dlr-rm.github.io/BlenderProc/README_BlenderProc4BOP.html
- 関連: Frontiers Robotics & AI（合成データ生成のサーベイ系）, arXiv:2506.00599

→ [synthetic-data-foundation.md](synthetic-data-foundation.md) §3.2 の候補（T-LESS / XYZ-IBD / IPD / ITODD / Fraunhofer IPA）と整合。BlenderProc 中心の自作生成基盤方針は支持される。

---

## 7. 文献カバレッジのギャップ（補足調査の残課題）

本バッチは「整合(1)・補完/陰関数/拡散(3)・正準化(4)」に集中。以下は**一次資料未確認**で、サーベイ完成に追加調査が必須（証拠不在＝不存在ではない）：

| サブ領域 | 未充足内容 | 重要度 |
|----------|-----------|--------|
| 2 | 古典 Poisson / TSDF融合 / scan integration の代表論文 | 中（ベースライン記述の出典） |
| 5 | **対称物体の曖昧性処理**（対称群考慮の評価・最適化） | **高**（FA部品は対称多。stage③の正準系一意性破れに直結） |
| 6 | bin-picking特化の自己教師あり3D復元、**NeRF/微分可能レンダリングの深度入力適用** | 中〜高（ファミリD の具体化） |
| 7 | **評価指標（CD/F-score）と被覆性・接地姿勢分布の解析** | **高**（A2前提の文献的裏付け） |
| 8 | GTメッシュ付きバラ積みデータセット・CADパーツ集・物理シミュ生成の網羅 | 高（M2 の基盤確定） |

---

## 8. 新規性主張への含意とリスク

- **強み**: 「被覆性（多数の安定接地姿勢の和集合が全周囲を覆うか）」を扱った研究が本バッチで見つからない → **A2 の定量解析自体が新規貢献になりうる**（[evaluation-harness.md](evaluation-harness.md) §6, [research-overview.md](research-overview.md) §5.2 の方針を支持）。
- **リスク1**: 同じ A2 の**文献的裏付けが無い**＝前提を自前で立証する責任が重い。被覆性解析を貢献として正面から扱う方針が妥当。
- **リスク2**: FurnSet（2026-04）・Lookalike3D（2026-03）は**新しいプレプリント**。新規性が近接2件に依存するため、査読・改訂・後続研究を継続監視（§7 と合わせ M1.5 で再確認）。
- **リスク3**: ファミリ B/C のカテゴリレベル手法を「未知カテゴリ・参照無し」で使うのは非自明。比較時に前提条件を厳密に揃えないと不公平になる。

---

## 9. 主要出典

| 文献 | arXiv / URL | 系統 |
|------|-------------|------|
| FurnSet (2026) | arXiv:2604.20093 | 最近接（繰り返しインスタンス joint recon, RGB/家具） |
| Lookalike3D (2026) | arXiv:2603.24713 | 同一/類似インスタンス検出（画像ペア分類） |
| Splat-and-Replace (SIGGRAPH2025) | arXiv:2506.06462 | 繰り返し3DGS登録で被覆補完 |
| TEASER++ (T-RO2020) | arXiv:2001.07715 | 古典ペアワイズ堅牢整合 |
| KISS-Matcher (2024) | arXiv:2409.15615 | 学習不要ペアワイズ（FPFH系） |
| SGHR (CVPR2023) | arXiv:2304.00467 | 参照無し多視点 pose-graph+IRLS |
| ODIN (CVPR2024) | arXiv:2404.00429 | 学習拡散ペアワイズ+global最適化 |
| correspondence-free joint opt (IROS2025) | arXiv:2506.18922 | 深度guided joint最適化 |
| DeepSDF (CVPR2019) | arXiv:1901.05103 | 陰関数形状表現・補完 |
| MV-DeepSDF (ICCV2023) | arXiv:2309.16715 | 複数部分観測→set-level latent→SDF |
| PVD (ICCV2021) | arXiv:2104.03670 | 拡散による生成・補完統一 |
| ConDor (CVPR2022) | arXiv:2201.07788 | 自己教師正準化（向き+位置） |
| SE(3)-equiv self-sup (NeurIPS2021) | arXiv:2111.00190 | 注釈無しカテゴリレベル正準系 |

---

## 10. M1.5 補足調査（対称性処理・接地姿勢分布）

> deep-research M1.5 はセッション上限で synthesis 失敗。**12件は3票検証済（confirmed）、残りは abstain（=未検証、棄却ではない。上限到達のため）**。以下は confirmed のみ高信頼で記録。未検証分は再開後に補完。

### 10.1 対称性処理（§7 の高優先ギャップ → クローズ）
FA部品の対称性で stage③ の正準系一意性が破れる問題への具体手法が揃った。共通の核は「対称物体は同一の見えに複数姿勢 → **1対1対応が破綻** → 多対多/対称群で扱う」：

| 手法 | 出典 | 要点 |
|------|------|------|
| **PS6D** | arXiv:2405.11257 | **点群・対称性考慮・bin-picking 向け**。対称回転集合 S から GT 距離最小の回転を選ぶ。実機 Fanuc で 91.7%。本研究に最も近い実用系 |
| GCPose | ICCV2023（Zhao） | 多対多対応を教師する symmetry-aware matching loss で対応曖昧性を除去 |
| SymCode | arXiv:2405.10557 | 1対多の対称認識サーフェスエンコーディングで一意対応の曖昧性を解消 |
| ES6D / A(M)GPD | CVPR2022 | 対称不変な姿勢距離（ADD-S 改良；損失地形の極小が全て正解に対応） |
| BOP-Distrib | arXiv:2408.17297 | **遮蔽下では非対称物体も多義** → 曖昧性は per-image で定義すべき。バラ積み（遮蔽多）に直結 |
| 反射対称面検出 | Springer s00371-024-03313-6 | 外れ値・欠損にロバストな対称面推定（部分深度からの正準系曖昧性に有用） |

→ baseline F2（対称縮退, [baseline-classical-geometry.md](baseline-classical-geometry.md) §7）と L2 姿勢評価（[evaluation-harness.md](evaluation-harness.md) §5/§7）の具体策。**ADD-S より A(M)GPD 系**が損失/評価に適すると判明。

### 10.2 接地姿勢分布＝A2 の古典的基礎（重要）
- **Goldberg et al. "Part Pose Statistics"**（goldberg.berkeley.edu/pubs/eps.pdf, confirmed 2-0）: 多面体剛体は凸包の**1面で静止** → 最終姿勢は面上の**離散確率ベクトル** $p_1,...,p_n$ で記述。**A2 の接地姿勢分布には part-feeding/orienting 分野の古典理論が存在**する。
- **含意（新規性の精緻化）**: 接地姿勢分布**そのもの**は既知（Goldberg を引用可）。本研究の新規性は「その分布を**全周囲復元の被覆性**として使う」点に絞る。「被覆解析に先行なし」とは言い切らず、**Goldberg を基礎として引用しつつ復元被覆の角度で差別化**する。

### 10.3 未検証だが関連（abstain, 再開後に確認）
- **能動マニピュレーションによる全周囲取得**（双腕で持ち替え／ロボットが物体を回す＋深度融合）: 本研究の**受動・多インスタンス**方式と対比される別パラダイム（関連研究として明確に区別する論点）。
- 評価指標慣行（Chamfer-L1=ONet, F-score@τ 既定 1%, Tanks&Temples の F-score, Mesh R-CNN は squared-L2）: いずれも教科書的事実で [evaluation-harness.md](evaluation-harness.md) の設計と整合（abstain は上限到達のため、内容は標準）。
- 古典スキャン統合（Curless-Levoy volumetric range / screened Poisson）: ソースは取得済み、主張検証は未完。
