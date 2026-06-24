# 合成データ基盤 — 技術選定 下調べ

> [research-overview.md](research-overview.md) §6（データ戦略=合成先行）の実装設計ノート。M2「合成データ生成基盤＋共通評価ハーネス」の技術選定材料。
>
> - ステータス: 下調べ Draft v0.1
> - 最終更新: 2026-06-23
> - 注: データセット選定は実行中の文献調査（deep-research, サブ領域8）の結果と最終的に突き合わせる。

---

## 1. このデータ基盤が満たすべき要件

本研究は「同一剛体の多数インスタンスをバラ積みし、上方1ショットで 2.5D 部分点群群を得て全周囲復元」なので、合成基盤には以下が必須：

| # | 要件 | 理由 |
|---|------|------|
| R1 | 同一物体 N 個を箱へ**物理ドロップ**し安定姿勢を生成 | A2（被覆性）検証の前提。接地姿勢分布が偏ることまで再現したい |
| R2 | **トップダウン単一視点**の深度レンダリング → 2.5D 点群 | 「ワンショット」制約の忠実な再現 |
| R3 | **GT メッシュ**（正準形状） | 最終点群 Ŝ を CD / F-score で評価する基準 |
| R4 | **GT 個体姿勢** $\{T_i\}$ | stage③ レジストレーションの定量評価 |
| R5 | **GT 個体セグメンテーション** | stage① はスコープ外＝GTで代替する前提（[research-overview.md](research-overview.md) §7） |
| R6 | **センサノイズ・欠損モデル**を後段で制御付与 | 合成→実機ドメインギャップの段階的検証 |
| R7 | 難易度を段階化できる**物体ソース**（対称/凹凸/サイズ） | アブレーション軸（§5.3）を回すため |
| R8 | **再現性**（シード固定・スクリプト化） | スペック駆動開発・比較実験の公平性 |
| R9 | データセットの**ライセンス遵守**（個人研究用途） | §5 参照。研究用途データはそのまま利用可。再配布・引用作法に注意 |

---

## 2. パイプライン構成（必要コンポーネント）

```
(a) 物体メッシュソース    →  (b) 物理シミュ（箱へドロップ, 姿勢生成）
                              ↓
(d) センサノイズ付与  ←  (c) トップダウン深度レンダリング → 2.5D点群
                              ↓
(e) GT エクスポート（メッシュ / 姿勢 / 個体セグ / 正準GT点群）
                              ↓
(f) 点群処理（評価ハーネス・可視化・サンプリング）
```

注: (b)〜(e) を**1ツールで束ねられるか**が選定の最大の分かれ目。

---

## 3. コンポーネント別 候補比較

### 3.1 シーン生成＋物理＋深度（b・c・e の中核）

| ツール | 物理ドロップ | 深度レンダ | GT出力(姿勢/セグ) | 重さ・要件 | ライセンス | 評価 |
|--------|:---:|:---:|:---:|------|------|------|
| **BlenderProc** (DLR-RM) | ◎ `bop_object_physics_positioning`（箱へfree-fall） | ◎ RGB/depth/normal/seg | ◎ BOP形式で姿勢・セグ・COCO | 軽量（Blender/CPU可、GPU任意） | GPL-3.0（生成物の利用は自由） | **本命**。要件 R1-R5 をほぼ素で満たす。BOP Challenge 2020 の学習画像生成実績 |
| **NVIDIA Isaac Sim 5.0 / Isaac Lab 2.2** | ◎ GPU PhysX, 大規模並列 | ◎ RTX photorealistic, Replicator SDG | ◎ アノテーション一式 | 重い（RTX GPU + Omniverse） | NVIDIA EULA（商用条件は要確認） | **ヘビー代替**。stereo depth ノイズ内蔵・大量生成・最高品質。設定コスト大 |
| **PyBullet** | ○ 簡単・無償 | △ 簡易レンダのみ（深度は可だが質素） | △ 自前実装 | 最軽量・Python | zlib（寛容） | 物理のみ切り出すなら有用。だがBlenderProcがBullet物理を内包し冗長になりがち |
| **MuJoCo** | ○ 接触精度高 | △ レンダは限定的 | △ 自前 | 軽量 | Apache-2.0 | 接触精度は良いがソフト接触で貫通あり。レンダ・GT出力は別途必要 |

> **所見**: (b)+(c)+(e) を一括で満たすのは BlenderProc と Isaac Sim の2択。**まず BlenderProc で最小構成を立ち上げ、フォトリアル/大量生成/高精度ステレオが必要になった段階で Isaac Sim へ拡張**するのが工数対効果で最適。PyBullet/MuJoCo は「物理だけ別エンジンで精密化したい」特殊ニーズが出た時の予備。

### 3.2 物体メッシュソース（a）

| ソース | 中身 | GTメッシュ | 対称/工業部品 | ライセンス注意 |
|--------|------|:---:|:---:|------|
| **BOP-Industrial**: XYZ-IBD / ITODD / IPD | バラ積み工業部品・複数インスタンス・遮蔽・多視点 | ◎ | ◎（FAに直結） | **要確認**。ITODD(MVTec)は研究用途限定の可能性、XYZ-IBD/IPDも個別ライセンス |
| **BOP-Classic**: T-LESS / IC-BIN ほか | T-LESS=テクスチャレス対称工業部品 | ◎ | ◎（対称性アブレーションに好適） | T-LESSは比較的寛容（要確認） |
| **ABC dataset** | 大規模CAD（機械部品中心） | ◎ | ○ | 要確認 |
| **ShapeNet / 手続き生成** | 汎用形状・難易度を自作可能 | ◎ | △ | ShapeNetは研究用途 |

> **所見**: 評価の主軸は **T-LESS（対称・テクスチャレス）＋ XYZ-IBD/IPD（バラ積み工業部品）**。これらは「同一物体が複数インスタンス・バラ積み」という本研究設定に最も近く、しかも**実機データとしても評価に使える**。難易度段階化には ABC / 手続き生成を併用。**商用流用の可否はライセンス次第（§5）**。

### 3.3 センサノイズ・欠損モデル（d）

| 手法 | 模擬対象 | 実装の入手性 | 備考 |
|------|---------|------|------|
| **SimKinect** | Kinect構造化光ノイズ（軸方向・影） | 既存実装あり | 古典的・軽量。深度マップに後処理付与 |
| **DREDS系（PBRアクティブステレオ）** | アクティブステレオの現実的ノイズ | arXiv 2208.03792 等 | 鏡面・透明物体まで含む。FA金属部品の難しさに対応 |
| **BlenSor** | レーザスキャナ（Velodyne等）/ ToF | Blenderアドオン | 距離バイアス＋per-rayガウシアン。スキャナ系を模すなら |
| **Isaac Sim 内蔵 depth noise** | ステレオカメラノイズ | Isaac Sim 5.0 | Isaacを使うなら統合済みで便利 |

> **所見**: フェーズ1は「クリーン深度 → ガウシアン/欠損の段階付与」で十分（ノイズ強度をアブレーション軸に）。FA金属部品の鏡面反射が効くと分かれば DREDS系へ。実センサに寄せるなら SimKinect（構造化光）か BlenSor（ToF/レーザ）を計測機に合わせて選ぶ。

### 3.4 点群・形状処理（f）

| ライブラリ | 用途 | 評価 |
|-----------|------|------|
| **Open3D** | 入出力・レジストレーション(FPFH/RANSAC/ICP)・表面再構成(Poisson)・可視化 | 古典幾何ベースライン（ファミリA）の中核。必須 |
| **trimesh** | メッシュ操作・**正準GT点群のサンプリング**（R3）・幾何計算 | GT点群生成・指標計算に必須 |
| **PyTorch3D** | 微分可能レンダ・CD/F-score・点群演算（GPU） | 学習系ファミリ(C/D)と指標計算で有用 |
| **NumPy/SciPy/scikit-learn** | 一般数値・最近傍・clustering | 補助 |

---

## 4. 推奨スタック

> **更新（2026-06-23, 検証反映）**: BlenderProc が箱内バラ積みを「設定すれば」再現できることを一次資料で確認（容器寸法・同一物体N複製・単一トップダウンの3点を設定変更する必要）。一方、本研究は**深度/点群が主・RGB は当面不要**で、Open3D `RaycastingScene` がローカル利用可。よって**幾何コアは PyBullet＋Open3D が軽量で反復に最適**との結論に精緻化。詳細設計・生成器アーキテクチャの比較は **[data-generation-design.md](data-generation-design.md)** に分離。本節はツール候補の一覧として残す。

**プライマリ（幾何コア; [data-generation-design.md](data-generation-design.md) §4-5）**
- 物体: 段1=**非対称・特徴豊富**な手続き生成/実部品 → ラダーで対称・平滑へ（同 §6）。実機評価兼用に **T-LESS / XYZ-IBD / IPD**
- 生成: **PyBullet（物理山積み）＋ Open3D RaycastingScene（単一トップダウン深度・相互遮蔽・個体セグGT）**
- ノイズ: クリーン→**ガウシアン＋欠損**を強度パラメータ化（アブレーション軸）
- GT点群: **trimesh / Open3D** でメッシュから一様サンプリング（評価基準 Ŝ_gt）
- 処理・評価: **Open3D ＋ trimesh ＋（学習系は）PyTorch3D**

**フォトリアル / 実部品 / sim-to-real 代替（後段）**
- **BlenderProc `bop_object_physics_positioning`**: RGB＋深度＋BOPセグ＋BOP実部品直結。要設定（§3）。
- **Isaac Sim 5.0 + Replicator**: 大量GPU生成・高精度ステレオ depth ノイズが要るとき。

**理由（要点）**
1. PyBullet＋Open3D は箱内物理山積み＋ワンショット単一視点深度（相互遮蔽が自動的に正しく入る）＋GT一式を**最小依存・全Python**で実現し、研究の核（参照無し groupwise registration）の反復が速い。
2. BOP-Industrial/Classic を物体ソースにすると、**同じデータが実機評価（フェーズ2）にも転用**でき、合成→実機の橋渡しが自然。
3. 段階運用（PyBullet+Open3D → BlenderProc/Isaac Sim）で初期工数を抑えつつ天井を確保。

---

## 5. ライセンス注意（個人研究用途）

**本プロジェクトは個人研究用途**のため、研究用途限定ライセンスのデータセット（ITODD/MVTec, ShapeNet 等）も**そのまま利用可**。商用流用の切り分けは不要。

- 守るべきは一般的な作法のみ: **再配布の制限**（データそのものを再公開しない）と**論文での出典明記・引用**。
- ツール側（BlenderProc=GPL, PyBullet=zlib, MuJoCo=Apache, Open3D=MIT, trimesh=MIT）は研究利用に支障なし。
- 結論: **データセット選定はライセンスより「本研究設定への適合度」で決める**（§3.2 の T-LESS / XYZ-IBD / IPD を主軸）。

---

## 6. リスク・確認事項

| 種別 | 内容 | 対応 |
|------|------|------|
| 確認 | BlenderProc 物理ドロップの**接触安定性**（薄板・複雑形状の貫通） | PoC で実形状を投下して目視確認。必要なら凸分解・ソルバ設定調整 |
| 確認 | トップダウン深度の**自己遮蔽・グレージング欠損**が実機相当か | 欠損モデルで明示制御し実機と突き合わせ |
| 確認 | GT セグ・姿勢の**座標系定義**（正準系の取り方が手法評価に影響） | 評価ハーネスで正準系規約を固定 |
| — | データセットのライセンス（§5） | 個人研究用途のため懸念なし（再配布・引用作法のみ留意） |
| リスク | 文献調査結果と**データ選定が食い違う** | deep-research 完了後に §3.2 を更新 |

---

## 7. 次アクション（PoC 提案）

最小の動作確認（"vertical slice"）を1本通すのが最短：

1. T-LESS の1物体を BlenderProc `bop_object_physics_positioning` で箱に N=20 個ドロップ。
2. トップダウン深度を1枚レンダ → 2.5D 点群化（Open3D）。
3. GT 姿勢・GT 個体セグ・GT 正準点群（trimesh サンプリング）を書き出し。
4. 各インスタンスの GT 姿勢で部分点群を正準系へ整列・融合 → **「上界（oracle）」の全周囲点群**を作り、CD/F-score/Coverage を測定。

→ これは (i) 基盤が要件 R1–R5 を満たすか、(ii) **A2 被覆性の上界**（GT姿勢を使えば全周囲が埋まるか）を同時に検証でき、評価ハーネスの雛形にもなる。

---

## 参考（一次情報）

- BlenderProc4BOP / 物理ポジショニング例: [docs](https://dlr-rm.github.io/BlenderProc/examples/datasets/bop_object_physics_positioning/README.html), [BlenderProc4BOP README](https://dlr-rm.github.io/BlenderProc/README_BlenderProc4BOP.html)
- Isaac Sim 5.0 / Isaac Lab 2.2（GA, SIGGRAPH 2025・depth noise・SDG）: [NVIDIA blog](https://developer.nvidia.com/blog/isaac-sim-and-isaac-lab-are-now-available-for-early-developer-preview/), [Advanced Sensor Physics](https://developer.nvidia.com/blog/advanced-sensor-physics-customization-and-model-benchmarking-coming-to-nvidia-isaac-sim-and-nvidia-isaac-lab/), [Isaac Sim SDG docs](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/synthetic_data_generation/index.html)
- BOP ベンチマーク / BOP-Industrial（XYZ-IBD, ITODD, IPD）: [BOP challenges](https://bop.felk.cvut.cz/challenges/), [XYZ-IBD](https://github.com/demianhj/XYZ-IBD)
- センサノイズ: BlenSor [ResearchGate](https://www.researchgate.net/publication/220844770_BlenSor_Blender_Sensor_Simulation_Toolbox), DREDS（PBRアクティブステレオ）[arXiv 2208.03792](https://arxiv.org/pdf/2208.03792)
- シミュレータ比較: [MuJoCo vs Isaac Sim vs PyBullet](https://robotwale.com/article/mujoco-vs-isaac-sim-vs-pybullet-a-practical-comparison)
