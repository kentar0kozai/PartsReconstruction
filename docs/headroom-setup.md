# Headroom セットアップ（入力トークン圧縮レイヤー）

> Claude Code のコスト主因＝**入力トークン**（反復的な JSON / ログ / ファイル / RAG データ）を、LLM に届く前に圧縮する OSS（Tejas Chopra, Netflix）。本プロジェクトの uv venv に導入済み。
>
> - 導入物: `headroom-ai 0.27.0`（`.venv\Scripts\headroom.exe`、Python 3.11）
> - 一次情報: [GitHub](https://github.com/chopratejas/headroom) / [docs](https://headroom-docs.vercel.app/docs)
> - テレメトリ: **v0.27.0 は既定 OFF**（`headroom proxy --help` で確認。記事の「既定ON」は旧版情報）

---

## 使い方（2通り）

> ⚠️ **結論先出し（2026-06-24 追記・実体験反映）**: **VSCode 拡張では headroom は実用的でない**。拡張は `claude` を内部で直接起動するため `headroom wrap` が効かず、唯一の経路 `ANTHROPIC_BASE_URL`→proxy は「proxy を常時起動し続ける」ことが必須。proxy 未起動だと Claude Code が API に接続できず**無限ループ／ハング**する（実際に発生し、設定を撤去して復旧）。**headroom が素直に効くのは CLI のみ**（下記 A）。拡張のルーティング（下記 B）は**非推奨**。拡張でのコスト削減は「重い deep-research Workflow を避ける」運用で行う。

### A. CLI で手軽に試す（安全・永続化なし）【拡張ではなく CLI 向け】
```
.venv\Scripts\activate
headroom wrap claude     # proxy起動＋ANTHROPIC_BASE_URL設定＋Claude Code CLI起動を一括（この場限り）
```
- `wrap` は proxy 起動・環境設定・CLI 起動を自動で行い、**終了すれば元に戻る**（設定を汚さない・壊れない）。
- オプション例: `headroom wrap claude --budget 5 --budget-period daily`（1日$5上限）。

### B. 永続ルーティング（⚠️ 非推奨・proxy 常時起動が必須）
```
.venv\Scripts\activate
headroom init claude     # Claude Code の settings env に ANTHROPIC_BASE_URL を書き込み durable 連携
headroom proxy           # 使用中は別ターミナルで proxy を起動し続ける
```
- ⚠️ **proxy が起動していないと Claude Code が API に到達できず動かなくなる**（VSCode 拡張も settings env を読むため同様）。常時 proxy 運用が前提。
- 解除: `headroom unwrap` または settings から `ANTHROPIC_BASE_URL` を削除。

### 状態確認・効果測定
```
headroom doctor          # proxy 稼働・ルーティング状態（現状: proxy未起動・未ルーティング）
headroom output-savings  # 出力トークン削減の見積/実測
headroom agent-savings   # Claude/Codex/Cursor のトークン節約レポート
```

---

## 役立つオプション（proxy / wrap 共通）
| オプション | 効果 |
|-----------|------|
| `--budget <USD> --budget-period {hourly\|daily\|monthly}` | 予算上限。超過で 429。**直接的コスト制御** |
| `--mode token`（既定） / `--mode cache` | token=最大圧縮 / cache=prefixキャッシュ命中率優先 |
| `--code-aware` | AST ベースのコード圧縮（`[all]` 導入済で利用可） |
| `--memory` / `--learn` | 永続メモリ・traffic 学習（MEMORY.md 等へ）。任意 |
| `--no-telemetry` | 既定 OFF だが明示する場合 |

---

## 期待値（正直に）
- **効果が大きい**: 反復データが多いターン（API/DB の JSON、ビルド/エラーログ、ファイルツリー、調査結果 JSON）。公称 90%超の例あり。
- **効果が小さい**: 短い会話・コードのみ・設計書↔コード生成中心（実運用の削減中央値は数%）。
- 本プロジェクトでは「大きな調査 JSON・ファイル/コマンド出力の読み戻し」で効く場面がある一方、ドキュメント執筆中心のターンでは限定的。

---

## 注意
- `headroom` は `.venv\Scripts` にあるため、**venv を有効化**するか full path で呼ぶ。
- subscription（Claude Code 定額）利用でも proxy は `--backend anthropic`（既定）で対応。`--no-subscription-tracking` で使用量ポーリングを無効化可。
- 圧縮処理自体はローカル完結（外部送信なし）。圧縮後データは従来通り LLM プロバイダへ送信。
