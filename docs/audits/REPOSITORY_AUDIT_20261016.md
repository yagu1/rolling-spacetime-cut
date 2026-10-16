> **監査対象の履歴スナップショット:** 以下の監査はGitHub `main=5b41a870` を基準として実施したものです。監査時点では4つの旧生成結果が未登録でした。整理コミットでは4点とも過去配布ZIPから**バイト単位で復元・追加**します。したがって整理コミット反映後は、旧生成結果に関する未登録の指摘は解消します。既存ソースコードは変更しません。

# `yagu1/rolling-spacetime-cut` リポジトリ監査報告

監査日: 2026-10-16  
対象: https://github.com/yagu1/rolling-spacetime-cut (`main`)  
確認HEAD: `5b41a870084576685e2c665db87b0362133e58f6`  
確認方法: GitHub APIで公開リポジトリの最新コミット・全ファイルツリー・コミット履歴を取得し、過去に生成されたGit BundleとZIPをローカルGitリポジトリへ復元して比較。Pythonテストを全件実行。

## 結論

- **全8段階のコミットと実装ソースは `main` に存在する。** 最新コミットは2026-10-16付の `5b41a870`。
- **最新配布ZIPの107ファイルすべてがGitHub HEADの追跡ファイルとバイト単位で一致**。Git BundleのHEADともコミットSHAが完全一致した。
- **全61テスト成功、0件失敗** (`python -m pytest -vv tests`, Python 3.13.5, 40.82秒)。
- **過去に配布したZIPにはあるがGit管理されていない生成ファイルが計4点**。ソースおよびテストの欠落はない。
- 安全性・TA連動・局所Graph Cutに関する将来の研究計画は、すべてが実装済みではない。この点をファイル欠落と区別すべき。

## 1. 開発履歴と実装対応表

| コミット | 機能 | 主なコードとドキュメント | 状態 |
|---|---|---|---|
| `d2f6bf8` | 初期2台のD* Lite + Graph Cut | `rolling_cut.py`, `render.py`, `tests/test_rolling_cut.py` | 反映済み |
| `16f166c` | 最短経路長×2の時間展開・優先指定なし比較 | `experiments/step_scaled_horizon.py`, `tests/test_step_scaled_horizon.py` | 反映済み |
| `077ae45` | Start/Goalシードと有向連結検証、一本橋、境界の動画 | `experiments/connected_cut_bridge.py`, `docs/figures/bridge_*.png/gif` | 反映済み |
| `44c693e` | 4台一本橋、4ラベル近似Cut | `experiments/four_agent_bridge.py`, `docs/figures/four_bridge/` | 反映済み |
| `a0ace83` | State Potential、機体半径・安全マージン | `experiments/state_margin_bridge.py`, `tests/test_state_margin.py` | ソース反映済み |
| `d5d280b` | Forward/Backward Reachability、Margin Feedback | `experiments/reachability_margin_cut.py`, `docs/figures/reachability_margin/` | 反映済み |
| `148b250` | Supervisor/Rover分離、Restrictionのみ配信、60秒予測10秒更新 | `experiments/hierarchical_restriction.py`, `experiments/compare_budget_restriction.py` | 反映済み |
| `5b41a87` | 局所障害物、迂回、橋の一時閉塞、停止・再開、3方式比較 | `experiments/contingency_restriction.py`, `experiments/run_contingency_batch.py` | 反映済み |

## 2. 過去のZIPと最新版の直接照合

| ZIP | ファイル数 | HEADと同じファイル数 | HEADにないファイル数 | 経緯 |
|---|---:|---:|---:|---|
| 初期版 | 12 | 9 | 0 | README等3ファイルはその後更新 |
| Step-scaled | 18 | 15 | 0 | README、requirements、gitignoreが更新 |
| Connected Bridge | 35 | 32 | 1 | 出力JSON未追跡 |
| Four Agent Bridge | 49 | 46 | 1 | 同じ出力JSON未追跡 |
| State/Margin | 56 | 50 | 4 | JSONと画像3点等未追跡（connectedのJSONを含む） |
| Reachability/Margin | 65 | 64 | 0 | READMEのみ更新 |
| Supervisor v0.3 | 79 | 78 | 0 | READMEのみ更新 |
| Contingency v0.4 | 107 | **107** | **0** | **GitHub HEADと全ファイル一致** |

### GitHubに存在しない過去の生成成果物（重複除去4件）

1. `output/connected_cut/report.json` — 生成コマンド: `python experiments/connected_cut_bridge.py`。`output/connected_cut/` は `.gitignore` 対象。
2. `output/state_margin/report.json` — 生成コマンド: `python experiments/state_margin_bridge.py`。
3. `output/state_margin/cut_comparison.png` — 生成コマンド: `python experiments/render_state_margin.py`（上記reportの生成後）。
4. `output/state_margin/margin_comparison.png` — 同上。

ソース・テストはGitHubにある。必要なら再生成してGitで追跡するか、生成出力は再生成可能な成果物としてリリース配布に整理する。監査ではGitHubを変更していない。

## 3. 再現テスト

- `python -m pytest --collect-only -q tests` : 61テスト検出。
- `python -m pytest -vv tests` : **61 passed in 40.82s**。
- 実行環境 : Python 3.13.5。`networkx`, `numpy`, `Pillow`, `matplotlib`, `pytest` を利用。
- README内に記述されたMarkdownの相対リンク : 欠落0件。
- ローカル作業ツリー : テスト実行後もGit追跡ファイルの未コミット変更なし。
- `.github/workflows/`にCIの定義なし（自動実行・バージョン行列による検証は未整備）。

## 4. 現在の研究要求と実装の差分（未実装であり、Git欠落ではない）

| 項目 | 現状 | 研究として残る課題 |
|---|---|---|
| TA (S/G Allocation) | 4台のS/G組合せは固定 | 複数割当候補の最適化とRP遅延をTAへ戻すフィードバック |
| RP（Constraint生成） | 一本橋の通行可能時間をランダム候補探索で割当 | 一般の交差・合流・広い領域の動的時空間Segmentation |
| Graph Cut | 二値・4ラベルの旧実験は保存 | 最新Supervisorへの局所Graph Cut統合は未実装（一本橋ではCut呼出0） |
| 探索方式 | Monte Carlo Random Shooting | MCTS/UCTではない。効率比較とアクション候補の体系化が必要 |
| PP | Rover側にD* Liteと時間依存A*、短距離障害物検出 | 現実の車体運動・制御・不確実性への対応 |
| Safety | 同期軌道の距離検証。最新シミュレータには真値を知る理想的な衝突停止シールド | Restriction内の独立した任意の経路変更に対する保証はない |
| System | 60秒先読み、10秒Restriction更新、1秒ローカル再計画 | 非同期通信、受領確認、版管理の安全な切替、通信断時の安全状態 |
| 評価 | 4台の一本橋＋側道、3シナリオ×3方式×10乱数Seed | 不規則地形、他台数、障害物分布、多数ベースライン |
| 3D+T / BST-ID | 今回のリポジトリには統合せず | 将来の任意拡張 |

### 最新実験のレポート値チェック

`output/contingency_restriction/seed_sweep_summary.csv` と整合する値：

- 通常時: RP 35.1 ± 0.3 s / 10秒固定経路配信 35.1 ± 0.3 s
- 橋手前の局所障害物: RP 38.2 ± 9.8 s / 固定経路配信 62.4 ± 5.0 s
- 橋中央の一時閉塞: RP 55.1 ± 0.3 s / 固定経路配信 65.1 ± 0.3 s
- 乱数が変化するのは候補選択。地図・障害物位置とタイミングは固定。
- 3方式比較は全方式に理想化された共通の衝突停止判定が適用されている。以前の無制約実験の「最小距離0m」と、最新の理想シールド付き無協調モデルの「衝突による停止」は、異なる条件であり直接比較しない。

## 5. ドキュメント・保守上の改善点

1. ルート `README.md` の冒頭に最新v0.3/v0.4は書かれているが、旧版中心の説明が混在し、「新規GitHubリポジトリを作成して公開」の古い手順も残る。全実験の目次を作成し、最新の目的（TA–RP–PP）を最初に置く。
2. 論文的な速報と、TA–RP–PPとして再定義した正式なProblem FormulationはGitHubのバージョン管理ファイルでは未確認。研究用の `docs/RESEARCH_NOTE.md` 等に移すとよい。
3. `.github/workflows/tests.yml`を追加し、全61テストをPull Request/Push時に実行する。将来的にはPython 3.11と3.13での実行を推奨。
4. 再現性確保のため `requirements.txt` の範囲指定に加えて、採択候補原稿で使用した実行環境の固定版と乱数Seed・パラメータ一覧を記録する。
5. 実験出力をすべてGitで保持するか、ソース・設定と生成済み図表を明示的に分離するかを決める。Git履歴を肥大化させる可能性も考慮する。
6. この監査はリポジトリの実装と既存テストの再現性確認であり、論文上の新規性や実機の安全性保証を認証するものではない。

## 監査者結論

**コミットとソースの持ち越し: 合格。全8段階の開発履歴はGitHub `main` に存在し、最新配布ZIPの107/107ファイルと一致する。**

**すべての過去生成成果物の保存: 一部未完。古い実験の4つの出力ファイルがGitHubにないが再生成可能。**

**ソフトウェア回帰テスト: 合格（61/61）。**

**研究計画の全機能完成: 未達。TA連動、一般化した局所Graph Cut、非同期安全保証などは意図的に今後の研究課題。**


---

## 補遺：2026-10-16 整理コミットでの是正内容

以下は監査完了後の整理作業であり、元の `5b41a870` の内容ではありません。

- 旧版READMEを `docs/archive/README_BEFORE_REORGANIZATION_20261016.md` にそのまま保存し、新READMEを研究の入口として再編。
- この監査報告、検証ログ、研究速報、TA–RP–PP問題設定、実験一覧、再現手順を `docs/` 以下に追加。
- 過去ZIPにのみ存在していた4件（`output/connected_cut/report.json`、`output/state_margin/report.json`、`output/state_margin/cut_comparison.png`、`output/state_margin/margin_comparison.png`）を、過去の配布ZIPから内容の再計算をせず復元。
- Python 3.13のpytestを起動するGitHub Actionsワークフローを追加。**ワークフローがGitHub上で成功するかはPush後に確認する必要がある。**
- 新たな実験結果や手法の完成を主張する変更は行っていない。

検証とコミットIDについては、整理版の `docs/audits/REPRODUCIBILITY_CHECK_20261016.md`、`docs/audits/pytest_full_after_docs_20261016.log`、`CHANGELOG.md` を参照。
