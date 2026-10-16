# TA–RP–PP による協調移動の研究フレームワーク

**状態:** 研究構想と予備実装の対応付け（2026-10-16）。ここに書く最適化問題・安全条件には**未実装の提案要素**を含む。

## 1. Research Question

> How can a supervisory system automatically plan minimally restrictive, safe, time-dependent movement permissions by considering both task allocation and decentralized path-planning feasibility, without dispatching complete robot trajectories?

日本語：**タスク割当（TA）と各ロボットの経路計画（PP）の実行可能性を同時に考慮しながら、安全性・待機可能性・ローカル再計画自由度を確保する時空間Restriction（RP）を、どう生成・更新するか。**

Traffic light / UTM は設計上の比喩であり、全UTMサービスが単なるRestriction管理から構成されるという主張ではない。

## 2. 三層の責務（経路は上位系から送らない）

| 層 | 入力 | 主な決定 | 出力 | このリポジトリの状態 |
|---|---|---|---|---|
| **Task Allocation (TA)** | タスク候補、S/G群、進捗、RP予測遅延 | 誰がどのStart–Goalに対応するか | S/G割当・タスク制約 | 現在は固定の4組。最適化・再割当は未実装 |
| **Restriction Planning (RP)** | タスク、地図、各機位置・状態、到達性、サイズ、安全余裕 | 誰が・どこを・いつ利用できるか | バージョン付き・時刻付きRestriction Map | 共有一本橋に対するMonte Carlo Random Shooting（Graph Cut呼出なし） |
| **Local Path Planning (PP)** | 自己位置、Goal、Global Map、Restriction、局所障害物 | 許可範囲内での経路選択・待機・再開 | ローバー自身の走行行動とテレメトリ | D* Lite cost-to-go + 時間展開A*。動的障害物時に再計画 |

上位RPが内部で予測パスを評価することと、そのパスを各機へ配信することは**異なる**。現在のrestriction-only方式ではパスを配信しない。

関連する既存のS/G割当着想：日本特許出願 [JP2022075273A](https://patents.google.com/patent/JP2022075273A/ja)。この特許を本リポジトリへ移植・再実装したわけではない。

## 3. Proposed Restriction Contract

世界座標を Ω、機体 \(i\) の現在状態を \(x_i(t)\)、Goalを \(G_i\) とする。

- 上位系が配信するのは時刻依存の許可領域 \(R_i(t)\subseteq\Omega\) と、その**有効範囲・バージョン・解放条件**。
- PPは \(R_i(t)\) を出てはならないが、その内部の経路はローカルに決める。
- サイズ・安全マージン・障害物を踏まえた安全条件は、実際の連続遷移に対して確認すべき。単にセル中心のラベルが異なるだけでは不十分。
- 局所障害物が出現しても、許可領域内で迂回できれば**通常運用内で局所再計画**。迂回できなければ**安全な待機**を優先。
- 待機状態が維持できない、通信が途絶えた、または進捗が失われた場合には、別途定義する安全停止・Contingencyへ移行する（**現実の通信断への保証は未実装**）。
- 長時間の待機や著しい遅延が生じた場合、RPからTAへ予測遅延を返してS/G再割当を検討する（未実装）。

## 4. 問題の概念的定式化（未実装の研究目標）

TAの割当 \(A\)、RPの時空間許可領域 \(R\)、各機の実行可能なローカル移動集合 \(\Pi_i(A,R_i)\) に対して、例えば

```text
min_{A,R}  C_task(A) + C_delay(A,R) + λ C_restriction(R)
                 + μ C_boundary_update(R) + ν C_wait(A,R)
subject to:
  - each rover can safely move or hold within its published restrictions;
  - all admitted synchronous movements obey body size and separation margins;
  - prior active grants cannot be revoked in an unsafe way;
  - progress/terminal feasibility is maintained or a declared contingency occurs.
```

この目的関数は研究上の**概念的な仕様**であり、現行コードがこの同時最適化を厳密に実現しているわけではない。現行TAは固定、RPは局所資源ゲートの候補評価、PPは各機側の再計画である。

## 5. 解法の方向

1. **競合の局所抽出:** 各ローバーの位置から約60秒先の到達可能領域を推定し、競合する共有資源や空間Region of Interest (ROI) を抽出。現行一本橋では共有資源を手指定。
2. **Soft Potential:** Goalへの進捗、斥力場、待機時間、境界変更、運用公平性などで候補Restrictionを評価。Hard safety constraintと混同しない。
3. **Resource Scheduling / Local Graph Cut:** 単一橋は時間スロットの権限割当。二経路、合流、交差点など境界が幾何的に曖昧なROIでは局所Graph Cutを検討（**最新Supervisorには未統合**）。
4. **Cooperative Candidate Search:** 現在はMonte Carlo Random Shooting。MCTS/UCTは比較対象候補であり、実装済みではない。
5. **Rolling Update:** 60秒先を評価して10秒ごとに更新。既に有効なGrantは途中で取り消さない。これを実機に拡張するにはACK、時刻同期、通信断時の停止・権限切替条件が必要。

## 6. 将来の安全性要件：Level 1 と Level 2

- **Level 1 — Trajectory witness safety（現行予備実験）:** 予測・実行した**特定の同期経路**で安全離隔を満たす。現行最新シミュレータは真値を知る理想的な衝突停止シールドも使う。
- **Level 2 — Restriction-preserving safety（研究課題）:** ローバーが許可集合内で自律的に許される範囲の経路変更を行っても安全を維持。動的Viability、停止可能集合、遷移占有領域、または独立したランタイムSafety Shieldが必要。

**重要:** Start–Goalの連結性はLevel 2を保証しない。Graph Cutの色ラベルも「任意に動ける安全なCorridor」を意味しない。

## 7. 主要評価仮説

- H1: 制約内での独立再計画が、固定経路を10秒単位で再発行するベースラインより、一時的障害物への対応遅延を減らす（同等の局所安全機能を有する比較も要追加）。
- H2: 共有資源に限定したRPが、全領域の硬い専有分割より、多くのNavigation Freedomを残す。
- H3: 局所切出しと計算予算制限が、ロボット数や地図サイズに対する計算負荷を下げる（未一般化）。
- H4: RP予測待機時間をTAへ戻すと、全タスクの完了効率が改善する（未検証）。

関連資料：[研究速報](PRELIMINARY_TECHNICAL_REPORT_20261016.md)・[実験一覧](EXPERIMENT_CATALOG.md)・[監査](../audits/REPOSITORY_AUDIT_20261016.md)。
