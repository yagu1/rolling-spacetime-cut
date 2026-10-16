# 研究速報（予備実験報告）— 動的時空間Restriction計画による複数ロボット協調移動

**仮題:** *Task- and Navigation-Aware Spatiotemporal Restriction Planning for Cooperative Multi-Robot Systems*  
**日付:** 2026-10-16  
**Status:** Working paper / Preliminary simulation report — 査読投稿原稿ではない  
**Code:** [rolling-spacetime-cut](https://github.com/yagu1/rolling-spacetime-cut)（監査元 `5b41a870`、研究コードそのものの変更なし）

## Abstract

本研究では、複数自律移動ロボットに個別の経路を配信するのではなく、上位系が**移動可能な空間と時間の制約（Restriction）を計画して配信する**階層型ナビゲーションを検討する。上位系は配送タスクのStart/Goal割当（TA）と、各ロボット側の局所経路計画（PP）の実行可能性を考慮し、競合する局所領域に対する時間依存の利用権限（RP）を生成する。ロボットは自らグローバルマップ、自己位置、Goalを用い、D* Liteの距離情報と時間依存A*でRestriction内を走行する。4台・一本橋の予備シミュレーションでは、60秒先読み・10秒周期でRestrictionを更新し、突然の局所障害物に対する迂回、および橋の一時閉塞時の待機・復帰を確認した。10乱数Seedの固定環境パイロットでは、Restriction方式は3条件すべてで10/10成功した。これは限定されたシミュレーションの動作実証であり、安全保証・一般的な交通管理方式に対する優位性を示すものではない。

**Keywords:** Multi-Robot Systems; Restriction Planning; Dynamic Space-Time Allocation; Autonomous Navigation; Contingency Absorption; Traffic Management.

## 1. Introduction and research motivation

Multi-Agent Path Finding (MAPF) やMulti-Robot Task Allocation (MRTA) は、複数ロボットのタスクと経路の協調を扱ってきた。一方、通信する上位系がすべての経路点を指定すると、ローバー側で局所障害物を発見した際に、既存経路からの逸脱、上位系の再発行、元経路への復帰が問題となる。

本研究の問いは、**Task Allocation (TA) とPath Planning (PP) の双方に依存する「Restriction Planning (RP)」を独立の計画問題として定式化できるか**、である。交通信号や空域利用制約のように、上位系は「いつ・どの領域へ進入できるか」を決定するが、ローバーの具体的な移動経路は原則として指定しない。

研究の中心仮説は、障害物による一時的な制約を、許可領域内の局所迂回、または安全な待機によって吸収でき、経路の中央再発行や上位系Contingencyへの移行を減らせることである。既存のTraffic Management、Layered Costmap、リソース予約と関連するため、概念を独占的な新発見として扱わず、**RPの自動生成・安全性・最小制約性・TA連動**を研究課題に置く。

## 2. TA–RP–PP architecture

- **TA:** 配送タスクのS/Gペアを割り当てる。以前の関連着想として [JP2022075273A](https://patents.google.com/patent/JP2022075273A/ja) がある。ただし、その割当アルゴリズムは本試作に移植されておらず、現在のS/G割当は**固定**。
- **RP:** 上位系がGlobal Map、ロボットの状態とGoal、物理寸法、安全余裕、競合エリアの時間利用状況を把握してRestrictionを決める。**上位系から完成経路は送信しない。**
- **PP:** 各ローバーは自らD* Liteコストと時間依存A*を利用する。局所障害物を短距離センサで発見し、制約内で迂回・停止・再開する。

提案アーキテクチャでは、RPによる予測遅延をTAへ返す設計が可能だが、**RP→TA再割当は未実装**である。上位系は評価用に内部でローバーの候補軌道をシミュレーションするが、それを通信出力に含めない。

## 3. Restriction generation and local replanning

位置 \(p\)、時刻 \(t\) におけるロボット \(i\) の許可集合を \(R_i(t)\) とすると、ローカルPPは \(p_i(t)\in R_i(t)\) を制約としてGoalへの移動コストを最小化する。ただし、時空間許可集合が経路連結性を持つだけでは、複数ロボットの衝突回避は保証されない。

初期研究段階ではXYT空間全体を2値／4値Graph Cutで分割し、D* Lite Potentialを用いた。単なるCutはStart–Goalの有向接続を保証しないため、Forward/Backward Reachabilityと安全離隔フィードバックを検討した。その実験コードは残しているが、現在の一本橋Supervisorへはまだ局所Cutを統合していない。

最新の一本橋モデルでは、共有通路を時間依存資源として扱う。**Monte Carlo Random Shooting**で通行方向・開始時刻等の候補を選択し、予測到着時間、Soft Repulsion、マージン、前回Grantとの変更量から評価する。これはMCTSではない。計画範囲は60秒、更新は10秒周期である。新Restrictionは、既に有効化された通行権を安全確認なしに取消さないという原則を採る。

障害物はローカルセンサで認識されるものとし、PPは約1秒周期で再計画する。迂回が無ければ静止候補を返し、障害物除去後はGoalへ移動を再開する。

## 4. Preliminary experimental method

**Environment:** 1mの離散Grid、4台（A/B左→右、C/D右→左）、中央一本橋、両岸に待機と局所迂回のためのセル。移動は直交・待機を許可。移動区間は同期した直線移動として補間。ロボット半径0.28m、壁への余裕0.10m、必要ロボット間中心距離0.90m。上位系の先読み60秒／更新10秒、ローカル再計画約1秒。

- **Nominal:** 障害物が現れない通常運行。
- **Approach detour:** 時刻1～18秒の間、一本橋手前の `(13,2)` が閉塞。橋自体の代替はないが、その手前の側道は利用可能。
- **Bridge closure:** 時刻6～23秒の間、橋上の `(7,2)` が閉塞。迂回できず、安全に待機してから再開する必要がある。

**Comparator:** (a) 動的Restriction配信＋各機再計画、(b) 10秒ごとに経路を配信し期間中は逸脱できない制限付き固定経路方式、(c) RPを持たない独立走行。3方式ともシミュレータ真値にアクセスする理想的な共通の緊急停止シールドを使う。よってこれは本格的なCBS/RHCRや高機能MPCとの比較ではない。

10種類の乱数Seedで変えるのは**上位系の候補割当の乱数**だけであり、地図、障害物の位置と発生時刻、センサ、機体モデルは固定した。

## 5. Results

表1は [seed_sweep_summary.csv](../../output/contingency_restriction/seed_sweep_summary.csv) に記録された値。完了時間は4台全員がGoalへ到着した時刻。±は成功試行の標本標準偏差（秒）。

| Scenario | Restriction RP | Fixed path dispatch | Uncoordinated |
|---|---:|---:|---:|
| Nominal | 35.1 ± 0.3 s (10/10) | 35.1 ± 0.3 s (10/10) | 0/10 completed |
| Approach obstacle | **38.2 ± 9.8 s (10/10)** | 62.4 ± 5.0 s (10/10) | 0/10 completed |
| Bridge closure | **55.1 ± 0.3 s (10/10)** | 65.1 ± 0.3 s (10/10) | 0/10 completed |

代表的な一試行（seed=17）では、橋手前の障害物に対しC/Dは次のRP更新（t=10s）より前の **t=2s** に側道を選択した。橋の一時閉塞ではローバーが待機し、障害物の除去後に再開した。同期実行軌道の最小中心間距離は1.00mで、設定した0.90mを満たした。比較実験は小規模な決定論的環境のパイロットであり、一般的な安全性・優位性を証明しない。

[シナリオ図：局所迂回](../../output/contingency_restriction/approach_detour_restriction_routes.png) / [通行時間帯](../../output/contingency_restriction/bridge_restriction_leases.png) / [10 Seed比較](../../output/contingency_restriction/pilot_seed_sweep.png)

現行シミュレーションは簡易な障害物発生モデルを用い、障害物がロボットの真下に突然現れるような事例は除外されている。

## 6. Limitations and future evaluation

1. **Restriction領域全体の安全保証がない:** 現在検証しているのはシミュレータの理想的な共通緊急停止シールドの下で生じた軌道。許可内の任意の移動に対して安全とはいえない。
2. **一般性がない:** 共有一本橋は明示的に指定しており、交差点、合流点、二経路、複雑な地図から競合領域を自動検出できない。
3. **TA未連動:** S/Gペア固定。RPの遅延評価をS/G再割当へ戻す実験が必要。
4. **高度な比較手法が必要:** CBS/ECBS、RHCR、Resource Reservation、Layered Costmap、中央経路のイベント駆動再計画などを、共通の障害物検出・停止機能の下で比較する。
5. **実機条件への対応なし:** 通信遅延、未受領、時計ずれ、位置推定誤差、加減速、車体形状、停止距離、非同期運動を導入する。
6. **Algorithmic extension:** ローカルGraph Cutを交差路等に適用し、必要ならMCTSでRestrictionの協調行動を探索する。Graph CutやMCTSを追加すること自体は研究上の新規性ではない。

## 7. Conclusion

パスを上位系から再配信せずにRestrictionを動的に更新することによって、ローバーが局所障害物の迂回・待機・復帰を独立に選べることを、4台の小規模シミュレーションで確認した。今後はRPをTAとPPの双方に依存する独立の計画問題として定式化し、許可領域内での多様なローカル行動に対する安全性、過度なRestrictionの回避、通信・計算予算、タスク遂行効率を評価する。

## Reproducibility and evidence

- 既存コード：`experiments/hierarchical_restriction.py`、`experiments/contingency_restriction.py`、`tests/test_contingency_restriction.py`。
- 実験仕様：[README_CONTINGENCY_RESTRICTION.md](../../experiments/README_CONTINGENCY_RESTRICTION.md)、[README_HIERARCHICAL_RESTRICTION.md](../../experiments/README_HIERARCHICAL_RESTRICTION.md)。
- 監査：[REPOSITORY_AUDIT_20261016.md](../audits/REPOSITORY_AUDIT_20261016.md)。

**No publication claim:** この報告は内部共有用の研究速報であり、査読で認められた結論や実機の安全性保証を意味しない。
