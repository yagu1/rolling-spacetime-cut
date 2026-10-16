# 実験カタログ（2026-10-16）

この表は時系列の整理です。**実験として保存されたもの**と**現行Supervisorに統合されたもの**を混同しないでください。旧Graph Cut実験は削除せず継続保存しています。

| Phase / Commit | 目的 | Entry point | 実験記録・説明 |
|---|---|---|---|
| v0 initial / `d2f6bf8` | 2台・D* Lite＋XYT二値Cut・10s予測/2s実行 | [`rolling_cut.py`](../../rolling_cut.py) | [`output/demo.json`](../../output/demo.json) |
| Step×2 / `16f166c` | 最大最短経路長×2のXYT Cut、優先固定なし・Joint A* | [`step_scaled_horizon.py`](../../experiments/step_scaled_horizon.py) | [`experiments/README.md`](../../experiments/README.md) |
| Connected / `077ae45` | 2台一本橋・Seed連結・Cutの時系列変形 | [`connected_cut_bridge.py`](../../experiments/connected_cut_bridge.py) | [`README_CONNECTED_CUT.md`](../../experiments/README_CONNECTED_CUT.md)、[`report.json`](../../output/connected_cut/report.json) |
| Four / `44c693e` | 4台一本橋・4ラベル近似Cut・通過順序の対称性 | [`four_agent_bridge.py`](../../experiments/four_agent_bridge.py) | [`README_FOUR_AGENT_BRIDGE.md`](../../experiments/README_FOUR_AGENT_BRIDGE.md) |
| Potential / `a0ace83` | State-dependent Potential・ロボット半径/安全距離 | [`state_margin_bridge.py`](../../experiments/state_margin_bridge.py) | [`README_STATE_MARGIN.md`](../../experiments/README_STATE_MARGIN.md)、[`report.json`](../../output/state_margin/report.json) |
| Reachability / `d5d280b` | Forward/Backward Reachability・経路Seedなしの連結性補正・安全距離フィードバック | [`reachability_margin_cut.py`](../../experiments/reachability_margin_cut.py) | [`README_REACHABILITY_MARGIN.md`](../../experiments/README_REACHABILITY_MARGIN.md) |
| Supervisor / `148b250` | 4台・60s予測/10s更新・Restriction-only配信・各機ローカルPP | [`hierarchical_restriction.py`](../../experiments/hierarchical_restriction.py) | [`README_HIERARCHICAL_RESTRICTION.md`](../../experiments/README_HIERARCHICAL_RESTRICTION.md) |
| Contingency / `5b41a87` | 突発障害物・側道迂回・一本橋閉塞・待機/復帰・10 Seed比較 | [`contingency_restriction.py`](../../experiments/contingency_restriction.py) | [`README_CONTINGENCY_RESTRICTION.md`](../../experiments/README_CONTINGENCY_RESTRICTION.md) |

## 重要な実装上の区別

- 初期版の全XYT Cutは**接続性保証のない探索的モデル**。後のReachability＋Margin版も、最後の安全補正では経路候補オラクルに依存する。
- Supervisor版・Contingency版の**本番系想定アーキテクチャ**は、局所資源の時刻付きRestrictionを出力するもので、**Graph Cut実行回数は0**。広い交差/合流領域への局所Cut導入は研究課題。
- 最新のTAは固定Start–Goal割当であり、以前のS/G対応特許を実装したものではない。
- `output/connected_cut/report.json` と `output/state_margin/` の3成果物は、監査で未登録と判明し、**過去ZIPから復元した履歴成果物**。今回新しく実験を回して生成した結果ではない。
- 新規プログラムは追加せず、コード自体のアルゴリズムは現在のコミットまで不変。

## Run / tests

```bash
python -m pip install -r requirements.txt
python -m pytest -q tests
python experiments/hierarchical_restriction.py --lookahead 60 --update 10 --budget 32
python experiments/contingency_restriction.py --budget 16 --seed 17
python experiments/run_contingency_batch.py --seeds 10 --budget 16
python experiments/render_contingency_restriction.py
```

詳細な再現条件は各 `experiments/README_*.md` と [再現監査](../audits/REPRODUCIBILITY_CHECK_20261016.md) を参照。
