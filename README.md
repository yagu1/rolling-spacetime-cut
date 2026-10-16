# Cooperative Restriction Planning for Multi-Robot Navigation

**Research prototype — 2D+T, October 2026. Not safety-certified.**

**Goal:** Study how **Task Allocation (TA)** and independent **Path Planning (PP)** can be coordinated by an intermediate **Restriction Planning (RP)** layer. The supervisor plans **which regions robots may use, and when**; each rover determines its own route using its global map, Goal, local sensing and D* Lite / time-expanded A*. The restriction plan is transmitted, **not a centrally computed trajectory**.

> **Safety note:** Current contingency simulations use a shared idealized ground-truth collision-stop shield. Safe execution of observed trajectories does **not** establish safety for arbitrary autonomous motion allowed within a published restriction. See the [research framework](docs/research/TA_RP_PP_RESEARCH_FRAMEWORK.md) and [preliminary report](docs/research/PRELIMINARY_TECHNICAL_REPORT_20261016.md).

## Start here

| Document | Purpose |
|---|---|
| [TA–RP–PP research framework](docs/research/TA_RP_PP_RESEARCH_FRAMEWORK.md) | Research question, architecture, RP constraints, what is implemented vs proposed |
| [Preliminary technical report (2026-10-16)](docs/research/PRELIMINARY_TECHNICAL_REPORT_20261016.md) | Motivation, methods, quantitative results, limitations, next experiments |
| [Experiment catalog](docs/research/EXPERIMENT_CATALOG.md) | All eight development stages with scripts, results, and milestone commits |
| [Repository audit](docs/audits/REPOSITORY_AUDIT_20261016.md) | File/history audit and original missing artifact findings, plus restoration note |
| [Reproducibility check](docs/audits/REPRODUCIBILITY_CHECK_20261016.md) | Test environment, exact commands and verified status |
| [Historical README (unaltered)](docs/archive/README_BEFORE_REORGANIZATION_20261016.md) | Full original long-form README preserved for backward reference |
| [CHANGELOG](CHANGELOG.md) | Chronological commit milestones |

## Demonstrated vs proposed

**Already demonstrated in a deterministic toy simulator:**

- Two-rover D* Lite + XYT min-cut experiments; time-window and Start/Goal reachability checks.
- Four-rover single-bridge multi-label space-time cut experiments, physical margins, forward/backward reachability and connectivity feedback (exploratory, without general completeness guarantees).
- A separate **60 s lookahead / 10 s update** supervisory resource restriction planner with per-rover local D* Lite + time-dependent A*. No path dispatch in restriction mode.
- Four-rover contingency scenarios: sudden bank-side obstruction (autonomous detour) and temporary bridge closure (stop, wait, resume). Ten Monte Carlo candidate-search seeds, fixed map and events.

**Not yet implemented or established:**

- TA optimization / S–G reassignment based on RP delays.
- Local geometric Graph Cut inside the current supervisor (the single-bridge restriction approach intentionally uses zero graph cuts).
- MCTS / UCT (the current supervisor uses Monte Carlo **Random Shooting**).
- Safe arbitrary rover replanning within a restriction, robust async messages, communication delay/loss handling, physical robot validation.

## Quick start

Python 3.10+; dependency versions and test environment are recorded in [the reproducibility notes](docs/audits/REPRODUCIBILITY_CHECK_20261016.md).

```bash
python -m pip install -r requirements.txt
python -m pytest -q tests

# Latest hierarchical supervisor (no central path dispatch)
python experiments/hierarchical_restriction.py --lookahead 60 --update 10 --budget 32
python experiments/render_hierarchical_restriction.py

# Dynamic obstacle / safe holding / resumption comparisons
python experiments/contingency_restriction.py --budget 16 --seed 17
python experiments/run_contingency_batch.py --seeds 10 --budget 16
python experiments/render_contingency_restriction.py
```

## Current preliminary observations (fixed four-rover map)

Figures are from the archived deterministic simulator, not general benchmarks. Mean final arrival time in seconds over ten candidate-selection random seeds (± sample SD, among successful trials):

| Condition | RP: restrictions + local replanning | Fixed route dispatch every 10 s |
|---|---:|---:|
| No obstacle | 35.1 ± 0.3 | 35.1 ± 0.3 |
| Temporary approach obstruction | **38.2 ± 9.8** | 62.4 ± 5.0 |
| Temporary single-lane bridge closure | **55.1 ± 0.3** | 65.1 ± 0.3 |

Both compared approaches completed all ten trials in each scenario. The deliberately simplified uncoordinated baseline completed none within the 90s limit; all methods share an idealized simulation safety shield. Baseline fairness and limits are discussed in [the experiment specification](experiments/README_CONTINGENCY_RESTRICTION.md).

- [Approach detour visualization](output/contingency_restriction/approach_detour_restriction_routes.png)
- [Bridge restriction lease timing](output/contingency_restriction/bridge_restriction_leases.png)
- [Pilot seed comparison](output/contingency_restriction/pilot_seed_sweep.png)

## Repository layout

```text
rolling_cut.py, render.py                  Initial two-rover prototype
experiments/                               Successive planning and contingency experiments
tests/                                     Regression tests
output/                                    Archived example outputs (some older artifacts restored)
docs/figures/                              Archived visualizations
docs/research/                             RP framework, report, experiment catalog
docs/audits/                               Repository audit, test logs, reproducibility notes
docs/archive/                              Previous README, retained verbatim
.github/workflows/                         Regression testing on GitHub (after push)
```

This is exploratory academic software. No real robot navigation, collision avoidance, or operational safety claim is made. If used for follow-on experiments, fix scenario, random seeds, software version, and commit hash in the manuscript.
