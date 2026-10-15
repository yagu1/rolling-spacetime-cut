# Four-agent bridge: state potential and robot margins

Run:

```bash
python experiments/state_margin_bridge.py
python experiments/render_state_margin.py
python -m pytest -q tests/test_state_margin.py
```

The study compares **static** vs **time-dependent start/goal distance potentials** in an *unseeded* 4-label alpha-expansion cut. The only pinned nodes are four t=0 starts and four t=H goals. There is no predetermined bridge priority.

**Observed:** both formulations leave only A connected from start to goal; B/C/D lack directed connectivity in their assigned label regions. Thus merely adding a time-dependent potential does *not* enforce reachability. Graph cut on its own is not a planner.

Separately, a 24-order prioritized time-expanded A* diagnostic is extended to check the minimum Euclidean distance of two simultaneously moving centers *throughout each unit-time transition*, not only at integer endpoints. This is **not** a seed-free solution: paths are used to check feasibility, and the planner enumerates priority orders.

1 grid unit is notionally 1 m and a transition takes 1 second. For two disk robots of radii r_i and r_j and required extra gap m, set required center distance `r_i+r_j+m`. Current tests use the combined minimum distance directly; no wall/corner swept-volume clearance is proven.

| Required separation | Feasible with order-enumeration in H=28 | Result |
|---|---|---|
| 0.0 | yes but overlapping trajectories | unsafe for finite-size robots |
| 0.6 | yes | A 14 / B 15 / C 26 / D 27 |
| 0.9 | not found | no proof of impossibility |
| 1.1 | not found | no proof of impossibility |
| 1.4 | not found | no proof of impossibility |

This demonstrates sensitivity to physical separation and exposes a research gap. The next algorithmic change should jointly optimize directed reachability, footprint clearance, and state-dependent region membership, rather than merely using smoothness penalties or fixing A* paths as seeds.

Files: `output/state_margin/report.json`, `cut_comparison.png`, `margin_comparison.png`.
