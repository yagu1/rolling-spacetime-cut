# Step-scaled, no-fixed-priority graph-cut experiment

This is an *additional experiment*, not a replacement for `rolling_cut.py`.
It starts from the exact `two_corridors()` scenario in the root prototype.

## Run

```bash
python experiments/step_scaled_horizon.py --factor 2
python experiments/render_step_scaled.py
python -m pytest -q
```

`H = ceil(factor * max(number of edges in each independently planned D* Lite route))`.
The default `factor=2` gives `H = 2 * max(14, 16) = 32` time steps.

- **Pass 1:** Symmetric XYT graph cut using D* Lite cost-to-go, start/goal pins,
  and spatial/temporal smoothness. No corridor priority, no designated leader.
  Constrained space-time A* is run on each part. **A is reachable,
  but B is disconnected** in the current toy example. Goal ownership is
  pinned only from each agent's earliest physically reachable time.
- **Comparator:** A joint-state A* with **no priority and no agent ordering**
  optimizes the sum of arrival steps under discrete conflict constraints. On
  this toy world it finds A=14, B=20 (sum=34) after expanding 155 states.
  This is a small-instance optimal benchmark, not the partitioning method.
- **Pass 2:** When connectivity fails, enumerate *both* alternatives (A's
  independent shortest path as the first feasibility witness, or B's).
  Compute a time-respecting path for the other agent while avoiding the first
  witness's occupied vertices and opposing-edge swaps. These path pairs are
  the source of **hard space-time reachability seeds** for a second cut.
- **Pass 3:** Recompute A* within the newly allocated cut regions.
  The second search also respects the first replanned trajectory as an
  edge-swap reservation; both orders are evaluated. Compare alternatives using
  sum of arrival steps, makespan, and cut boundary cost. The selection objective
  has no predetermined A/B priority.

## Deterministic results

| Factor | Horizon | Plain cut A/B reachable? | Symmetric repair |
|---:|---:|---|---|
| 1.0 | 16 | yes/no | no feasible candidate |
| 1.25 | 20 | yes/no | A upper at 14, B lower at 20 |
| 1.5 | 24 | yes/no | A upper at 14, B lower at 20 |
| 2.0 | 32 | yes/no | A upper at 14, B lower at 20 |

At factor 2, the competing feasible repaired cut gives A lower (22),
B upper (16). Its sum of arrival steps (38) is greater than the selected
A-upper/B-lower candidate (34).

## Important limitations

The no-priority *first cut* is **not** sufficient (one agent fails to
reach its goal). The successful solution
uses time-expanded A* to generate feasible route witnesses before a second Graph Cut,
which is materially different from claiming that the Graph Cut alone found
alternative routes. The enumeration over both possible first witnesses is
symmetric but is not a general globally optimal MAPF solver.

This experiment does not yet implement D* Lite in the time-expanded domain.
Instead, D* Lite provides admissible spatial cost-to-go heuristics, and A*
searches `(x, y, t)` under cut-ownership constraints. The graph cut itself
uses symmetric spatial/temporal Potts smoothing.

Two vehicles are idealized as point occupants of cells at integer time ticks.
Vertex and opposite-edge conflicts are checked for computed route pairs. No
continuous-time safety guarantee, swept-volume test, kinematic model, or
between-tick collision checking is claimed. Goals are held occupied after
arrival. See `README.md` for prototype context.

A key next research task is to remove these explicit feasibility-witness
branches by integrating directed reachability/flow constraints *inside* the
partition optimization (or developing a cut–route co-optimization loop). The
current branch enumeration is symmetric as a *decision procedure*, but planning
one route before the other within each branch is still a form of conditional
ordered reservation, not a fully simultaneous graph-cut optimization.

The old root prototype's very large `1e7` seed weights sometimes produce a
numerically inconsistent cut partition with NetworkX's default preflow solver.
This experiment uses `edmonds_karp`, 1e4 seed weights, and checks the returned
cut capacity against the extracted partition. It intentionally leaves the
existing root prototype unchanged.

