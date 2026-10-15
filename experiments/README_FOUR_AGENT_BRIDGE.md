# Four-agent one-lane bridge: multi-label XYT cut experiment

This extends the two-agent `connected_cut_bridge.py` experiment to **four named
robots** (A/B left-to-right, C/D right-to-left). It is a small discrete routing
experiment, **not a proven safe real-world multi-robot system**.

## Reproduce

Python 3.10+; from the repository root:

```sh
python -m pip install -r requirements.txt
python experiments/four_agent_bridge.py
python experiments/four_agent_bridge.py --tie-side right --out output/four_bridge/right_first.json
python experiments/render_four_agent_bridge.py
python -m pytest -q
```

Files:

- `experiments/four_agent_bridge.py` — four-robot bridge/banks scenario, D*
  cost-to-go, 24-order prioritized XYT A* candidate schedules, four-label
  alpha-expansion Graph Cut, a distinct *unseeded* comparison cut, and
  independent re-search in allocated label regions.
- `experiments/render_four_agent_bridge.py` — X–T cut interfaces, eight XY
  time slices, directed XYT Start–Goal paths, bridge occupancy timeline,
  time-slice GIF, offline HTML slider, reverse-direction equal-cost comparison.
- `tests/test_four_agent_bridge.py` — collision, witnesses, cut labeling,
  monotone crossing, identical-score reverse priority ordering and alpha
  expansion energy-correctness tests.
- `output/four_bridge/report.json` — default A/B-first record and all 29 XY
  time slices; `right_first.json` — symmetric reverse solution.

## Scenario

The *central* bridge is `(x,2)` for `x=3,…,11`: nine connected single-width
cells with no passing branch. Both banks have a small 3x3 staging area:
left `x=0,1,2` and right `x=12,13,14`, `y=1,2,3`.

| Robot | Start | Goal | Independent shortest path |
|---|---|---|---:|
| A | (0,1) | (12,1) | 14 steps |
| B | (0,3) | (12,3) | 14 steps |
| C | (14,1) | (2,1) | 14 steps |
| D | (14,3) | (2,3) | 14 steps |

The XYT horizon is `2 * max(individual D* path length)` = 28 steps. The grid
has 27 traversable XY cells, hence `27*(28+1)=783` XYT label nodes.

At each integer step a robot moves to one orthogonal neighbor or waits.
On the narrow bridge, going backward *toward its starting bank* is forbidden
(a motion/no-turn-back rule), avoiding unrealistic entering-then-retreating
behavior. Upon reaching its destination it remains there (absorbing goal).
Same-cell and opposing-edge swap conflicts are excluded at integer ticks;
footprint, crossing swept-volume, delay uncertainty and continuous-time
separation are NOT modeled.

## What is optimized (and what isn't)

1. D* Lite calculates static cost-to-go for each robot.
2. Time-expanded A* plans robot paths against previous occupied vertices and
   opposing edge transitions. **All 4! = 24 planning orders** are enumerated.
   Feasible schedules are ranked by sum of first arrival steps, then makespan;
   no agent or *side* is assigned fixed priority in advance. Within an order,
   planning is prioritized, so this heuristic does **not** prove global
   minimum-cost multi-agent paths.
3. Four different labels are assigned to all `(x,y,t)` cells. This is not a
   binary Source/Sink cut. We approximately minimize the *multiway* Potts
   labeling energy by **alpha-expansion** using repeated binary s/t min cuts:

   `E(L) = sum_v D_v(L(v)) + λ_xy sum_spatial [L(u)!=L(v)] + λ_t sum_temporal [L(u)!=L(v)]`.

   `D_v` uses distance from the robot start, D* cost-to-go and soft time
   attainability. The confirmed collision-free trajectories are **pinned as
   hard seeds**, so each agent has at least one directed Start–Goal witness in
   the allocated region. These pins are NOT equivalent to a connected
   multiway-cut optimization.
4. A new shortest directed path is computed *inside each agent's allotted
   label* from its Start at `t=0` to its absorbing Goal at `t=H`, and all
   candidate paths are explicitly conflict-checked. In general, exclusive
   vertex labels do **not** exclude opposing-edge swap collisions, so only
   checked paths may be called collision-free.
5. A separate *unseeded* four-label Cut (with only Start at `t=0` and Goal at
   `t=H` pins) is run for comparison. As expected, it does **not** guarantee
   directed reachability for all four robots.

## Observed deterministic result

| Measure | Result |
|---|---|
| Max independent shortest path | 14 steps |
| Planning horizon | 28 steps |
| XYT Cut nodes | 783 |
| Candidate orders | 24 |
| Feasible candidate schedules | 8 |
| Selected left-first planning order | A,B,C,D |
| Individual arrivals | A=14, B=15, C=26, D=27 |
| Sum of arrival steps / makespan | 82 / 27 |
| Bridge traversals | A t=4..12; B t=5..13; C t=16..24; D t=17..25 |
| Vertex / opposing-edge swap conflicts | 0 in reported witness/replanned paths |
| Unseeded Cut Start–Goal reachability | A yes; B,C,D no |
| Pinned Cut Start–Goal reachability | A,B,C,D yes |

The **opposite-direction tie** (`--tie-side right`) yields C=14, D=15,
A=26, B=27: the **same** sum-of-arrivals = 82 and makespan = 27.
The optional side preference only resolves *exact* cost ties; it does not
change the primary cost or force a corridor reservation.

The central-bridge X–T heatmap makes the moving A/B interface and C/D
interface visible. It also shows an abrupt full-bridge recoloring at the
changeover between the directions. This reflects current Potts time regularization,
not a smooth physical motion or a proof that all region subparts are connected.

Without bank holding/passing branches, replacing left and right endpoints on
a pure one-dimensional path is topologically impossible, regardless of `H`:
the left-to-right ordering of mutually collision-free robots cannot reverse.
This is a topological **proof for the pure-path toy case**, not a search
failure extrapolated to arbitrary maps.

## Figures

- `docs/figures/four_bridge/bridge_x_t_cut.png` — 4-color X–T central bridge
  (black = computed space-time cut interfaces, curves = witness paths).
- `docs/figures/four_bridge/bridge_four_time_slices.png` — eight XY slices.
- `docs/figures/four_bridge/four_directed_seed_paths.png` — directed 3D XYT
  Start–Goal witnesses (goal-time columns dotted).
- `docs/figures/four_bridge/bridge_passage_timeline.png` — bridge occupancy
  timeline for all four agents.
- `docs/figures/four_bridge/equal_cost_opposite_order.png` — opposite flow
  order without a prespecified permanent agent priority.
- `docs/figures/four_bridge/four_bridge_cut.gif` — all 29 computed XY slices.
- `docs/figures/four_bridge/four_bridge_cut_slider.html` — standalone, offline
  time slider to inspect cut borders and individual robot positions.
- `docs/figures/four_bridge/no_bays_four_impossible.png` — pure-line case.

## Next research questions

- The *routing first, path-seeded cut second* heuristic must be replaced by a
  principled connected multiway cut / multi-commodity flow / joint planner for
  evidence that the **cut itself** causes alternative routes to be selected.
- Disconnected islands of A/B/C/D in individual XY slices and dead-end areas
  of the full directed XYT graph should be identified by forward/backward
  reachability and removed or penalized.
- Compare time-dependent allocation, less discontinuous cut boundaries, and
  more realistic no-turn-back constraints; evaluate dynamic priority/fairness
  under 4, 6, 8 agents; compare with CBS/RHCR, not only heuristic A*.
- Add vehicle footprints, buffered separation corridors, safe stopping, and
  real-world time/space discretization before treating as a safety algorithm.
