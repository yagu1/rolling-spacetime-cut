# Reachability-constrained potential cut — four robots, one-lane bridge

**Status:** exploratory research code (October 2026). It is **not** a complete multi-agent connected graph-cut solver and **must not** be used to control physical robots.

## Research question

Can time-varying, goal-directed potentials be combined with forward/backward reachability and real robot safety margins so that a *multi-label space-time cut* produces usable space-time regions, without fixing complete route trajectories as hard graph-cut seeds?

This iteration investigates two different requirements that must not be conflated:

1. **Directed connectivity**: in every robot's labeled XYT region, a permitted (time-forward) start-to-goal walk exists.
2. **Joint safety**: there exists a simultaneous selection of the four walks whose **swept centerlines** maintain both obstacle clearance and pairwise disk separation at every real-valued instant within each time step.

A connected colored region does **not** imply collision-free motion, and its entire interior is not yet guaranteed safe for arbitrary local replanning.

## Reproduce

From the repository root:

```bash
python -m pip install -r requirements.txt
python experiments/reachability_margin_cut.py --horizon 30 \
    --body-radius 0.28 --wall-margin 0.10 --pair-margin 0.34
python experiments/render_reachability_margin.py
python experiments/run_reachability_margin_matrix.py
python -m pytest -q tests/test_reachability_margin_cut.py
```

Main program: [`reachability_margin_cut.py`](reachability_margin_cut.py). Renderer: [`render_reachability_margin.py`](render_reachability_margin.py).

Outputs:

- `output/reachability_margin/report.json` — configurations, cuts, connectivity feedback history, paths, geometry checks, frame labels.
- `output/reachability_margin/matrix.json` and `matrix.csv` — 4-condition parameter sweep.
- `docs/figures/reachability_margin/bridge_xt_boundary.png` — **X–T** ownership, three stages of optimization.
- `docs/figures/reachability_margin/cut_slice_comparison.png` — snapshots of ownership and robots at four times.
- `docs/figures/reachability_margin/margin_and_tube.png` — minimum separation versus required margin, conservative deviation slack.
- `docs/figures/reachability_margin/viable_cones.png` — size of per-agent forward/backward viable state sets through time.
- `docs/figures/reachability_margin/four_robot_state_margin.gif` — positions, path traces, disk radius and half pair-margin halo.

## Geometry and reference model

- 4 mobile disks (A,B left to right; C,D right to left), one 9-cell central lane, 3×3 banks at both ends.
- 1 grid unit = 1 m; one time step = 1 s. Only orthogonal motion and waiting are permitted.
- Disk body radius `r=0.28 m`; extra wall margin `m_w=0.10 m`.
- Required wall separation of the **centerline**: `r + m_w = 0.38 m`.
- Extra pair margin `m_p=0.34 m`.
- Required inter-robot center separation: `2*r + m_p = 0.90 m`.
- Straight synchronized interpolation is assumed between each time step's centers. Clearance is evaluated at the minimum distance *over each continuous movement segment*, not only at integer sample times.
- The free environment is the union of unit squares centered at walkable grid nodes. Each center transition is tested against the full environment wall boundary. Robot orientation, 3D dynamics, uncertain state estimation, control latency, and acceleration are not modeled.

## Algorithm and its limitations

### 1. Directed viability cones (no route seed)

For robot `i` and a horizon `H`, construct a directed time-expanded graph of valid center transitions. Its possible space-time states are

```
Vi(t) = ForwardFromStart_i(t) INTERSECT BackwardToGoal_i(t)
```

At each time slice `t`, `Vi(t)` contains **all states** on at least one wall-safe start-to-goal walk of robot `i`. No single path is selected. Bridge direction (no reversing once inside) and wall margin are included in transition generation. Goal remains occupied after arrival.

### 2. State-dependent multi-label graph cut (no route seed)

For cell `(p,t)`, use a cost based on D* Lite cost-to-go, distance traveled from the start, the minimum detour excess, and time-dependent expected progress:

```
D_i(p,t) = 100 * [
    0.11 * max(0, f_i(p) + g_i(p) - d_i)
  + 0.10 * abs(f_i(p) - t*d_i/H)
  + 0.02 * g_i(p)
]
```

States outside the individual viability cone receive **very high unary cost**, though this prototype's labeling still assigns every grid-time node one of four labels (so this is not a mathematical hard ban on those assignments). Starts at time 0 and goals at `H` are hard terminal seeds. A 4-label Potts energy is approximated by alpha expansion, using spatial and temporal cut-boundary penalties. No complete path is pinned.

### 3. Directed reachability feedback

Following each cut, compute all forward-reachable and backward-reachable states **inside each assigned label**. If a robot is disconnected, find the least-label-edit directed walk through its viable cone, and use it as **finite unary pressure** on the next cut. This is a sequential heuristic, not a guarantee of completeness or of globally optimal region size; however no entire route is hard-pinned.

**Observed for H=30:** initial cut connects 1/4 robots; directed feedback connects 4/4 after three feedback updates. The four independently recomputed connected paths nevertheless have *zero* swept pairwise center separation during some transitions: **they collide**. A second criterion is necessary.

### 4. Margin-aware joint-safety feedback

A separate feasibility oracle enumerates all 24 prioritized A* planning orders and keeps its shortest feasible plan for the specified swept-pair margin. This **does not fix any robot's priority beforehand**; nevertheless it is a prioritized-planning oracle and is not exact 4-robot simultaneous optimization.

The four synchronized candidate trajectories are returned to the **cut as finite, soft potential pressure**. They are **not hard pinned**. We then perform a new 4-label graph cut and run an independent **joint-state four-robot A*** constrained to each robot's newly allocated region. If joint A* finds a feasible path combination, its continuous-time swept clearance is verified. Only **that concrete joint trajectory** (and the conservative deviation envelope derived below) is certified safe, not arbitrary trajectories within the colored regions.

This safety feedback uses full candidate paths supplied **after** the reachability cut fails the margin criterion. Therefore the method has not solved the harder open question of computing joint-safe, directed-connected regions from state potentials and graph cuts *without any path oracle*. It is a constrained, empirical prototype for developing a more rigorous flow-constrained optimizer.

### 5. Conservative trajectory-centered safety tube

For each synchronized transition `t -> t+1`, let `d_pair(t)` denote minimum pairwise center distance across its continuous motion and `d_wall(t)` the minimum centerline distance to any wall. A common path-deviation radius is bounded by:

```
δ(t) ≤ min( d_wall(t) - (r+m_w),
            [d_pair(t) - (2r+m_p)]/2 )
```

This is a conservative local bound if each centerline can deviate at most `δ(t)` over **that entire transition**. It does not imply that any motion within a full graph-cut label region is safe; it shows a small certified *trajectory tube*. In the 0.9 m example, the critical pair separation is 1.0 m, so the available common deviation radius can be just 0.05 m even though the bridge-wall clearance slack is 0.12 m.

## Results (deterministic toy example)

| Horizon | Required pair-center distance | Initial cut | Directed-repair cut | Final safety feedback | Arrivals | Interpretation |
|---|---:|---|---|---|---|---|
| 28 | 0.60 m | 1/4 connected | 3/4 connected (this run) | **Feasible** | 14 / 15 / 26 / 27 | Four synchronized paths verified |
| 28 | 0.90 m | 1/4 connected | 3/4 connected (this run) | Not found | — | 24-order oracle found no plan; does not prove infeasibility |
| 30 | 0.90 m | 1/4 connected | **4/4 connected** | **Feasible** | 14 / 16 / 28 / 30 | Minimum swept separation **1.00 m** (required 0.90 m) |
| 30 | 1.10 m | 1/4 connected | 4/4 connected | Not found | — | No plan found by the tested heuristic; does not prove infeasibility |

All arrival figures are **simulation steps**, not measured physical flight/drive times. For H=30, 0.90 m case, 8 of 24 order-based candidates were feasible; after the soft cut feedback, a 4-agent joint A* restricted to the cut found a feasible synchronized plan with a sum of arrival times of **88 steps** (optimal **only within that final label assignment**, not globally). Its minimum center-to-wall distance was 0.50 m.

## What remains genuinely open

1. **Direct connectivity constraints in labeling**: hard multicommodity directed-flow conservation or lazy reachability inequalities, without soft path-oracle feedback.
2. **Region-level safety**: mutually exclusive **swept transition** sets with body footprints rather than only safe nominal trajectories and trajectory tubes. Include an unassigned buffer label.
3. **Dynamic state**: estimate velocity/heading/uncertainty and state transition feasibility; permit safe temporal hold states and penalize corridor ownership switching.
4. **Optimization**: compare exact small-case MILP or CP-SAT, CBS, ECBS and bounded/suboptimal search. Graph-cut-only energy minimization does not establish global optimality.
5. **Scalability**: multiple bridges, different-width corridors, 3D+T, asynchronous motion, disturbance/error margins, online replanning.
