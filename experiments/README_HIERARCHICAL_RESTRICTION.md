# Hierarchical, restriction-only 4-rover navigation (experimental)

This experiment implements the new separation between **supervisory restriction
allocation** and **onboard autonomous path planning**. The supervisor knows
all tasks, positions, goals, and the global map, but *does not transmit paths*.
It transmits per-rover time-valid restriction maps; local rovers use their own
D* Lite map potentials and time-dependent A* to choose their actual paths.

**This is a proof of concept, not a collision-avoidance guarantee.** Communications
are synchronous and error-free in the simulation, all robot locations and the
map are known to the supervisor, motion is a linearly interpolated 1m/s grid
transition, and there is no localization uncertainty, sensing delay, robot
turning radius, acceleration bound or actuation mismatch. An infeasible sampled
allocation is a *search failure*, not a proof that no feasible coordination exists.

## Why this version differs from the Graph Cut demos

The earlier alpha-expansion demos performed a 4-label partition over the entire
XYT lattice. A one-cell-wide chokepoint has a much simpler resource structure:
the conflict is **when** a robot may enter the bridge, not a geometrically
ambiguous 2D cut. The supervisor therefore uses a **Monte Carlo random shooting
search** over candidate bottleneck access windows. This is **not MCTS**, and
it does **not** run a full XYT Graph Cut. Wide intersecting corridors would
require a local geometric-cut/refinement stage in a later extension.

Bank/staging cells remain jointly traversable (not assigned exclusive A/B/C/D
labels). The entire shared narrow lane `(x,2)` is time gated; parked-robot goal
cells are excluded from each other robot's maps to prevent waiting in an
occupied goal bay. Thus the communication payload represents a **region plus
validity interval**, not a centerline route. Bottleneck entry windows for
same-direction rovers can overlap if swept-clearance validation permits.

## Computational structure

- **Global / static onboard D***: each rover maintains cost-to-go values on the
  map. Dynamic restrictions are separate; the rover performs an ordinary
  time-expanded A* over `(x,y,t)` to select wait/move actions. This is not a
  fully incremental space-time D* implementation.
- **Supervisor**: every 10 seconds, inspect rover telemetry and compute a
  lookahead up to 60 seconds. Only competing shared-resource cells are considered
  for allocation. Sample a small number of candidate group orders, release
  headways, and switch times (default 32).
- **Simulated rollout and soft repulsion**: the supervisor asks each local planner
  to plan *hypothetically* under the candidate restriction, then checks the
  resulting synchronized trajectories. For robot pair `(i,j)` at time `t`,
  the soft score uses
  `exp(-0.5 * (max(0,d_ij(t)-d_min)/sigma)^2)` with `sigma=0.65m`,
  while the hard exclusion is `d_ij(t)>=d_min`. Here `d_ij` is the continuous
  minimum over the two robots' synchronous linear swept segments.
- **Assignment score**: sum of remaining arrival times, maximum delay,
  soft repulsion, restriction volume, and changes from the prior grant.
- **Deployment**: publish ONLY `RestrictionMap` with `version`, `issued_at`,
  `valid_to`, bottleneck cell set, a permitted time interval, and exclusion
  cells around occupied goal pockets. No route is sent.
- **Execution**: each rover recomputes its own path under the published
  restriction, executes 10 seconds, reports new position, and repeats.
  Already activated grants are preserved to avoid revoking the lane under a
  robot. The update itself is still instantaneous in this simplified demo.

Although a global map is available onboard, the finite-time path search only
expands states reachable within its temporal horizon. A lightweight spatial
reachability envelope determines the competing local resource ROI; the demo map
is so small that every grid cell lies in a 60-second envelope.

## Reproduction

```bash
python -m pip install -r requirements.txt
python experiments/hierarchical_restriction.py --lookahead 60 --update 10 --budget 32
python experiments/render_hierarchical_restriction.py
python -m pytest -q tests/test_hierarchical_restriction.py
python experiments/compare_budget_restriction.py
```

Outputs appear in `output/hierarchical_restriction/`:

- `report.json`: actual trajectories, issued restriction payloads, update events,
  candidate search metrics and safety checks.
- `restriction_schedule.png`: granted resource windows vs independently chosen
  robot motion, plus supervisor compute time at each update.
- `restriction_slices.png`: robot positions and who can use the bridge at each
  selected time; the rest of the map remains shared.
- `swept_clearance_and_repulsion.png`: minimum continuous pairwise separation
  and the soft repulsive score along executed trajectories.
- `four_rovers_restrictions.gif`: dynamic gates and robot positions.
- `baseline_comparison.png`: unrestricted individually optimal but colliding
  routes, versus safe but time-staggered restriction-only execution.
- `update_summary.csv`: per-update search count, runtime and locality.
- `budget_study.csv` / `.png`: 10-seed random shooting comparison at budgets
  of 2, 8, 16, 32 and 64 candidate allocations per update.

### Example, deterministic seed 17

- A/B start left, C/D start right, one-cell lane with staging bays.
- `H=60` steps, update period `10` steps, `32` candidates/update.
- Robot radius `0.28m`, static wall margin `0.10m`, required robot-center
  separation `0.90m` (a joint clearance parameter independent of wall margin).
- Robot arrival times: A=16, B=14, C=30, D=33 seconds.
- When each rover plans without bridge access restrictions, each finds a
  14-second shortest path but the synchronized trajectories collide
  (minimum swept center distance 0m). The unregulated baseline is **unsafe**.
- Actual minimum swept-center distance: `1.00m`, no collision in this exact
  synchronized grid simulation; all reach Goal.
- Whole-XYT Graph Cut calls: `0`. Monte Carlo query time is reported in
  `update_summary.csv` (strongly dependent on hardware and scenario).

## Known limitations and next research steps

1. The shared bridge is explicitly given as a resource (all `y=2` cells), not
   automatically detected. Identify local competing bottlenecks from reachability
   overlap, occupancy risk and uncertainty, and optionally apply *local* Graph
   Cut in wider ambiguous spaces rather than everywhere.
2. Candidate generation is a stochastic **ordering/slot sampler**, not MCTS;
   it cannot claim globally optimal allocation and can miss feasible solutions.
   For many robots, compare MCTS/UCT over high-level actions (`wait`, `open
   corridor`, `change direction`) against random shooting with equal budgets.
3. Safe rollout of particular independent paths does NOT guarantee every path
   allowed by the distributed restriction maps will be collision-free. For a
   true safety contract, each granted region must remain safe for all allowable
   robot actions, or the system must add a runtime monitor / braking shield.
4. Communication acknowledgement, clock synchronization, version retention,
   delayed updates, policy expiry and safe-stop behavior are NOT implemented.
5. Receding horizon lookahead and committed resource release need a formal
   viability/terminal condition to rule out deadlock, starvation and livelock.
6. The current proof-of-concept is 2D+T. Extend to 3D+T and add BST-ID/XYHT as
   an optional interchange format for distributed restriction maps.
