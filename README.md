# Rolling Space-Time Graph Cut + D* Lite — exploratory prototype

2026-10 / Multi-agent 2D+T safe-region allocation proof of concept.

> **Research prototype, not an assured collision-avoidance system.** This
> demonstration uses a discretized map and time steps, and **does not** prove
> that graph cuts preserve directed reachability or real-world continuous
> separation. No flight use is intended.

## Goal

Two vehicles have individual shortest paths through the **same single-cell-wide
upper passage**, traveling in opposite directions. We would like to allocate
space-time regions instead of only picking collision-free centerline paths.
When they cannot both use the passage simultaneously, the lower-priority robot
should switch to the lower bypass if that is preferable to waiting.

This implementation demonstrates that behavior through a combination of:

1. **D* Lite**: maintains each robot's global spatial cost-to-go and shortest
   path; supports obstacle-cost changes (`DStarLite.change_obstacle`).
2. **XYT graph cut (s/t minimum cut)**: labels each walkable `(x, y, t)` as
   owned by A or B, using D* cost-to-go and corridor-priority unary costs, plus
   spatial and temporal Potts smoothing terms.
3. **Directed time-expanded path search**: dynamic programming on directed
   `(x, y, t) -> (x', y', t+1)` transitions, including wait moves, constrained
   to the owner label. The global D* cost-to-go is the horizon-terminal cost.
4. **Reachability feedback**: if an unconstrained cut gives a robot no path
   through the window, obtain a candidate bypass avoiding the protected
   shared path, add that bypass as *soft unary evidence*, and cut again.
5. **Rolling horizon**: plan 10 s, execute only 2 s, update state, and repeat.

The shared upper passage is identified by **overlap of the initial independent
shortest paths**, then temporarily priority-protected until A reaches its goal.
For clarity, this prototype is deliberately conservative; it does not
solve the general problem of dynamically selecting arbitrary conflict zones,
proving the reachability of all allocated regions, or finding an optimal schedule.

## Quick start

Python 3.10+; tested with Python 3.11 and NetworkX.

```bash
python -m pip install -r requirements.txt
python rolling_cut.py --horizon 10 --execute 2 --steps 26
python render.py
python -m pytest -q
```

Running `rolling_cut.py` produces `output/demo.json`. `render.py` produces:

- `output/independent_routes.png`: independent D* Lite shortest paths.
- `output/allocated_routes.png`: executed trajectories after reallocation.
- `output/cut_snapshots.png`: Graph Cut labels at three XYT time slices.
- `output/simulation.gif`: vehicle positions as the 2-second windows advance.

### Observed deterministic demonstration

| Robot | Independent path | Path after allocation | Arrival time |
|---|---|---|---|
| A (priority) | Short upper corridor | Upper corridor | 14 s |
| B | Short upper corridor | Longer lower bypass | 20 s |

The recorded execution has **0 same-cell collisions** and **0 opposite-edge
swaps** in this specific scenario. The initial Graph Cut disconnects B; a
second cut with reachability feedback recovers a bypass. These observations
are *not* an overall safety proof.

## Toy map

Symbols: `.` traversable, `#` blocked, `A`, `B` start positions.

```text
A..............
..###########.B
#.###########.#
#.###########.#
#.............#
```

A starts at `(0,2)` and moves right to `(14,2)`; B starts at `(14,3)` and
moves left to `(0,3)`. The top corridor is shorter for both; the lower
corridor is four steps longer for B. One cell and one transition are set to
1 spatial unit and 1 second for this toy case. Movement is orthogonal or wait.

## Mathematical sketch

For each free cell `(x,y)` and discrete time `t`, create the space-time vertex
`v=(x,y,t)`. Forward motion edges are allowed only when `t' = t+1` and the
underlying spatial neighbor/hold is valid.

The binary label `L(v) ∈ {A,B}` is chosen through a two-terminal min cut of:

```text
E(L) = Σ_v D_v[L(v)] + λ Σ_(u,v)∈spatial [L(u)≠L(v)]
                        + μ Σ_(u,v)∈temporal [L(u)≠L(v)]
```

`D_v` combines distance-from-current-state, D* cost-to-go, priority on the
shared corridor, and (if required) an escape-route witness. Starts at the
current time and goals are pinned to their respective labels.

**Important:** minimizing `E(L)` does not enforce agent-specific directed
start-to-goal connectivity. We **check** each cut by running constrained
windowed path searches. If either fails, we update the unary costs and
re-cut. This feedback is a heuristic, not a completeness guarantee.

The spatial global D* potential is `g_i(x,y)`; it is used in the time-window
search as a terminal estimate. This is **not** a full time-dependent D* Lite
implementation; time-dependent feasibility is solved by dynamic programming
on the acyclic time-expanded motion graph in each window.

## Source files

- `rolling_cut.py`: scenario, D* Lite, XYT min cut, rolling simulation, validation.
- `render.py`: PNG/GIF output generation.
- `tests/test_rolling_cut.py`: shortest-path, obstacle-update, conflict-check,
  rolling execution, and alternate window tests.
- `output/demo.json`: archived run metadata and a few cut label snapshots.

## Major limitations / next research steps

1. **General conflict detection**: current priority-protected shared corridor
   comes from overlap of two initial shortest paths, and is locked until A
   arrives. Generalize the resource/corridor detection and dynamically compute
   time-dependent passage ownership.
2. **Connectivity-aware optimization**: min-cut labels need not preserve
   directed reachability; replace the current bypass-recut heuristic with a
   principled flow/reachability-constrained model, or derive conditions under
   which a cut is safe.
3. **Collision guarantees**: only checks same-cell and opposing-edge collisions
   at integer times. Add physical footprints, swept-volume conflicts,
   separation margins and uncertainty between samples. Adjacent cells are not
   automatically safe for finite-size robots.
4. **Robust receding horizon**: this specific example works, but partial
   plans can deadlock/livelock. Add invariant safety states, commitment,
   safe-stop options, and robust horizon-terminal conditions.
5. **Compare waiting against detour**: experiment with alternative route
   lengths, time headways, priority rules and dynamic obstacle changes; compare
   against WHCA*, RHCR and CBS.
6. **3D+T and BST-ID**: after algorithmic validation, replace grid cells with
   hierarchical `(x,y,h,t)` cells / an XYHT spatial-index layer. BST-ID is
   not required for this minimal algorithm demo.

## GitHub

This is intentionally kept separate from `yagu1/bst-id` because it is a
multi-agent planning algorithm prototype, not yet a BST-ID library feature.
To publish as a new repository, create a repository (private or public) and:

```bash
git remote add origin https://github.com/<owner>/<new-repo>.git
git push -u origin main
```

If using the bundled `rolling_spacetime_cut.bundle`, first run:

```bash
git clone rolling_spacetime_cut.bundle rolling_spacetime_cut
cd rolling_spacetime_cut
```

then add the remote. The ZIP distribution is also usable without Git.
