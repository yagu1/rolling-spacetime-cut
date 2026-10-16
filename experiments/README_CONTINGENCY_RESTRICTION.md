# Experiment E1: Contingency absorption with restriction-only navigation

**Status:** exploratory, discrete simulator — not a safety-certified planner or a paper-quality comparative benchmark.

## Research question

Can a supervisory system distribute time-bounded **spatial restrictions**, rather
than trajectories, and thereby allow independent local D* / time-expanded A*
planners to **detour around a newly detected obstacle**, **wait safely** when
no path currently exists, and **resume** without new path instructions?

The delivery S/G assignment is fixed in this experiment. The Task Allocation
(TA) mechanism and any TA feedback are not yet tested.

## Architecture

- **TA (fixed input here):** four delivery-style S/G pairs. A and B cross the
  one-lane bridge left-to-right; C and D cross right-to-left.
- **Restriction Planning (RP):** Monte Carlo **Random Shooting**, not MCTS,
  evaluates a small budget of candidate passage time windows. The supervisor
  broadcasts only `RestrictionMap` objects (resource mask, temporal access,
  version, validity, protected parking). It may use predicted rover local plans
  *internally* as candidate feasibility evidence; it does not transmit them.
- **PP (on each rover):** local sensor range 2 Manhattan grid cells, incremental
  D* Lite potential updates on obstacle appearance/removal, and an independent
  time-expanded A* respecting the latest restriction. When no path is currently
  found, it returns a **safe-hold candidate** (stay in place).
- **Execution simulation:** an **idealized joint transition shield**, identical
  across all compared methods, rejects moves whose synchronized swept-center
  distance falls below 0.90 m. This simulator oracle uses ground truth; it
  is **not** a decentralized collision-avoidance implementation. All claims
  of collision-free operation are conditional on this oracle and toy dynamics.

The supervisor has a 60 s lookahead and sends restrictions every 10 s. Rovers
locally replan every 1 s. The bottleneck is a 1D resource gate, therefore this
experiment intentionally invokes **zero XYT Graph Cuts**; future research will
need geometric local cuts at junctions and wide shared regions.

## Scenarios

1. `none`: nominal four-rover passage, no unexpected obstacle.
2. `approach_detour`: at **t=1 s**, obstacle `(13,2)` appears on the right-bank
   approach, and is removed at **t=18 s**. There are side paths around the
   obstacle, but not a second bridge. Rovers C and D can individually choose
   the upper/lower bank-side alternatives without a path update from RP.
3. `bridge_closure`: at **t=6 s**, the bridge cell `(7,2)` is blocked and
   reopens at **t=23 s**. No full bypass exists, requiring a safe holding
   phase before crossing can resume.

Obstacle intervals are inclusive of their start and exclusive of their end.
Obstacle activation is constructed not to overlap a rover's occupied cell in
this scenario. True real-world hazards need a more general response policy.

### Comparator definitions (deliberately limited)

| Label | Planner definition | What is allowed between 10 s updates |
| --- | --- | --- |
| **RP / restriction** | Same resource-grant supervisor; each rover has its own D* and time A* | Autonomous detour / hold / resume inside its published restriction |
| **central_path** | Same supervisor produces both grants and a full centerline route every 10 s | May emergency-stop on hazard but **cannot** depart from that route before next dispatch |
| **uncoordinated** | Independent local D* / time A*, **without** supervisory grants | Emergency-stop by the **same idealized simulation safety shield** |

**Fairness caveat:** central-path dispatch is a deliberately restricted
controller rather than a state-of-the-art centralized MAPF solver or robust MPC.
Likewise the uncoordinated baseline has a deliberately simplified short-range
safety shield. The results cannot establish superiority over CBS, RHCR, traffic
reservations, industrial traffic management or Nav2.

## Primary one-seed reproducible observation (seed=17, budget=16)

| Scenario | Method | Four goals reached? | Last arrival | Emergency stop events | Minimum swept-center separation |
| --- | --- | --- | ---: | ---: | ---: |
| Nominal | Restriction | Yes | 36 s | 0 | 1.00 m |
| Nominal | Fixed-path dispatch | Yes | 36 s | 0 | 1.00 m |
| Nominal | No RP | No by 90 s | N/A | 332 | 1.00 m |
| Approach obstruction | Restriction | Yes | **36 s** | 2 | 1.00 m |
| Approach obstruction | Fixed-path dispatch | Yes | **65 s** | 16 | 1.00 m |
| Approach obstruction | No RP | No by 90 s | N/A | 328 | 1.00 m |
| Bridge closure | Restriction | Yes | **55 s** | 17 | 1.00 m |
| Bridge closure | Fixed-path dispatch | Yes | **66 s** | 9 | 1.00 m |
| Bridge closure | No RP | No by 90 s | N/A | 298 | 1.00 m |

For the approach event, C and D select the locally available side lanes at
**t=2**, before the next RP update at t=10. In the bridge-closure event, the
lead rovers wait while the bridge is blocked and later continue. The emergency
stop count is **not** an overall safety or contingency cost: the RP method
can intentionally wait more often than the fixed-path baseline.

## Small randomized pilot (10 different Monte Carlo seeds, fixed scenarios)

| Scenario | Mode | Success rate within 90 s | Mean last arrival among successful runs |
| --- | --- | ---: | ---: |
| Nominal | Restriction | 10/10 | 35.1 ± 0.3 s |
| Nominal | Fixed-path dispatch | 10/10 | 35.1 ± 0.3 s |
| Approach obstruction | Restriction | 10/10 | 38.2 ± 9.8 s |
| Approach obstruction | Fixed-path dispatch | 10/10 | 62.4 ± 5.0 s |
| Bridge closure | Restriction | 10/10 | 55.1 ± 0.3 s |
| Bridge closure | Fixed-path dispatch | 10/10 | 65.1 ± 0.3 s |
| All scenarios | No RP | 0/10 each | not applicable |

This sweep varies the supervisor's **random candidate search only**: the same
map, robot dimensions, event location, event timing, sensor model and safety
shield are used in every run. Do not interpret the variation as resilience to
random physical disturbances. Means are shown for successful trials only;
unfinished trials are not assigned fictitious completion times.

## Reproduce

From repository root, with Python 3.10+:

```bash
python -m pip install -r requirements.txt
python experiments/contingency_restriction.py --budget 16 --seed 17
python experiments/run_contingency_batch.py --seeds 10 --budget 16
python experiments/render_contingency_restriction.py
python -m pytest -q tests/test_contingency_restriction.py
```

**Outputs** in `output/contingency_restriction/`:

- `summary.csv`: one deterministic run per scenario/method.
- `seed_sweep.csv`, `seed_sweep_summary.csv`: all 90 pilot trials and aggregates.
- `*_restriction.json`, `*_central_path.json`, `*_uncoordinated.json`:
  per-second positions, held states, observed hazards, and issued restrictions.
- `approach_detour_restriction_routes.png`: actual local detour geometry.
- `approach_recovery_time.png`: arrival-progress comparison around t=1 hazard.
- `bridge_hold_and_resume.png`: two methods during blocked bridge.
- `bridge_restriction_leases.png`: issued bridge access times versus occupancy.
- `pilot_seed_sweep.png`: 10-seed mean last-arrival comparison.
- `approach_detour_restriction.gif`, `bridge_closure_restriction.gif`:
  animated independent rover motion.

The two route maps and the GIFs visualize actual simulated positions; the
supervisor never broadcasts these trajectories in restriction-only mode.

## Limits and next experiments

1. **Robust RP-to-PP contract:** The present *shared simulation shield* enforces
   ground-truth spacing. The published restriction regions **alone do not**
   guarantee every independent rover trajectory remains separated. Replace the
   oracle with decentralized obstacle detection/safety filtering; study what
   safety can be guaranteed by RP while paths remain unconstrained.
2. **Unexpected occupancy before an obstacle spawns:** An obstacle can appear
   on a rover location in real life; the synthetic scenarios avoid this case.
3. **Time-window commitments:** Simulator communications are synchronous,
   instant and reliable. Add lease acknowledgement, bridge-clear confirmation,
   delayed/out-of-order updates and safe fallbacks after communication loss.
4. **Safe-holding semantics:** A one-step stationary cell is not automatically a
   viable refuge under continuing dynamic traffic. Introduce safe-stop states,
   stable resource ownership and bounded waiting/recovery properties.
5. **RP generality:** Shared bridge is hand-coded as a resource. Automatically
   detect local conflict zones and compare resource gates with local Graph Cut.
6. **Baseline quality:** Add CBS/ECBS or RHCR, time-space resource reservations,
   and a centralized planner that supports event-driven route updates.
7. **TA coupling:** Add candidate S/G matching and feed predicted restriction
   delay to the task allocator. This experiment fixes the S/G assignment.
8. **Geometric realism:** Continuous obstacle swept volumes, heading, braking,
   localization uncertainty, asynchronous motion, realistic map geometry, and
   ROS 2/Nav2 with independently running rover nodes.

The central research claim being tested is **contingency absorption without
central trajectory re-issuance**, not that graph cuts or random shooting are
already shown to dominate existing motion planning algorithms.
