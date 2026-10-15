"""Four-rover hierarchical navigation with *restriction-only* supervisory control.

Proof of concept; NOT a safety-certified multi-robot planner.

Supervisor: finite-budget randomized search over time-dependent access grants for
an automatically identified one-lane shared resource. It calls rover planners
for simulated rollouts but publishes ONLY per-rover restriction grants; it never
publishes or forces a centerline path. Restriction areas outside the bottleneck
remain fully shared. Soft swept-separation repulsion and hard swept-clearance
checks score the candidate grants. Active grants are not revoked on replanning.

Rovers: independent global D* Lite cost-to-go + time-expanded local A* subject
only to their own map, goal, position and restriction grant.

In the single-lane bridge scenario the safety boundary is a 1-D chokepoint gate;
a full spatiotemporal Graph Cut would not add a meaningful geometric boundary.
The experiment *intentionally performs zero full XYT graph cuts*. Graph Cut can
be inserted only for locally ambiguous, wide conflict regions in future work.

Run: python experiments/hierarchical_restriction.py
"""
from __future__ import annotations

import argparse
import heapq
import itertools
import json
import math
import random
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from rolling_cut import Agent, DStarLite
from experiments.four_agent_bridge import make_four_bridge
from experiments.reachability_margin_cut import safe_moves
from experiments.state_margin_bridge import dist_to_segment_zero

Point = tuple[int, int]


@dataclass(frozen=True)
class AccessWindow:
    """Time window over an entire bottleneck, not a planned trajectory."""
    open_at: int
    close_at: int


@dataclass(frozen=True)
class RestrictionMap:
    """Payload published to ONE rover (no route, no trajectory)."""
    version: int
    issued_at: int
    valid_to: int
    resource_cells: frozenset[Point]
    access: AccessWindow | None
    reserved_goal_cells: frozenset[Point] = frozenset()

    def permits(self, p: Point, t: int) -> bool:
        if p in self.reserved_goal_cells:
            return False
        return p not in self.resource_cells or (
            self.access is not None and self.access.open_at <= t < self.access.close_at
        )

    def payload(self) -> dict:
        return {'version': self.version, 'issued_at': self.issued_at,
                'valid_to': self.valid_to,
                'restricted_resource': 'single_lane_bridge_and_approaches',
                'resource_cell_count': len(self.resource_cells),
                'resource_cells_xy': [list(p) for p in sorted(self.resource_cells)],
                'protected_parking_cells_xy': [list(p) for p in sorted(self.reserved_goal_cells)],
                'access_window': asdict(self.access) if self.access else None,
                'contents': 'resource-level allowed/denied mask; no path waypoints'}


class Rover:
    """Own navigation stack. Supervisor cannot set a path on this class."""
    def __init__(self, agent: Agent, graph, *, radius: float, wall_margin: float):
        self.agent = agent
        self.graph = graph
        self.pos = agent.start
        self.dstar = DStarLite(graph, agent.start, agent.goal)
        self.safe = safe_moves(graph, [agent], radius, wall_margin)[agent.name]
        self.restriction: RestrictionMap | None = None
        self.trace = [self.pos]
        self.arrived_at: int | None = None

    def accept_restriction(self, restriction: RestrictionMap) -> None:
        if self.restriction and restriction.version <= self.restriction.version:
            raise ValueError('non-monotonic restriction version')
        if not restriction.permits(self.pos, restriction.issued_at):
            raise ValueError('restriction would immediately revoke occupied cell')
        self.restriction = restriction

    def local_plan(self, position: Point, now: int, horizon: int,
                   restriction: RestrictionMap) -> list[Point] | None:
        """Independent time-expanded A*; D* Lite supplies the cost-to-go.

        Future restrictions matter only on the single shared resource. Outside
        that resource no pre-selected corridor/path is imposed by supervision.
        """
        goal = self.agent.goal
        if position == goal:
            return [position] * (horizon + 1)
        if not restriction.permits(position, now):
            return None
        q = []
        ct = itertools.count()
        start = (position, now)
        # Use integer step count as primary cost; waiting contributes a small
        # secondary tie-break, keeping the D* distance heuristic admissible.
        heapq.heappush(q, (self.dstar.g[position], 0., next(ct), start))
        best = {start: 0.}
        parent = {start: None}
        hit = None
        while q:
            _, g, _, (p, t) = heapq.heappop(q)
            if g != best[(p, t)]:
                continue
            if p == goal:
                hit = (p, t)
                break
            if t >= now + horizon:
                continue
            for v in self.safe.get(p, ()):  # includes wait
                if not restriction.permits(v, t + 1):
                    continue
                if not math.isfinite(self.dstar.g[v]):
                    continue
                # Prefer moving rather than waiting, but waiting is available.
                inc = 1.0 + (0.10 if v == p else 0.0)
                new = (v, t + 1)
                score = g + inc
                if score >= best.get(new, math.inf) - 1e-10:
                    continue
                best[new] = score
                parent[new] = (p, t)
                heapq.heappush(q, (score + self.dstar.g[v], score, next(ct), new))
        if hit is None:
            return None
        rev = []
        while hit is not None:
            rev.append(hit[0])
            hit = parent[hit]
        route = rev[::-1]
        route.extend([goal] * (horizon + 1 - len(route)))
        return route

    def execute(self, path: list[Point], seconds: int, now: int) -> None:
        assert self.restriction is not None
        for k in range(1, seconds + 1):
            nxt = path[k]
            assert nxt in self.safe.get(self.pos, ())
            assert self.restriction.permits(nxt, now + k)
            self.pos = nxt
            self.trace.append(nxt)
            if self.arrived_at is None and self.pos == self.agent.goal:
                self.arrived_at = now + k
        self.dstar.move_start(self.pos)


def swept_clearance(paths: dict[str, list[Point]], required: float) -> tuple[bool, float, float]:
    """Continuous straight swept-center checks for synchronized unit steps.

    Also return soft repulsive energy centered around the clearance distance.
    Never substitute this soft energy for the hard distance requirement.
    """
    min_dist = math.inf
    repulsion = 0.0
    for a, b in itertools.combinations(sorted(paths), 2):
        pa, pb = paths[a], paths[b]
        for t in range(1, min(len(pa), len(pb))):
            d = dist_to_segment_zero(pa[t-1], pa[t], pb[t-1], pb[t])
            min_dist = min(min_dist, d)
            if d + 1e-9 < required:
                return False, min_dist, repulsion
            # A smooth cooperative repulsion for *near* motion, zero at distance.
            repulsion += math.exp(-0.5 * (max(0., d - required) / 0.65) ** 2)
    return True, min_dist, repulsion


class Supervisor:
    """Randomized rolling search over boundary/resource allocations only."""
    def __init__(self, rovers: dict[str, Rover], *, lookahead: int = 60,
                 update_every: int = 10, candidate_budget: int = 32,
                 safety_distance: float = 0.90, seed: int = 17):
        self.rovers = rovers
        self.lookahead = lookahead
        self.update_every = update_every
        self.budget = candidate_budget
        self.required = safety_distance
        self.rng = random.Random(seed)
        self.resource = frozenset((x, 2) for x in range(0, 15))
        self.windows: dict[str, AccessWindow] = {}
        self.version = 0
        self.history = []

    def _conflict_scope(self, now:int) -> dict:
        """A rolling reachability envelope; do not expand all of global XYT.

        Reachable spatial cells are computed from CURRENT positions and the
        finite lookahead. Only shared-resource cells in two+ envelopes are
        treated as potential conflict region. The full graph is tiny here,
        so all cells happen to lie within the 60-second envelope.
        """
        cones = {n: {p for p,d in rover.graph.distances(rover.pos).items()
                     if d <= self.lookahead} for n,rover in self.rovers.items()
                 if rover.pos != rover.agent.goal}
        roi = {p for p in self.resource if sum(p in c for c in cones.values()) >= 2}
        return {'live_rovers':len(cones),
                'union_reachable_cells':len(set().union(*cones.values())) if cones else 0,
                'competing_bottleneck_cells':len(roi),
                'estimated_full_XYT_node_count': len(next(iter(self.rovers.values())).graph.cells)*(self.lookahead+1),
                'bottleneck_XYT_node_equivalent':len(roi)*(self.lookahead+1)}

    def _candidate_windows(self, now: int, order: list[str],
                           headway: int, crossing: int,
                           switch_gap: int, delay: int) -> dict[str, AccessWindow]:
        # Never revoke an existing grant that may already be in use.
        frozen = {
            n: w for n, w in self.windows.items()
            if (w.open_at <= now and self.rovers[n].pos != self.rovers[n].agent.goal)
        }
        slots = dict(frozen)
        start = now + delay
        idx = 0
        if frozen:
            # Allow not-yet-activated rovers of the SAME direction to join
            # the existing committed phase rather than unnecessarily opening
            # a new exclusive phase after it. Their existing grant is useful
            # as a stable proposal; the swept clearance check is still hard.
            active_directions = {self._side(n) for n in frozen}
            if len(active_directions) != 1:
                raise RuntimeError('opposite directions held simultaneous committed bridge access')
            current_side = next(iter(active_directions))
            end = max(w.close_at for w in frozen.values())
            while idx < len(order) and self._side(order[idx]) == current_side:
                name = order[idx]
                old = self.windows.get(name)
                earliest = max(now, min(w.open_at for w in frozen.values()) + headway)
                release = max(earliest, old.open_at) if old else earliest
                if release >= end:
                    break
                slots[name] = AccessWindow(release, end)
                idx += 1
            start = max(start, end + switch_gap)
        while idx < len(order):
            run = [order[idx]]
            direction = self._side(order[idx])
            idx += 1
            while idx < len(order) and self._side(order[idx]) == direction:
                run.append(order[idx]); idx += 1
            releases = [start + k * headway for k in range(len(run))]
            phase_end = max(releases) + crossing
            for name, release in zip(run, releases):
                slots[name] = AccessWindow(release, phase_end)
            start = phase_end + switch_gap
        return slots

    def _side(self, name: str) -> str:
        a = self.rovers[name].agent
        return 'L' if a.goal[0] > a.start[0] else 'R'

    def _propose(self, now: int, k: int) -> dict[str, AccessWindow]:
        frozen = {
            n for n, w in self.windows.items()
            if (w.open_at <= now and self.rovers[n].pos != self.rovers[n].agent.goal)
        }
        todo = [n for n, r in self.rovers.items()
                if n not in frozen and r.pos != r.agent.goal]
        if k == 0:
            order = sorted(todo)
            headway, crossing, switch_gap, delay = 4, 14, 2, 0
        elif k == 1:
            order = sorted(todo, reverse=True)
            headway, crossing, switch_gap, delay = 4, 14, 2, 0
        else:
            order = self.rng.sample(todo, len(todo))
            headway = self.rng.choice([3, 4, 5, 6])
            crossing = self.rng.choice([13, 14, 15, 16])
            switch_gap = self.rng.choice([1, 2, 3])
            delay = self.rng.choice([0, 0, 0, 1, 2])
        return self._candidate_windows(now, order, headway, crossing, switch_gap, delay)

    def _evaluate(self, now: int, windows: dict[str, AccessWindow],
                  *, version: int) -> dict | None:
        restrictions = {}
        paths = {}
        for name, rover in self.rovers.items():
            # Finished robots remain outside the shared bottleneck in this map.
            restriction = RestrictionMap(version, now, now+self.lookahead,
                                         self.resource, windows.get(name),
                                         frozenset(r.agent.goal for other,r in self.rovers.items() if other != name))
            path = rover.local_plan(rover.pos, now, self.lookahead, restriction)
            if path is None:
                return None
            restrictions[name] = restriction
            paths[name] = path
        safe, minimum, repulsion = swept_clearance(paths, self.required)
        if not safe:
            return None
        arrival = {n: next(i for i, p in enumerate(path)
                           if p == self.rovers[n].agent.goal)
                   for n, path in paths.items()}
        # Additional soft costs: close passes, maximum lateness, and how much
        # restriction an operator imposes. A small edit cost prevents churn.
        delays = [arrival[n] - self.rovers[n].dstar.g[self.rovers[n].pos]
                  for n in sorted(self.rovers)]
        restriction_volume = sum(
            sum(not (w.open_at <= t < w.close_at)
                for t in range(now, now+self.lookahead))
            for w in windows.values()
        )
        churn = sum(abs(w.open_at - self.windows[n].open_at)
                    for n, w in windows.items() if n in self.windows)
        score = (sum(arrival.values()) + 0.20*max(delays)
                 + 0.09*repulsion + 0.001*restriction_volume
                 + 0.03*churn)
        return {'score': score, 'paths': paths, 'restrictions': restrictions,
                'arrival_remaining': arrival, 'repulsion': repulsion,
                'minimum_distance': minimum, 'churn': churn,
                'restriction_volume': restriction_volume, 'windows': windows}

    def update(self, now: int) -> dict:
        start = time.perf_counter()
        feasible = []
        for k in range(self.budget):
            windows = self._propose(now, k)
            trial = self._evaluate(now, windows, version=self.version+1)
            if trial is not None:
                feasible.append(trial)
        elapsed = time.perf_counter() - start
        if not feasible:
            raise RuntimeError('no verified restriction schedule within candidate budget; rovers must stop safely')
        best = min(feasible, key=lambda s: (s['score'],
                                             sorted((n,w.open_at,w.close_at) for n,w in s['windows'].items())))
        self.version += 1
        self.windows = best['windows']
        for name, rover in self.rovers.items():
            rover.accept_restriction(best['restrictions'][name])
        info = {'now': now, 'version': self.version,
                'candidates': self.budget, 'feasible_candidates': len(feasible),
                'search_seconds': elapsed, 'score': best['score'],
                'predicted_arrival_remaining': best['arrival_remaining'],
                'predicted_min_swept_distance_m': best['minimum_distance'],
                'repulsion_energy': best['repulsion'],
                'restriction_volume': best['restriction_volume'],
                'windows': {n:asdict(w) for n,w in self.windows.items()},
                'cut_calls': 0, 'full_spacetime_cut_nodes_allocated': 0,
                'scope':self._conflict_scope(now)}
        self.history.append(info)
        return best


def run(lookahead: int=60, update_every: int=10, budget:int=32,
        safety_distance:float=.90, seed:int=17, max_time:int=90):
    graph, agents = make_four_bridge(with_bays=True)
    rovers = {a.name:Rover(a,graph,radius=.28,wall_margin=.10) for a in agents}
    coordinator = Supervisor(rovers,lookahead=lookahead,update_every=update_every,
                             candidate_budget=budget,safety_distance=safety_distance,seed=seed)
    timeline = []
    clock = 0
    while clock < max_time:
        trial = coordinator.update(clock)
        n = min(update_every,max_time-clock)
        # The execution step is independent of the supervisor: each rover
        # replans using only its latest restriction, own goal and global map.
        planned = {name: r.local_plan(r.pos,clock,lookahead,r.restriction)
                   for name,r in rovers.items()}
        assert all(p is not None for p in planned.values())
        ok, d, _ = swept_clearance({a:p[:n+1] for a,p in planned.items()},safety_distance)
        if not ok:
            raise RuntimeError('execution preview violates swept clearance')
        before = {name:r.pos for name,r in rovers.items()}
        for name, rover in rovers.items():
            rover.execute(planned[name],n,clock)
        timeline.append({'time':clock,'positions_before':before,
                         'positions_after':{a:r.pos for a,r in rovers.items()},
                         'minimum_swept_distance_m':d, 'version':coordinator.version,
                         'executed':{name:[list(p) for p in planned[name][1:n+1]]
                                     for name in planned},
                         'restriction_payloads':{name:r.restriction.payload() for name,r in rovers.items()}})
        clock += n
        if all(r.pos == r.agent.goal for r in rovers.values()):
            break
    completed = all(r.arrived_at is not None for r in rovers.values())
    traces={name:r.trace for name,r in rovers.items()}
    min_ok,min_dist,_=swept_clearance(traces,safety_distance)
    # Independent rover baseline: global map + goals, with NO bridge time gate.
    # Goal parking cells are protected in both cases for a fair geometric test.
    bare = {a.name: Rover(a,graph,radius=.28,wall_margin=.10) for a in agents}
    unregulated={n:r.local_plan(r.pos,0,lookahead,
            RestrictionMap(1,0,lookahead,coordinator.resource,
                           AccessWindow(0,lookahead),
                           frozenset(other.agent.goal for m,other in bare.items() if m!=n)))
                 for n,r in bare.items()}
    baseline_clear, baseline_d, _ = swept_clearance(unregulated,safety_distance)
    baseline_arrivals={n:next(i for i,p in enumerate(path) if p==bare[n].agent.goal)
                       for n,path in unregulated.items()}
    return {
        'method':'hierarchical restriction supervisor + independent per-rover D* Lite / temporal A*',
        'assumptions':['1 grid cell = 1 meter; 1 step = 1 second',
                       'A/B start left; C/D start right; single-lane shared bridge',
                       'grant revisions are instant and synchronous in simulation',
                       'only the shared resource is restricted; all bank cells remain open',
                       'repulsion is soft supervisor rollout score, not physical safety proof',
                       'continuous linear swept-center pairwise and wall checks; no uncertainty model'],
        'unrestricted_baseline':{'all_rovers_arrive_if_independent':True,
                                  'arrival_time_s':baseline_arrivals,
                                  'margin_satisfied':baseline_clear,
                                  'minimum_swept_center_distance_m':baseline_d},
        'settings':{'lookahead_seconds':lookahead,'update_seconds':update_every,
                    'candidate_budget':budget,'minimum_center_distance_m':safety_distance,
                    'rover_radius_m':.28,'wall_extra_margin_m':.10,
                    'total_map_cells':len(graph.cells),
                    'resource_cells':len(coordinator.resource)},
        'completed':completed,'total_simulated_seconds':clock,
        'arrivals':{name:r.arrived_at for name,r in rovers.items()},
        'traces':{name:[list(p) for p in path] for name,path in traces.items()},
        'full_trace_clearance_pass':min_ok,'minimum_swept_distance_m':min_dist,
        'supervisor_updates':coordinator.history,'timeline':timeline,
        'supervisor_payload_contains_paths':False,
        'whole_spacetime_graph_cut_calls':0
    }


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--lookahead',type=int,default=60)
    parser.add_argument('--update',type=int,default=10)
    parser.add_argument('--budget',type=int,default=32)
    parser.add_argument('--separation',type=float,default=.90)
    parser.add_argument('--seed',type=int,default=17)
    parser.add_argument('--out',default='output/hierarchical_restriction/report.json')
    args=parser.parse_args()
    result=run(args.lookahead,args.update,args.budget,args.separation,args.seed)
    output = ROOT / args.out
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2),encoding='utf8')
    print('completed',result['completed'],'arrivals',result['arrivals'])
    print('minimum swept distance',result['minimum_swept_distance_m'])
    print('updates',[(r['now'],r['feasible_candidates'],round(r['search_seconds'],3))
                     for r in result['supervisor_updates']])
    print('saved',output)


if __name__ == '__main__':
    main()
