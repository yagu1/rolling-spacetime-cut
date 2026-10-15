"""Connectivity-aware space-time cut experiments (2 corridors / single bridge).

This is a small, auditable research experiment, NOT a collision avoidance system.

Important distinction:
    A binary graph cut enforces start/goal labels, not directed connectivity.
    We obtain a collision-checked two-agent *joint A-star* path witness first,
    pin its occupied XYT nodes, run a symmetric binary graph cut, and search
    paths again in the resulting ownership regions. The witness guarantees
    existence of a directed start-to-goal path within each pinned region;
    the min-cut itself does not discover or guarantee that path.

The bridge without holding bays is intentionally infeasible for two agents
exchanging endpoints on a one-cell-wide topological path. Time alone cannot
reverse their ordering without vertex/edge-swap conflicts.

Run: python experiments/connected_cut_bridge.py
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import sys

import networkx as nx

from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rolling_cut import Agent, DStarLite, GridGraph, path_conflicts  # noqa: E402
from experiments.step_scaled_horizon import astar_timed, pad  # noqa: E402

Point = tuple[int, int]
INF = math.inf


def make_scenario(name: str) -> tuple[GridGraph, list[Agent]]:
    """Bridge has one central lane and side holding/exit spaces at both banks."""
    if name == 'two_corridors':
        return GridGraph.two_corridors(), [
            Agent('A', (0, 2), (14, 2)),
            Agent('B', (14, 3), (0, 3)),
        ]
    if name == 'bridge_with_bays':
        # One horizontal line x=2..10,y=2. The y=1/y=3 pocket cells at
        # x=1,2 and x=10,11 give each agent somewhere to wait/exit.
        cells = {(x, 2) for x in range(2, 11)}
        cells.update({(1, 1), (1, 2), (1, 3), (2, 1), (2, 3),
                      (10, 1), (10, 3), (11, 1), (11, 2), (11, 3)})
        return GridGraph(cells), [
            Agent('A', (1, 1), (11, 1)),
            Agent('B', (11, 3), (1, 3)),
        ]
    if name == 'bridge_no_bays':
        # Topological path, no spare node on which agents can pass.
        return GridGraph({(x, 2) for x in range(9)}), [
            Agent('A', (0, 2), (8, 2)),
            Agent('B', (8, 2), (0, 2)),
        ]
    raise ValueError(f'Unknown scenario: {name}')


def joint_astar_tiebreak(
    graph: GridGraph,
    agents: list[Agent],
    dstars: dict[str, DStarLite],
    horizon: int,
    favor: Optional[str] = None,
) -> Optional[dict]:
    """Exact two-agent joint A* on the finite discrete horizon.

    Primary objective: min sum of arrival times. ``favor`` adds a tiny
    secondary arrival-time bias, smaller than any 1-step primary difference;
    there is NEVER an exclusive corridor reservation or prescribed ordering.
    Returned timing is exact for the discrete model and stated constraints.
    """
    a, b = agents
    if favor not in (None, a.name, b.name):
        raise ValueError(f'favor must be None, {a.name}, or {b.name}')
    if a.start == b.start:
        return None
    eps = 0.0 if favor is None else 1.0 / (10.0 * (horizon + 1))
    w_a = 1.0 + (eps if favor == a.name else 0.0)
    w_b = 1.0 + (eps if favor == b.name else 0.0)
    h_a, h_b = dstars[a.name].g, dstars[b.name].g

    state0 = (0, a.start, b.start)
    best: dict[tuple[int, Point, Point], float] = {state0: 0.0}
    parent: dict[tuple[int, Point, Point], Optional[tuple[int, Point, Point]]] = {state0: None}
    counter = 0
    heap = [(w_a*h_a[a.start] + w_b*h_b[b.start], 0.0, counter, state0)]
    expanded = 0
    while heap:
        _, cost, _, state = heapq.heappop(heap)
        if cost > best.get(state, INF) + 1e-10:
            continue
        t, pa, pb = state
        expanded += 1
        if pa == a.goal and pb == b.goal:
            paired = []
            while state is not None:
                paired.append((state[1], state[2]))
                state = parent[state]
            paired.reverse()
            paths = {a.name: [p[0] for p in paired],
                     b.name: [p[1] for p in paired]}
            arrival = {z.name: next(i for i,p in enumerate(paths[z.name]) if p == z.goal)
                       for z in agents}
            assert not path_conflicts(paths, [a.name, b.name])
            return {
                'paths': paths,
                'arrival_steps': arrival,
                'sum_of_costs': sum(arrival.values()),
                'makespan': max(arrival.values()),
                'expanded_states': expanded,
                'tie_breaker': favor or 'none',
                'weighted_objective': cost,
            }
        if t >= horizon:
            continue
        if t + max(h_a[pa], h_b[pb]) > horizon:
            continue
        nxt_a = [pa] if pa == a.goal else sorted((pa, *graph.neighbors(pa)))
        nxt_b = [pb] if pb == b.goal else sorted((pb, *graph.neighbors(pb)))
        for qa in nxt_a:
            for qb in nxt_b:
                if qa == qb:  # collision at the next time slice
                    continue
                if qa == pb and qb == pa:  # head-on edge swap
                    continue
                step = (w_a if pa != a.goal else 0.0) + (w_b if pb != b.goal else 0.0)
                score = cost + step
                nxt = (t + 1, qa, qb)
                if score >= best.get(nxt, INF) - 1e-10:
                    continue
                if t+1 + max(h_a[qa], h_b[qb]) > horizon:
                    continue
                best[nxt] = score
                parent[nxt] = state
                counter += 1
                f = score + w_a*h_a[qa] + w_b*h_b[qb]
                heapq.heappush(heap, (f, score, counter, nxt))
    return None



def graphcut_end_terminal_seeds(
    graph: GridGraph, agents: list[Agent], dstars: dict[str,DStarLite],
    horizon: int, *, required: Optional[dict[str,list[Point]]] = None,
    spatial_lambda:float=0.45, temporal_lambda:float=0.6, hard_weight:float=10000.,
) -> tuple[dict[tuple[Point,int],str],dict]:
    """XYT min-cut with EXACT start-at-0 and goal-at-H terminal seeds.

    Since GOAL is absorbing (goal wait has zero extra arrival cost), reaching
    it at ANY time <=H is equivalent to being at GOAL on the final t=H plane.
    Thus we do not incorrectly pin GOAL at *every earlier* t.

    Binary cut itself does not impose directed reachability. If ``required``
    is provided, it must be an already feasible pair of time-indexed paths.
    Those witness cells become hard seeds to guarantee existence.
    """
    a,b=agents
    names=[a.name,b.name]
    start_dist={agent.name:graph.distances(agent.start) for agent in agents}
    src='source_A';snk='sink_B'
    flow=nx.DiGraph()
    flow.add_nodes_from([src,snk])
    nodes={(p,t):(p[0],p[1],t) for p in graph.cells for t in range(horizon+1)}
    for (p,t),v in nodes.items():
        ac=0.045*(start_dist[a.name].get(p,10**4)+dstars[a.name].g[p])
        bc=0.045*(start_dist[b.name].get(p,10**4)+dstars[b.name].g[p])
        if (t==0 and p==a.start) or (t==horizon and p==a.goal):
            bc += hard_weight
        if (t==0 and p==b.start) or (t==horizon and p==b.goal):
            ac += hard_weight
        if required is not None:
            if required[a.name][t] == p: bc += hard_weight
            if required[b.name][t] == p: ac += hard_weight
        flow.add_edge(src,v,capacity=bc)
        flow.add_edge(v,snk,capacity=ac)
    for (p,t),v in nodes.items():
        for q in graph.neighbors(p):
            if p<q:
                w=nodes[(q,t)]
                flow.add_edge(v,w,capacity=spatial_lambda)
                flow.add_edge(w,v,capacity=spatial_lambda)
        if t<horizon:
            w=nodes[(p,t+1)]
            flow.add_edge(v,w,capacity=temporal_lambda)
            flow.add_edge(w,v,capacity=temporal_lambda)
    value,(source_side,sink_side)=nx.minimum_cut(
        flow,src,snk,capacity='capacity',flow_func=nx.algorithms.flow.edmonds_karp)
    actual=sum(data['capacity'] for u,v,data in flow.edges(data=True)
               if u in source_side and v in sink_side)
    if not math.isclose(actual,value,abs_tol=1e-5,rel_tol=1e-9):
        raise RuntimeError('inconsistent cut capacity')
    labels={(p,t):a.name if node in source_side else b.name
            for (p,t),node in nodes.items()}
    info={'value':round(value,5), 'nodes':len(nodes),
          'label_counts':{k:sum(v==k for v in labels.values()) for k in names},
          'hard_path_witnesses':required is not None,
          'terminal_pins':'Start at t=0, Goal at t=H only; earlier arrival waits at goal',
          'temporal_lambda':temporal_lambda,'spatial_lambda':spatial_lambda}
    return labels,info

def directed_core(
    graph: GridGraph, labels: dict[tuple[Point, int], str],
    agent: Agent, horizon: int,
) -> dict:
    """Forward-from-start and backward-to-any-goal reachability in XYT.

    Their intersection contains precisely nodes lying on at least one directed
    start-to-goal walk *within that label*. Isolated assigned nodes are NOT
    falsely advertised as a connected safe corridor. The goal is absorbing.
    """
    name = agent.name
    forward: set[tuple[Point, int]] = set()
    if labels.get((agent.start, 0)) == name:
        forward.add((agent.start, 0))
    for t in range(horizon):
        for p in graph.cells:
            if (p, t) not in forward:
                continue
            moves = (p,) if p == agent.goal else (p, *graph.neighbors(p))
            for q in moves:
                if labels.get((q, t + 1)) == name:
                    forward.add((q, t + 1))

    backward: set[tuple[Point, int]] = {
        (agent.goal, t) for t in range(horizon + 1)
        if labels.get((agent.goal, t)) == name
    }
    for t in range(horizon - 1, -1, -1):
        for p in graph.cells:
            if labels.get((p, t)) != name:
                continue
            moves = (p,) if p == agent.goal else (p, *graph.neighbors(p))
            if any((q, t + 1) in backward for q in moves):
                backward.add((p, t))
    core = forward & backward
    assigned = sum(1 for owner in labels.values() if owner == name)
    return {
        'start_reaches_goal': any((agent.goal, t) in forward for t in range(horizon + 1)),
        'assigned_nodes': assigned,
        'forward_nodes': len(forward),
        'directed_viable_nodes': len(core),
        'nonviable_assigned_nodes': assigned - len(core),
        'core_cells_by_t': [
            [[p[0], p[1]] for p in sorted(graph.cells) if (p, t) in core]
            for t in range(horizon + 1)
        ],
    }


def label_frames(graph: GridGraph, labels: dict[tuple[Point, int], str],
                 horizon: int) -> list[dict]:
    out = []
    for t in range(horizon + 1):
        frame = {'t': t, 'A': [], 'B': [], 'boundary_edges': [],
                 'spatial_boundary_count': 0, 'temporal_switch_count': 0}
        for p in sorted(graph.cells):
            frame[labels[(p, t)]].append([p[0], p[1]])
            for q in graph.neighbors(p):
                if p < q and labels[(p, t)] != labels[(q, t)]:
                    frame['boundary_edges'].append([list(p), list(q)])
            if t > 0 and labels[(p, t)] != labels[(p, t - 1)]:
                frame['temporal_switch_count'] += 1
        frame['spatial_boundary_count'] = len(frame['boundary_edges'])
        out.append(frame)
    return out


def verify_seed_paths(
    graph: GridGraph, agents: list[Agent], labels: dict[tuple[Point, int], str],
    required: dict[str, list[Point]], horizon: int,
) -> None:
    for agent in agents:
        path = required[agent.name]
        assert len(path) == horizon + 1
        assert path[0] == agent.start
        assert agent.goal in path
        for t, p in enumerate(path):
            assert labels[(p, t)] == agent.name, (agent.name, t, p)
            if t:
                assert p == path[t-1] or p in graph.neighbors(path[t-1])
    assert path_conflicts(required, [a.name for a in agents]) == []


def get_report(name: str, *, scale: float = 2.0, favor: Optional[str] = None,
               include_pure: bool = True) -> dict:
    graph, agents = make_scenario(name)
    dstars = {a.name: DStarLite(graph, a.start, a.goal) for a in agents}
    independent = {a.name: dstars[a.name].path() for a in agents}
    max_steps = max(len(p) - 1 for p in independent.values())
    horizon = math.ceil(max_steps * scale)
    if horizon < max_steps:
        raise ValueError('horizon must be >= longest single-agent path')
    common = {
        'scenario': name,
        'grid': [list(p) for p in sorted(graph.cells)],
        'agents': {a.name: {
            'start': list(a.start), 'goal': list(a.goal),
            'independent_steps': len(independent[a.name]) - 1,
            'independent_route': [list(p) for p in independent[a.name]],
        } for a in agents},
        'horizon_steps': horizon,
        'horizon_factor': scale,
        'space_time_nodes': len(graph.cells) * (horizon + 1),
    }
    if include_pure:
        bare_labels, bare_info = graphcut_end_terminal_seeds(graph, agents, dstars, horizon)
        bare_paths = {a.name: astar_timed(graph, a, dstars[a.name], horizon,
                                          ownership=bare_labels) for a in agents}
        common['bare_cut'] = {
            'info': bare_info,
            'reachable': {name: path is not None for name, path in bare_paths.items()},
            'frames': label_frames(graph, bare_labels, horizon),
        }
    joint = joint_astar_tiebreak(graph, agents, dstars, horizon, favor)
    if joint is None:
        common['joint'] = None
        common['connected_cut'] = None
        common['conclusion'] = (
            'No collision-free goal-swap path within the tested time horizon. '
            'For bridge_no_bays the underlying one-cell-wide path has fixed '
            'agent order; exchanging endpoints is impossible at ANY horizon '
            'without an extra passing/holding branch or another operation.'
        )
        return common

    required = {k: pad(v, horizon) for k, v in joint['paths'].items()}
    # Joint-state A* already validates movement and collision constraints.
    labels, info = graphcut_end_terminal_seeds(graph, agents, dstars, horizon, required=required)
    verify_seed_paths(graph, agents, labels, required, horizon)
    cores = {a.name: directed_core(graph, labels, a, horizon) for a in agents}
    if not all(v['start_reaches_goal'] for v in cores.values()):
        raise AssertionError('Pinned cut unexpectedly lost directed start-goal path')

    replanned = {a.name: astar_timed(graph, a, dstars[a.name], horizon,
                                    ownership=labels) for a in agents}
    recovered = all(p is not None for p in replanned.values())
    replanned_conflicts = None
    if recovered:
        replanned_conflicts = path_conflicts(
            {name: pad(p, horizon) for name, p in replanned.items()},
            [a.name for a in agents]
        )
        # If independently re-searched paths happen to edge-swap (possible
        # even under exclusive XYT vertex labels), keep the validated witness
        # as the explicit safe result; do not claim safety of the new paths.
    common['joint'] = {
        'arrival_steps': joint['arrival_steps'],
        'sum_of_costs': joint['sum_of_costs'],
        'makespan': joint['makespan'],
        'expanded_states': joint['expanded_states'],
        'tie_breaker': joint['tie_breaker'],
        'paths': {name: [list(p) for p in path] for name, path in required.items()},
    }
    common['connected_cut'] = {
        'info': info,
        'path_seeded': True,
        'guarantee': 'Directed start-to-goal reachability of the pinned path '
                     'witnesses only, NOT global connectedness of all labels.',
        'frames': label_frames(graph, labels, horizon),
        'cores': cores,
        'replanned_reachable': {n: p is not None for n,p in replanned.items()},
        'replanned_arrival_steps': {
            n: len(p) - 1 if p is not None else None for n,p in replanned.items()
        },
        'replanned_conflicts': replanned_conflicts,
        'replanned_paths': None if not recovered else {
            n: [list(v) for v in pad(p, horizon)] for n, p in replanned.items()
        },
    }
    return common


def bridge_temporal_variant(base: dict, temporal_lambda: float) -> dict:
    """Re-cut the SAME connected witnesses with a different temporal penalty."""
    graph, agents = make_scenario('bridge_with_bays')
    horizon = base['horizon_steps']
    dstars = {a.name: DStarLite(graph, a.start, a.goal) for a in agents}
    required = {k: [tuple(p) for p in v] for k,v in base['joint']['paths'].items()}
    labels, info = graphcut_end_terminal_seeds(graph, agents, dstars, horizon,
                                 required=required, temporal_lambda=temporal_lambda)
    verify_seed_paths(graph, agents, labels, required, horizon)
    rows = label_frames(graph, labels, horizon)
    return {
        'temporal_lambda': temporal_lambda, 'info': info,
        'frames': rows,
        'cores': {a.name: directed_core(graph, labels, a, horizon) for a in agents},
        'max_bridge_cells_switched_in_one_step': max(
            sum(labels[((x,2),t)] != labels[((x,2),t-1)] for x in range(2,11))
            for t in range(1,horizon+1)),
        'time_slices_with_fragmented_bridge_labels': sum(
            sum(labels[((x,2),t)] != labels[((x+1,2),t)] for x in range(2,10)) > 1
            for t in range(horizon+1)),
    }


def run_all(scale: float = 2.0) -> dict:
    scenarios = {
        'two_corridors': get_report('two_corridors', scale=scale),
        'bridge_with_bays': get_report('bridge_with_bays', scale=scale),
        'bridge_no_bays': get_report('bridge_no_bays', scale=scale),
    }
    return {
        'model': 'unit grid steps; synchronous one-step moves/wait; vertex and head-on swap exclusions; goals absorb',
        'claim_limitations': [
            'Binary min-cut is only a labeling method and does not impose path connectivity.',
            'A two-agent joint A* supplies the feasible seed paths BEFORE the connected cut.',
            'Start-to-goal existence is certified, not every labeled vertex being on a usable path.',
            'Only discrete vertex and head-on swap conflicts are checked; no continuous swept-volume separation guarantee.',
            'With no passing branches, opposite endpoint exchange on a one-cell-wide path is impossible regardless of horizon.',
        ],
        'scenarios': scenarios,
        'bridge_temporal_smoothing': bridge_temporal_variant(scenarios['bridge_with_bays'], 3.0),
        'bridge_tie_break': {
            'favor_A': get_report('bridge_with_bays', scale=scale, favor='A', include_pure=False),
            'favor_B': get_report('bridge_with_bays', scale=scale, favor='B', include_pure=False),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--factor', type=float, default=2.0)
    parser.add_argument('--out', type=Path, default=Path('output/connected_cut/report.json'))
    args = parser.parse_args()
    report = run_all(scale=args.factor)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf8')
    for name, entry in report['scenarios'].items():
        print(f'[{name}] XYT nodes={entry["space_time_nodes"]}, horizon={entry["horizon_steps"]}')
        print('  bare cut reachable:',entry['bare_cut']['reachable'])
        print('  joint:',None if entry['joint'] is None else entry['joint']['arrival_steps'])
        if entry['connected_cut']:
            print('  connected cut:', {n: c['start_reaches_goal'] for n,c in
                                      entry['connected_cut']['cores'].items()})
            print('  replanned:',entry['connected_cut']['replanned_arrival_steps'],
                  'conflicts:',entry['connected_cut']['replanned_conflicts'])
    print('Bridge tie breaking:',{name: r['joint']['arrival_steps'] if r['joint'] else None
                                  for name,r in report['bridge_tie_break'].items()})
    print('Temporal smoothing:', {k: report['bridge_temporal_smoothing'][k] for k in
        ('max_bridge_cells_switched_in_one_step', 'time_slices_with_fragmented_bridge_labels')})
    print('Saved:',args.out)

if __name__ == '__main__':
    main()
