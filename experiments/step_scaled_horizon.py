"""No fixed priority: full-horizon XYT min-cut + path reachability feedback.

Experiment for a 2-agent, 2-corridor toy world. The time window is derived as
ceil(scale * max(unconstrained shortest path lengths)) rather than fixed seconds.

Run from the repo root: python experiments/step_scaled_horizon.py

This experiment deliberately separates three claims:
  * A plain binary min-cut using symmetric D* costs does NOT guarantee reachability.
  * Symmetric feasibility repair can enumerate either ordering of corridor
    access without assigning a fixed priority; a separate joint-state A* has
    no ordering at all and verifies the optimum on this tiny map.
  * Once paths certify feasible partition seeds, a graph cut partitions the
    space-time region; a fresh A* searches each allocated region.

The second approach is a *heuristic feasibility-repair demonstrator*, not a
proof that graph cut alone discovers alternative routes or is globally optimal.
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from itertools import permutations
from pathlib import Path
from typing import Optional

import networkx as nx

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from rolling_cut import Agent, DStarLite, GridGraph, path_conflicts  # noqa: E402

Point = tuple[int, int]
INF = float('inf')


def pad(path: list[Point], horizon: int) -> list[Point]:
    assert 1 <= len(path) <= horizon + 1
    return path + [path[-1]] * (horizon + 1 - len(path))


def astar_timed(
    graph: GridGraph,
    agent: Agent,
    dstar: DStarLite,
    horizon: int,
    *,
    reservations: Optional[list[Point]] = None,
    ownership: Optional[dict[tuple[Point, int], str]] = None,
) -> Optional[list[Point]]:
    """Earliest-arrival A* over (XY, discrete t), with terminal cost D* g.

    When a robot reaches goal it holds position through the remaining horizon.
    Reservations exclude vertex collisions and opposing-edge swaps. Ownership
    constrains the robot to cells assigned to it by the space-time min cut.
    """
    if reservations is not None and agent.start == reservations[0]:
        return None
    if ownership is not None and ownership.get((agent.start, 0)) != agent.name:
        return None
    heap = [(dstar.g[agent.start], 0, 0, agent.start)]
    parent: dict[tuple[int, Point], Optional[tuple[int, Point]]] = {(0, agent.start): None}
    count = 0
    while heap:
        f, t, _, p = heapq.heappop(heap)
        if t + dstar.g[p] > horizon:
            continue
        if p == agent.goal:
            route: list[Point] = []
            state: Optional[tuple[int, Point]] = (t, p)
            while state is not None:
                route.append(state[1])
                state = parent[state]
            route.reverse()
            return route
        if t >= horizon:
            continue
        q_candidates = sorted((*graph.neighbors(p), p))
        for q in q_candidates:
            n_t = t + 1
            if reservations is not None:
                # Reservation persists at the other robot's goal.
                other_now = reservations[t]
                other_next = reservations[n_t]
                if q == other_next or (p == other_next and q == other_now):
                    continue
            if ownership is not None and ownership.get((q, n_t)) != agent.name:
                continue
            if q in dstar.blocked:
                continue
            key = (n_t, q)
            if key in parent:
                continue
            if n_t + dstar.g[q] > horizon:
                continue
            parent[key] = (t, p)
            count += 1
            heapq.heappush(heap, (n_t + dstar.g[q], n_t, count, q))
    return None


def graphcut_xyt(
    graph: GridGraph,
    agents: list[Agent],
    dstars: dict[str, DStarLite],
    horizon: int,
    *,
    required: Optional[dict[str, list[Point]]] = None,
    spatial_lambda: float = 0.45,
    temporal_lambda: float = 0.6,
    hard_weight: float = 10000.0,
) -> tuple[dict[tuple[Point, int], str], dict]:
    """Symmetric binary s/t min-cut. No corridor lock or priority variable.

    ``required`` can pin an already collision-checked feasible path witness
    for each agent. This is an explicit reachability feedback step, NOT a
    consequence of the binary graph-cut objective alone.
    """
    a, b = agents
    f = nx.DiGraph()
    source, sink = 'source_A', 'sink_B'
    from_start = {z.name: graph.distances(z.start) for z in agents}
    f.add_nodes_from([source, sink])
    nodes = {(p, t): (p[0], p[1], t) for p in graph.cells for t in range(horizon + 1)}

    for (p, t), node in nodes.items():
        costs = {}
        for z in agents:
            # Spatial costs-to-go from D* Lite, no agent-specific weights.
            total = from_start[z.name].get(p, 10**4) + dstars[z.name].g[p]
            costs[z.name] = 0.045 * total
        if t == 0 and p == a.start:
            costs[b.name] += hard_weight
        if t == 0 and p == b.start:
            costs[a.name] += hard_weight
        # The goal is admissible at ANY arrival time. Do not artificially pin
        # its cell before the earliest possible arrival (unlike the original
        # prototype); after that, it is reserved because arrival is terminal.
        if p == a.goal and t >= dstars[a.name].g[a.start]:
            costs[b.name] += hard_weight
        if p == b.goal and t >= dstars[b.name].g[b.start]:
            costs[a.name] += hard_weight
        if required is not None:
            if required[a.name][t] == p:
                costs[b.name] += hard_weight
            if required[b.name][t] == p:
                costs[a.name] += hard_weight
        # SOURCE-side is A: source->node is penalty for assigning B;
        # node->sink is penalty for assigning A.
        f.add_edge(source, node, capacity=costs[b.name])
        f.add_edge(node, sink, capacity=costs[a.name])

    for (p, t), node in nodes.items():
        for q in graph.neighbors(p):
            if p < q:
                neighbor = nodes[(q, t)]
                f.add_edge(node, neighbor, capacity=spatial_lambda)
                f.add_edge(neighbor, node, capacity=spatial_lambda)
        if t < horizon:
            other_t = nodes[(p, t + 1)]
            f.add_edge(node, other_t, capacity=temporal_lambda)
            f.add_edge(other_t, node, capacity=temporal_lambda)

    # Use Edmonds-Karp for the small demonstrator. On this graph, the default
    # preflow solver yielded an inconsistent min-cut partition with 1e7 seeds.
    value, (src_side, dst_side) = nx.minimum_cut(
        f, source, sink, capacity='capacity', flow_func=nx.algorithms.flow.edmonds_karp
    )
    actual = sum(data['capacity'] for u, v, data in f.edges(data=True)
                 if u in src_side and v in dst_side)
    if not math.isclose(value, actual, abs_tol=1e-5, rel_tol=1e-9):
        raise RuntimeError('min-cut solver returned inconsistent cut value')
    labels = {(p, t): a.name if node in src_side else b.name
              for (p, t), node in nodes.items()}
    info = {
        'value': round(value, 5),
        'label_counts': {name: sum(z == name for z in labels.values())
                         for name in (a.name, b.name)},
        'hard_path_witnesses': bool(required),
        'nodes': len(nodes),
    }
    return labels, info



def joint_astar(
    graph: GridGraph,
    agents: list[Agent],
    dstars: dict[str, DStarLite],
    horizon: int,
) -> Optional[dict]:
    """Small two-agent *joint-state* A* baseline, no order or priority.

    Optimal for sum of arrival steps on this discretized two-agent graph with
    vertex/edge-swap constraints, held goal occupancy and unit time per move.
    This is a research comparator, not a scalable substitute for MAPF methods.
    """
    a, b = agents
    h_a, h_b = dstars[a.name].g, dstars[b.name].g
    initial = (0, a.start, b.start)
    best = {initial: 0}
    parent: dict[tuple[int, Point, Point], Optional[tuple[int, Point, Point]]] = {
        initial: None
    }
    queue = [(h_a[a.start] + h_b[b.start], 0, 0, a.start, b.start)]
    expansions = 0
    while queue:
        _, cost, t, pa, pb = heapq.heappop(queue)
        state = (t, pa, pb)
        if cost != best.get(state):
            continue
        expansions += 1
        if pa == a.goal and pb == b.goal:
            paired = []
            while state is not None:
                paired.append((state[1], state[2]))
                state = parent[state]
            paired.reverse()
            paths = {
                a.name: [p[0] for p in paired],
                b.name: [p[1] for p in paired],
            }
            arrival = {z.name: next(t for t,p in enumerate(paths[z.name]) if p==z.goal)
                       for z in agents}
            assert sum(arrival.values()) == cost
            return {'arrival_steps': arrival, 'sum_of_costs': cost,
                    'makespan': max(arrival.values()),
                    'expanded_states': expansions,
                    'paths': paths}
        if t >= horizon:
            continue
        # A robot stays at its goal. All pair actions are considered together;
        # neither is planned first and there is no priority coefficient.
        possible_a = [pa] if pa == a.goal else [pa, *graph.neighbors(pa)]
        possible_b = [pb] if pb == b.goal else [pb, *graph.neighbors(pb)]
        for qa in possible_a:
            for qb in possible_b:
                if qa == qb or (qa == pb and qb == pa):
                    continue
                new_cost = cost + (pa != a.goal) + (pb != b.goal)
                next_state = (t + 1, qa, qb)
                if new_cost >= best.get(next_state, INF):
                    continue
                if t + 1 + max(h_a[qa], h_b[qb]) > horizon:
                    continue
                best[next_state] = new_cost
                parent[next_state] = (t, pa, pb)
                bound = new_cost + h_a[qa] + h_b[qb]
                heapq.heappush(queue, (bound, new_cost, t + 1, qa, qb))
    return None


def feasibility_branches(
    graph: GridGraph,
    agents: list[Agent],
    dstars: dict[str, DStarLite],
    horizon: int,
) -> list[dict]:
    """Enumerate both symmetric alternatives, not a fixed-priority rule.

    Construct one shortest-path witness, then find the other with time-indexed
    reservations. This is an explicit feasibility-repair *search*; Graph Cut
    then allocates regions around both witnesses and both are re-searched.
    """
    branches = []
    lookup = {a.name: a for a in agents}
    for first, second in permutations(agents, 2):
        first_route = astar_timed(graph, first, dstars[first.name], horizon)
        if first_route is None:
            branches.append({'first': first.name, 'status': 'first-unreachable'})
            continue
        second_route = astar_timed(
            graph, second, dstars[second.name], horizon,
            reservations=pad(first_route, horizon)
        )
        if second_route is None:
            branches.append({'first': first.name, 'status': 'second-unreachable'})
            continue
        required = {first.name: pad(first_route, horizon),
                    second.name: pad(second_route, horizon)}
        assert not path_conflicts(required, [a.name for a in agents])
        labels, info = graphcut_xyt(graph, agents, dstars, horizon, required=required)
        # The vertex-wise cut cannot prevent opposing-edge swaps by itself.
        # Rerun the second search with the first *computed* path as a space-time
        # reservation. Both possible orders are examined symmetrically below.
        first_in_cut = astar_timed(
            graph, first, dstars[first.name], horizon, ownership=labels)
        second_in_cut = (None if first_in_cut is None else astar_timed(
            graph, second, dstars[second.name], horizon,
            ownership=labels, reservations=pad(first_in_cut, horizon)))
        computed = {first.name:first_in_cut, second.name:second_in_cut}
        if any(computed[z.name] is None for z in agents):
            branches.append({'first': first.name, 'status': 'cut-disconnected', 'cut': info})
            continue
        computed_full = {name: pad(p, horizon) for name, p in computed.items()}
        conflicts = path_conflicts(computed_full, [a.name for a in agents])
        if conflicts:
            branches.append({'first': first.name, 'status': 'conflict', 'conflicts': conflicts})
            continue
        arrivals = {name: len(computed[name]) - 1 for name in computed}
        route_types = {name: ('lower' if any(y == 6 for x, y in computed[name]) else 'upper')
                       for name in computed}
        branches.append({
            'first': first.name,
            'status': 'feasible',
            'arrival_steps': arrivals,
            'sum_of_costs': sum(arrivals.values()),
            'makespan': max(arrivals.values()),
            'route_types': route_types,
            'cut': info,
            'witness': {name: [list(p) for p in required[name]] for name in required},
            'replanned_paths': {name: [list(p) for p in computed_full[name]] for name in computed_full},
            'labels': labels,
        })
    return branches


def run(scale: float = 2.0) -> dict:
    graph = GridGraph.two_corridors()
    agents = [Agent('A', (0, 2), (14, 2), 1.0),
              Agent('B', (14, 3), (0, 3), 1.0)]
    dstars = {a.name: DStarLite(graph, a.start, a.goal) for a in agents}
    independent = {a.name: dstars[a.name].path() for a in agents}
    max_shortest = max(len(path) - 1 for path in independent.values())
    horizon = math.ceil(max_shortest * scale)
    if horizon < max_shortest:
        raise ValueError('scale must produce horizon >= max shortest path')

    original_labels, initial_cut = graphcut_xyt(graph, agents, dstars, horizon)
    initial_paths = {a.name: astar_timed(graph, a, dstars[a.name], horizon,
                                        ownership=original_labels)
                     for a in agents}
    # Independent check: symmetric joint-state A* has no priority/order and
    # provides the globally lowest sum of arrivals for this tiny 2-agent case.
    joint = joint_astar(graph, agents, dstars, horizon)
    branches = feasibility_branches(graph, agents, dstars, horizon)
    accepted = [b for b in branches if b['status'] == 'feasible']
    # Select the lowest total flight time; ties resolved by makespan/cut value.
    # This does not privilege A or B in the optimization objective.
    selected = min(accepted, key=lambda b: (b['sum_of_costs'], b['makespan'],
                                            b['cut']['value'])) if accepted else None

    def cut_slices(labels):
        return {str(t): {a.name: [list(p) for p in sorted(graph.cells)
                                 if labels[(p, t)] == a.name] for a in agents}
                for t in sorted(set([0, horizon // 4, horizon // 2, 3 * horizon // 4, horizon]))}

    report = {
        'source_commit': 'd2f6bf80c9636524c6e33160a1549f56700db508',
        'parameters': {'horizon_factor': scale, 'max_independent_steps': max_shortest,
                       'horizon_steps': horizon, 'grid_cells': len(graph.cells),
                       'space_time_nodes': len(graph.cells) * (horizon + 1)},
        'independent': {name: {'steps': len(p)-1, 'path': [list(v) for v in p]}
                        for name, p in independent.items()},
        'pure_symmetric_cut': {
            'cut': initial_cut,
            'reachable': {name: p is not None for name, p in initial_paths.items()},
            'arrival_steps': {name: len(p)-1 if p is not None else None
                              for name, p in initial_paths.items()},
            'slices': cut_slices(original_labels),
        },
        'joint_astar_no_priority': None if joint is None else {
            k: v if k != 'paths' else {name: [list(p) for p in path] for name,path in v.items()}
            for k,v in joint.items()
        },
        'reachability_feedback': {
            'branch_summaries': [{k:v for k,v in b.items() if k not in ('witness','replanned_paths','labels')}
                                 for b in branches],
            'selected': None if selected is None else {
                k: v for k,v in selected.items() if k != 'labels'
            },
            'selected_cut_slices': None if selected is None else cut_slices(selected['labels']),
        },
        'caveat': 'Reciprocal timed A* branches supply feasible seed paths before graph cut; '
                  'the initial symmetric cut alone cannot guarantee connectivity. '
                  'This is a heuristic demonstration, not a complete MAPF or collision safety proof.',
    }
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--factor', type=float, default=2.0)
    ap.add_argument('--out', type=Path, default=Path('output/step_scaled_no_priority.json'))
    args = ap.parse_args()
    report = run(args.factor)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print('Independent shortest:', {n:v['steps'] for n,v in report['independent'].items()})
    print('Horizon:',report['parameters']['horizon_steps'])
    print('Naive cut reaches goal:',report['pure_symmetric_cut']['reachable'])
    print('Joint A* no priority:',None if report['joint_astar_no_priority'] is None
          else {k:report['joint_astar_no_priority'][k]
                for k in ('arrival_steps','sum_of_costs','expanded_states')})
    for branch in report['reachability_feedback']['branch_summaries']:
        print('Repair branch:',branch)
    sel = report['reachability_feedback']['selected']
    print('Selected:',None if sel is None else {
        k:sel[k] for k in ('first','arrival_steps','sum_of_costs','makespan','route_types')})
    print('Saved',args.out)


if __name__=='__main__':
    main()
