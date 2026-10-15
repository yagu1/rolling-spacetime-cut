"""Four-agent single-bridge proof-of-concept regression tests."""
import itertools
import pytest

from experiments.four_agent_bridge import (
    make_four_bridge, independent_dstar, route_by_enumerated_orders,
    run_four, conflict_details, unary_costs, alpha_move, compute_energy,
)


@pytest.fixture(scope='module')
def four_report():
    return run_four(factor=2.,cut_passes=2)


def test_scenario_initial_length_and_time_nodes(four_report):
    r=four_report
    assert r['space_time_nodes']==27*29==783
    assert all(a['independent_steps']==14 for a in r['agents'].values())
    assert len(r['independent_path_conflicts'])>0
    assert r['horizon']==28


def test_all_four_reach_goals_and_never_collide(four_report):
    r=four_report;s=r['schedule'];c=r['multiway_cut']
    assert s['tested_orders']==24 and s['feasible_orders']==8
    assert s['sum_of_arrivals']==82 and s['makespan']==27
    assert s['arrival_steps']==dict(A=14,B=15,C=26,D=27)
    assert all(c['start_goal_directed_reachable'].values())
    assert c['replanned_path_conflicts']==[]
    assert conflict_details(s['paths'])==[]
    assert conflict_details(c['replanned_paths'])==[]
    names=list(r['agents'])
    for t,frame in enumerate(c['frames']):
        owner={tuple(p):name for name,points in frame['regions'].items() for p in points}
        assert len(owner)==27
        assert set(owner)=={tuple(p) for p in r['grid']}
        for name in names:
            pos=tuple(s['paths'][name][t])
            assert owner[pos]==name
            assert tuple(c['replanned_paths'][name][t]) in owner
            assert owner[tuple(c['replanned_paths'][name][t])]==name
    history=c['statistics']['energy_history']
    assert all(a>b for a,b in zip(history,history[1:]))


def test_bridge_is_monotonic_in_direction(four_report):
    for name,seq in four_report['schedule']['paths'].items():
        forward=1 if name in 'AB' else -1
        for a,b in zip(seq,seq[1:]):
            if a[1]==2 and 3<=a[0]<=11:
                assert (b[0]-a[0])*forward >= 0


def test_bare_four_way_cut_does_not_preserve_four_reachable_paths(four_report):
    r=four_report
    assert any(not ok for ok in r['unseeded_cut']['start_goal_directed_reachable'].values())
    assert r['unseeded_cut']['statistics']['seeded_witness_paths'] is False
    assert r['multiway_cut']['statistics']['seeded_witness_paths'] is True


def test_symmetric_equivalent_schedules():
    graph,agents=make_four_bridge()
    ds,_=independent_dstar(graph,agents)
    left=route_by_enumerated_orders(graph,agents,ds,28,tie_side='left')
    right=route_by_enumerated_orders(graph,agents,ds,28,tie_side='right')
    assert (left['sum_of_arrivals'],left['makespan'])==(82,27)
    assert (right['sum_of_arrivals'],right['makespan'])==(82,27)
    assert left['planning_order'][0] in ('A','B')
    assert right['planning_order'][0] in ('C','D')
    assert right['arrival_steps']==dict(C=14,D=15,A=26,B=27)
    assert conflict_details(left['paths'])==[]
    assert conflict_details(right['paths'])==[]


def test_monolane_without_bays_infeasible_topological_exchange():
    r=run_four(with_bays=False)
    assert r['feasible'] is False
    assert 'order' in r['proof']


def test_short_horizon_ordered_search_not_found():
    graph,agents=make_four_bridge()
    ds,_=independent_dstar(graph,agents)
    r=route_by_enumerated_orders(graph,agents,ds,21)
    assert not r['found']
    assert 'not a proof' in r['explanation']


def test_alpha_expansion_two_nodes_agrees_with_exhaustive_binary_move():
    a=((0,0),0);b=((1,0),0)
    nodes=[a,b]
    for l0,l1,alpha in itertools.product('ABC',repeat=3):
        unary={a:{'A':1,'B':7,'C':4},b:{'A':5,'B':0,'C':9}}
        edges=[(a,b,11)]
        labels={a:l0,b:l1}
        result=alpha_move(labels,alpha,unary,edges,pinned={})
        outcome=compute_energy(result,unary,edges)
        expected=min(compute_energy({a:aa,b:bb},unary,edges)
                     for aa in (l0,alpha) for bb in (l1,alpha))
        assert outcome==expected,(l0,l1,alpha,outcome,expected)


def test_alpha_expansion_honors_hard_pin():
    a=((0,0),0);b=((1,0),0)
    unary={a:{k:0 for k in 'ABCD'},b:{k:0 for k in 'ABCD'}}
    out=alpha_move({a:'A',b:'B'},'B',unary,[(a,b,10)],pinned={a:'A',b:'B'})
    assert out=={a:'A',b:'B'}
