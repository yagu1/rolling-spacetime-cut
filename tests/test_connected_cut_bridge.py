"""Checks for directed XYT connectivity, moving cuts, and one-lane bridge."""
import pytest

from experiments.connected_cut_bridge import (
    directed_core, get_report, joint_astar_tiebreak, make_scenario, run_all,
)
from rolling_cut import Agent, DStarLite, path_conflicts


@pytest.fixture(scope='module')
def all_report():
    return run_all(scale=2.0)


def test_single_bridge_without_passing_branch_is_infeasible(all_report):
    case=all_report['scenarios']['bridge_no_bays']
    assert case['joint'] is None
    assert case['bare_cut']['reachable'] != {'A':True,'B':True}
    graph,agents=make_scenario('bridge_no_bays')
    ds={a.name:DStarLite(graph,a.start,a.goal) for a in agents}
    # More waiting time still cannot reverse order on a path graph.
    assert joint_astar_tiebreak(graph,agents,ds,40) is None


def test_bridge_bays_connect_both_seeds_to_goals(all_report):
    case=all_report['scenarios']['bridge_with_bays']
    assert case['horizon_steps']==24
    assert case['space_time_nodes']==19*25
    assert not all(case['bare_cut']['reachable'].values())
    assert case['joint']['sum_of_costs']==33
    assert sorted(case['joint']['arrival_steps'].values())==[12,21]
    assert case['connected_cut']['replanned_reachable']=={'A':True,'B':True}
    assert case['connected_cut']['replanned_conflicts']==[]
    for label in ('A','B'):
        core=case['connected_cut']['cores'][label]
        assert core['start_reaches_goal']
        assert core['directed_viable_nodes']>0
        assert 0<core['directed_viable_nodes']<core['assigned_nodes']


def test_witness_paths_are_time_directed_and_collision_free(all_report):
    for case in all_report['scenarios'].values():
        if case['joint'] is None:
            continue
        graph, agents=make_scenario(case['scenario'])
        H=case['horizon_steps']
        paths={name:[tuple(v) for v in seq] for name,seq in case['joint']['paths'].items()}
        assert all(len(path)==H+1 for path in paths.values())
        for a in agents:
            route=paths[a.name]
            assert route[0]==a.start and route[-1]==a.goal
            for p,q in zip(route,route[1:]):
                assert p==q or q in graph.neighbors(p)
        assert path_conflicts(paths,['A','B'])==[]


def test_two_corridor_routes_diverge_without_fixed_priority(all_report):
    case=all_report['scenarios']['two_corridors']
    assert case['horizon_steps']==32
    assert case['joint']['sum_of_costs']==34
    assert case['joint']['arrival_steps']=={'A':14,'B':20}
    assert case['connected_cut']['replanned_conflicts']==[]
    assert all(v['start_reaches_goal'] for v in case['connected_cut']['cores'].values())
    assert any(y==6 for x,y in case['joint']['paths']['B'])


def test_bridge_tie_bias_only_resolves_equal_primary_cost(all_report):
    a=all_report['bridge_tie_break']['favor_A']['joint']
    b=all_report['bridge_tie_break']['favor_B']['joint']
    assert a['sum_of_costs']==b['sum_of_costs']==33
    assert a['arrival_steps']=={'A':12,'B':21}
    assert b['arrival_steps']=={'A':21,'B':12}


def test_spatial_boundaries_are_exact_label_transitions(all_report):
    for case in all_report['scenarios'].values():
        if case['connected_cut'] is None:
            continue
        frames=case['connected_cut']['frames']
        graph,_=make_scenario(case['scenario'])
        for t,frame in enumerate(frames):
            lab={tuple(p):key for key in ('A','B') for p in frame[key]}
            edges={(tuple(a),tuple(b)) for a,b in frame['boundary_edges']}
            assert frame['t']==t
            assert len(edges)==frame['spatial_boundary_count']
            assert set(lab)==set(graph.cells)
            expected={(p,q) for p in graph.cells for q in graph.neighbors(p)
                      if p<q and lab[p]!=lab[q]}
            assert edges==expected


def test_temporal_smoothing_tradeoff_detected(all_report):
    baseline=all_report['scenarios']['bridge_with_bays']['connected_cut']['frames']
    smooth=all_report['bridge_temporal_smoothing']
    h=len(baseline)-1
    def max_bridge_flip(frames):
        def lab(t,x):
            return 'A' if [x,2] in frames[t]['A'] else 'B'
        return max(sum(lab(t,x)!=lab(t-1,x) for x in range(2,11))
                   for t in range(1,h+1))
    assert max_bridge_flip(baseline)>smooth['max_bridge_cells_switched_in_one_step']
    assert smooth['time_slices_with_fragmented_bridge_labels']>0
    assert all(v['start_reaches_goal'] for v in smooth['cores'].values())


def test_directed_core_does_not_claim_all_label_cells_are_reachable():
    graph,agents=make_scenario('bridge_no_bays')
    # Label every XYT vertex A except the start/goal of B; this creates many
    # assigned A cells that cannot be reached from its start without B cells.
    H=5
    labels={(p,t):'A' for p in graph.cells for t in range(H+1)}
    for t in range(H+1):
        labels[(agents[1].start,t)]='B'
    result=directed_core(graph,labels,agents[0],H)
    assert result['directed_viable_nodes']<=result['assigned_nodes']
    assert not result['start_reaches_goal']  # goal occupied by B all times
