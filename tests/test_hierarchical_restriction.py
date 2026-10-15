"""Tests for restriction-only supervisor / rover planner separation."""
import pytest
from experiments.hierarchical_restriction import (
    AccessWindow, RestrictionMap, Rover, Supervisor, run, swept_clearance,
)
from experiments.four_agent_bridge import make_four_bridge


def setup():
    graph, agents = make_four_bridge(with_bays=True)
    rovers = {a.name: Rover(a, graph, radius=.28, wall_margin=.10) for a in agents}
    return graph, rovers, Supervisor(rovers,lookahead=60,update_every=10,candidate_budget=32)


def test_restrictions_are_time_gated_resource_not_route():
    _, rovers, superv = setup()
    r = RestrictionMap(1, 0, 60, superv.resource, AccessWindow(4,18),
                       frozenset(v.agent.goal for k,v in rovers.items() if k!='A'))
    assert not r.permits((5,2),3)
    assert r.permits((5,2),4)
    assert r.permits((6,2),5)
    assert not r.permits((5,2),18)
    assert r.permits((1,1),3)
    assert not r.permits(rovers['B'].agent.goal,3)
    payload = r.payload()
    assert 'waypoints' not in payload and 'path' not in payload
    assert payload['access_window']=={'open_at':4,'close_at':18}
    assert len(payload['resource_cells_xy'])==15
    assert 'protected_parking_cells_xy' in payload


def test_independent_rover_dstar_and_no_bridge_permission():
    graph, rovers, superv = setup()
    a = rovers['A']
    closed = RestrictionMap(1,0,60,superv.resource,None)
    assert a.local_plan(a.pos,0,60,closed) is None
    open_all = RestrictionMap(1,0,60,superv.resource,AccessWindow(0,60))
    path = a.local_plan(a.pos,0,60,open_all)
    assert path is not None and path[0]==a.pos and a.agent.goal in path
    assert a.dstar.g[a.pos]==14


def test_no_revocation_when_agent_active():
    graph, rovers, superv = setup()
    first=superv.update(0)
    original={n:w for n,w in superv.windows.items() if w.open_at<=10}
    for name,r in rovers.items():
        p=r.local_plan(r.pos,0,60,r.restriction)
        assert p is not None
        r.execute(p,10,0)
    superv.update(10)
    assert all(superv.windows[n]==window for n,window in original.items()
                  if rovers[n].pos!=rovers[n].agent.goal)


def test_four_rover_sixty_second_local_lookahead():
    result=run(lookahead=60,update_every=10,budget=32,safety_distance=.90,seed=17)
    assert result['completed']
    assert result['arrivals']=={'A':16,'B':14,'C':30,'D':33}
    assert result['minimum_swept_distance_m']>=.9
    assert result['full_trace_clearance_pass']
    assert result['whole_spacetime_graph_cut_calls']==0
    assert result['unrestricted_baseline']['minimum_swept_center_distance_m']==0.0
    assert not result['unrestricted_baseline']['margin_satisfied']
    assert [u['now'] for u in result['supervisor_updates']]==[0,10,20,30]
    assert all(u['candidates']==32 and u['feasible_candidates']>0 for u in result['supervisor_updates'])
    assert all(u['scope']['competing_bottleneck_cells']<=15 for u in result['supervisor_updates'])
    assert all('planned_paths' not in log for log in result['timeline'])
    assert all('path' not in r for log in result['timeline'] for r in log['restriction_payloads'].values())


@pytest.mark.parametrize('budget', [2,8,16,32])
def test_candidate_budget_keeps_collision_clearance(budget):
    result=run(budget=budget,seed=17)
    assert result['completed']
    assert result['minimum_swept_distance_m']>=.9


def test_hard_center_separation_not_a_soft_penalty():
    p={'A':[(0,0),(1,0)],'B':[(1,0),(0,0)]}
    assert not swept_clearance(p,.90)[0]
    # A soft repulsion never turns a submargin transition into admissible.
    assert swept_clearance({'A':[(0,0),(1,0)],'B':[(0,2),(1,2)]},.9)[0]
