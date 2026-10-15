"""Regression tests: run with `python -m pytest -q`."""
import pytest
from rolling_cut import GridGraph, DStarLite, Agent, path_conflicts, run


def test_shortest_paths_choose_same_upper_route():
    graph=GridGraph.two_corridors()
    a=DStarLite(graph,(0,2),(14,2))
    b=DStarLite(graph,(14,3),(0,3))
    assert len(a.path())-1==14
    assert len(b.path())-1==16
    assert len(set(a.path()) & set(b.path())) >= 10


def test_dstar_lite_reacts_to_obstacle_updates():
    graph=GridGraph.two_corridors()
    dstar=DStarLite(graph,(0,2),(14,2))
    assert dstar.g[(0,2)] == 14
    dstar.change_obstacle((7,2), True)
    p=dstar.path()
    assert (7,2) not in p
    assert len(p)-1 > 14
    assert len(p)-1 == dstar.g[(0,2)]
    dstar.change_obstacle((7,2), False)
    assert dstar.g[(0,2)]==14
    assert len(dstar.path())-1 == 14


def test_detect_vertex_and_swap_collisions():
    both={'A':[(0,0),(1,0)],'B':[(1,0),(0,0)]}
    assert any('edge swap' in e for e in path_conflicts(both,['A','B']))
    together={'A':[(0,0),(1,0)],'B':[(2,0),(1,0)]}
    assert any('vertex collision' in e for e in path_conflicts(together,['A','B']))


def test_rolling_cut_diverges_and_both_reach_goal():
    result=run(steps=26,horizon=10,execute=2,verbose=False)
    a=result['agents']['A']; b=result['agents']['B']
    assert a['arrived'] and b['arrived']
    assert a['arrival_time_s']==14
    assert b['arrival_time_s']==20
    assert any(p[1]==6 for p in b['trace'])
    assert not any(p[1]==6 for p in b['independent_shortest_path'])
    assert result['collisions']==[]
    assert len(result['phases'])>1
    assert result['phases'][0]['info']['cut_attempts']>=2
    for phase in result['phases']:
        assert len(phase['planned_paths']['A']) == 11
        assert len(phase['planned_paths']['B']) == 11


@pytest.mark.parametrize('horizon,execute',[(8,2),(10,1),(12,3)])
def test_other_window_lengths_remain_collision_free(horizon,execute):
    result=run(steps=30,horizon=horizon,execute=execute,verbose=False)
    assert not result['collisions']
    assert all(a['arrived'] for a in result['agents'].values())
