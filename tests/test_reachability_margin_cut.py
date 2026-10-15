"""Regression tests for the 4-agent connected-cut + geometric margin experiment."""
import math
import pytest

from experiments.four_agent_bridge import make_four_bridge, independent_dstar
from experiments.reachability_margin_cut import (
    boundaries, segment_distance, safe_moves, viability_cones, joint_astar,
    repair_soft_flow, dist_to_segment_zero, run,
)

@pytest.fixture(scope='module')
def result_09():
    return run(H=30,body_radius=.28,wall_margin=.1,pair_margin=.34,max_joint_states=12000)


def test_individual_viability_is_directed_not_just_spatial():
    g,agents=make_four_bridge(True)
    safe=safe_moves(g,agents,.28,.1)
    cones=viability_cones(g,agents,safe,30)
    for a in agents:
        assert cones[a.name]['goal_reachable_alone']
        assert cones[a.name]['core'][0]=={a.start}
        assert cones[a.name]['core'][30]=={a.goal}
        assert 0 < cones[a.name]['viable_state_count'] < (31*len(g.cells))
    too_short=viability_cones(g,agents,safe,10)
    assert all(not v['goal_reachable_alone'] for v in too_short.values())


def test_robot_wall_clearance_disallows_oversized_body():
    g,agents=make_four_bridge(True)
    okay=safe_moves(g,agents,.28,.1)
    assert (4,2) in okay['A'][(3,2)]
    large=safe_moves(g,agents,.53,.10)
    assert (4,2) not in large['A'].get((3,2),())
    assert not viability_cones(g,agents,large,30)['A']['goal_reachable_alone']


def test_wall_distance_of_bridge_motion_is_half_a_cell():
    g,_=make_four_bridge(True)
    d=min(segment_distance((4,2),(5,2),*wall) for wall in boundaries(g))
    assert d==pytest.approx(.5)


def test_continuous_collision_detector_sees_edge_swap_and_perpendicular_nearmiss():
    assert dist_to_segment_zero((0,0),(1,0),(1,0),(0,0)) == pytest.approx(0.)
    assert dist_to_segment_zero((0,0),(1,0),(1,1),(0,1)) == pytest.approx(1.)
    assert dist_to_segment_zero((0,0),(1,0),(0,1),(0,0)) == pytest.approx(2**-.5)


def test_margin_aware_state_coupling_preserves_directed_connectivity(result_09):
    assert sum(result_09['initial_cut']['connected'].values())==1
    assert all(result_09['directed_feedback_cut']['connected'].values())
    assert result_09['directed_feedback_cut']['hard_complete_path_pins'] is False
    assert result_09['directed_feedback_cut']['history'][-1]['iteration']>=1


def test_independent_replanning_can_be_unsafe_even_when_all_connected(result_09):
    g=result_09['directed_feedback_cut']['independent_path_geometry']
    assert g['minimum_pair_center_m'] < result_09['model']['pair_required_center_m']
    assert not g['safe_traj']
    assert not result_09['directed_feedback_cut']['joint_restricted']['found']


def test_soft_margin_feedback_has_no_hard_route_pins_and_joint_safe_solution(result_09):
    s=result_09['safety_feedback_cut']
    assert s is not None
    assert all(s['connected'].values())
    assert not s['hard_complete_path_pins']
    assert s['candidate_cells_retained']==s['candidate_cell_total']
    assert s['joint_restricted']['found']
    assert s['joint_restricted']['arrival']=={'A':14,'B':16,'C':28,'D':30}
    assert s['joint_restricted']['optimal_cost']==88  # within this owned region
    assert s['geometry']['safe_traj']
    assert s['geometry']['minimum_pair_center_m']==pytest.approx(1.0)
    assert s['geometry']['minimum_wall_center_m']==pytest.approx(.5)
    assert result_09['model']['pair_required_center_m']==pytest.approx(.9)
    assert s['geometry']['per_transition'][2]['max_shared_path_deviation_m']==pytest.approx(.05)


def test_all_four_paths_follow_valid_directed_moves_and_own_every_step(result_09):
    g,agents=make_four_bridge(True)
    safe=safe_moves(g,agents,.28,.1)
    cut=result_09['safety_feedback_cut']
    frames=cut['frames']
    for a in agents:
        n=a.name
        pts=list(map(tuple,cut['joint_paths'][n]))
        assert pts[0]==a.start and pts[-1]==a.goal
        assert len(pts)==31
        for t,(p,q) in enumerate(zip(pts,pts[1:])):
            assert q in safe[n].get(p,())
            assert list(p) in frames[t]['regions'][n]
        assert list(pts[-1]) in frames[30]['regions'][n]


def test_wall_tolerance_rejects_invalid_input():
    with pytest.raises(ValueError):run(H=30,body_radius=-.1)
    with pytest.raises(ValueError):run(H=0)


def test_28_step_horizon_lower_margin_can_work():
    r=run(H=28,body_radius=.28,wall_margin=.1,pair_margin=.04)
    assert r['safety_feedback_cut']['joint_restricted']['found']
    assert r['safety_feedback_cut']['geometry']['minimum_pair_center_m']>=.6-1e-8
    assert r['safety_feedback_cut']['joint_restricted']['arrival']=={'A':14,'B':15,'C':26,'D':27}


def test_28_step_high_margin_oracle_failure_is_not_infeasibility_proof():
    r=run(H=28,body_radius=.28,wall_margin=.1,pair_margin=.34)
    assert not r['safety_oracle']['found']
    assert r['safety_feedback_cut'] is None
