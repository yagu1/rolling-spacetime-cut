"""Contingency experiments: deterministic regression, not formal safety proof."""
import math
import pytest
from experiments.contingency_restriction import (
    SCENARIOS, ContingencyRover, current_obstacles, make_contingency_bridge,
    no_gate, run_case, shield_moves,
)


def test_approach_event_does_not_materialize_under_robot():
    assert current_obstacles(SCENARIOS['approach_detour'],0)==set()
    assert current_obstacles(SCENARIOS['approach_detour'],1)=={(13,2)}
    assert current_obstacles(SCENARIOS['approach_detour'],18)==set()


def test_sensor_range_observation_and_release():
    g, agents=make_contingency_bridge()
    a=next(x for x in agents if x.name=='C')
    r=ContingencyRover(a,g,sensor_radius=1)
    assert not r.scan({(13,2)},0) # two taxicab units away, outside 1-cell sensor radius
    r.pos=(13,1)
    assert r.scan({(13,2)},1)
    assert r.scan(set(),2)


def test_incremental_dstar_updates_on_detection():
    g,agents=make_contingency_bridge()
    r=ContingencyRover(next(a for a in agents if a.name=='C'),g,sensor_radius=2)
    r.pos=(13,1)
    assert r.scan({(13,2)},1)
    assert (13,2) in r.known_obstacles
    assert math.isinf(r.dstar.g[(13,2)])
    assert r.scan(set(),2)
    assert (13,2) not in r.known_obstacles
    assert math.isfinite(r.dstar.g[(13,2)])


def test_local_nav_detours_around_visible_approach_obstacle():
    g,agents=make_contingency_bridge()
    rovers={a.name:ContingencyRover(a,g) for a in agents}
    r=rovers['C'];r.pos=(13,1)
    r.scan({(13,2)},1)
    mask=no_gate(r,1,60,{(x,2) for x in range(15)},rovers)
    p=r.local_plan(r.pos,1,60,mask)
    assert p[1]==(13,0)
    assert (13,2) not in p[:18]
    assert p[-1]==r.agent.goal


def test_approach_detour_only_restriction_mode_replans_before_ten_seconds():
    rp=run_case('approach_detour','restriction',budget=8,seed=17)
    central=run_case('approach_detour','central_path',budget=8,seed=17)
    assert rp['all_arrived'] and central['all_arrived']
    assert rp['traces']['C'][2]==[13,0]  # locally selected at t=1
    assert central['traces']['C'][2]==[13,1] # follows fixed route until next dispatch
    assert rp['completion_time']<central['completion_time']
    assert rp['path_dispatch_messages']==0
    assert central['path_dispatch_messages']>0
    assert not rp['restriction_payload_contains_paths']


def test_bridged_blockage_causes_wait_and_recovery():
    rp=run_case('bridge_closure','restriction',budget=8,seed=17)
    assert rp['all_arrived']
    assert any('hold' in v for row in rp['timeline'] for v in row['status'].values())
    assert rp['safe_hold_plans']['D']>0
    assert rp['arrival_times']['D']>23
    assert rp['minimum_swept_distance']>=.9


@pytest.mark.parametrize('scenario',['none','approach_detour','bridge_closure'])
def test_comparable_nominal_and_obstacle_runs(scenario):
    # All methods receive the same map, agents, sensing model and idealized shield.
    for mode in ('restriction','central_path','uncoordinated'):
        result=run_case(scenario,mode,budget=8,seed=17)
        assert result['safety_distance_satisfied']
        assert result['minimum_swept_distance']>=.9
        assert result['cut_calls']==0
        assert result['simulation_steps']<=90
        assert result['restriction_payload_contains_paths'] is False
        if mode!='uncoordinated': assert result['all_arrived']


def test_no_rp_can_deadlock_despite_emergency_shield():
    x=run_case('bridge_closure','uncoordinated',budget=8,seed=17,max_time=90)
    assert not x['all_arrived']
    assert x['emergency_stops']>0
    assert x['minimum_swept_distance']>=.9


def test_dynamic_supervisor_messages_contain_no_waypoints():
    r=run_case('approach_detour','restriction',seed=17,budget=8)
    assert r['restriction_messages']>0
    assert r['restriction_payload_bytes']>0
    assert r['path_payload_bytes']==0
    assert all('waypoint' not in k and 'path' not in k
        for message in r['restriction_payloads'] for k in message['payload'])


def test_seed_sweep_keeps_feasible_four_rover_safety():
    for seed in range(5):
        for scenario in ('approach_detour','bridge_closure'):
            r=run_case(scenario,'restriction',seed=seed,budget=8)
            assert r['all_arrived'],(scenario,seed)
            assert r['minimum_swept_distance']>=.9
