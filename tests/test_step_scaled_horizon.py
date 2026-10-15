"""Tests for the no-fixed-priority horizon experiment."""

from experiments.step_scaled_horizon import run


def test_double_length_horizon_and_repair():
    result = run(2.0)
    assert result['parameters']['max_independent_steps'] == 16
    assert result['parameters']['horizon_steps'] == 32
    assert result['parameters']['space_time_nodes'] == 1188
    assert result['pure_symmetric_cut']['reachable'] == {'A': True, 'B': False}
    branches = result['reachability_feedback']['branch_summaries']
    assert [b['status'] for b in branches] == ['feasible', 'feasible']
    assert [b['sum_of_costs'] for b in branches] == [34, 38]
    selected = result['reachability_feedback']['selected']
    assert selected is not None
    assert selected['arrival_steps'] == {'A': 14, 'B': 20}
    assert selected['route_types'] == {'A': 'upper', 'B': 'lower'}


def test_shorter_horizon_cannot_support_the_bypass():
    result = run(1.0)
    assert result['parameters']['horizon_steps'] == 16
    assert result['reachability_feedback']['selected'] is None


def test_intermediate_horizon_recovers_one_alternative():
    result = run(1.25)
    assert result['parameters']['horizon_steps'] == 20
    branches = result['reachability_feedback']['branch_summaries']
    assert [b['status'] for b in branches] == ['feasible', 'second-unreachable']
    assert result['reachability_feedback']['selected']['arrival_steps'] == {'A': 14, 'B': 20}
