from experiments.state_margin_bridge import dist_to_segment_zero,run

def test_swept_separation_detects_crossing():
    assert dist_to_segment_zero((0,0),(1,0),(1,0),(0,0)) == 0
    assert abs(dist_to_segment_zero((0,0),(1,0),(0,1),(1,1))-1)<1e-9

def test_margin_and_state_comparison():
    report=run()
    assert report['horizon']==28
    assert not all(report['unseeded_cuts']['state']['reachable'].values())
    assert report['margin_aware_planning']['0.6']['found']
    assert report['margin_aware_planning']['0.6']['minimum_distance_m']['distance']>=.6
    assert not report['margin_aware_planning']['0.9']['found']
