import numpy as np
import pytest
from turtlebot_swerve.live_path import resample_path,accept_path

def test_duplicate_removal_and_spacing():
    p=resample_path([[0,0],[0,0],[.12,0]])
    np.testing.assert_allclose(p[:,0],[0,.05,.1,.12])

def test_short_path_is_valid():
    assert len(resample_path([[0,0],[.02,0]]))==2

@pytest.mark.parametrize('change',[{'frame_id':'odom'},{'stamp_ns':0},{'slice_stamp_ns':0},{'valid':False},{'points':[[0,0],[float('nan'),0]]},{'points':[[0,0],[2,0]]}])
def test_rejects_bad_path(change):
    p=dict(valid=True,frame_id='map',stamp_ns=4_000_000_000,slice_stamp_ns=4_000_000_000,points=[[0,0],[1,0]])
    p.update(change)
    with pytest.raises(ValueError):accept_path(p,4_100_000_000,[1,0])


def test_replanning_does_not_reset_mission_progress():
    from turtlebot_swerve.live_path import mission_progress
    assert mission_progress([0,1,2,3,4],2,4)==.5
    assert mission_progress([0,1,2],0,4)==.5
    assert mission_progress([0,1,2],2,4)==1.
    assert mission_progress([0,5],0,4)==0.
