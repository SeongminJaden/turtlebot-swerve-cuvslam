import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from tracking_recovery import continuation_anchor,stable_poses

def test_reanchor_keeps_visual_origin_and_only_adds_odometry_increment():
    v=[10,20,0,0,0,0,1];old=[1,2,0,0,0,0,1];new=[1.1,1.8,0,0,0,0,1]
    out=continuation_anchor(v,old,new)
    assert np.allclose(out[:3,3],[10.1,19.8,0])

def test_stationary_multiframe_reacquisition_required():
    poses=[np.eye(4) for _ in range(8)]
    assert not stable_poses(poses[:7])
    assert stable_poses(poses)
    poses[0][0,3]=.03
    assert not stable_poses(poses)

def test_image_time_odometry_interpolation_rejects_missing_bracket():
    from tracking_recovery import interpolate_pose
    h=[{'stamp_ns':0,'pose':[0,0,0,0,0,0,1]}, {'stamp_ns':10,'pose':[1,0,0,0,0,0,-1]}]
    assert np.allclose(interpolate_pose(h,5),[.5,0,0,0,0,0,1])
    assert interpolate_pose(h,11) is None
