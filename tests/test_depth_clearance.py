import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import numpy as np
from depth_clearance import obstacle_points,corridor_clearance

def test_usd_optical_axis_to_robot_forward():
    # camera -Z -> robot +X; camera +Y -> robot +Z
    t=np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,.2],[0,0,0,1.]])
    pts=obstacle_points(np.ones((3,3)),t,100,stride=1)
    assert pts.shape==(9,2)
    assert np.allclose(pts[:,0],1.)
    assert corridor_clearance(pts,0)==1.

def test_corridor_excludes_side_wall_and_behind():
    assert corridor_clearance([[.6,.5],[-1,0],[1.2,.1]],0)==1.2
    assert np.isclose(corridor_clearance([[0,.7]],np.pi/2),.7)

def test_invalid_depth_and_floor_not_obstacles():
    t=np.eye(4)
    assert obstacle_points(np.array([[np.nan,np.inf,0.]]),t,100,stride=1).shape==(0,2)
