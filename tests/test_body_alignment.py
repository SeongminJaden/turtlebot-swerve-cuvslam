import math
import numpy as np
from turtlebot_swerve.body_alignment import route_heading,alignment_speed_cap


def test_straight_path_faces_forward():
    p=np.c_[np.arange(0,1,.05),np.zeros(20)]
    yaw,straight=route_heading(p,4)
    assert straight and yaw==0
    assert alignment_speed_cap(yaw,-math.pi/4,straight)==0
    assert alignment_speed_cap(yaw,0,straight)==.92


def test_corner_preview_and_vertical_straight():
    p=np.r_[np.c_[np.arange(0,1,.05),np.zeros(20)],np.c_[np.ones(20),np.arange(0,1,.05)]]
    yaw,straight=route_heading(p,18)
    assert not straight and 0<yaw<math.pi/2
    yaw,straight=route_heading(p,24)
    assert straight and abs(yaw-math.pi/2)<1e-8
    assert alignment_speed_cap(yaw,0,straight)==0


def test_heading_wrap_does_not_stop_aligned_base():
    assert alignment_speed_cap(math.pi,-math.pi,True)==.92
