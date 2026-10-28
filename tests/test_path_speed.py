import numpy as np
from turtlebot_swerve.path_speed import PathSpeedEnvelope

def test_straight_and_goal_braking():
    route=np.c_[np.linspace(0,5,101),np.zeros(101)]
    e=PathSpeedEnvelope(route)
    assert abs(e.at(0)-.92)<1e-9
    assert e.at(99)<.21 and e.at(100)==0
    assert np.all(e.limit[:-1]**2<=e.limit[1:]**2+.8*np.diff(e.arc)+1e-9)

def test_bend_is_slower_and_lookahead_brakes():
    route=np.vstack([np.c_[np.linspace(0,2,41),np.zeros(41)],np.c_[np.full(40,2),np.linspace(.05,2,40)]])
    e=PathSpeedEnvelope(route)
    assert e.at(40)<.2 and e.at(0)>.8
    assert e.at(30,.8)<=e.at(30,0)
