import numpy as np
import pytest
from turtlebot_swerve.map_slice import reproject_slice


def test_translation_preserves_distances_and_unknown():
    source = np.array([[0., 1., -1000.], [.5, 0., 2.]])
    tf = np.eye(4); tf[:2, 3] = [2., 3.]
    result, origin = reproject_slice(source, [0, 0], .1, tf, -1000.)
    np.testing.assert_allclose(origin, [2, 3])
    np.testing.assert_allclose(result[:2, :3], source)


def test_quarter_turn_moves_obstacle_and_preserves_unknown():
    tf = np.eye(4); tf[:2, :2] = [[0, -1], [1, 0]]
    result, origin = reproject_slice([[0., 2.], [-1000., 3.]], [0, 0], 1., tf, -1000.)
    np.testing.assert_allclose(origin, [-1, 0])
    np.testing.assert_allclose(result, [[-1000., 0.], [3., 2.]])


def test_outside_rotated_grid_remains_unknown():
    a=np.pi/4; tf=np.eye(4); tf[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
    result,_=reproject_slice(np.zeros((3,3)),[0,0],.1,tf,-1000.)
    assert np.any(result == -1000.)
    assert set(np.unique(result)) == {-1000., 0.}


def test_rejects_nonplanar_transform():
    tf=np.eye(4);tf[2,3]=.2
    with pytest.raises(ValueError,match='planar'):
        reproject_slice(np.zeros((2,2)),[0,0],.1,tf,-1000.)
