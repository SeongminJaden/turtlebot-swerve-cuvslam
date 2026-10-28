import numpy as np
import pytest
from turtlebot_swerve.camera_quality import camera_quality


def test_blank_image_is_not_good_texture():
    q=camera_quality(np.full((48,64,3),128),np.ones((48,64)))
    assert q['texture_tile_fraction']==0 and q['laplacian_variance']==0
    assert q['depth_valid_fraction']==1 and q['saturated_fraction']==0


def test_invalid_depth_and_exposure_are_reported():
    d=np.full((48,64),np.inf);d[:24]=1
    q=camera_quality(np.full((48,64,3),255),d)
    assert q['depth_valid_fraction']==.5 and q['near_field_depth_valid_fraction']==0
    assert q['saturated_fraction']==1


def test_texture_coverage_uses_image_and_not_depth():
    rng=np.random.default_rng(3)
    q=camera_quality(rng.integers(20,230,(48,64,3)),np.zeros((48,64)))
    assert q['texture_tile_fraction']>.8 and q['depth_valid_fraction']==0


def test_rejects_mismatched_frames():
    with pytest.raises(ValueError):camera_quality(np.zeros((48,64,3)),np.zeros((24,32)))
