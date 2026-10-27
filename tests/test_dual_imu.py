import numpy as np
import pytest
from turtlebot_swerve.dual_imu import Sample,DualImuFusion,rotation,interpolate

def sample(t,w,f):return Sample(t,np.array(w,float),np.array(f,float),np.eye(3)*1e-4,np.eye(3)*.01)
def pair(t,w,a,alpha,rs,qs):
 out=[]
 for r,q in zip(rs,qs):
  R=rotation(q);force=a+np.cross(alpha,r)+np.cross(w,np.cross(w,r))
  out.append(sample(t,R.T@w,R.T@force))
 return out

def test_rotated_board_and_offset_with_tangential_acceleration():
 rs=np.array([[.2,.1,.15],[-.12,-.05,.15]]);qs=[[0,0,0,1],[0,0,1,0]];f=np.array([.4,-.1,9.81]);alpha=np.array([.1,.2,.5]);w=np.array([.3,-.2,1.])
 fusion=DualImuFusion(rs,qs,alpha_tau=0)
 assert fusion.update(pair(1,w,f,alpha,rs,qs)) is None
 result=fusion.update(pair(1.01,w+alpha*.01,f,alpha,rs,qs))
 assert np.allclose(result['accel'],f,atol=1e-10)
 assert np.allclose(result['gyro'],w+alpha*.01)
 assert np.linalg.eigvalsh(result['accel_cov']).min()>=0

def test_uncompensated_mean_is_wrong_even_for_symmetric_boards():
 rs=np.array([[.06,0,.166],[-.06,0,.166]]);qs=[[0,0,0,1]]*2;w=np.array([0,1.,0]);f=np.array([0,0,9.81]);alpha=np.zeros(3)
 samples=pair(1,w,f,alpha,rs,qs)
 assert not np.allclose(np.mean([s.accel for s in samples],axis=0),f)
 fusion=DualImuFusion(rs,qs,alpha_tau=0);fusion.update(samples)
 assert np.allclose(fusion.update(pair(1.01,w,f,alpha,rs,qs))['accel'],f)

def test_disagreement_rejects_both_not_arbitrary_bad_sensor_selection():
 fusion=DualImuFusion(np.zeros((2,3)),[[0,0,0,1]]*2)
 with pytest.raises(ValueError,match='Gyro disagreement'):fusion.update([sample(1,[0,0,0],[0,0,9.81]),sample(1,[0,0,2],[0,0,9.81])])

def test_interpolation_and_stale_rejection():
 ss=[sample(1,[0,0,0],[0,0,9.81]),sample(1.02,[0,0,2],[0,0,9.81])]
 assert np.allclose(interpolate(ss,1.01).gyro,[0,0,1])
 with pytest.raises(ValueError):interpolate(ss,1.03)
 with pytest.raises(ValueError):interpolate(ss,1.01,max_gap=.005)

def test_gap_and_repeated_stamp_never_fabricate_alpha():
 f=DualImuFusion(np.zeros((2,3)),[[0,0,0,1]]*2)
 s=lambda t:[sample(t,[0,0,0],[0,0,9.81])]*2
 assert f.update(s(1)) is None
 with pytest.raises(ValueError,match='Nonmonotonic'):f.update(s(1))
 assert f.update(s(2)) is None
 assert np.allclose(f.update(s(2.01))['accel'],[0,0,9.81])
