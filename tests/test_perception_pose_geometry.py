"""Geometry/loss regression checks for the custom CenterPose training harness."""
import sys
import unittest
from unittest.mock import patch
from pathlib import Path
import cv2
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'perception/scripts'))
from train_centerpose import CORNERS,SYMMETRIES,SIZE,crop_image,loss_for,infer_single,infer

class PoseGeometryTests(unittest.TestCase):
    def test_three_scale_median_resists_one_bad_depth(self):
        estimates=[dict(translation_valid=True,rotation_valid=True,valid=True,
                        center_camera_m=[0.,0.,z]) for z in (.4,.401,.8)]
        with patch('train_centerpose.infer_crop_views',return_value=estimates):
            result=infer(None,None,None,None,views='scale3_median')
        np.testing.assert_allclose(result['center_camera_m'],[0.,0.,.401])

    def test_crop_ensemble_excludes_invalid_translation_and_preserves_rotation(self):
        estimates=[dict(translation_valid=True,rotation_valid=True,
                       center_camera_m=[0.,0.,.4],rotation_matrix=np.eye(3).tolist(),valid=True)]
        estimates += [dict(translation_valid=True,center_camera_m=[.006,0.,.4]) for _ in range(5)]
        estimates += [dict(translation_valid=False,center_camera_m=[99.,99.,99.])]
        with patch('train_centerpose.infer_crop_views',return_value=estimates):
            result=infer(None,None,None,None,views='all7_mean')
        np.testing.assert_allclose(result['center_camera_m'],[.005,0.,.4])
        self.assertEqual(result['translation_views_used'],6)
        np.testing.assert_array_equal(result['rotation_matrix'],np.eye(3))

    def test_crop_ensemble_does_not_turn_failed_views_into_success(self):
        failure=dict(translation_valid=False,rotation_valid=False,valid=False)
        with patch('train_centerpose.infer_crop_views',return_value=[failure.copy() for _ in range(7)]):
            result=infer(None,None,None,None,views='all7_mean')
        self.assertFalse(result['translation_valid'])
        self.assertFalse(result['valid'])

    def test_symmetries_are_eight_proper_rigid_rotations(self):
        self.assertEqual(len({tuple(p) for p in SYMMETRIES}),8)
        for p in SYMMETRIES:
            rotation=np.linalg.lstsq(CORNERS,CORNERS[p],rcond=None)[0]
            np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(rotation),1)
            np.testing.assert_allclose(CORNERS@rotation,CORNERS[p],atol=1e-12)

    def test_exact_projections_recover_translation(self):
        K=np.array([[733.,0,320],[0,733.,320],[0,0,1.]])
        for dims in ([.03,.03,.06],[.205,.205,.07]):
            xyz=CORNERS*np.array(dims)/[.03,.03,.06]
            for rv in ([.2,.4,.1],[1.57,0,2.4]):
                t=np.array([.02,-.03,.35]);uv,_=cv2.projectPoints(xyz,np.array(rv),t,K,None)
                ok,_,est=cv2.solvePnP(xyz,uv,K,None,flags=cv2.SOLVEPNP_EPNP)
                self.assertTrue(ok);np.testing.assert_allclose(est.ravel(),t,atol=1e-7)

    def test_depth_normalization_round_trip_under_crop_scale(self):
        im=np.zeros((640,640,3),np.uint8)
        for box in ([20,30,70,90],[100,130,500,540]):
            _,m=crop_image(im,box)
            for z in (.2,.4,.8):
                logz=np.log(z*SIZE/(m[0,0]*733*.06))
                recovered=np.exp(logz)*733*.06*m[0,0]/SIZE
                self.assertAlmostEqual(recovered,z,places=6)

    def test_metric_depth_inference_recovers_known_center(self):
        image=np.zeros((640,640,3),np.uint8)
        K=np.array([[733.,0,320],[0,733.,320],[0,0,1.]])
        translation=np.array([.02,-.03,.4])
        pixels,_=cv2.projectPoints(CORNERS,np.array([.3,.1,.4]),translation,K,None)
        pixels=pixels.reshape(8,2)
        box=[*pixels.min(0),*pixels.max(0)]
        _,m=crop_image(image,box)
        center=K@translation;center=center[:2]/center[2]
        center_grid=(m[:,:2]@center+m[:,2])/4
        ij=center_grid.astype(int)
        corners_grid=(pixels@m[:,:2].T+m[:,2])/4
        out={k:torch.zeros(1,n,32,32) for k,n in [('hm',1),('hps',16),('reg',2),('scale',3)]}
        out['hm'].fill_(-20);out['hm'][0,0,ij[1],ij[0]]=20
        out['reg'][0,:,ij[1],ij[0]]=torch.tensor(center_grid-ij)
        out['hps'][0,:,ij[1],ij[0]]=torch.tensor((corners_grid-ij).flatten())
        out['scale'][0,0,ij[1],ij[0]]=float(np.log(translation[2]*SIZE/(m[0,0]*K[0,0]*.06)))
        class FixedModel:
            metric_depth_head=True
            def __call__(self,x):return [out]
        prediction=infer_single(FixedModel(),image,box,K,device='cpu')
        self.assertTrue(prediction['translation_valid'])
        self.assertTrue(prediction['rotation_valid'])
        np.testing.assert_allclose(prediction['center_camera_m'],translation,atol=1e-7)

    def test_corner_loss_accepts_symmetric_ordering(self):
        ij=torch.tensor([[16,16]]);heat=torch.zeros(1,1,32,32);heat[0,0,16,16]=1
        corners=torch.arange(16,dtype=torch.float32).reshape(8,2)
        def output(points):
            result={k:torch.zeros(1,n,32,32) for k,n in [('hm',1),('hps',16),('reg',2),('scale',3)]}
            result['hps'][0,:,16,16]=points.flatten()
            return result
        target=corners.flatten()[None];reg=torch.zeros(1,2);depth=torch.zeros(1)
        original=loss_for(output(corners),heat,ij,target,reg,depth)
        equivalent=loss_for(output(corners[SYMMETRIES[5]]),heat,ij,target,reg,depth)
        self.assertAlmostEqual(float(original),float(equivalent),places=5)
        wrong=loss_for(output(corners*0),heat,ij,target,reg,depth)
        self.assertGreater(float(wrong),float(original))

if __name__=='__main__':unittest.main()
