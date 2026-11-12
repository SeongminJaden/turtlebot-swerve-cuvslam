import copy
import unittest
from turtlebot_swerve.dice_mission import plan
from turtlebot_swerve.mission_verification import verify_placements,verify_return

class VerificationTests(unittest.TestCase):
    def setUp(self):
        pucks=[dict(id=i,color='R' if i<6 else 'G',shape='cylinder' if i%2==0 else 'cuboid') for i in range(12)]
        self.plan=plan('RGRRGRGGYGYG','P2-01',pucks)
        self.frames={loc['location']:[dict(time_s=10+i*.2,frame_id=f"{loc['location']}-{i}",source='camera',pucks=copy.deepcopy(loc['targets'])) for i in range(11)] for loc in self.plan['locations']}
    def test_synthetic_valid_contract(self):
        self.assertTrue(verify_placements(self.plan,self.frames,9)['passed'])
    def test_target_copy_source_rejected(self):
        self.frames['L1'][0]['source']='planner'
        self.assertFalse(verify_placements(self.plan,self.frames,9)['passed'])
    def test_stale_and_missing(self):
        self.assertFalse(verify_placements(self.plan,self.frames,10)['passed'])
        self.frames['L1'][-1]['pucks']=[]
        self.assertFalse(verify_placements(self.plan,self.frames,9)['passed'])
    def test_wrong_position_or_nan(self):
        for offset in (.011,float('nan')):
            f=copy.deepcopy(self.frames);f['L1'][2]['pucks'][0]['center_m'][0]+=offset
            self.assertFalse(verify_placements(self.plan,f,9)['passed'])
    def test_unstable(self):
        self.frames['L1']=self.frames['L1'][:5]
        self.assertFalse(verify_placements(self.plan,self.frames,9)['passed'])
    def test_return_entire_chassis(self):
        samples=[dict(time_s=i*.2,speed_mps=0,yaw_rate_radps=0,chassis_polygon_m=[(.2,.2),(.8,.2),(.8,.8),(.2,.8)]) for i in range(16)]
        self.assertTrue(verify_return(samples,(0,0,1,1)))
        samples[-1]['chassis_polygon_m'][0]=(-.01,.2)
        self.assertFalse(verify_return(samples,(0,0,1,1)))
