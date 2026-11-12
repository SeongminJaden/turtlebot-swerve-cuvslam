import importlib.util
from pathlib import Path
import unittest
import math
spec=importlib.util.spec_from_file_location('placement',Path(__file__).resolve().parents[1]/'perception/scripts/placement.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class PlacementTests(unittest.TestCase):
    def test_supported_and_separated_all_resting_faces(self):
        rests=set();xs=[]
        for seed in range(1000):
            layout=m.sample_layout(seed)
            for i,p in enumerate(layout):
                self.assertLessEqual(abs(p['x'])+p['extent_x'],.1025)
                self.assertLessEqual(abs(p['y'])+p['extent_y'],.1025)
                self.assertEqual(p['center_z'],.015 if p['class_id']%2 and p['tilt'] in (90,270) else .03)
                if p['class_id']%2:rests.add((p['axis'],p['tilt']))
                xs.append(round(p['x'],4))
                for q in layout[i+1:]:self.assertGreater(math.hypot(p['x']-q['x'],p['y']-q['y']),p['radius']+q['radius'])
        self.assertEqual(rests,set(m.CUBOID_RESTS))
        self.assertGreater(len(set(xs)),500)
    def test_reproducible(self):
        self.assertEqual(m.sample_layout(4),m.sample_layout(4))
