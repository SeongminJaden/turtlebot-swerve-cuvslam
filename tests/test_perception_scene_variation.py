import sys,random,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'perception/scripts'))
from scene_variation import carton_surface,sample_lighting

class SceneVariationTests(unittest.TestCase):
    def test_dents_stay_inside_reference_envelope_and_preserve_supported_top(self):
        for mode in ('intact','side_dent','corner_dent','top_sag','compressed'):
            for seed in range(20):
                pts,uv,counts,idx,meta=carton_surface(random.Random(seed),mode)
                p=np.array(pts);self.assertTrue(np.isfinite(p).all());self.assertTrue((abs(p)<=.50000001).all())
                self.assertEqual(len(uv),len(pts));self.assertEqual(sum(counts),len(idx));self.assertLess(max(idx),len(pts))
                if mode in ('intact','side_dent'):
                    np.testing.assert_allclose(p[4*49:5*49,2],.5)
                if mode=='compressed':self.assertLess(p[:,2].max(),.5)
    def test_seed_reproduction(self):
        a=carton_surface(random.Random(5),'side_dent');b=carton_surface(random.Random(5),'side_dent');self.assertEqual(a,b)
        self.assertEqual(sample_lighting(random.Random(5),'dark'),sample_lighting(random.Random(5),'dark'))
    def test_light_direction_and_temperature_diverse(self):
        lights=[sample_lighting(random.Random(s),'normal') for s in range(100)]
        self.assertGreater(max(x['dome_kelvin'] for x in lights)-min(x['dome_kelvin'] for x in lights),4000)
        self.assertGreater(max(x['sun_rotation_deg'][2] for x in lights)-min(x['sun_rotation_deg'][2] for x in lights),300)
if __name__=='__main__':unittest.main()
