import unittest
from turtlebot_swerve.dice_mission import plan, DICE


def inventory(red=6):
    return [dict(id=i,color='R' if i<red else 'G',shape='cylinder' if i%2==0 else 'cuboid') for i in range(12)]

class MissionTests(unittest.TestCase):
    def test_q1(self):
        result=plan('RGRRGRGGYGYG','P2-01',inventory())
        self.assertEqual(result['counts'],{'R':4,'G':6,'Y':2})
        self.assertEqual([x['group'] for x in result['locations']],['Y','R','G'])
        ids=[p['id'] for x in result['locations'] for p in x['targets']]
        self.assertEqual(sorted(ids),list(range(12)))
    def test_ties_both_directions(self):
        for barcode in ('P2-01','P2-02','P2-03','P2-04'):
            self.assertEqual([x['group'] for x in plan('RYG'*4,barcode,inventory())['locations']],list('RYG'))
    def test_variable_colors(self):
        p=inventory(8)
        self.assertEqual(plan('R'*5+'Y'*3+'G'*4,'P2-02',p)['locations'][0]['group'],'R')
    def test_impossible(self):
        with self.assertRaises(ValueError):plan('R'*6+'Y'*5+'G','P2-01',inventory(2))
    def test_bad_inputs(self):
        for seq,code,p in [('R'*12,'P2-01',inventory()),('R?'+ 'YG'*5,'P2-01',inventory()),('RYG'*4,'bad',inventory()),('RYG'*4,'P2-01',inventory()[:-1])]:
            with self.assertRaises(ValueError):plan(seq,code,p)
    def test_orientation(self):
        self.assertEqual(DICE[2],((-1,1),(1,-1)))
        self.assertTrue(all(x in (-1,1) for x,y in DICE[6]))


class SignalTests(unittest.TestCase):
    def test_repeated_color_and_end(self):
        from turtlebot_swerve.dice_mission import SignalRecorder
        recorder=SignalRecorder()
        sequence='RGRRGRGGYGYG'
        for tick in range(271):
            time=tick/10
            color=sequence[tick//20] if tick<240 and tick%20<10 else 'OFF'
            recorder.observe(time,color)
        self.assertTrue(recorder.complete)
        self.assertEqual(''.join(recorder.sequence),sequence)
    def test_drop_is_error_latched(self):
        from turtlebot_swerve.dice_mission import SignalRecorder
        recorder=SignalRecorder()
        recorder.observe(0,'R')
        with self.assertRaises(ValueError):recorder.observe(1,'OFF')
        with self.assertRaises(ValueError):recorder.observe(1.1,'R')
    def test_unknown(self):
        from turtlebot_swerve.dice_mission import SignalRecorder
        with self.assertRaises(ValueError):SignalRecorder().observe(0,'UNKNOWN')

if __name__=='__main__':unittest.main()
