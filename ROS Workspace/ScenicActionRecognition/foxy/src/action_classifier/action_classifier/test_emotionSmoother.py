import unittest
import numpy as np
from EmotionSmoother import EmotionSmoother

BASE = np.array([0.2, 0.1, 0.4, 0.3])
SAME_DOM = np.array([0.05, 0.05, 0.8, 0.1])
DIFF_DOM_BEL_THRES = np.array([0.5, 0.025, 0.45, 0.025])


class TestEmotionSmoother(unittest.TestCase):

    def test_same_dom(self):
        s = EmotionSmoother()
        s.smooth(BASE)
        smooth = s.smooth(SAME_DOM)
        for i, a in enumerate(smooth):
            self.assertEqual(a, SAME_DOM[i])

    def test_bel_thres(self):
        s = EmotionSmoother()
        s.smooth(BASE)
        smooth = s.smooth(DIFF_DOM_BEL_THRES)
        for i, a in enumerate(smooth):
            self.assertEqual(a, BASE[i])


if __name__ == "__main__":
    unittest.main()
