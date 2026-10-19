#!/usr/bin/env python3
"""Causal synthetic motion and history integrity; no developer truth inputs."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from publication_prediction import PublicationPrediction


class PublicationPredictionTest(unittest.TestCase):
    def turning(self, predictor=None):
        p = predictor or PublicationPrediction()
        for t, v in [(10., [1., 0.]), (10.1, [.8, .2]),
                     (10.2, [.6, .4]), (10.3, [0., 1.])]:
            p.observe(t, 1, v)
        return p

    def test_straight_motion_keeps_full_age_compensation(self):
        p = PublicationPrediction()
        for t in (10., 10.1, 10.2, 10.3):
            p.observe(t, 1, [1., -.2])
        xy, meta = p.predict([5., 4.], [1., -.2], 10.3, 10.64)
        np.testing.assert_allclose(xy, [5.34, 3.932])
        self.assertEqual(meta['mode'], 'constant_velocity')

    def test_turn_bounds_old_velocity_displacement_but_preserves_real_age(self):
        p = self.turning()
        xy, meta = p.predict([5., 4.], [0., 1.], 10.3, 10.64)
        effective = .15 * (1 - math.exp(-.34 / .15))
        np.testing.assert_allclose(xy, [5., 4. + effective])
        self.assertEqual(meta['mode'], 'turn_velocity_decay')
        self.assertAlmostEqual(meta['measurement_age'], .34)
        self.assertAlmostEqual(meta['effective_horizon'], effective)
        self.assertLess(meta['effective_horizon'], .15)

    def test_observed_braking_also_bounds_displacement(self):
        p = PublicationPrediction()
        for t, v in [(10., 1.), (10.1, .8), (10.2, .5), (10.3, .3)]:
            p.observe(t, 1, [v, 0.])
        _, meta = p.predict([0., 0.], [.3, 0.], 10.3, 10.64)
        self.assertEqual(meta['mode'], 'turn_velocity_decay')

    def test_one_noisy_frame_and_low_speed_do_not_establish_a_turn(self):
        p = PublicationPrediction()
        p.observe(10., 1, [1., 0.])
        p.observe(10.1, 1, [-1., 0.])
        self.assertEqual(p.predict([0., 0.], [-1., 0.], 10.1, 10.44)[1]['mode'], 'constant_velocity')
        p.reset()
        for t, v in [(10., [.1, 0.]), (10.1, [0., .1]), (10.2, [-.1, 0.])]:
            p.observe(t, 1, v)
        self.assertEqual(p.predict([0., 0.], [-.1, 0.], 10.2, 10.54)[1]['mode'], 'constant_velocity')

    def test_source_handoff_gap_and_reset_clear_turn_evidence(self):
        for action in ('source', 'gap', 'reset'):
            p = self.turning()
            if action == 'reset':
                p.reset()
                stamp, source = 10.4, 1
            else:
                stamp, source = (10.4, 2) if action == 'source' else (10.8, 1)
            p.observe(stamp, source, [0., 1.])
            self.assertEqual(len(p.samples), 1)
            self.assertEqual(p.predict([0., 0.], [0., 1.], stamp, stamp+.34)[1]['mode'], 'constant_velocity')

    def test_old_and_duplicate_times_cannot_manufacture_motion_evidence(self):
        p = PublicationPrediction()
        p.observe(10., 1, [1., 0.])
        p.observe(10.1, 1, [1., 0.])
        self.assertFalse(p.observe(9.9, 1, [-1., 0.]))
        for _ in range(10):
            p.observe(10.1, 1, [0., 1.])
        self.assertEqual(len(p.samples), 2)

    def test_prediction_is_pure_and_remains_continuous_at_the_same_posterior(self):
        p = self.turning()
        saved = [(t, d, v.copy()) for t, d, v in p.samples]
        positions = [p.predict([0., 0.], [0., 1.], 10.3, 10.3 + dt)[0][1]
                     for dt in (0., .1, .2, .3, .6, .9)]
        self.assertEqual(positions[0], 0.)
        self.assertTrue(all(a < b for a, b in zip(positions, positions[1:])))
        self.assertLess(positions[-1], .15)
        self.assertEqual([(t, d) for t, d, _ in saved], [(t, d) for t, d, _ in p.samples])
        for (_, _, a), (_, _, b) in zip(saved, p.samples):
            np.testing.assert_array_equal(a, b)

    def test_explicit_disable_keeps_cv_prediction(self):
        p = self.turning(PublicationPrediction(enabled=False))
        xy, meta = p.predict([0., 0.], [0., 1.], 10.3, 10.64)
        np.testing.assert_allclose(xy, [0., .34])
        self.assertEqual(meta['mode'], 'constant_velocity')


if __name__ == '__main__':
    unittest.main()
