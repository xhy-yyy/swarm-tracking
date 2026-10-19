#!/usr/bin/env python3
"""Offline regression for image-time yaw feedback and its target association."""
import math
import unittest
from unittest.mock import patch

import numpy as np

import test_tracking_publication as publication_tests
from test_vision_estimators import Estimator, FakeStream, synth
import swarm_fusion as sf
import swarm_vision as sv


class TrackingViewTest(unittest.TestCase):
    setUp = publication_tests.TrackingPublicationTest.setUp
    det = publication_tests.TrackingPublicationTest.det
    ack = publication_tests.TrackingPublicationTest.ack
    step = publication_tests.TrackingPublicationTest.step
    settle = publication_tests.TrackingPublicationTest.settle

    def image_det(self, color='green', did=1, x=10.0, method=sf.M_DEPTH,
                  bearing=0.12, pixels=-17.0):
        d = self.det(color, did, x, method=method)
        d.header.frame_id = 'world'
        d.image_bearing_valid = True
        d.image_bearing, d.image_error_x = bearing, pixels
        return d

    def views(self, did=1):
        return self.f.view_pubs[did - 1].messages

    def test_owner_view_is_immediate_and_preserves_image_epoch(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        before_state = len(self.f.state_pubs[0].messages)
        before_judge = len(self.f.actor_pubs['green'].messages)
        self.now += .025  # No assign or publish timer runs in this interval.
        d = self.image_det()
        d.header.stamp = sf.rospy.Time.from_sec(self.now - .005)
        self.f.det_cb(sf.DetectionArray(dets=[d]))
        self.assertEqual(len(self.views()), 1)
        msg = self.views()[0]
        self.assertEqual(msg.header.stamp, d.header.stamp)
        self.assertEqual(msg.header.frame_id, 'world')
        self.assertEqual((msg.assignment, msg.drone_id), (tr.assignment, 1))
        self.assertAlmostEqual(msg.bearing, d.image_bearing)
        self.assertEqual(msg.horizontal_error_px, -17.)
        self.assertAlmostEqual(msg.confidence, .95)
        self.assertEqual(len(self.f.state_pubs[0].messages), before_state)
        self.assertEqual(len(self.f.actor_pubs['green'].messages), before_judge)
        self.f.assign_cb(None)
        self.f.publish_cb(None)
        self.assertEqual(len(self.views()), 1)  # No cached-image timer republishing.

    def test_unassigned_nonowner_duplicate_old_and_rejected_frames_do_not_steer(self):
        d = self.image_det()
        self.f.det_cb(sf.DetectionArray(dets=[d]))
        self.assertEqual(self.views(), [])
        self.f.assign_cb(None)
        self.now += .1
        d = self.image_det()
        self.f.det_cb(sf.DetectionArray(dets=[d]))
        self.assertEqual(len(self.views()), 1)
        self.f.det_cb(sf.DetectionArray(dets=[d]))  # Same image, even if KF accepts it.
        old = self.image_det()
        old.header.stamp = sf.rospy.Time.from_sec(self.now - .05)
        self.f.det_cb(sf.DetectionArray(dets=[old]))
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(did=3)]))
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(x=100.)]))
        self.assertEqual(len(self.views()), 1)
        self.assertEqual(self.views(3), [])

    def test_camera_validity_and_finite_values_are_required_without_changing_kf(self):
        self.settle([('green', 1, 10)])
        for validity, bearing, pixels, confidence in (
                (False, .1, 3., .95), (True, float('nan'), 3., .95),
                (True, .1, float('nan'), .95), (True, .1, 3., float('nan'))):
            self.now += .1
            d = self.image_det(bearing=bearing, pixels=pixels)
            d.image_bearing_valid, d.conf = validity, confidence
            self.f.det_cb(sf.DetectionArray(dets=[d]))
            self.assertAlmostEqual(self.f.tracks['green'].last_meas, self.now)
        self.assertEqual(self.views(), [])

    def test_reassignment_binds_only_a_new_accepted_owner_observation(self):
        self.settle([('green', 1, 10)])
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det()]))
        old_assignment = self.views()[0].assignment
        tr = self.f.tracks['green']
        tr.assignment = 'green/new-owner-task'
        tr.assigned = 3
        self.f.assign_cb(None)
        self.assertEqual(len(self.views()), 1)
        self.assertEqual(self.views(3), [])
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(did=1)]))
        self.assertEqual(len(self.views()), 1)
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(did=3)]))
        self.assertEqual(self.views(3)[0].assignment, tr.assignment)
        self.assertNotEqual(self.views(3)[0].assignment, old_assignment)

    def test_monocular_feedback_requires_existing_navigation_acceptance(self):
        self.settle([('green', 1, 10)])
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(method=sf.M_JOINT)]))
        self.assertEqual(self.views(), [])  # Existing depth hold ignores this mono frame.
        self.now += 1.0
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(method=sf.M_JOINT)]))
        self.assertTrue(self.f.tracks['green'].bearing_only_nav)
        self.assertEqual(len(self.views()), 1)
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(method=sf.M_JOINT)]))
        self.assertEqual(len(self.views()), 2)

    def test_newer_observer_image_does_not_suppress_delayed_owner_yaw_feedback(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        for _ in range(20):
            self.now += .1
            owner = self.image_det(did=1, bearing=.12, pixels=-17.)
            observer = self.image_det(did=3, x=10.3, bearing=-.5, pixels=100.)
            owner.header.stamp = sf.rospy.Time.from_sec(self.now - .15)
            observer.header.stamp = sf.rospy.Time.from_sec(self.now - .02)
            self.f.det_cb(sf.DetectionArray(dets=[observer, owner]))
        self.assertGreater(len(self.views()), 15)
        self.assertEqual(self.views(3), [])
        self.assertEqual(self.views()[-1].assignment, tr.assignment)
        self.assertAlmostEqual(self.now - self.views()[-1].header.stamp.to_sec(), .15)
        self.assertTrue(all(abs(view.bearing - .12) < 1e-6 for view in self.views()))
        self.assertTrue(all(view.horizontal_error_px == -17. for view in self.views()))
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertAlmostEqual(tr.x[0], 10.0, places=5)

    def test_observer_cannot_switch_monocular_owner_to_depth_or_publish_its_view(self):
        self.settle([('green', 1, 50, 0.0, sf.M_JOINT)])
        tr = self.f.tracks['green']
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(did=3, x=10.)]))
        self.assertTrue(tr.bearing_only_nav)
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertAlmostEqual(tr.x[0], 50.0)
        self.assertEqual(self.views(), [])
        self.assertEqual(self.views(3), [])
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det(x=50., method=sf.M_JOINT)]))
        self.assertEqual(len(self.views()), 1)
        self.assertEqual(self.views()[0].assignment, tr.assignment)

    def test_two_red_boxes_use_accepted_track_identity_in_both_orders(self):
        self.settle([('red', 1, 10), ('red', 1, 40)], n=20)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        # The current dispatcher sends idle drone 2 toward the second target
        # using drone 1's shared observation. It still must not receive drone
        # 1's image-time steering feedback before observing that target itself.
        self.assertEqual((a.assigned, b.assigned), (1, 2))
        self.assertEqual(b.shared_source, 1)
        for boxes in ((40., 10.), (10., 40.)):
            self.now += .1
            ds = [self.image_det('red', 1, x, bearing=x / 100., pixels=x) for x in boxes]
            self.f.det_cb(sf.DetectionArray(dets=ds))
            self.assertEqual(self.views()[-1].assignment, a.assignment)
            self.assertAlmostEqual(self.views()[-1].bearing, .1)
            self.assertEqual(self.views()[-1].horizontal_error_px, 10.)
        self.assertEqual(len(self.views()), 2)
        self.assertEqual(self.views(2), [])

    def test_parallel_red_views_are_independent_of_judge_quality(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        before_a = len(self.f.actor_pubs['red1'].messages)
        before_b = len(self.f.actor_pubs['red2'].messages)
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[
            self.image_det('red', 2, 40, bearing=.4),
            self.image_det('red', 1, 10, bearing=.1)]))
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual(self.views(1)[0].assignment, a.assignment)
        self.assertEqual(self.views(2)[0].assignment, b.assignment)
        self.assertAlmostEqual(self.views(1)[0].bearing, .1)
        self.assertAlmostEqual(self.views(2)[0].bearing, .4)
        # A localisation quality failure stops only that track's judge stream;
        # both owners still receive their own accepted image-time yaw feedback.
        b.last_residual = self.f.max_pub_residual + 1.0
        self.ack(1)
        self.ack(2)
        self.f.publish_cb(None)
        self.assertEqual(len(self.f.actor_pubs['red1'].messages), before_a + 1)
        self.assertEqual(len(self.f.actor_pubs['red2'].messages), before_b)
        self.assertEqual(a.last_pub, self.now)
        self.assertLess(b.last_pub, self.now)

    def test_ambiguous_red_identity_suppresses_view(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        b.x[:2] = a.x[:2]
        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.image_det('red', 1, 10)]))
        self.assertGreater(a.identity_blocked_until, self.now)
        self.assertEqual(self.views(1), [])
        self.assertEqual(self.views(2), [])

    def test_later_ambiguity_in_batch_suppresses_earlier_accepted_red_view(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        self.now += .1
        own = self.image_det('red', 1, 10)
        own.header.stamp = sf.rospy.Time.from_sec(self.now - .05)
        ambiguous = self.image_det('red', 1, 25)
        ambiguous.header.stamp = sf.rospy.Time.from_sec(self.now - .01)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']

        def cost(tr, det, stamp):
            return .1 if det.x == 25. or tr is a else float('inf')

        with patch.object(self.f, '_red_cost', side_effect=cost):
            self.f.det_cb(sf.DetectionArray(dets=[own, ambiguous]))
        self.assertEqual(a.last_meas, own.header.stamp.to_sec())
        self.assertGreater(a.identity_blocked_until, self.now)
        self.assertEqual(self.views(), [])
        self.now += .6
        self.f.det_cb(sf.DetectionArray())
        self.assertEqual(self.views(), [])  # Blocked candidate is never cached for later.


class ImageBearingGeometryTest(unittest.TestCase):
    def quality(self, source='camera_imu', valid=True):
        return dict(source=source, depth_valid=valid, position_sigma=.1,
                    lever_arm_sigma=.03, time_position_sigma=0., attitude_sigma=0.,
                    rgb_depth_dt=0., speed=0.)

    def estimate(self, quality):
        estimator = Estimator()
        estimator.bearing_corr_deg = 1.5  # Localisation calibration must stay separate.
        box, depth, pos, rotation = synth((6., 1.), (0., 0.), 0.)
        # Deliberately offset the surface centroid: it must not replace the
        # image-centre bearing used by yaw feedback.
        with patch.object(sv, 'depth_surface_cluster', return_value=(
                np.array([6., 1.4]), dict(surface_scatter=0., reason='accepted_surface'))):
            result = estimator._estimate(FakeStream(), box, depth, pos, rotation,
                                         pose_quality=quality)
        self.assertIsNotNone(result)
        centre_u, centre_v = (box[0] + box[2]) / 2., (box[1] + box[3]) / 2.
        ray = estimator._ray_world(rotation, centre_u, centre_v)
        expected = math.atan2(ray[1], ray[0])
        return result, expected, centre_u - sv.CAMERA_CX

    def test_image_ray_uses_actual_camera_orientation_and_not_depth_bearing(self):
        result, expected, error_px = self.estimate(self.quality())
        self.assertTrue(result['image_bearing_valid'])
        self.assertAlmostEqual(result['image_bearing'], expected)
        self.assertAlmostEqual(result['image_error_x'], error_px)
        self.assertGreater(abs(result['bearing'] - result['image_bearing']), .03)

    def test_relay_or_missing_imu_cannot_supply_valid_image_feedback(self):
        for quality in (None, self.quality('relay_navigation', False),
                        self.quality('camera_imu', False)):
            result, _, _ = self.estimate(quality)
            self.assertFalse(result['image_bearing_valid'])


if __name__ == '__main__':
    unittest.main()
