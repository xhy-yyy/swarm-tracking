#!/usr/bin/env python3
"""离线验证一台机跟甲时共享乙坐标、空闲机接近并独立获取乙。"""
import unittest

import test_tracking_publication as fixture

sf = fixture.sf


class SharedTargetAssignmentTest(unittest.TestCase):
    setUp = fixture.TrackingPublicationTest.setUp
    det = fixture.TrackingPublicationTest.det
    ack = fixture.TrackingPublicationTest.ack
    step = fixture.TrackingPublicationTest.step
    settle = fixture.TrackingPublicationTest.settle
    pubs = fixture.TrackingPublicationTest.pubs
    step_red_boxes = fixture.TrackingPublicationTest.step_red_boxes

    def share_blue(self, method=sf.M_DEPTH):
        self.settle([('green', 1, 10)], n=6)
        green = self.f.tracks['green']
        token = green.assignment
        self.step([('green', 1, 10), ('blue', 1, 40, 0., method)])
        blue = self.f.tracks['blue']
        self.assertEqual((green.assigned, green.assignment), (1, token))
        self.assertEqual((blue.assigned, blue.shared_source), (2, 1))
        return blue

    def test_busy_camera_shares_extra_target_and_keeps_original_task(self):
        blue = self.share_blue()
        green = self.f.tracks['green']
        token, count = green.assignment, len(self.pubs('green'))
        self.assertEqual(self.f.drone_mode[1], sf.MODE_REACQUIRE)
        self.assertAlmostEqual(self.f.state_pubs[1].messages[-1].state.pose.pose.position.x, 40.)
        self.assertNotIn(2, blue.accepted_seen_by)
        self.assertFalse(self.f._tracking_ready(blue, self.now))
        self.assertEqual(self.pubs('blue'), [])

        for k in range(12):
            self.step([('green', 1, 10), ('blue', 1, 40. + .1 * k)])
        target = self.f.state_pubs[1].messages[-1]
        self.assertGreater(target.state.pose.pose.position.x, 40.8)
        self.assertAlmostEqual(target.measurement_stamp.to_sec(), self.now)
        self.assertEqual(target.assignment, blue.assignment)
        self.assertEqual((green.assigned, green.assignment), (1, token))
        self.assertGreater(len(self.pubs('green')), count)
        self.assertEqual(self.pubs('blue'), [])

    def test_shared_monocular_coordinates_are_navigation_only(self):
        blue = self.share_blue(sf.M_JOINT)
        self.step([('green', 1, 10), ('blue', 1, 42., 0., sf.M_JOINT)])
        target = self.f.state_pubs[1].messages[-1]
        self.assertTrue(target.bearing_only)
        self.assertTrue(target.depth_stamp.is_zero())
        self.assertAlmostEqual(target.state.pose.pose.position.x, 42.)
        self.assertEqual((blue.shared_source, blue.bearing_nav_drone), (1, 1))
        self.assertEqual(self.pubs('blue'), [])

    def test_remote_acquisition_has_time_to_travel_beyond_normal_loss_window(self):
        blue = self.share_blue()
        token = blue.assignment
        for _ in range(100):
            self.step([('green', 1, 10), ('blue', 1, 40.)])
        self.assertEqual((blue.assigned, blue.shared_source, blue.assignment), (2, 1, token))
        self.assertEqual(self.f.drone_mode[1], sf.MODE_REACQUIRE)
        self.assertNotIn(2, blue.accepted_seen_by)
        self.assertEqual(self.pubs('blue'), [])

    def test_slower_recipient_camera_acquires_with_real_timestamp_and_new_credential(self):
        blue = self.share_blue()
        old_token, old_stamp = blue.assignment, blue.last_meas
        self.now += .1
        recipient = self.det('blue', 2, 40.1)
        recipient.header.stamp = sf.rospy.Time.from_sec(self.now - .3)
        self.assertLess(recipient.header.stamp.to_sec(), old_stamp)
        self.f.det_cb(sf.DetectionArray(dets=[
            self.det('green', 1, 10), self.det('blue', 1, 40.), recipient]))
        self.f.assign_cb(None)
        self.assertEqual((blue.assigned, blue.shared_source, blue.last_meas_drone), (2, 0, 2))
        self.assertNotEqual(blue.assignment, old_token)
        self.assertEqual(blue.owner_observation_assignment, blue.assignment)
        target = self.f.state_pubs[1].messages[-1]
        self.assertEqual(target.assignment, blue.assignment)
        self.assertAlmostEqual(target.measurement_stamp.to_sec(), self.now - .3)
        self.assertAlmostEqual(target.state.pose.pose.position.x, 40.1)
        self.ack(2, old_token)
        self.f.publish_cb(None)
        self.assertFalse(self.f._tracking_ready(blue, self.now))
        self.assertEqual(self.pubs('blue'), [])
        self.ack(2)
        self.f.publish_cb(None)
        self.assertTrue(self.f._tracking_ready(blue, self.now))
        self.settle([('green', 1, 10), ('blue', 2, 40.1)], n=6)
        self.assertTrue(self.pubs('blue'))

        stamp, position = blue.last_meas, blue.x.copy()
        self.step([('green', 1, 10), ('blue', 1, 41.)])
        self.assertEqual(blue.last_meas, stamp)
        self.assertTrue((blue.x == position).all())
        self.assertEqual(blue.last_meas_drone, 2)

    def test_real_observer_is_preferred_over_nearer_unseeing_aircraft(self):
        self.settle([('green', 1, 10)], n=6)
        self.step([('green', 1, 10), ('blue', 1, 10.), ('blue', 2, 10.1)])
        blue = self.f.tracks['blue']
        self.assertEqual((blue.assigned, blue.shared_source, blue.last_meas_drone), (2, 0, 2))
        self.assertLess(abs(self.f.drone_pos[2][0] - 10.), abs(self.f.drone_pos[1][0] - 10.))

    def test_multiple_extra_people_use_different_idle_aircraft(self):
        self.settle([('green', 1, 10)], n=6)
        self.step([('green', 1, 10), ('blue', 1, 40), ('brown', 1, 20)])
        tracks = [self.f.tracks[s] for s in ('green', 'blue', 'brown')]
        self.assertEqual([tr.assigned for tr in tracks], [1, 2, 3])
        self.assertEqual([tr.shared_source for tr in tracks], [0, 1, 1])
        self.assertEqual(len({tr.assignment for tr in tracks}), 3)

    def test_shared_dispatch_respects_reject_and_not_yet_airborne_aircraft(self):
        self.settle([('green', 1, 10)], n=6)
        self.f.reject[(2, 'blue')] = self.now + 10.
        self.f.drone_pos[2][2] = 0.
        self.step([('green', 1, 10), ('blue', 1, 40)])
        self.assertEqual(self.f.tracks['blue'].assigned, 0)
        self.f.drone_pos[2][2] = 3.
        self.step([('green', 1, 10), ('blue', 1, 40)])
        self.assertEqual((self.f.tracks['blue'].assigned, self.f.tracks['blue'].shared_source), (3, 1))

    def test_all_busy_queue_then_dispatch_when_a_recipient_becomes_idle(self):
        self.settle([('green', 1, 10), ('blue', 2, 40), ('brown', 3, 20)], n=6)
        self.step([('green', 1, 10), ('blue', 2, 40), ('brown', 3, 20), ('white', 1, 35)])
        self.assertEqual(self.f.tracks['white'].assigned, 0)
        self.f._release(self.f.tracks['blue'], self.now)
        self.f.reject[(2, 'blue')] = self.now + 10.
        self.ack(2, '')
        self.f.assign_cb(None)
        white = self.f.tracks['white']
        self.assertEqual((white.assigned, white.shared_source), (2, 1))
        self.assertEqual(self.f.tracks['green'].assigned, 1)
        self.assertEqual(self.f.tracks['brown'].assigned, 3)

    def test_stale_or_rejected_coordinates_cannot_start_shared_dispatch(self):
        self.settle([('green', 1, 10)], n=6)
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        self.step([('green', 1, 10), ('blue', 1, 40)])
        blue = self.f.tracks['blue']
        stamp = blue.last_meas
        self.now += self.f.owner_seen_timeout + .1
        self.f.drone_pos[1][2] = 3.
        self.f.det_cb(sf.DetectionArray(dets=[self.det('blue', 1, 100)]))
        self.f.assign_cb(None)
        self.assertEqual(blue.last_meas, stamp)
        self.assertEqual(blue.assigned, 0)

    def test_shared_acquisition_expires_even_when_source_keeps_seeing_target(self):
        blue = self.share_blue()
        self.now = blue.assigned_since + self.f.shared_acquire_timeout + .1
        self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, 10), self.det('blue', 1, 40)]))
        self.ack(2)
        self.f.assign_cb(None)
        self.assertEqual((blue.assigned, blue.shared_source), (0, 0))
        self.assertIn(2, self.f.retiring)
        self.assertEqual(self.f.drone_mode[1], sf.MODE_OFF)

    def test_source_loss_cannot_turn_old_coordinates_into_fresh_measurements(self):
        blue = self.share_blue()
        stamp = blue.last_meas
        self.now += 3.
        self.ack(2)
        self.f.assign_cb(None)
        target = self.f.state_pubs[1].messages[-1]
        self.assertEqual(target.measurement_stamp.to_sec(), stamp)
        self.assertLessEqual(target.state.header.stamp.to_sec(), stamp + self.f.pub_max_age)
        self.now += self.f.lost_timeout
        self.f.assign_cb(None)
        self.assertEqual((blue.assigned, blue.shared_source), (0, 0))

    def test_recipient_at_frozen_shared_point_returns_to_patrol(self):
        # 2026-10-15 实跑: 发现机不再看到后, 接手机在冻结点旁空等到 25s。
        blue = self.share_blue()
        self.assertLess(abs(self.f.drone_pos[1][0] - 40.), self.f.reacq_arrive_radius)
        frozen_at = self.now
        while blue.assigned and self.now < frozen_at + 10.:
            self.step([('green', 1, 10)])
        waited = self.now - frozen_at
        self.assertEqual((blue.assigned, blue.shared_source, blue.blind_acquire), (0, 0, False))
        self.assertGreaterEqual(waited, self.f.reacq_frozen_age + self.f.reacq_arrive_hold)
        self.assertLess(waited, self.f.reacq_frozen_age + self.f.reacq_arrive_hold + .5)
        self.assertIn(2, self.f.retiring)
        self.assertEqual(self.f.drone_mode[1], sf.MODE_OFF)
        self.assertEqual(blue.queued_meas, blue.last_meas)
        self.assertEqual(self.f.tracks['green'].assigned, 1)

    def test_recipient_still_flying_to_frozen_point_keeps_task(self):
        self.f.drone_pos[1][:2] = (120., 0.)
        self.f.drone_pos[2][:2] = (5., 90.)
        blue = self.share_blue()
        for _ in range(150):
            self.step([('green', 1, 10)])
        self.assertEqual((blue.assigned, blue.shared_source), (2, 1))
        self.assertEqual(self.f.drone_mode[1], sf.MODE_REACQUIRE)
        # 飞到点附近才开始计时。
        self.f.drone_pos[1][:2] = (44., 0.)
        for _ in range(int(self.f.reacq_arrive_hold / .1) - 2):
            self.step([('green', 1, 10)])
        self.assertEqual(blue.assigned, 2)
        self.settle([('green', 1, 10)], n=4)
        self.assertEqual(blue.assigned, 0)

    def test_recipient_at_point_waits_while_source_keeps_seeing(self):
        blue = self.share_blue()
        for _ in range(60):
            self.step([('green', 1, 10), ('blue', 1, 40)])
        self.assertEqual((blue.assigned, blue.shared_source), (2, 1))

    def test_stale_shared_source_is_replaced_by_another_drone_seeing_target(self):
        # 3 号机正跟 brown, 也看到 blue; 发现机 1 已看不到 -> 改用 3 号机坐标引导接手机。
        self.f.drone_pos[1][:2] = (120., 0.)
        self.settle([('green', 1, 10), ('brown', 3, 20)], n=6)
        self.step([('green', 1, 10), ('brown', 3, 20), ('blue', 1, 40)])
        blue = self.f.tracks['blue']
        self.assertEqual((blue.assigned, blue.shared_source), (2, 1))
        token = blue.assignment
        for k in range(30):
            self.step([('green', 1, 10), ('brown', 3, 20), ('blue', 3, 40. + .05 * k)])
        self.assertEqual((blue.assigned, blue.shared_source, blue.assignment), (2, 3, token))
        self.assertTrue(blue.blind_acquire)
        target = self.f.state_pubs[1].messages[-1]
        self.assertGreater(target.state.pose.pose.position.x, 41.)
        self.assertAlmostEqual(target.measurement_stamp.to_sec(), self.now)
        self.assertEqual(self.pubs('blue'), [])
        self.assertEqual(self.f.tracks['brown'].assigned, 3)

    def test_blind_owner_hands_target_to_closer_idle_observer(self):
        self.settle([('green', 1, 10)])
        green = self.f.tracks['green']
        self.f.drone_pos[0][:2] = (30., 0.)   # 负责机丢框后飘远
        for _ in range(50):
            self.step([('green', 3, 10)])
            if green.assigned == 3:
                break
        self.assertEqual(green.assigned, 3)
        self.assertLess(self.now - green.accepted_seen_by[1], self.f.owner_release_timeout)
        self.assertEqual(green.last_meas_drone, 3)
        self.settle([('green', 3, 10)], n=6)
        self.assertTrue(self.f._tracking_ready(green, self.now))

    def test_owner_still_seeing_candidates_is_not_handed_over(self):
        self.settle([('green', 1, 10)])
        green = self.f.tracks['green']
        token = green.assignment
        self.f.drone_pos[0][:2] = (30., 0.)
        for _ in range(25):   # 负责机看到 6m 外的新位置(门控外, 等确认), 3 号机看到旧位置
            self.step([('green', 1, 16), ('green', 3, 10)])
        self.assertEqual((green.assigned, green.assignment, green.shared_source), (1, token, 0))

    def lock_green_then_see_blue(self, lock_steps):
        # 1 号机锁定 green 且同时看见 35m 外的 blue; 两架空闲机都在 80m 以外。
        self.f.drone_pos[1][:2] = (120., 0.)
        self.f.drone_pos[2][:2] = (5., 90.)
        self.settle([('green', 1, 10)], n=lock_steps)
        self.step([('green', 1, 10), ('blue', 1, 40)])
        return self.f.tracks['green'], self.f.tracks['blue']

    def test_nearly_done_close_discoverer_takes_target_instead_of_far_idle_aircraft(self):
        green, blue = self.lock_green_then_see_blue(150)
        self.assertGreater(green.lock_seconds(self.now), 13.0)
        self.assertEqual(blue.assigned, 0)
        self.assertEqual(self.f.observer_waiting.get('blue'), 1)
        # green 被裁判删除: 1 号机消除确认后空出, 继续看见 blue 就直接接手。
        for _ in range(60):
            self.step([('blue', 1, 40)])
            if blue.assigned:
                break
        self.assertEqual(green.status, sf.ST_ELIMINATED)
        self.assertEqual((blue.assigned, blue.shared_source), (1, 0))
        self.assertNotIn('blue', self.f.observer_waiting)

    def test_discoverer_far_from_finishing_still_shares_immediately(self):
        green, blue = self.lock_green_then_see_blue(30)
        self.assertEqual((blue.assigned, blue.shared_source), (2, 1))
        self.assertEqual(green.assigned, 1)

    def test_waiting_ends_as_soon_as_discoverer_stops_seeing_target(self):
        green, blue = self.lock_green_then_see_blue(150)
        self.assertEqual(blue.assigned, 0)
        for _ in range(int(self.f.observer_wait_seen / .1) + 2):
            self.step([('green', 1, 10)])
        self.assertIn(blue.assigned, (2, 3))
        self.assertEqual((blue.shared_source, green.assigned), (1, 1))

    def test_close_red_handoff_preserves_current_independent_person_evidence(self):
        for _ in range(25):
            self.step_red_boxes()
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual(a.assigned, 1)
        self.assertIn(b.assigned, (2, 3))
        self.assertEqual(b.shared_source, 1)
        owner, epoch, old_token = b.assigned, b.identity_epoch, b.assignment
        self.assertTrue(self.f._reds_distinct(a, b, self.now))
        self.step([('red', owner, 12.8)])
        self.assertEqual((b.assigned, b.shared_source), (owner, 0))
        self.assertEqual(b.identity_epoch, epoch)
        self.assertNotEqual(b.assignment, old_token)
        self.assertTrue(self.f._reds_distinct(a, b, self.now))
        self.assertNotIn(owner, self.f.retiring)


if __name__ == '__main__':
    unittest.main()
