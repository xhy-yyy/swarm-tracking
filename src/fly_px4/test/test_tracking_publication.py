#!/usr/bin/env python3
"""离线执行真实检测、分配、回执和发布回调；不启动裁判/Gazebo，不读取真值。"""
import ast
import copy
import json
import math
import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import swarm_fusion as sf


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class OfflineRedJudge:
    """执行现用裁判的真实红人回调；时钟/删除服务全部为离线替身。"""

    def __init__(self, now, actor_positions):
        path = Path(os.environ.get(
            'SCORE_CAL_PATH', str(Path(__file__).resolve().parents[2] / 'swarm_simulation/scripts/competition/score_cal.py')))
        if not path.is_file():
            raise unittest.SkipTest('现用裁判不存在: %s' % path)
        functions = {
            '_publish_score', '_progress_score', '_finish', '_reset_detection',
            '_delete_actor', '_process_red_detection', 'actor_info1_callback',
            'actor_info2_callback',
        }
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        contract_names = {'actor_num', 'actor_id_dict', 'err_threshold',
                          'DETECTION_INTERVAL', 'DETECTION_DURATION'}
        contract = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in tree.body
            if isinstance(node, ast.Assign) and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in contract_names
        }
        if set(contract) != contract_names:
            raise AssertionError('裁判缺少预期计分常量: %s' % path)
        selected = [node for node in tree.body
                    if isinstance(node, ast.FunctionDef) and node.name in functions]
        if {node.name for node in selected} != functions:
            raise AssertionError('裁判缺少预期红人回调: %s' % path)
        self.deleted = {}
        self.state = {
            'actors_pos': [None] * 6, 'left_actors': list(range(6)),
            'find_actors': list(range(6)), 'count_flag': [False] * 6,
            'topic_arrive_time': [0.0] * 6, 'find_time': [0.0] * 6,
            'find_actor_pub': [], 'mission_finished': False,
            'find_finish': 0, 'target_finish': 0,
            'score': 0.0, 'time_usage': 0.0, 'start_time': now(),
            'sensor_cost': 0.0, 'uav_loss_count': 0, 'uav_loss_penalty': 100.0,
            'flag_1': 0, 'flag_2': 1, '_now': now,
            'rospy': SimpleNamespace(signal_shutdown=lambda reason: None),
            'print': lambda *args, **kwargs: None,
            **contract,
        }
        for actor_id, (x, y) in actor_positions.items():
            self.state['actors_pos'][actor_id] = SimpleNamespace(x=x, y=y)

        def delete_model(name):
            actor_id = int(name.split('_')[-1])
            self.deleted[actor_id] = (now(), self.state['find_time'][actor_id])
            return SimpleNamespace(success=True)

        self.state['del_model'] = delete_model
        module = ast.Module(body=selected, type_ignores=[])
        exec(compile(module, str(path), 'exec'), self.state)

    def receive(self, slot, msg):
        callback = 'actor_info1_callback' if slot == 'red1' else 'actor_info2_callback'
        self.state[callback](msg)


class TrackingPublicationTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.params = {'~drone_num': 3, '~warmup_sec': 0.0,
                       '~publish_on_measurement': False,  # Timer-path regression cases below.
                       '~elim_confirm_gap': 2.0, '~lost_timeout': 25.0,
                       '~owner_release_timeout': 8.0,
                       '~completion_confirm_sec': 1.0}
        patches = [
            patch.object(sf.rospy, 'get_param', side_effect=lambda k, d: self.params.get(k, d)),
            patch.object(sf.rospy, 'get_time', side_effect=lambda: self.now),
            patch.object(sf.rospy.Time, 'now', side_effect=lambda: sf.rospy.Time.from_sec(self.now)),
            patch.object(sf.rospy, 'Publisher', side_effect=lambda *a, **k: Publisher()),
            patch.object(sf.rospy, 'Subscriber'), patch.object(sf.rospy, 'Timer'),
            patch.object(sf.rospy, 'loginfo'), patch.object(sf.rospy, 'logwarn'),
            patch.object(sf.rospy, 'logwarn_throttle'),
            patch.object(sf.SwarmFusion, 'check_sole_publisher'),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.f = sf.SwarmFusion()
        self.f.drone_pos = [np.array([5., 0., 3.]), np.array([35., 0., 3.]),
                            np.array([5., 2., 3.])]

    def det(self, color, did, x, y=0.0, method=sf.M_DEPTH):
        d = sf.Detection()
        d.header.stamp = sf.rospy.Time.from_sec(self.now)
        d.drone_id, d.cls_id = did, sf.CLS_NAMES.index(color)
        d.x, d.y, d.conf = x, y, 0.95
        d.sigma_along, d.sigma_cross = 0.15, 0.1
        pos = self.f.drone_pos[did - 1]
        d.bearing = math.atan2(y - pos[1], x - pos[0])
        d.slant = max(math.hypot(x - pos[0], y - pos[1]), 5.0)
        d.method = method
        return d

    def ack(self, did, token=None, age=0.0):
        if token is None:
            token = next((tr.assignment for tr in self.f.tracks.values()
                          if tr.assigned == did), '')
        self.f.tracking_cb(sf.Header(stamp=sf.rospy.Time.from_sec(self.now - age),
                                     frame_id=token), did - 1)

    def step(self, observations=(), acknowledge=True, dt=0.1):
        self.now = round(self.now + dt, 5)
        arr = sf.DetectionArray()
        arr.dets = [self.det(*args) for args in observations]
        self.f.det_cb(arr)
        self.f.assign_cb(None)
        if acknowledge:
            for did in range(1, self.f.drone_num + 1):
                self.ack(did)
        self.f.publish_cb(None)

    def settle(self, observations, n=15, acknowledge=True):
        for _ in range(n):
            self.step(observations, acknowledge=acknowledge)

    def pubs(self, slot):
        return self.f.actor_pubs[slot].messages

    def step_red_boxes(self, xs=(10., 12.8), acknowledge=True):
        self.now = round(self.now + .1, 5)
        dets = []
        for x in xs:
            d = self.det('red', 1, x)
            d.u, d.box_w, d.v_top, d.v_bot = 100. + x * 10., 10., 100., 180.
            dets.append(d)
        self.f.det_cb(sf.DetectionArray(dets=dets))
        self.f.assign_cb(None)
        if acknowledge:
            for did in range(1, self.f.drone_num + 1):
                self.ack(did)
        self.f.publish_cb(None)

    def test_one_camera_close_reds_publish_both_without_second_owner(self):
        # 没有其他已起飞的空闲机，仍验证一台相机可上报两人的独立深度。
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        for _ in range(25):
            self.step_red_boxes()
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual((a.assigned, b.assigned), (1, 0))
        token = a.assignment
        before = {s: len(self.pubs(s)) for s in ('red1', 'red2')}
        for _ in range(20):
            self.step_red_boxes((12.8, 10.))  # 框输出顺序变化不换人。
        self.assertEqual((a.assigned, a.assignment, b.assigned), (1, token, 0))
        self.assertTrue(self.f._reds_distinct(a, b, self.now))
        self.assertTrue(self.f._judge_authorized(b, self.now))
        for s, x in (('red1', 10.), ('red2', 12.8)):
            self.assertEqual(len(self.pubs(s)) - before[s], 20)
            self.assertTrue(all(m.cls == 'red' and abs(m.x - x) < .1
                                for m in self.pubs(s)))
            self.assertGreater(self.f.tracks[s].lock_seconds(self.now), 2.)

    def test_close_red_measurement_event_publishes_both_immediately(self):
        for _ in range(20):
            self.step_red_boxes()
        self.f.publish_on_measurement = True
        before = {s: len(self.pubs(s)) for s in ('red1', 'red2')}
        self.now += .025
        ds = [self.det('red', 1, x) for x in (10., 12.8)]
        for d in ds:
            d.u, d.box_w, d.v_top, d.v_bot = 100. + d.x * 10., 10., 100., 180.
        self.f.det_cb(sf.DetectionArray(dets=ds))
        for s in before:
            self.assertEqual(len(self.pubs(s)), before[s] + 1)

    def test_close_moving_reds_keep_separate_coordinates_and_one_owner(self):
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        for i in range(80):
            xs = (10. + .08 * i, 12.8 + .08 * i)
            self.step_red_boxes(xs if i % 2 else xs[::-1])
        tracks = sorted((self.f.tracks['red1'], self.f.tracks['red2']),
                        key=lambda t: t.x[0])
        self.assertAlmostEqual(tracks[1].x[0] - tracks[0].x[0], 2.8, delta=.05)
        self.assertEqual(sum(bool(t.assigned) for t in tracks), 1)
        for tr, expected in zip(tracks, (16.32, 19.12)):
            self.assertAlmostEqual(self.pubs(tr.slot)[-1].x, expected, delta=.1)
            self.assertGreater(tr.lock_seconds(self.now), 6.)

    def test_red_counters_continue_when_flight_assignment_is_released(self):
        self.settle([('red', 1, 10.), ('red', 1, 40.)], n=25)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        starts = (a.pub_run_start, b.pub_run_start)
        self.f._release(a, self.now)
        self.settle([('red', 1, 10.), ('red', 1, 40.)], n=5)
        self.assertEqual((a.pub_run_start, b.pub_run_start), starts)
        self.assertEqual((a.last_pub, b.last_pub), (self.now, self.now))

    def test_deleted_red_tail_blacklist_does_not_hide_nearby_survivor(self):
        for _ in range(25):
            self.step_red_boxes((10., 11.5))
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        a.status = sf.ST_ELIMINATED
        self.f._release(a, self.now)
        self.f.blacklist.append((10., 0., 2., self.now + 3., 'red'))
        before = len(self.pubs('red2'))
        self.step_red_boxes((11.5,))
        self.assertEqual(len(self.pubs('red2')), before + 1)
        self.assertAlmostEqual(b.last_meas, self.now, places=8)
        self.assertEqual(a.status, sf.ST_ELIMINATED)

    def test_duplicate_boxes_cannot_reuse_previous_close_pair_evidence(self):
        for _ in range(25):
            self.step_red_boxes()
        before = {s: len(self.pubs(s)) for s in ('red1', 'red2')}
        self.f.publish_on_measurement = True
        self.now += .1
        ds = [self.det('red', 1, x) for x in (10., 12.8)]
        for d in ds:
            d.u, d.box_w, d.v_top, d.v_bot = 200., 20., 100., 180.
        self.f.det_cb(sf.DetectionArray(dets=ds))
        self.f.assign_cb(None)
        self.f.publish_cb(None)
        self.assertEqual({s: len(self.pubs(s)) for s in before}, before)

    def test_duplicate_image_boxes_do_not_create_second_close_red(self):
        for _ in range(25):
            self.now += .1
            ds = [self.det('red', 1, x) for x in (10., 12.8)]
            for d in ds:
                d.u, d.box_w, d.v_top, d.v_bot = 200., 20., 100., 180.
            self.f.det_cb(sf.DetectionArray(dets=ds))
            self.f.assign_cb(None)
            self.ack(1)
            self.f.publish_cb(None)
        self.assertTrue(self.f.tracks['red1'].inited)
        self.assertFalse(self.f.tracks['red2'].inited)
        self.assertEqual(self.pubs('red2'), [])

    def test_two_cameras_nearby_do_not_prove_two_people(self):
        self.settle([('red', 1, 10.), ('red', 2, 12.8)], n=25)
        self.assertEqual(sum(t.inited for t in self.f.tracks.values()), 1)
        self.assertEqual(self.pubs('red2'), [])

    def test_close_pair_evidence_expires_and_does_not_survive_reinitialization(self):
        for _ in range(20):
            self.step_red_boxes()
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertTrue(self.f._reds_distinct(a, b, self.now))
        self.assertFalse(self.f._reds_distinct(a, b, self.now + 2.1))
        a.init_from(self.det('red', 1, 10.), self.now)
        self.assertFalse(self.f._reds_distinct(a, b, self.now))

    def test_unassigned_red_depth_loss_stops_only_its_publication_when_distinct(self):
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        self.settle([('red', 1, 10.), ('red', 1, 40.)], n=20)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual((a.assigned, b.assigned), (1, 0))
        self.assertTrue(self.pubs('red2'))
        self.settle([('red', 1, 10.), ('red', 1, 40., 0., sf.M_JOINT)], n=20)
        count = len(self.pubs('red2'))
        self.step([('red', 1, 10.), ('red', 1, 40., 0., sf.M_JOINT)])
        self.assertEqual(len(self.pubs('red2')), count)
        self.assertEqual(a.last_pub, self.now)
        self.assertEqual(b.lock_seconds(self.now), 0.)

    def test_one_camera_close_reds_eliminate_both_with_current_judge(self):
        for swapped in (False, True):
            with self.subTest(swapped=swapped):
                self.now = 100.
                self.f = sf.SwarmFusion()
                self.f.elim_confirm_gap = 5.
                # 双红计分与交换身份验证独立于其他飞机接手：只有1号机起飞。
                self.f.drone_pos = [np.array([5., 0., 3.]),
                                    np.array([5., 0., 0.]), np.array([5., 0., 0.])]
                identities = {'red1': 5 if swapped else 4,
                              'red2': 4 if swapped else 5}
                judge = OfflineRedJudge(lambda: self.now,
                                        {identities['red1']: (10., 0.),
                                         identities['red2']: (12.8, 0.)})
                delivered = {'red1': 0, 'red2': 0}
                for _ in range(240):
                    xs = [x for s, x in (('red1', 10.), ('red2', 12.8))
                          if identities[s] not in judge.deleted]
                    self.step_red_boxes(xs)
                    self.assertLessEqual(sum(bool(t.assigned)
                                             for t in self.f.tracks.values()), 1)
                    for s in delivered:
                        for msg in self.pubs(s)[delivered[s]:]:
                            judge.receive(s, msg)
                        delivered[s] = len(self.pubs(s))
                self.assertEqual(set(judge.deleted), {4, 5})
                for removed, started in judge.deleted.values():
                    self.assertGreaterEqual(removed - started, 15.)
                    self.assertLessEqual(removed - started, 15.11)
                self.assertEqual([self.f.tracks[s].status for s in delivered],
                                 [sf.ST_ELIMINATED, sf.ST_ELIMINATED])

    def test_new_depth_publishes_between_timer_ticks_and_only_its_updated_track(self):
        self.settle([('green', 1, 10), ('blue', 2, 40)])
        self.f.publish_on_measurement = True
        before_green, before_blue = len(self.pubs('green')), len(self.pubs('blue'))
        self.now += .025  # Next regular timer tick is still 75 ms away.
        d = self.det('green', 1, 10.01)
        d.header.stamp = sf.rospy.Time.from_sec(self.now-.005)
        self.f.det_cb(sf.DetectionArray(dets=[d]))
        self.assertEqual(len(self.pubs('green')), before_green+1)
        self.assertEqual(len(self.pubs('blue')), before_blue)
        diag = json.loads(self.f.diagnostics_pub.messages[-1].data)
        self.assertEqual(diag['publication_trigger'], 'measurement')
        self.assertEqual(diag['state_stamp'], self.now)
        self.assertEqual(diag['vision_receive_stamp'], self.now)
        self.assertEqual(diag['measurement_update_stamp'], self.now)
        # Coalesce a timer callback at the identical clock stamp for the same
        # posterior; a later timer still publishes without a new observation.
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before_green+1)
        self.now += .075
        self.ack(1)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before_green+2)
        self.assertEqual(json.loads(self.f.diagnostics_pub.messages[-1].data)['publication_trigger'], 'timer')

    def test_turn_guard_bounds_published_mean_without_changing_kf_or_cadence(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        self.now += .025
        tr.last_meas = tr.last_pred = self.now - .34
        tr.last_depth_t = tr.last_meas
        tr.accepted_seen_by[1] = tr.last_meas
        tr.x = np.array([10., 0., 0., 1.])
        tr.P = np.eye(4) * .02
        tr.publication_predictor.reset()
        for dt, velocity in [(-.3, [1., 0.]), (-.2, [.8, .2]),
                             (-.1, [.6, .4]), (0., [0., 1.])]:
            tr.publication_predictor.observe(tr.last_meas+dt, 1, velocity)
        x_before, P_before = tr.x.copy(), tr.P.copy()
        cv_state, cv_cov = tr.prediction(self.now)
        before = len(self.pubs('green'))
        self.ack(1)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before+1)
        self.assertLess(self.pubs('green')[-1].y, .15)
        self.assertAlmostEqual(cv_state[1], .34)
        np.testing.assert_array_equal(tr.x, x_before)
        np.testing.assert_array_equal(tr.P, P_before)
        diag = json.loads(self.f.diagnostics_pub.messages[-1].data)
        self.assertEqual(diag['publication_prediction']['mode'], 'turn_velocity_decay')
        self.assertAlmostEqual(diag['publication_prediction']['measurement_age'], .34)
        np.testing.assert_allclose(diag['cov'], cv_cov[:2, :2])
        self.now += .1
        self.ack(1)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before+2)
        # Guarding the mean cannot make expired/uncertain data publishable.
        self.f.pub_max_age = .4
        self.now += .1
        self.ack(1)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before+2)

    def test_only_accepted_depth_updates_contribute_to_turn_history(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        saved = [(t, d, v.copy()) for t, d, v in tr.publication_predictor.samples]
        for method, x, delta in [(sf.M_JOINT, 10., .02), (sf.M_DEPTH, 100., .04)]:
            self.now += delta
            self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, x, method=method)]))
        old = self.det('green', 1, 10.)
        old.header.stamp = sf.rospy.Time.from_sec(tr.last_meas-.1)
        self.f.det_cb(sf.DetectionArray(dets=[old]))
        self.assertEqual(len(saved), len(tr.publication_predictor.samples))
        for (ta, da, va), (tb, db, vb) in zip(saved, tr.publication_predictor.samples):
            self.assertEqual((ta, da), (tb, db))
            np.testing.assert_array_equal(va, vb)

    def test_measurement_event_keeps_identity_and_quality_gates(self):
        self.settle([('green', 1, 10)])
        self.f.publish_on_measurement = True
        before = len(self.pubs('green'))
        self.now += .025
        self.ack(1, 'green/incorrect-assignment')
        self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, 10)]))
        self.assertEqual(len(self.pubs('green')), before)
        self.ack(1)
        self.f.max_pos_std = .01
        self.now += .025
        self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, 10)]))
        self.assertEqual(len(self.pubs('green')), before)

    def test_mono_rejected_and_old_measurements_do_not_trigger_extra_publications(self):
        self.settle([('green', 1, 10)])
        self.f.publish_on_measurement = True
        before = len(self.pubs('green'))
        self.now += .025
        mono = self.det('green', 1, 10, method=sf.M_JOINT)
        rejected = self.det('green', 1, 100)
        old = self.det('green', 1, 10)
        old.header.stamp = sf.rospy.Time.from_sec(self.f.tracks['green'].last_meas-.1)
        for d in (mono, rejected, old):
            self.f.det_cb(sf.DetectionArray(dets=[d]))
            self.assertEqual(len(self.pubs('green')), before)

    def test_measurement_events_publish_each_red_to_its_own_topic(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        self.f.publish_on_measurement = True
        before = {slot: len(self.pubs(slot)) for slot in ('red1', 'red2')}
        self.now += .025
        arr = sf.DetectionArray(dets=[self.det('red', 2, 40), self.det('red', 1, 10)])
        self.f.det_cb(arr)
        self.assertEqual(len(self.pubs('red1')), before['red1']+1)
        self.assertEqual(len(self.pubs('red2')), before['red2']+1)
        self.assertEqual(self.f.tracks['red1'].last_pub, self.now)
        self.assertEqual(self.f.tracks['red2'].last_pub, self.now)
        self.assertLess(abs(self.pubs('red1')[-1].x-10), .1)
        self.assertLess(abs(self.pubs('red2')[-1].x-40), .1)

        # 单个红人的新深度只触发自己的入口；不会广播到另一人的入口。
        before = {slot: len(self.pubs(slot)) for slot in ('red1', 'red2')}
        self.now += .025
        self.f.det_cb(sf.DetectionArray(dets=[self.det('red', 2, 40)]))
        self.assertEqual(len(self.pubs('red1')), before['red1'])
        self.assertEqual(len(self.pubs('red2')), before['red2']+1)

    def test_measurement_events_and_timer_keep_publications_during_five_hz_depth(self):
        self.settle([('green', 1, 10)])
        self.f.publish_on_measurement = True
        publication_times = []
        for i in range(200):
            self.now += .1
            self.ack(1)
            self.f.publish_cb(None)
            publication_times.append(self.now)
            if i % 2 == 0:
                self.now += .025
                self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, 10)]))
                publication_times.append(self.now)
                self.now -= .025
        actual = [json.loads(m.data)['state_stamp'] for m in self.f.diagnostics_pub.messages]
        actual = [t for t in actual if t >= publication_times[0]]
        self.assertEqual(len(actual), len(publication_times))
        self.assertLessEqual(max(np.diff(actual)), .100001)

    def test_good_measurement_does_not_wait_to_become_stale_before_publication(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        # Good position posterior, but uncertain velocity plus the real image
        # pipeline delay increases covariance at publication time.
        tr.P = np.diag([0.03, 0.03, 1.0, 1.0])
        self.now += 0.25
        self.ack(1)
        self.assertLess(tr.pos_std, self.f.max_pos_std)
        self.assertGreater(tr.predicted_pos_std(self.now), self.f.max_pos_std)
        self.assertLess(tr.predicted_pos_std(self.now), self.f.pub_max_pos_std)
        before = len(self.pubs('green'))
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before + 1)
        self.now += 0.16  # Crossing 0.4 s must not suddenly grant permission.
        self.ack(1)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before + 2)

    def test_delayed_depth_updates_keep_ten_hz_publication(self):
        self.settle([('green', 1, 10)])
        samples = []
        longest_run = 0.0
        for i in range(240):
            self.now = round(self.now + 0.1, 5)
            if i % 2 == 0:  # 5 Hz accepted measurements, 240 ms pipeline.
                d = self.det('green', 1, 10)
                d.header.stamp = sf.rospy.Time.from_sec(self.now - 0.24)
                d.sigma_along, d.sigma_cross = 0.30, 0.17
                self.f.det_cb(sf.DetectionArray(dets=[d]))
            self.f.assign_cb(None)
            self.ack(1)
            before = len(self.pubs('green'))
            self.f.publish_cb(None)
            longest_run = max(longest_run, self.f.tracks['green'].lock_seconds(self.now))
            if i >= 40:  # Let velocity covariance settle first.
                samples.append(len(self.pubs('green')) - before)
        self.assertEqual(sum(samples), len(samples))
        # No judge is deleting the synthetic person; its response watchdog may
        # later reset the shadow counter even though publication stayed live.
        self.assertGreater(longest_run, 15.0)

    def test_bad_posterior_is_not_authorized_by_getting_older(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        tr.P = np.diag([0.10, 0.10, 0.01, 0.01])
        self.now += 0.45
        self.ack(1)
        before = len(self.pubs('green'))
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before)

    def test_incidental_colors_never_publish_until_their_owner_tracks(self):
        obs = [('green', 1, 10), ('blue', 1, 20), ('brown', 1, 30)]
        self.settle(obs)
        self.assertTrue(self.pubs('green'))
        for s in ('blue', 'brown'):
            self.assertTrue(self.f.tracks[s].inited)
            tr = self.f.tracks[s]
            self.assertEqual(tr.assigned, 2 if s == 'blue' else 3)
            self.assertEqual(tr.shared_source, 1)
            self.assertEqual(self.f.drone_mode[tr.assigned - 1], sf.MODE_REACQUIRE)
            self.assertFalse(self.f._tracking_ready(tr, self.now))
            self.assertEqual(self.pubs(s), [])
            self.assertIsNone(self.f.tracks[s].pub_run_start)
        self.settle(obs + [('blue', 2, 20)])
        self.assertEqual(self.f.tracks['blue'].assigned, 2)
        self.assertTrue(self.pubs('blue'))
        self.assertEqual(self.pubs('brown'), [])

    def test_matching_fresh_actual_track_ack_is_required(self):
        obs = [('green', 1, 10)]
        self.settle(obs, acknowledge=False)
        tr = self.f.tracks['green']
        self.assertEqual(tr.assigned, 1)
        self.assertEqual(self.pubs('green'), [])
        self.ack(1, 'green/old-assignment')
        self.f.publish_cb(None)
        self.assertEqual(self.pubs('green'), [])
        self.ack(1)
        self.f.publish_cb(None)
        self.assertTrue(self.pubs('green'))
        count = len(self.pubs('green'))
        for _ in range(8):
            self.step(obs, acknowledge=False)
        count = len(self.pubs('green'))
        self.step(obs, acknowledge=False)
        self.assertEqual(len(self.pubs('green')), count)
        # 过期/乱序回执不能重新获得许可。
        self.ack(1, tr.assignment, age=1.0)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), count)

    def test_two_reds_track_and_publish_in_parallel_with_independent_counters(self):
        obs = [('red', 1, 10), ('red', 2, 40)]
        self.settle(obs)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual((a.assigned, b.assigned), (1, 2))
        self.assertTrue(self.f._tracking_ready(b, self.now))
        self.assertEqual((a.last_pub, b.last_pub), (self.now, self.now))
        self.assertGreater(a.lock_seconds(self.now), 0.0)
        self.assertGreater(b.lock_seconds(self.now), 0.0)
        # 第二红人须连续确认，因此两条连续发布计数的起点各自独立。
        self.assertGreater(b.pub_run_start, a.pub_run_start)
        self.assertTrue(self.pubs('red1'))
        self.assertTrue(all(abs(m.x - 10) < 0.1 for m in self.pubs('red1')))
        self.assertTrue(self.pubs('red2'))
        self.assertTrue(all(abs(m.x - 40) < 0.1 for m in self.pubs('red2')))
        self.assertTrue(all(m.cls == 'red' for slot in ('red1', 'red2')
                            for m in self.pubs(slot)))

    def test_reds_disappearing_in_order_confirm_independently_and_keep_live_owner(self):
        self.f.elim_confirm_gap = 5.0  # 与比赛 launch 的消失确认窗一致
        self.f.lock_need = 100.0  # 本用例专测消失确认，不模拟裁判仍可见时的否决重试。
        obs = [('red', 1, 10), ('red', 2, 40)]
        self.settle(obs, n=157)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        token_b = b.assignment
        self.assertGreater(a.lock_seconds(self.now), 13.0)
        self.assertGreater(b.lock_seconds(self.now), 13.0)
        start_b = b.pub_run_start
        # 丢一秒目标不等于消除，第二红人仍保持独立发布和计时。
        self.settle([('red', 2, 40)], n=10)
        self.assertNotEqual(a.status, sf.ST_ELIMINATED)
        self.assertEqual(b.last_pub, self.now)
        self.settle([('red', 2, 40)], n=43)
        self.assertEqual(a.status, sf.ST_ELIMINATED)
        self.assertEqual((b.assigned, b.assignment), (2, token_b))
        self.assertEqual(b.pub_run_start, start_b)
        self.assertGreater(b.lock_seconds(self.now), 18.0)
        self.assertTrue(all(abs(m.x - 10) < 0.1 for m in self.pubs('red1')))
        self.assertTrue(all(abs(m.x - 40) < 0.1 for m in self.pubs('red2')))

        self.settle([], n=53)
        self.assertEqual((a.status, b.status), (sf.ST_ELIMINATED, sf.ST_ELIMINATED))
        self.assertEqual((a.assigned, b.assigned), (0, 0))

    def test_full_run_disappearance_confirms_quickly_partial_run_keeps_long_window(self):
        self.f.elim_confirm_gap = 5.0      # 与比赛 launch 一致
        self.f.elim_confirm_gap_full = 2.0
        self.f.lock_need = 100.0           # 只测消失确认，不模拟裁判否决重试
        # 发满一整轮 (>=15s) 后消失: 约 2s 确认并放出负责机。
        self.settle([('green', 1, 10)], n=157)
        green = self.f.tracks['green']
        self.assertGreaterEqual(green.lock_seconds(self.now), self.f.elim_full_run)
        self.settle([], n=15)
        self.assertNotEqual(green.status, sf.ST_ELIMINATED)
        self.settle([], n=12)
        self.assertEqual((green.status, green.assigned), (sf.ST_ELIMINATED, 0))
        # 只发了 ~13.5s 就消失 (加上消失后 pub_max_age 的外推尾巴仍不满 15s):
        # 仍按 5s 窗口, 3s 时不能判消除。
        self.settle([('blue', 1, 10)], n=137)
        blue = self.f.tracks['blue']
        self.assertGreater(blue.lock_seconds(self.now), 13.0)
        self.assertLess(blue.lock_seconds(self.now) + self.f.pub_max_age, self.f.elim_full_run)
        self.settle([], n=30)
        self.assertNotEqual(blue.status, sf.ST_ELIMINATED)
        self.settle([], n=25)
        self.assertEqual(blue.status, sf.ST_ELIMINATED)

    def stale_sole_red(self, owner_xy):
        # red1 已消除; red2 只有一帧远处单目, 之后 10s 无观测 -> 排队重派给离保存点最近的空闲机。
        self.f.tracks['red1'].status = sf.ST_ELIMINATED
        self.f.drone_pos[1][:2] = owner_xy
        self.f.drone_pos[2][:2] = (30., 20.)
        self.step([('red', 1, 40., 0., sf.M_JOINT)])
        self.settle([], n=100)
        return self.f.tracks['red2']

    def test_sole_red_owner_sighting_far_from_saved_point_rebuilds_track(self):
        red = self.stale_sole_red((40., -30.))
        self.assertEqual(red.assigned, 3)
        for k in range(20):   # 负责机自己在横向 12m 外连续看到 2s: 身份门控失败也必须能重建
            self.step([('red', 3, 40.0 + 0.05 * k, 12.0, sf.M_JOINT)])
        self.assertEqual(red.assigned, 3)
        self.assertLess(math.hypot(red.x[0] - 41.0, red.x[1] - 12.0), 0.5)
        self.assertIn(3, red.seen_by)

    def test_sole_red_other_drone_sighting_takes_over_from_blind_owner(self):
        red = self.stale_sole_red((40., -5.))
        self.assertEqual(red.assigned, 2)
        for k in range(21):
            self.step([('red', 3, 40.0 + 0.05 * k, 12.0, sf.M_JOINT)])
        self.assertEqual(red.assigned, 3)
        self.assertLess(math.hypot(red.x[0] - 41.0, red.x[1] - 12.0), 0.5)

    def test_sole_red_single_glimpse_does_not_move_track(self):
        red = self.stale_sole_red((40., -5.))
        self.step([('red', 3, 40.0, 12.0, sf.M_JOINT)])
        self.settle([], n=3)
        self.assertEqual(red.assigned, 2)
        self.assertLess(math.hypot(red.x[0] - 40.0, red.x[1]), 0.5)

    def test_two_live_reds_keep_identity_gate_for_assigned_tracks(self):
        self.settle([('red', 1, 10), ('red', 2, 40)], n=6)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        owners = (a.assigned, b.assigned)
        self.settle([], n=30)          # 两条都断流 >1.5s 但仍各自认领
        for k in range(20):
            self.step([('red', 3, 25.0 + 0.05 * k, 12.0, sf.M_JOINT)])
        self.assertEqual((a.assigned, b.assigned), owners)
        self.assertLess(abs(a.x[0] - 10.0), 1.0)
        self.assertLess(abs(b.x[0] - 40.0), 1.0)

    def test_red_measurement_gap_only_resets_its_own_publication_counter(self):
        obs = [('red', 1, 10), ('red', 2, 40)]
        self.settle(obs)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        start_b = b.pub_run_start
        token_b = b.assignment
        self.settle([('red', 2, 40)], n=40)
        self.assertEqual(a.lock_seconds(self.now), 0.0)
        self.assertGreater(b.lock_seconds(self.now), 4.0)
        self.assertEqual(b.pub_run_start, start_b)
        self.assertEqual((b.assigned, b.assignment), (2, token_b))
        stale_count = len(self.pubs('red1'))
        self.step([('red', 2, 40)])
        self.assertEqual(len(self.pubs('red1')), stale_count)
        self.assertEqual(b.last_pub, self.now)

        self.settle(obs, n=6)
        self.assertLess(a.lock_seconds(self.now), 1.0)
        self.assertGreater(b.lock_seconds(self.now), 4.5)
        self.assertEqual(b.pub_run_start, start_b)

    def test_red_tracking_ack_gap_does_not_stop_reliable_coordinates(self):
        obs = [('red', 1, 10), ('red', 2, 40)]
        self.settle(obs)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        start_b = b.pub_run_start
        for _ in range(20):
            self.step(obs, acknowledge=False)
            self.ack(2)
            self.f.publish_cb(None)
        self.assertEqual(a.last_pub, self.now)
        self.assertGreater(a.lock_seconds(self.now), 2.0)
        self.assertEqual(b.last_pub, self.now)
        self.assertEqual(b.pub_run_start, start_b)
        self.assertGreater(b.lock_seconds(self.now), 2.0)

    def test_reds_disappearing_together_both_confirm_elimination(self):
        self.f.elim_confirm_gap = 5.0
        self.settle([('red', 1, 10), ('red', 2, 40)], n=157)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertGreater(min(a.lock_seconds(self.now), b.lock_seconds(self.now)), 13.0)
        self.settle([], n=10)
        self.assertNotEqual(a.status, sf.ST_ELIMINATED)
        self.assertNotEqual(b.status, sf.ST_ELIMINATED)
        self.settle([], n=43)
        self.assertEqual((a.status, b.status), (sf.ST_ELIMINATED, sf.ST_ELIMINATED))
        self.assertEqual((a.assigned, b.assigned), (0, 0))
        self.assertAlmostEqual(a.eliminated_at, b.eliminated_at, delta=0.11)

    def test_lost_red_reacquires_without_interrupting_other_red(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        token_b = b.assignment
        before_b = len(self.pubs('red2'))
        # 第一红人未计满就丢失/瞬移；第二红人继续跟踪、独立发布。
        self.settle([('red', 2, 40)], n=255)
        self.settle([('red', 3, -20), ('red', 2, 40)], n=30)
        self.assertEqual(a.assigned, 3)
        self.assertAlmostEqual(a.x[0], -20.0)
        self.assertEqual((b.assigned, b.assignment), (2, token_b))
        self.assertEqual(b.last_pub, self.now)
        self.assertGreater(len(self.pubs('red2')), before_b)
        self.assertTrue(all(abs(m.x - 40) < 0.1 for m in self.pubs('red2')))
        self.assertTrue(any(m.x < 0 for m in self.pubs('red1')))

    def test_restart_waits_for_unknown_active_controller_to_exit(self):
        self.ack(1, 'green/previous-fusion-session')
        self.step([('green', 1, 10)], acknowledge=False)
        self.assertEqual(self.f.tracks['green'].assigned, 0)
        self.assertIn(1, self.f.retiring)
        self.ack(1, '')
        self.step([('green', 1, 10)])
        self.assertEqual(self.f.tracks['green'].assigned, 1)

    def test_same_red_seen_by_multiple_drones_has_one_owner(self):
        self.settle([('red', 1, 10), ('red', 2, 10.1), ('red', 3, 9.9)])
        live = [t for t in self.f.tracks.values() if t.cls == 'red' and t.inited]
        self.assertEqual(len(live), 1)
        owner = live[0].assigned
        self.assertGreater(owner, 0)
        self.settle([('red', 1, 10), ('red', 2, 10), ('red', 3, 10)])
        self.assertEqual(live[0].assigned, owner)
        self.assertEqual(sum(bool(t.assigned) for t in self.f.tracks.values()), 1)

    def test_two_red_boxes_in_one_camera_do_not_mix_tracks(self):
        # 先无接手机可用，再让2号机起飞并亲眼观察第二红人。
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        self.settle([('red', 1, 10), ('red', 1, 40)], n=20)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertTrue(a.inited and b.inited)
        self.assertEqual(sum(bool(t.assigned) for t in (a, b)), 1)
        self.assertAlmostEqual(a.x[0], 10.0)
        self.assertAlmostEqual(b.x[0], 40.0)
        self.f.drone_pos[1][2] = 3.
        self.settle([('red', 1, 40), ('red', 1, 10), ('red', 2, 40)])
        self.assertEqual((a.assigned, b.assigned), (1, 2))
        self.assertAlmostEqual(a.x[0], 10.0)
        self.assertAlmostEqual(b.x[0], 40.0)

    def test_uncertain_monocular_ranges_do_not_create_duplicate_red(self):
        self.f.drone_pos[1] = self.f.drone_pos[0].copy()
        self.settle([('red', 1, 50, 0.0, sf.M_JOINT),
                     ('red', 2, 70, 0.0, sf.M_JOINT)])
        self.assertEqual(sum(t.inited for t in self.f.tracks.values()), 1)
        self.assertEqual(self.pubs('red1'), [])
        self.assertEqual(self.pubs('red2'), [])

    def test_another_drone_depth_does_not_steal_a_live_red_owner(self):
        self.settle([('red', 1, 10, 0.0, sf.M_JOINT)])
        a = self.f.tracks['red1']
        token = a.assignment
        self.settle([('red', 1, 10, 0.0, sf.M_JOINT), ('red', 2, 10)])
        self.assertEqual((a.assigned, a.assignment), (1, token))
        self.assertTrue(a.bearing_only_nav)
        self.assertEqual(a.last_meas_drone, 1)
        self.assertFalse(self.f.tracks['red2'].inited)
        self.assertEqual(self.pubs('red1'), [])
        self.assertEqual(self.pubs('red2'), [])

        # 只有负责机自己恢复深度，才切回坐标KF并开始自己的裁判流。
        self.settle([('red', 1, 10), ('red', 2, 10)], n=6)
        self.assertFalse(a.bearing_only_nav)
        self.assertEqual(a.last_meas_drone, 1)
        self.assertTrue(self.pubs('red1'))

    def test_nonowner_depth_cannot_perturb_owner_state_or_trigger_publication(self):
        for color in ('green', 'red'):
            with self.subTest(color=color):
                self.f = sf.SwarmFusion()
                self.f.drone_pos = [np.array([5., 0., 3.]), np.array([35., 0., 3.]),
                                    np.array([5., 2., 3.])]
                self.settle([(color, 1, 10)])
                slot = 'red1' if color == 'red' else color
                tr = self.f.tracks[slot]
                state, covariance = tr.x.copy(), tr.P.copy()
                previous_det = tr.last_det
                previous_times = (tr.last_pred, tr.last_meas, tr.last_depth_t,
                                  tr.last_pub, tr.pub_run_start)
                accepted_by_owner = tr.accepted_seen_by[1]
                token = tr.assignment
                before = len(self.pubs(slot))
                self.f.publish_on_measurement = True
                self.now += .025
                self.f.det_cb(sf.DetectionArray(dets=[self.det(color, 2, 10.35)]))
                np.testing.assert_array_equal(tr.x, state)
                np.testing.assert_array_equal(tr.P, covariance)
                self.assertIs(tr.last_det, previous_det)
                self.assertEqual((tr.last_pred, tr.last_meas, tr.last_depth_t,
                                  tr.last_pub, tr.pub_run_start), previous_times)
                self.assertEqual(tr.accepted_seen_by[1], accepted_by_owner)
                self.assertEqual((tr.assigned, tr.assignment), (1, token))
                self.assertFalse(tr.bearing_only_nav)
                self.assertEqual(tr.last_meas_drone, 1)
                self.assertAlmostEqual(tr.seen_by[2], self.now, places=6)
                self.assertEqual(len(self.pubs(slot)), before)

    def test_only_owner_observations_switch_between_monocular_and_depth_navigation(self):
        self.settle([('green', 1, 50, 0.0, sf.M_JOINT)])
        tr = self.f.tracks['green']
        token = tr.assignment
        self.settle([('green', 1, 50, 0.0, sf.M_JOINT), ('green', 2, 10)], n=85)
        self.assertEqual((tr.assigned, tr.assignment), (1, token))
        self.assertTrue(tr.bearing_only_nav)
        self.assertAlmostEqual(tr.x[0], 50.0)
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertLess(self.now - tr.accepted_seen_by[1], .11)
        self.assertEqual(self.pubs('green'), [])

        self.settle([('green', 1, 10), ('green', 2, 10)], n=8)
        self.assertFalse(tr.bearing_only_nav)
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertTrue(self.pubs('green'))
        previous_state = tr.x.copy()
        previous_stamp = tr.last_meas
        self.now += 1.1
        self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 2, 50, method=sf.M_JOINT)]))
        self.assertFalse(tr.bearing_only_nav)
        np.testing.assert_array_equal(tr.x, previous_state)
        self.assertEqual(tr.last_meas, previous_stamp)

        self.now += .1
        self.f.det_cb(sf.DetectionArray(dets=[self.det('green', 1, 50, method=sf.M_JOINT)]))
        self.assertTrue(tr.bearing_only_nav)
        self.assertAlmostEqual(tr.x[0], 50.0)
        self.assertEqual(tr.last_meas_drone, 1)

    def test_newer_observer_images_cannot_starve_delayed_owner_measurements(self):
        for color in ('green', 'red'):
            with self.subTest(color=color):
                self.f = sf.SwarmFusion()
                self.f.drone_pos = [np.array([5., 0., 3.]), np.array([35., 0., 3.]),
                                    np.array([5., 2., 3.])]
                self.settle([(color, 1, 10)])
                slot = 'red1' if color == 'red' else color
                tr = self.f.tracks[slot]
                token = tr.assignment
                for _ in range(30):
                    self.now = round(self.now + .1, 5)
                    owner, observer = self.det(color, 1, 10), self.det(color, 2, 10.3)
                    owner.header.stamp = sf.rospy.Time.from_sec(self.now - .15)
                    observer.header.stamp = sf.rospy.Time.from_sec(self.now - .02)
                    self.f.det_cb(sf.DetectionArray(dets=[observer, owner]))
                    self.f.assign_cb(None)
                    self.ack(1)
                    self.f.publish_cb(None)
                self.assertEqual((tr.assigned, tr.assignment), (1, token))
                self.assertEqual(tr.last_meas_drone, 1)
                self.assertAlmostEqual(self.now - tr.last_meas, .15, places=6)
                self.assertAlmostEqual(self.now - tr.accepted_seen_by[1], .15, places=6)
                self.assertAlmostEqual(self.now - tr.seen_by[2], .02, places=6)
                self.assertTrue(self.f._tracking_ready(tr, self.now))
                self.assertAlmostEqual(tr.x[0], 10.0, places=5)
                self.assertAlmostEqual(tr.x[2], 0.0, places=5)
                self.assertEqual(tr.last_pub, self.now)

    def test_idle_observer_can_approach_unassigned_monocular_target_seen_by_busy_source(self):
        # 目标首次发现时其他飞机未起飞；2号机随后带着自身方位观测认领。
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        self.settle([('green', 1, 10), ('blue', 1, 50, 0.0, sf.M_JOINT)])
        tr = self.f.tracks['blue']
        self.assertEqual(tr.assigned, 0)
        self.assertEqual(tr.bearing_nav_drone, 1)
        self.f.drone_pos[1][2] = 3.
        self.step([('green', 1, 10), ('blue', 2, 52, 0.0, sf.M_JOINT)])
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.bearing_nav_drone, 2)
        self.assertEqual(tr.last_meas_drone, 2)
        self.assertAlmostEqual(tr.x[0], 52.0)
        self.assertAlmostEqual(self.f.target_pubs[1].messages[-1].pose.pose.position.x, 52.0)
        self.assertEqual(self.pubs('blue'), [])

    def test_idle_slow_camera_can_claim_target_continuously_seen_by_busy_fast_camera(self):
        # 2号机尚未起飞，保留未认领航迹的慢相机缓存接管回归。
        self.f.drone_pos[1][2] = self.f.drone_pos[2][2] = 0.
        self.settle([('green', 1, 10), ('blue', 1, 20)])
        tr = self.f.tracks['blue']
        self.assertEqual(tr.assigned, 0)
        previous_fast_stamp = tr.last_meas
        self.f.drone_pos[1][2] = 3.
        self.now += .1
        slow = self.det('blue', 2, 20.2)
        slow.header.stamp = sf.rospy.Time.from_sec(self.now - .3)
        self.assertLess(slow.header.stamp.to_sec(), previous_fast_stamp)
        self.f.det_cb(sf.DetectionArray(dets=[
            self.det('green', 1, 10), self.det('blue', 1, 20), slow]))
        # 慢相机图像相对主KF旧，但对2号机自己的流是新的有效关联观测。
        self.assertAlmostEqual(tr.observer_detections[2][1], self.now - .3, places=6)
        self.f.assign_cb(None)
        self.assertEqual(self.f.tracks['green'].assigned, 1)
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.last_meas_drone, 2)
        self.assertAlmostEqual(tr.x[0], 20.2)
        self.assertAlmostEqual(tr.last_meas, self.now - .3, places=6)
        target = self.f.state_pubs[1].messages[-1]
        self.assertAlmostEqual(target.measurement_stamp.to_sec(), self.now - .3, places=6)
        self.ack(2)
        self.f.publish_cb(None)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.assertEqual(self.pubs('blue'), [])

        # 认领后两机仍以原来的时延继续观察；快相机不能饿死2号机本人更新。
        for _ in range(6):
            self.now += .1
            slow = self.det('blue', 2, 20.2)
            slow.header.stamp = sf.rospy.Time.from_sec(self.now - .3)
            self.f.det_cb(sf.DetectionArray(dets=[
                self.det('green', 1, 10), self.det('blue', 1, 20), slow]))
            self.f.assign_cb(None)
            self.ack(1)
            self.ack(2)
            self.f.publish_cb(None)
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.last_meas_drone, 2)
        self.assertAlmostEqual(tr.last_meas, self.now - .3, places=6)
        self.assertTrue(self.f._tracking_ready(tr, self.now))
        self.assertTrue(self.pubs('blue'))
        self.assertTrue(all(abs(msg.x - 20.2) < .1 for msg in self.pubs('blue')))

    def test_observer_cache_has_its_own_clock_and_rejects_old_images_before_handoff(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        previous_owner_stamp = tr.last_meas
        self.now += .1
        observer = self.det('green', 2, 10.3)
        observer.header.stamp = sf.rospy.Time.from_sec(self.now - .3)
        self.f.det_cb(sf.DetectionArray(dets=[observer]))
        cached_det, cached_stamp = tr.observer_detections[2]
        self.assertLess(cached_stamp, previous_owner_stamp)
        self.assertAlmostEqual(cached_det.x, 10.3)
        self.assertEqual(tr.last_meas, previous_owner_stamp)
        old = self.det('green', 2, 10.0)
        old.header.stamp = sf.rospy.Time.from_sec(cached_stamp - .1)
        self.f.det_cb(sf.DetectionArray(dets=[old]))
        self.assertEqual(tr.observer_detections[2][1], cached_stamp)
        self.assertAlmostEqual(tr.observer_detections[2][0].x, 10.3)
        self.assertAlmostEqual(tr.seen_by[2], cached_stamp, places=6)

        self.f.count_lock_enable = False
        self.f.reject_cb(sf.Int8(data=1), 0)
        self.ack(1, '')
        self.f.assign_cb(None)
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.last_meas, cached_stamp)
        self.assertEqual(tr.last_meas_drone, 2)
        self.assertAlmostEqual(tr.x[0], 10.3)
        self.ack(2)
        self.assertFalse(self.f._tracking_ready(tr, self.now))

    def test_owner_reinitialization_invalidates_observers_of_previous_target_position(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        self.step([('green', 1, 10), ('green', 2, 10)])
        self.assertIn(2, tr.observer_detections)
        old_token = tr.assignment

        # A持续看到新位置并通过真实重初始化确认；B每帧仍看到旧10，
        # 其缓存本来会保持新鲜，不能让缓存年龄过期掩盖跨重建的身份问题。
        for _ in range(40):
            self.step([('green', 1, 20), ('green', 2, 10)])
            if abs(tr.x[0] - 20.0) < .1:
                break
        self.assertAlmostEqual(tr.x[0], 20.0, delta=.1)
        self.assertEqual((tr.assigned, tr.assignment), (1, old_token))
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertLess(self.now - tr.seen_by[2], .11)

        # 重建后A退出。B可以接收共享新20坐标去重捕，但旧10框不能成为
        # 本机获取证据，更不能把已确认的新目标位置20直接拖回10。
        self.f.count_lock_enable = False
        self.f.reject_cb(sf.Int8(data=1), 0)
        self.ack(1, '')
        self.f.assign_cb(None)
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.shared_source, 1)
        self.assertEqual(self.f.drone_mode[1], sf.MODE_REACQUIRE)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.assertAlmostEqual(tr.x[0], 20.0, delta=.1)
        shared_token = tr.assignment
        before = len(self.pubs('green'))
        self.step([('green', 2, 10)])
        self.assertEqual((tr.assigned, tr.assignment, tr.shared_source),
                         (2, shared_token, 1))
        self.assertAlmostEqual(tr.x[0], 20.0, delta=.1)
        self.assertEqual(len(self.pubs('green')), before)

        # B实际看见新20并重新通过关联后，才结束共享；旧凭证不能授权发布。
        self.step([('green', 2, 20)], acknowledge=False)
        self.assertEqual(tr.assigned, 2)
        self.assertEqual(tr.shared_source, 0)
        self.assertNotEqual(tr.assignment, shared_token)
        self.assertAlmostEqual(tr.x[0], 20.0, delta=.1)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.ack(2, shared_token)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), before)
        self.ack(2)
        self.assertTrue(self.f._tracking_ready(tr, self.now))
        self.step([('green', 2, 20)])
        self.assertTrue(self.f._tracking_ready(tr, self.now))

    def test_invalid_raw_observer_frames_cannot_assign_old_target_position(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        for i in range(90):
            self.step([('green', 2, 100 if i % 2 else -100)])
            self.assertNotEqual(tr.assigned, 2)
        self.assertEqual(tr.assigned, 0)
        self.assertNotEqual(tr.status, sf.ST_ELIMINATED)
        self.assertAlmostEqual(tr.x[0], 10.0)
        self.assertEqual(tr.last_meas_drone, 1)
        self.assertLess(self.now - tr.seen_by[2], .11)
        self.assertEqual(self.f.drone_mode[1], sf.MODE_OFF)

    def test_new_assignment_has_lease_window_without_fabricating_owner_measurement(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        tr.assigned, tr.assignment = 2, 'green/new-task-with-old-history'
        tr.assigned_since, tr.last_tracking_ack = self.now, None
        tr.accepted_seen_by[2] = self.now - self.f.owner_release_timeout - 10.0
        before = len(self.pubs('green'))
        for _ in range(10):
            self.now += .1
            self.ack(2)
            self.f.assign_cb(None)
            self.f.publish_cb(None)
        self.assertEqual((tr.assigned, tr.assignment), (2, 'green/new-task-with-old-history'))
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.assertEqual(len(self.pubs('green')), before)

    def test_handoff_waits_for_old_drone_exit_and_rejects_old_token(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        old_token = tr.assignment
        self.f.count_lock_enable = False
        self.f.reject_cb(sf.Int8(data=1), 0)
        count = len(self.pubs('green'))
        self.assertEqual(tr.assigned, 0)
        for _ in range(5):
            self.step([('green', 2, 10)], acknowledge=False)
            self.ack(1, old_token)  # 旧机还在 TRACK 的释放滞回中
        self.assertEqual(tr.assigned, 0)
        self.assertEqual(len(self.pubs('green')), count)
        self.ack(1, '')
        self.step([('green', 2, 10)], acknowledge=False)
        self.assertEqual(tr.assigned, 2)
        self.assertNotEqual(tr.assignment, old_token)
        self.ack(2, old_token)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), count)
        self.ack(2)
        self.f.publish_cb(None)
        self.assertEqual(len(self.pubs('green')), count)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.step([('green', 2, 10)])  # 新认领后本人实际刷新，不用接管缓存代替。
        self.assertGreater(len(self.pubs('green')), count)

    def test_lost_owner_handoff_waits_for_exit_then_adopts_new_owner_observation(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        old_token = tr.assignment
        for _ in range(85):
            self.step([('green', 2, 10.3)], acknowledge=False)
            self.ack(1, old_token)
        self.assertEqual(tr.assigned, 0)
        self.assertIn(1, self.f.retiring)
        before = len(self.pubs('green'))
        self.assertNotEqual(tr.status, sf.ST_ELIMINATED)

        self.ack(1, '')
        self.step([('green', 2, 10.3)], acknowledge=False)
        self.assertEqual(tr.assigned, 2)
        self.assertNotEqual(tr.assignment, old_token)
        self.assertEqual(tr.last_meas_drone, 2)
        self.assertAlmostEqual(tr.x[0], 10.3, delta=.05)
        self.ack(2)
        self.f.publish_cb(None)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.assertEqual(len(self.pubs('green')), before)
        self.settle([('green', 2, 10.3)], n=6)
        self.assertTrue(self.f._tracking_ready(tr, self.now))
        self.assertGreater(len(self.pubs('green')), before)

    def test_other_cameras_cannot_hide_owner_losing_target(self):
        # 先发布到13秒以上，覆盖“接近计满后的消除确认保护”分支；其他飞机的
        # 新量测也不能替已跟丢的旧负责机刷新这层保护。
        self.settle([('green', 1, 10)], n=135)
        owner = self.f.tracks['green'].assigned
        self.assertGreater(self.f.tracks['green'].lock_seconds(self.now), 13.0)
        # 旁观者证明人仍存在，不能更新A的KF，也不能将A丢框误判成裁判删人。
        previous_measurement = self.f.tracks['green'].last_meas
        # 消除确认保护窗 (elim_confirm_gap + 1s) 内旁观者一律不动 KF。
        self.settle([('green', 2, 10)], n=29)
        tr = self.f.tracks['green']
        self.assertNotEqual(tr.status, sf.ST_ELIMINATED)
        self.assertEqual(tr.assigned, owner)
        self.assertEqual(tr.last_meas, previous_measurement)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        # 保护窗过后, 更远的旁观者只用来引导仍在认领的盲负责机, 不计分、不判消除。
        self.settle([('green', 2, 10)], n=3)
        self.assertEqual((tr.assigned, tr.shared_source, tr.last_meas_drone), (owner, 2, 2))
        self.assertEqual(self.f.drone_mode[owner - 1], sf.MODE_REACQUIRE)
        self.assertNotEqual(tr.status, sf.ST_ELIMINATED)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        # 1号负责机自己超过8s无有效量测后退出，2号机方可合法接管。
        self.settle([('green', 2, 10)], n=55)
        tr = self.f.tracks['green']
        self.assertNotEqual(tr.assigned, owner)
        self.assertEqual(tr.assigned, 2)

    def test_rejected_owner_detections_cannot_refresh_assignment_lease(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        token = tr.assignment
        last_accepted = tr.accepted_seen_by[1]

        # 1号机仍不断报框，但位置在两处大幅跳变，始终过不了门控和新位置持续
        # 确认。旧实现会用 raw seen_by 每帧续租，导致它永久占着 green。
        for i in range(90):
            bad_x = 20 if i % 2 == 0 else -10
            self.step([('green', 1, bad_x)])
            if tr.assigned == 0:
                break

        self.assertEqual(tr.accepted_seen_by[1], last_accepted)
        self.assertLess(self.now - tr.seen_by[1], 0.2)  # 候选框仍持续出现
        self.assertEqual((tr.assigned, tr.assignment), (0, ''))
        self.assertIn(1, self.f.retiring)
        self.assertIn(sf.MODE_OFF,
                      [m.data for m in self.f.mode_pubs[0].messages])
        self.assertNotEqual(token, '')

    def test_approach_detection_gap_keeps_owner_and_reacquire_commands(self):
        # 实跑回归：red1/white 在“观测年龄2s、TRACK回执正常”时被弹回巡逻。
        # 远距单目接近本来就不计分，转弯/俯仰短暂丢框必须保留认领。
        for color in ('white', 'red'):
            with self.subTest(color=color):
                slot = 'red1' if color == 'red' else color
                did = 1 if color == 'white' else 2
                obs = [(color, did, 50, 0.0, sf.M_JOINT)]
                self.settle(obs)
                tr = self.f.tracks[slot]
                self.assertEqual(tr.assigned, did)
                token = tr.assignment
                before = len(self.f.mode_pubs[did - 1].messages)
                self.settle([], n=35)
                self.assertEqual((tr.assigned, tr.assignment), (did, token))
                self.assertEqual(self.f.drone_mode[did - 1], sf.MODE_REACQUIRE)
                self.assertNotIn(sf.MODE_OFF, [m.data for m in
                    self.f.mode_pubs[did - 1].messages[before:]])
                self.assertEqual(self.pubs(slot), [])
                self.settle(obs, n=3)
                self.assertEqual((tr.assigned, tr.assignment), (did, token))
                self.assertEqual(self.f.drone_mode[did - 1], sf.MODE_APPROACH)

    def test_brief_ack_gap_after_long_tracking_does_not_release(self):
        obs = [('green', 1, 10)]
        self.settle(obs, n=80)  # 认领已超过旧判断中的5秒
        tr = self.f.tracks['green']
        token = tr.assignment
        self.settle(obs, n=8, acknowledge=False)
        count = len(self.pubs('green'))
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.settle(obs, n=12, acknowledge=False)
        self.assertEqual((tr.assigned, tr.assignment), (1, token))
        self.assertEqual(len(self.pubs('green')), count)  # 仍然停发过期回执的坐标
        self.step(obs)
        self.assertEqual((tr.assigned, tr.assignment), (1, token))
        self.assertGreater(len(self.pubs('green')), count)

    def test_sustained_ack_loss_still_releases_controller(self):
        obs = [('green', 1, 10)]
        self.settle(obs, n=80)
        self.settle(obs, n=52, acknowledge=False)
        self.assertEqual(self.f.tracks['green'].assigned, 0)
        self.assertIn(1, self.f.retiring)

    def test_short_owner_gap_with_other_observer_does_not_trigger_handoff(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        token = tr.assignment
        self.settle([('green', 2, 10)], n=35)
        self.assertEqual((tr.assigned, tr.assignment), (1, token))
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.assertEqual(self.f.drone_mode[0], sf.MODE_REACQUIRE)

    def test_dead_controller_handoff_waits_for_command_lease_expiry(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        self.f.count_lock_enable = False
        self.f.reject_cb(sf.Int8(data=1), 0)
        self.settle([('green', 2, 10)], n=29, acknowledge=False)
        self.assertEqual(tr.assigned, 0)
        self.settle([('green', 2, 10)], n=3, acknowledge=False)
        self.assertEqual(tr.assigned, 2)
        # 租约过期只允许改派，并不代替新机的实际跟踪回执。
        self.assertFalse(self.f._tracking_ready(tr, self.now))

    def test_runtime_subscriptions_use_only_our_observations_and_feedback(self):
        topics = [call.args[0] for call in sf.rospy.Subscriber.call_args_list]
        allowed = ['/vision/detections']
        for i in range(3):
            allowed += ['/typhoon_h480_%d/odom_world_relay' % i,
                        '/tracking/reject_%d' % (i + 1),
                        '/tracking/active_%d' % (i + 1)]
        self.assertCountEqual(topics, allowed)

    def test_latest_observer_being_busy_does_not_block_idle_observer(self):
        self.settle([('green', 1, 10)])
        self.settle([('green', 1, 10), ('blue', 2, 25), ('blue', 1, 25)])
        self.assertEqual(self.f.tracks['green'].assigned, 1)
        self.assertEqual(self.f.tracks['blue'].assigned, 2)

    def test_overlapping_red_tracks_cannot_keep_two_owners(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        b.x[:2] = a.x[:2] + [0.2, 0.0]
        self.f.assign_cb(None)
        self.assertEqual(a.assigned, 1)
        self.assertEqual(b.assigned, 0)
        self.assertFalse(self.f._tracking_ready(a, self.now))
        self.assertIn(2, self.f.retiring)

    def test_overlapping_red_tracks_keep_existing_second_owner_and_block_both_topics(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        token_b = b.assignment
        self.f._release(a, self.now)
        a.x[:2] = b.x[:2] + [0.2, 0.0]
        before = {slot: len(self.pubs(slot)) for slot in ('red1', 'red2')}
        self.f.assign_cb(None)
        self.assertEqual(a.assigned, 0)
        self.assertEqual((b.assigned, b.assignment), (2, token_b))
        self.assertNotIn(2, self.f.retiring)
        self.assertGreater(a.identity_blocked_until, self.now)
        self.assertGreater(b.identity_blocked_until, self.now)
        self.assertFalse(self.f._tracking_ready(b, self.now))
        self.f.publish_cb(None)
        self.assertEqual({slot: len(self.pubs(slot)) for slot in before}, before)

    def test_ambiguous_red_detection_updates_neither_track_and_blocks_both_topics(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        # 增大身份位置不确定度，让两条航迹都能解释同一个红框。
        a.P[:2, :2] = b.P[:2, :2] = np.eye(2) * 100.0
        previous = (a.last_meas, b.last_meas)
        self.now += 0.1
        self.f.feed_reds([(self.det('red', 1, 25), self.now)], self.now)
        self.assertEqual((a.last_meas, b.last_meas), previous)
        self.assertGreater(a.identity_blocked_until, self.now)
        self.assertGreater(b.identity_blocked_until, self.now)
        self.assertFalse(self.f._judge_authorized(a, self.now))
        self.assertFalse(self.f._judge_authorized(b, self.now))

    def test_parallel_reds_apply_hold_timeout_independently(self):
        self.settle([('red', 1, 10), ('red', 2, 40)])
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        b.assigned_since = self.now - self.f.track_max_hold - 10.0
        token_a = a.assignment
        self.step([('red', 1, 10), ('red', 2, 40)])
        self.assertEqual((a.assigned, a.assignment), (1, token_a))
        self.assertEqual(a.last_pub, self.now)
        self.assertEqual((b.assigned, b.assignment), (0, ''))
        self.assertNotEqual(b.status, sf.ST_ELIMINATED)
        self.assertIn(2, self.f.retiring)
        self.assertIsNone(b.pub_run_start)

    def test_last_queued_red_is_dispatched_and_publishes_after_reacquire(self):
        # 实跑回归：red2 被忙碌的 1 号机看到时三机都有任务，所以一直“机0”。
        # red1 消除后若仍只接受 2s 内观察者，就再也没有飞机会去 red2 方位重捕。
        occupied = [('green', 1, 10), ('blue', 2, 20), ('red', 3, 40)]
        self.settle(occupied, n=135)
        self.settle(occupied + [('red', 1, 80)], n=6)
        a, b = self.f.tracks['red1'], self.f.tracks['red2']
        self.assertEqual((a.assigned, b.assigned), (3, 0))
        self.assertIsNotNone(a.last_pub)
        self.assertIsNotNone(b.last_pub)  # 未认领仍可上报当前可靠深度。

        # 让第一红人消失并继续给 1/2 号机各自原任务，只有刚释放的 3 号机可补位。
        self.settle([('green', 1, 10), ('blue', 2, 20)], n=24)
        self.assertEqual(a.status, sf.ST_ELIMINATED)
        self.assertEqual(b.assigned, 3)
        self.assertEqual(self.f.drone_mode[2], sf.MODE_REACQUIRE)
        old_count = len(self.pubs('red2'))
        self.step([('green', 1, 10), ('blue', 2, 20)])
        self.assertEqual(len(self.pubs('red2')), old_count)  # 旧点只导航。

        # 3 号机到达后重新得到深度并回显 TRACK，才开始发布第二红人的独立入口。
        old_counts = {slot: len(self.pubs(slot)) for slot in ('red1', 'red2')}
        self.settle([('green', 1, 10), ('blue', 2, 20), ('red', 3, 80)], n=3)
        self.assertEqual(len(self.pubs('red1')), old_counts['red1'])
        self.assertGreater(len(self.pubs('red2')), old_counts['red2'])
        self.assertAlmostEqual(self.pubs('red1')[-1].x, 40.0, delta=0.2)
        self.assertAlmostEqual(self.pubs('red2')[-1].x, 80.0, delta=0.2)

    def test_parallel_red_publications_eliminate_both_with_current_judge_and_swapped_identities(self):
        for swapped in (False, True):
            with self.subTest(swapped_actor_identities=swapped):
                self.now = 100.0
                self.f = sf.SwarmFusion()
                self.f.elim_confirm_gap = 5.0
                self.f.drone_pos = [np.array([5., 0., 3.]), np.array([35., 0., 3.]),
                                    np.array([5., 2., 3.])]
                # 红航迹编号取自视觉首次发现顺序，不依赖裁判真实模型编号。
                actor_of_slot = {'red1': 5 if swapped else 4,
                                 'red2': 4 if swapped else 5}
                positions = {actor_of_slot['red1']: (10.0, 0.0),
                             actor_of_slot['red2']: (40.0, 0.0)}
                judge = OfflineRedJudge(lambda: self.now, positions)
                delivered = {'red1': 0, 'red2': 0}
                for _ in range(240):
                    obs = [('red', did, x) for slot, did, x in
                           (('red1', 1, 10), ('red2', 2, 40))
                           if actor_of_slot[slot] not in judge.deleted]
                    self.step(obs)
                    for slot in delivered:
                        messages = self.pubs(slot)
                        for message in messages[delivered[slot]:]:
                            judge.receive(slot, message)
                        delivered[slot] = len(messages)

                self.assertEqual(set(judge.deleted), {4, 5})
                for removed_at, started_at in judge.deleted.values():
                    self.assertGreaterEqual(removed_at - started_at, 15.0)
                    self.assertLessEqual(removed_at - started_at, 15.11)
                removed_times = [value[0] for value in judge.deleted.values()]
                self.assertLessEqual(max(removed_times) - min(removed_times), 0.5)
                self.assertLessEqual(max(removed_times) - 100.0, 16.0)
                self.assertEqual([self.f.tracks[s].status for s in delivered],
                                 [sf.ST_ELIMINATED, sf.ST_ELIMINATED])
                self.assertTrue(all(abs(m.x - 10.0) < 0.1 for m in self.pubs('red1')))
                self.assertTrue(all(abs(m.x - 40.0) < 0.1 for m in self.pubs('red2')))

    def test_queued_red_is_dispatched_once_per_observation(self):
        # 实跑: 同一个旧点被 6->5->6 号机轮流空飞 8s。到点没看到后不再重复排队。
        red = self.stale_sole_red((40., -5.))
        self.assertEqual(red.assigned, 2)
        self.assertTrue(red.blind_acquire)
        for _ in range(60):
            self.step([])
        self.assertEqual(red.assigned, 0)
        self.assertEqual(red.queued_meas, red.last_meas)
        self.settle([], n=60)
        self.assertEqual(red.assigned, 0)
        # 新观测重新允许认领。
        self.step([('red', 3, 40.5, 0., sf.M_JOINT)])
        self.assertEqual(red.assigned, 3)

    def test_ancient_queued_red_is_not_dispatched(self):
        b = self.f.tracks['red2']
        b.init_from(self.det('red', 1, 80), self.now)
        self.f._mark_seen(b, 1, self.now)
        self.f.tracks['red1'].status = sf.ST_ELIMINATED
        self.now += self.f.lost_timeout + 0.1
        self.f.assign_cb(None)
        self.assertEqual(b.assigned, 0)

    def test_eliminated_normal_target_revives_and_waits_for_exclusive_owner(self):
        self.settle([('green', 1, 10)])
        tr = self.f.tracks['green']
        old_token = tr.assignment
        old_pub_count = len(self.pubs('green'))

        # 模拟内部误判消除。旧机尚未退出 TRACK 时，2号机的持续视觉可以复活
        # 航迹，但不得立即认领，避免两架飞机同时跟同一人。
        tr.status = sf.ST_ELIMINATED
        tr.eliminated_at = self.now
        tr.clear_reinit_candidate()
        self.f._release(tr, self.now)
        self.step([('green', 2, 10)], acknowledge=False)
        self.ack(1, old_token)
        self.assertEqual(tr.status, sf.ST_ELIMINATED)  # 单帧不复活
        for _ in range(2):
            self.step([('green', 2, 10)], acknowledge=False)
            self.ack(1, old_token)
        self.assertEqual(tr.status, sf.ST_TRACKED)
        self.assertEqual(tr.assigned, 0)
        self.assertIn(1, self.f.retiring)
        self.assertEqual(len(self.pubs('green')), old_pub_count)

        # 旧机明确退出才认领；新机ACK和认领前缓存仍不冒充本次任务实际观测。
        self.ack(1, '')
        self.step([('green', 2, 10)])
        self.assertEqual(tr.assigned, 2)
        self.assertNotEqual(tr.assignment, old_token)
        self.assertEqual(len(self.pubs('green')), old_pub_count)
        self.assertFalse(self.f._tracking_ready(tr, self.now))
        self.step([('green', 2, 10)])
        self.assertGreater(len(self.pubs('green')), old_pub_count)
        self.assertEqual(sum(t.assigned == 2 for t in self.f.tracks.values()), 1)

    def test_internal_all_eliminated_waits_for_patrol_verification(self):
        for tr in self.f.tracks.values():
            tr.inited = True
            tr.status = sf.ST_ELIMINATED
        self.f.publish_cb(None)
        self.assertFalse(self.f.complete_pub.messages[-1].data)
        self.assertEqual(self.f.all_eliminated_since, self.now)

        # 完成确认窗口内出现一帧普通目标，哪怕尚未累计到复活门限，也要取消降落计时。
        self.now += 0.5
        arr = sf.DetectionArray()
        arr.dets = [self.det('green', 1, 10)]
        self.f.det_cb(arr)
        self.f.publish_cb(None)
        self.assertFalse(self.f.complete_pub.messages[-1].data)
        self.assertIsNone(self.f.all_eliminated_since)

        # 候选消失后重新完整等待确认窗，才允许 mission_complete。
        self.f.tracks['green'].clear_reinit_candidate()
        self.now += 0.1
        self.f.publish_cb(None)
        self.assertFalse(self.f.complete_pub.messages[-1].data)
        self.now += 1.1
        self.f.publish_cb(None)
        self.assertTrue(self.f.complete_pub.messages[-1].data)


if __name__ == '__main__':
    unittest.main(verbosity=2)
