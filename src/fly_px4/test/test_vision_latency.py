#!/usr/bin/env python3
"""Real vision tick with deterministic fake inference and ROS/wall clocks."""
import collections
import json
import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import swarm_vision as sv


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)
        self.copies = 0

    def cpu(self):
        self.copies += 1
        return self

    def numpy(self):
        return self.values


class Boxes:
    def __init__(self, boxes=None, classes=None, confidences=None):
        self.xyxy = Tensor([[1., 0., 4., 3.]] if boxes is None else boxes)
        self.cls = Tensor([0] if classes is None else classes)
        self.conf = Tensor([.95] if confidences is None else confidences)

    def __len__(self):
        return len(self.cls.values)


class VisionLatencyTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.04
        self.events, self.detections, self.diagnostics = [], [], []
        self.estimate_calls, self.annotation_labels = [], []
        self.prediction_kwargs = None
        for p in [patch.object(sv.time, 'monotonic', side_effect=lambda: self.now),
                  patch.object(sv.rospy.Time, 'now', side_effect=lambda: sv.rospy.Time.from_sec(self.now)),
                  patch.object(sv.rospy, 'logwarn_throttle')]:
            p.start()
            self.addCleanup(p.stop)
        n = self.node = sv.SwarmVision.__new__(sv.SwarmVision)
        n.conf, n.half, n.device, n.max_batch = .8, True, 'cpu', 6
        n.red_conf = .8
        n.publish_annotated = n.require_camera_info = True
        n.lever_arm_model, n.sync_wait_sec = 'gimbal_kinematics', .025
        n.n_drop_pose = n.n_pub = 0
        n.n_drop_range = n.n_drop_consist = n.n_veto_downgrade = 0
        n.health_lock = threading.Lock()
        n.tracking_input_counts = collections.Counter()
        n.batch_timings = collections.deque(maxlen=100)
        n.rr_cursor = 0
        n.cv2 = SimpleNamespace(FONT_HERSHEY_SIMPLEX=0, LINE_AA=16,
                                rectangle=lambda *a: None,
                                putText=lambda image, label, *a: self.annotation_labels.append(label))
        self.results = [SimpleNamespace(boxes=Boxes()), SimpleNamespace(boxes=Boxes())]
        n.model = SimpleNamespace(predict=self.predict)
        n.pub = SimpleNamespace(publish=self.publish_detections)
        n.diag_pub = SimpleNamespace(publish=self.publish_diagnostic)
        n.ann_pubs = [SimpleNamespace(publish=self.publish_annotation) for _ in range(2)]
        n.streams = [self.stream(1), self.stream(2)]
        n._estimate = self.estimate

    def stream(self, did):
        stamp = 100.+(did-1)*.01
        frame = (np.zeros((4, 6, 3), dtype=np.uint8), stamp, 1,
                 dict(image_receipt_stamp=stamp+.02, image_ready_stamp=stamp+.021,
                      image_decode_wall_ms=1.))
        quality = dict(pos=np.zeros(3), R=np.eye(3), body_position=[0., 0., 0.],
                       depth_valid=True, source='camera_imu',
                       lever_arm_model='gimbal_kinematics_from_orientation')
        def query(stamp, deadline):
            self.now += .004
            return quality, (np.ones((4, 6)), 0.), SimpleNamespace(height=4, width=6), True
        return SimpleNamespace(drone_id=did, ns='test%d' % did, mode=1,
                               take_frame=lambda: frame, mark_done=lambda seq: None,
                               _epoch_valid=lambda stamp: True, query_inputs=query)

    def predict(self, frames, **kwargs):
        self.prediction_kwargs = kwargs
        self.now += .05
        return self.results

    def estimate(self, *args):
        self.estimate_calls.append((int(args[5]), np.asarray(args[1]).tolist()))
        self.now += .006
        return dict(u=2., v_top=0., v_bot=3., box_w=3., x=5., y=0.,
                    sigma_along=.2, sigma_cross=.1, bearing=0., slant=5.,
                    method=sv.M_DEPTH, truncated=False,
                    image_bearing_valid=True, image_bearing=0., image_error_x=0.,
                    depth_diagnostics=dict(reason='accepted_surface'))

    def publish_detections(self, msg):
        self.events.append(('coordinates', self.now))
        self.detections.append(msg)

    def publish_diagnostic(self, msg):
        self.events.append(('diagnostic', self.now))
        self.diagnostics.append(json.loads(msg.data))
        self.now += .005

    def publish_annotation(self, msg):
        self.events.append(('display', self.now))
        self.now += .025

    def test_coordinates_precede_diagnostics_and_all_annotation_work(self):
        self.node._tick()
        self.assertEqual(self.events[0][0], 'coordinates')
        self.assertEqual(len(self.detections[0].dets), 2)  # Preserve the whole multi-camera batch.
        self.assertAlmostEqual(self.events[0][1], 100.11)
        self.assertAlmostEqual(self.now, 100.17)
        self.assertEqual(self.node.n_pub, 2)
        # All batch members share the actual coordinate publication epoch;
        # each still retains its own original acquisition timestamp.
        for d, diagnostic in zip(self.detections[0].dets, self.diagnostics):
            self.assertAlmostEqual(d.pipeline_lag, 100.11-d.header.stamp.to_sec())
            timing = diagnostic['timing']
            self.assertAlmostEqual(timing['vision_publish_stamp'], 100.11)
            self.assertAlmostEqual(timing['inference_wall_ms'], 50.)
            self.assertAlmostEqual(timing['coordinate_postprocess_wall_ms'], 20.)
            self.assertAlmostEqual(timing['input_query_wall_ms'], 4.)
            self.assertAlmostEqual(timing['estimate_wall_ms'], 6.)
            self.assertAlmostEqual(timing['image_receipt_stamp'], diagnostic['image_stamp']+.02)
        self.assertTrue(np.allclose(self.node.batch_timings[-1], [.05, .02, .06]))

    def test_rendering_reuses_cpu_boxes_without_second_gpu_transfer(self):
        self.node._tick()
        for result in self.results:
            self.assertEqual([result.boxes.xyxy.copies, result.boxes.cls.copies,
                              result.boxes.conf.copies], [1, 1, 1])

    def test_bad_pose_is_still_rejected_and_diagnosed_after_valid_coordinates(self):
        self.node.streams[1].query_inputs = lambda *a: (None, None, None, False)
        self.node._tick()
        self.assertEqual(len(self.detections[0].dets), 1)
        self.assertEqual(self.events[0][0], 'coordinates')
        self.assertEqual(self.node.n_drop_pose, 1)
        self.assertTrue(self.diagnostics[0]['accepted'])
        self.assertFalse(self.diagnostics[1]['accepted'])
        self.assertEqual(self.diagnostics[1]['reason'], 'no_bracketed_camera_pose')

    def test_annotation_failure_keeps_coordinates_and_other_camera_display(self):
        self.node.ann_pubs[0] = SimpleNamespace(publish=lambda msg: (_ for _ in ()).throw(RuntimeError('display')))
        self.node._tick()
        self.assertEqual(len(self.detections[0].dets), 2)
        self.assertEqual(len(self.diagnostics), 2)
        self.assertEqual(sum(name == 'display' for name, _ in self.events), 1)

    def test_red_threshold_filters_before_geometry_and_matches_annotations(self):
        self.node.conf, self.node.red_conf = .7, .8
        boxes = [[float(i), 0., float(i + 1), 3.] for i in range(5)]
        self.results = [SimpleNamespace(boxes=Boxes(
            boxes, [4, 4, 4, 0, 0], [.79, .80, .95, .70, .69]))]
        self.node.streams = self.node.streams[:1]

        self.node._tick()

        self.assertAlmostEqual(self.prediction_kwargs['conf'], .7)
        self.assertEqual(self.estimate_calls, [(4, boxes[1]), (4, boxes[2]), (0, boxes[3])])
        detections = self.detections[0].dets
        self.assertEqual([d.cls_id for d in detections], [4, 4, 0])
        np.testing.assert_allclose([d.conf for d in detections], [.80, .95, .70])
        self.assertEqual(self.annotation_labels, ['red 0.80', 'red 0.95', 'green 0.70'])
        self.assertEqual(self.node.n_pub, 3)

    def test_lower_red_threshold_sets_inference_floor_and_keeps_other_class_gate(self):
        self.node.conf, self.node.red_conf = .9, .8
        boxes = [[float(i), 0., float(i + 1), 3.] for i in range(3)]
        self.results = [SimpleNamespace(boxes=Boxes(
            boxes, [4, 0, 0], [.85, .85, .95]))]
        self.node.streams = self.node.streams[:1]

        self.node._tick()

        self.assertAlmostEqual(self.prediction_kwargs['conf'], .8)
        self.assertEqual(self.estimate_calls, [(4, boxes[0]), (0, boxes[2])])
        detections = self.detections[0].dets
        self.assertEqual([d.cls_id for d in detections], [4, 0])
        np.testing.assert_allclose([d.conf for d in detections], [.85, .95])
        self.assertEqual(self.annotation_labels, ['red 0.85', 'green 0.95'])

    def test_color_callback_keeps_timing_with_the_selected_frame(self):
        stream = sv.DroneStream.__new__(sv.DroneStream)
        stream.lock, stream.updated = threading.Lock(), threading.Event()
        stream.frame_seq = -1
        stream._epoch_valid = lambda stamp: True
        msg = sv.Image(height=4, width=6, encoding='bgr8', step=18,
                       data=np.zeros((4, 6, 3), dtype=np.uint8).tobytes())
        msg.header.stamp = sv.rospy.Time.from_sec(100.)
        stream._color_cb(msg)
        selected = stream.frame
        self.now += .01
        stream._color_cb(msg)
        self.assertEqual(selected[2], 0)
        self.assertEqual(stream.frame[2], 1)
        self.assertAlmostEqual(selected[3]['image_receipt_stamp'], 100.04)
        self.assertAlmostEqual(stream.frame[3]['image_receipt_stamp'], 100.05)


if __name__ == '__main__':
    unittest.main(verbosity=2)
