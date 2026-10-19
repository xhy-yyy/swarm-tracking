#!/usr/bin/env python3
"""Observe camera/body IMUs; publish gimbal targets only for confirmed faults."""
import json
from dataclasses import replace
import os
import sys
import threading
import time

import rospy
from mavros_msgs.msg import MountControl
from sensor_msgs.msg import Imu
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gimbal_recovery_logic import GimbalRecovery, ImuObservation


class SwarmGimbalRecovery:
    def __init__(self):
        # YAML 总开关只在启动时读取；关闭时不创建检测或云台控制接口。
        self.enabled = rospy.get_param('~enabled', True)
        if not isinstance(self.enabled, bool):
            raise ValueError('enabled must be a YAML boolean: true or false')
        if not self.enabled:
            rospy.loginfo('Gimbal detection/recovery disabled by ~enabled; no IMU subscriptions or mount commands.')
            return
        count = int(rospy.get_param('~drone_num', 6))
        prefix = str(rospy.get_param('~ns_prefix', 'typhoon_h480')).strip('/')
        hz = float(rospy.get_param('~rate_hz', 20.))
        if not 1 <= count <= 32 or not prefix or not 1 <= hz <= 50:
            raise ValueError('invalid drone_num, ns_prefix or rate_hz')
        self.period = 1./hz
        parameters = {key: rospy.get_param('~' + key, value)
                      for key, value in GimbalRecovery.DEFAULTS.items()}
        parameters.update({
            'camera_imu_reference': rospy.get_param('~camera_imu_reference', 'initial'),
            'camera_imu_initial_rpy_deg': rospy.get_param('~camera_imu_initial_rpy_deg', [0., 25., 0.]),
            'camera_mount_rpy_deg': rospy.get_param('~camera_mount_rpy_deg', [0., 25., 0.])})
        self.monitors = [GimbalRecovery(**parameters) for _ in range(count)]
        self.observations = [[None, None] for _ in range(count)]
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.commands, self.status, self.subscribers = [], [], []
        self.previous_status = [None] * count
        self.status_time = [0.] * count
        for index in range(count):
            ns = '/%s_%d' % (prefix, index)
            self.commands.append(rospy.Publisher(ns + '/mavros/mount_control/command',
                                                 MountControl, queue_size=1))
            self.status.append(rospy.Publisher('/gimbal_recovery/status_%d' % (index+1),
                                               String, queue_size=1, latch=True))
            for kind, topic in enumerate((ns + '/imu_gazebo', ns + '/mavros/imu/data')):
                self.subscribers.append(rospy.Subscriber(topic, Imu, self.imu_callback,
                    callback_args=(index, kind), queue_size=1, tcp_nodelay=True))
        rospy.on_shutdown(self.stop.set)
        p = self.monitors[0].params
        rospy.loginfo('Gimbal recovery observes %d drones; fault_angle_deg=%.1f, fault_sec=%.1f, '
                      'require_stationary=%s; healthy flight sends no mount commands.',
                      count, p['fault_angle_deg'], p['fault_sec'], p['require_stationary'])

    def imu_callback(self, message, key):
        index, kind = key
        q, omega = message.orientation, message.angular_velocity
        observation = ImuObservation(message.header.stamp.to_sec(), time.monotonic(),
            (q.x, q.y, q.z, q.w), (omega.x, omega.y, omega.z),
            message.orientation_covariance[0] != -1 and message.angular_velocity_covariance[0] != -1)
        # Keep only the latest sample. Duplicate stamps cannot refresh wall age.
        with self.lock:
            previous = self.observations[index][kind]
            if previous is not None and previous.stamp == observation.stamp:
                observation = replace(observation, received=previous.received)
            self.observations[index][kind] = observation

    def step(self):
        if not self.enabled:
            return
        now, wall = rospy.Time.now().to_sec(), time.monotonic()
        with self.lock:
            samples = [tuple(pair) for pair in self.observations]
        for index, (monitor, (camera, body)) in enumerate(zip(self.monitors, samples)):
            decision = monitor.step(now, wall, camera, body,
                                    self.commands[index].get_num_connections() > 0)
            if decision.command:
                command = MountControl()
                command.header.stamp = rospy.Time.now()
                command.mode = MountControl.MAV_MOUNT_MODE_MAVLINK_TARGETING
                command.roll = command.pitch = command.yaw = 0.
                self.commands[index].publish(command)
                rospy.logwarn('Gimbal drone %d: persistent roll=%.1f pitch=%.1f deg, recenter %d/%d',
                    index+1, decision.roll_deg, decision.pitch_deg,
                    decision.attempts, monitor.params['max_attempts'])
            state = (decision.state, decision.reason)
            if state != self.previous_status[index] or wall-self.status_time[index] >= 1.:
                self.status[index].publish(String(data=json.dumps({
                    'state': decision.state, 'reason': decision.reason,
                    'roll_deg': round(decision.roll_deg, 2) if camera and decision.state != 'NO_FEEDBACK' else None,
                    'pitch_deg': round(decision.pitch_deg, 2) if camera and decision.state != 'NO_FEEDBACK' else None,
                    'attempts': decision.attempts}, allow_nan=False)))
                if state != self.previous_status[index] and decision.state == 'COOLDOWN':
                    log = rospy.loginfo if decision.reason == 'recovered' else rospy.logwarn
                    log('Gimbal drone %d: %s; observation continues during cooldown.', index+1, decision.reason)
                self.previous_status[index], self.status_time[index] = state, wall

    def run(self):
        if not self.enabled:
            rospy.spin()
            return
        while not rospy.is_shutdown() and not self.stop.is_set():
            self.step()
            self.stop.wait(self.period)


def main():
    rospy.init_node('swarm_gimbal_recovery')
    SwarmGimbalRecovery().run()


if __name__ == '__main__':
    main()
