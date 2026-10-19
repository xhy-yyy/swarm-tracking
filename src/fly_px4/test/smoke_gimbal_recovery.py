#!/usr/bin/env python3
"""Verify six event-driven recovery routes on a private ROS master, using IMUs."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_gimbal_recovery import observations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir')
    args = parser.parse_args()
    temporary = tempfile.TemporaryDirectory(prefix='event-gimbal-ros-')
    outdir = Path(args.outdir or temporary.name)
    outdir.mkdir(parents=True, exist_ok=True)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', ROS_HOME=str(outdir/'home'), ROS_LOG_DIR=str(outdir/'ros'))
    env.pop('ROS_HOSTNAME', None)
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ.update({k: env[k] for k in ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOME', 'ROS_LOG_DIR')})
    children, log_files, results = [], [], []
    timer = None

    def start(command, label):
        log = (outdir/(label + '.log')).open('w')
        log_files.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)

    def wait(predicate, label, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                results.append(label)
                print('PASS:', label, flush=True)
                return
            if any(p.poll() is not None for p in children):
                raise AssertionError('ROS process exited during ' + label)
            time.sleep(.04)
        raise AssertionError('Timed out: ' + label)

    def silent(duration, indices, label):
        with lock:
            counts = [len(commands[i]) for i in indices]
        time.sleep(duration)
        with lock:
            assert counts == [len(commands[i]) for i in indices], label
        results.append(label)
        print('PASS:', label, flush=True)

    try:
        start(['roscore', '-p', str(port)], 'master')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15.
        while True:
            try:
                if master.getUri('/gimbal_recovery_smoke')[0] == 1:
                    break
            except OSError:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError('Private ROS master failed to start')
            time.sleep(.05)

        import rospy
        from mavros_msgs.msg import MountControl
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String
        from swarm_gimbal_recovery import SwarmGimbalRecovery

        rospy.init_node('gimbal_recovery_smoke', anonymous=True, disable_signals=True)
        callback_probe = SwarmGimbalRecovery.__new__(SwarmGimbalRecovery)
        callback_probe.lock = threading.Lock()
        callback_probe.observations = [[None, None]]
        duplicate = Imu()
        duplicate.header.stamp = rospy.Time.now()
        duplicate.orientation.w = 1.
        callback_probe.imu_callback(duplicate, (0, 0))
        received = callback_probe.observations[0][0].received
        duplicate.orientation_covariance[0] = -1
        callback_probe.imu_callback(duplicate, (0, 0))
        assert callback_probe.observations[0][0].received == received
        assert not callback_probe.observations[0][0].available
        results.append('duplicate sensor stamps never refresh age or hide invalid feedback')
        lock = threading.Lock()
        scenarios = [dict(camera_rpy=(0., 0., i*20.), body_rpy=(0., 0., i*20.),
                          camera_rate=(0., 0., 0.), body_rate=(0., 0., 0.),
                          camera_enabled=True, body_enabled=True) for i in range(6)]
        commands, statuses = [[] for _ in range(6)], [{} for _ in range(6)]
        imu_publishers, handles = [], []
        for index in range(6):
            ns = '/gimbal_smoke_%d' % index
            imu_publishers.append([
                rospy.Publisher(ns+'/imu_gazebo', Imu, queue_size=1),
                rospy.Publisher(ns+'/mavros/imu/data', Imu, queue_size=1)])

            def command_cb(message, index=index):
                with lock:
                    commands[index].append(dict(wall=time.monotonic(), mode=message.mode,
                        roll=message.roll, pitch=message.pitch, yaw=message.yaw))

            def status_cb(message, index=index):
                with lock:
                    statuses[index] = json.loads(message.data)

            handles.extend([
                rospy.Subscriber(ns+'/mavros/mount_control/command', MountControl, command_cb, queue_size=10),
                rospy.Subscriber('/gimbal_recovery/status_%d' % (index+1), String, status_cb, queue_size=10)])

        def publish(_):
            with lock:
                current = [dict(scenario) for scenario in scenarios]
            stamp = rospy.Time.now()
            for index, scenario in enumerate(current):
                camera, body = observations(stamp.to_sec(), time.monotonic(),
                    camera_rpy=scenario['camera_rpy'], body_rpy=scenario['body_rpy'],
                    camera_rate=scenario['camera_rate'], body_rate=scenario['body_rate'])
                for kind, sample in enumerate((camera, body)):
                    if not scenario[('camera_enabled', 'body_enabled')[kind]]:
                        continue
                    message = Imu()
                    message.header.stamp = stamp
                    (message.orientation.x, message.orientation.y,
                     message.orientation.z, message.orientation.w) = sample.quaternion
                    (message.angular_velocity.x, message.angular_velocity.y,
                     message.angular_velocity.z) = sample.angular_velocity
                    imu_publishers[index][kind].publish(message)

        timer = rospy.Timer(rospy.Duration(.02), publish)
        # 只在隔离测试参数中开启检测，保留用户 YAML 的总开关状态。
        configuration = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                        'config/gimbal_recovery.yaml').read_text())
        configuration['enabled'] = True
        test_config = outdir.resolve() / 'test_config.yaml'
        test_config.write_text(yaml.safe_dump(configuration))
        start(['roslaunch', 'fly_px4', 'swarm_gimbal_recovery.launch',
               'ns_prefix:=gimbal_smoke', 'drone_num:=6',
               'config_file:=' + str(test_config)], 'recovery')
        wait(lambda: all(s.get('state') == 'MONITORING' for s in statuses),
             'six healthy sensors initialize without recenter', timeout=12.)
        silent(2., range(6), 'healthy flight publishes zero mount commands')
        with lock:
            scenarios[0]['camera_rpy'] = (45., 0., 0.)
        silent(.8, range(6), 'transient tilt is ignored')
        with lock:
            scenarios[0]['camera_rpy'] = (0., 0., 0.)
        silent(.8, range(6), 'transient ends without a command')

        with lock:
            scenarios[0].update(camera_rpy=(45., 0., 0.), body_rpy=(25., 0., 0.),
                                camera_rate=(30., 0., 0.), body_rate=(40., 0., 0.))
        silent(2.8, range(6), 'persistent tilt waits three seconds even during motion')
        wait(lambda: len(commands[0]) == 1, 'persistent roll triggers only drone 0 despite motion')
        assert all(not c for c in commands[1:])
        with lock:
            scenarios[0]['camera_rpy'] = (0., 0., 0.)
        wait(lambda: statuses[0].get('reason') == 'recovered', 'IMU feedback confirms recovery')
        silent(1., range(6), 'recovered gimbal sends no further commands')

        with lock:
            scenarios[1]['camera_rpy'] = (0., -45., 20.)
            scenarios[2].update(camera_rpy=(45., 0., 40.), body_enabled=False)
        wait(lambda: len(commands[1]) == 1, 'persistent pitch reaches only drone 1')
        assert not commands[2]
        wait(lambda: statuses[1].get('reason') == 'recovery_timeout',
             'unresponsive gimbal exhausts bounded retries', timeout=8.)
        assert 1 <= len(commands[1]) <= 3
        assert all(not c for c in commands[2:])
        silent(2., range(6), 'cooldown and missing body feedback publish no commands')

        publishers, subscribers, services = master.getSystemState('/gimbal_recovery_smoke')[2]
        node_name = '/swarm_gimbal_recovery'
        actual_publish = {topic for topic, nodes in publishers if node_name in nodes}
        actual_subscribe = {topic for topic, nodes in subscribers if node_name in nodes}
        expected_publish = {'/rosout'} | {
            '/gimbal_smoke_%d/mavros/mount_control/command' % i for i in range(6)} | {
            '/gimbal_recovery/status_%d' % (i+1) for i in range(6)}
        expected_subscribe = {'/gimbal_smoke_%d%s' % (i, suffix)
            for i in range(6) for suffix in ('/imu_gazebo', '/mavros/imu/data')}
        assert actual_publish == expected_publish, actual_publish
        assert actual_subscribe == expected_subscribe, actual_subscribe
        assert {s for s, nodes in services if node_name in nodes} == {
            node_name+'/get_loggers', node_name+'/set_logger_level'}
        assert all(c['mode'] == 2 and c['roll'] == c['pitch'] == c['yaw'] == 0.
                   for vehicle_commands in commands for c in vehicle_commands)
        for vehicle_commands in commands:
            assert all(b['wall']-a['wall'] >= 3.
                       for a, b in zip(vehicle_commands, vehicle_commands[1:]))
        results.append('ROS graph exposes only IMUs, mount control and diagnostics')

        for enabled in ('true', 'false'):
            listing = subprocess.check_output(['roslaunch', '--nodes', 'fly_px4',
                'swarm_fly_px4_1.launch', 'gimbal_recovery_enabled:='+enabled], env=env, text=True)
            assert ('/swarm_gimbal_recovery' in listing.splitlines()) == (enabled == 'true'), listing
            assert all('/fly_px4_1_%d' % (i+1) in listing.splitlines() for i in range(6)), listing
        results.append('flight launch preserves all six flight nodes and recovery toggle')
        (outdir/'result.json').write_text(json.dumps(dict(passed=True, checks=results,
            command_counts=[len(c) for c in commands], commands=commands,
            publishers=sorted(actual_publish), subscribers=sorted(actual_subscribe)), indent=2)+'\n')
        print('PASS:', results[-1], flush=True)
        return 0
    except Exception:
        for log in log_files:
            log.flush()
        for name in ('recovery.log', 'master.log'):
            path = outdir/name
            if path.exists():
                print(path.read_text()[-4000:], file=sys.stderr)
        raise
    finally:
        if timer:
            timer.shutdown()
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in log_files:
            log.close()
        temporary.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
