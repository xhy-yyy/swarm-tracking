#!/usr/bin/env python3
"""Run the real tracking yaw path on a private localhost ROS master.

No Gazebo or PX4 is needed. The runner only stops processes it starts and
retains controller logs plus the received setpoints/diagnostics in /tmp.
Source ROS and catkin_ws/devel/setup.bash before running this file.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

WORKSPACE = Path(__file__).resolve().parents[3]


def run_isolated():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    output = Path(tempfile.mkdtemp(prefix='tracking_yaw_'))
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1',
               ROS_LOG_DIR=str(output / 'ros'), YAW_SMOKE_OUTPUT_DIR=str(output))
    env['LD_LIBRARY_PATH'] = ':'.join(
        p for p in env.get('LD_LIBRARY_PATH', '').split(':') if 'racer_ws' not in p)
    processes, files = [], []

    def start(args, name):
        log = (output / (name + '.log')).open('w')
        files.append(log)
        process = subprocess.Popen(args, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)

    try:
        master_process = start(['roscore', '-p', str(port)], 'master')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 8.
        while time.monotonic() < deadline:
            if master_process.poll() is not None:
                raise RuntimeError('private ROS master exited')
            try:
                if master.getUri('/yaw_smoke_runner')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('private ROS master did not become ready')
        config = {'patrol': {'defaults': {
            'z': 4., 'start_delay': 0., 'track_z': 4., 'track_stuck_sec': 60.,
            'standoff_radius': 5., 'track_vel_loop_enter_dist': 9.,
            'track_vel_loop_exit_dist': 12.}, 'routes': {'drone_1': {
                'z': 4., 'start_delay': 0., 'waypoints': [
                    {'x': 0., 'y': 0., 'z': 4.}, {'x': 20., 'y': 0., 'z': 4.}]}}}}
        master.setParam('/yaw_smoke_runner', '/yaw_smoke_task', config)
        controller = start([str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'),
                            '__name:=yaw_smoke_task', '_drone_ns:=yaw_smoke_0',
                            '_drone_id:=1', '_drone_num:=1', '_init_x:=0', '_init_y:=0',
                            '_min_expl_z:=4', '_max_expl_z:=4',
                            '_cmd_timeout:=0.15', '_stuck_yaw_sec:=60', '_frozen_sec:=60',
                            '_track_release_sec:=0.2'], 'controller')
        worker = start([sys.executable, __file__, '--worker'], 'checks')
        if worker.wait(timeout=35) != 0:
            raise RuntimeError('tracking yaw integration checks failed')
        if controller.poll() is not None:
            raise RuntimeError('controller exited during tracking yaw checks')
        print((output / 'checks.log').read_text())
        print('Tracking yaw ROS regression PASSED. Logs:', output)
        return 0
    except Exception:
        print('Tracking yaw ROS regression logs:', output)
        for name in ('checks.log', 'controller.log', 'master.log'):
            path = output / name
            if path.exists():
                print(name + ':\n' + path.read_text()[-4500:])
        raise
    finally:
        cleanup = []
        for process in reversed(processes):
            stop(process)
            cleanup.append(dict(pid=process.pid, args=process.args, returncode=process.returncode))
        for log in files:
            log.close()
        (output / 'processes.json').write_text(json.dumps(cleanup, indent=2))


def worker():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import PositionTarget, State
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32MultiArray, Header, Int8
    from track_msgs.msg import TrackingTarget, TrackingView

    rospy.init_node('yaw_smoke_checks')
    namespace = '/yaw_smoke_0/mavros'
    raw, diagnostics, phases = [], [], []
    rospy.Subscriber(namespace + '/setpoint_raw/local', PositionTarget,
                     lambda msg: raw.append((time.monotonic(), msg)), queue_size=200)
    rospy.Subscriber('/tracking/yaw_diag_1', Float32MultiArray,
                     lambda msg: diagnostics.append((time.monotonic(), list(msg.data))),
                     queue_size=200)
    publishers = {
        'state': rospy.Publisher(namespace + '/state', State, queue_size=1),
        'pose': rospy.Publisher(namespace + '/local_position/pose', PoseStamped, queue_size=1),
        'velocity': rospy.Publisher(namespace + '/local_position/velocity_local',
                                    TwistStamped, queue_size=1),
        'mode': rospy.Publisher('/tracking/mode_1', Int8, queue_size=1),
        'target': rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1),
        'view': rospy.Publisher('/tracking/view_1', TrackingView, queue_size=1),
        'cmd': rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1),
        'map': rospy.Publisher('/ego_planner_1/grid_map/occupancy_inflate',
                               PointCloud2, queue_size=1)}
    assignment, angle = 'yaw-smoke:person-1', 0.
    checks = []
    metrics = {}

    def check(condition, message):
        if not condition:
            raise AssertionError(message)
        checks.append(message)
        print('PASS:', message, flush=True)

    def wrap(value):
        return math.atan2(math.sin(value), math.cos(value))

    def run(name, duration, view='valid', speed=0., distance=20., command=False, mode=1):
        nonlocal angle
        begin = time.monotonic()
        initial = angle
        count = 0
        rate = rospy.Rate(50)
        while time.monotonic() - begin < duration and not rospy.is_shutdown():
            elapsed = time.monotonic() - begin
            current = initial + speed * elapsed
            now = rospy.Time.now()
            publishers['state'].publish(State(connected=True, armed=True, mode='OFFBOARD'))
            pose = PoseStamped()
            pose.header.stamp = now
            pose.pose.position.z = 4.
            actual_yaw = raw[-1][1].yaw if raw else 0.
            pose.pose.orientation.z, pose.pose.orientation.w = (
                math.sin(actual_yaw / 2.), math.cos(actual_yaw / 2.))
            publishers['pose'].publish(pose)
            velocity = TwistStamped()
            velocity.header.stamp = now
            publishers['velocity'].publish(velocity)
            publishers['mode'].publish(Int8(data=mode))
            target = TrackingTarget()
            target.state.header = Header(stamp=now, frame_id='world')
            target.state.child_frame_id = assignment
            los = math.radians(current + 10.)
            target.state.pose.pose.position.x = distance * math.cos(los)
            target.state.pose.pose.position.y = distance * math.sin(los)
            target.state.pose.pose.position.z = 1.0191
            target.state.pose.pose.orientation.w = 1.
            target.state.twist.covariance[0] = .01
            target.state.twist.covariance[7] = .01
            target.measurement_stamp = now
            target.depth_stamp, target.assignment = now, assignment
            publishers['target'].publish(target)
            if view is not None and count % 5 == 0:
                observation = TrackingView()
                lag = .8 if view == 'stale' else .02
                observation.header = Header(stamp=now - rospy.Duration(lag), frame_id='world')
                observation.assignment = 'wrong-person' if view == 'assignment' else assignment
                observation.drone_id = 2 if view == 'drone' else 1
                observation.bearing = math.radians(current - speed * lag + (
                    70. if view != 'valid' else 0.))
                observation.horizontal_error_px, observation.confidence = -17., .95
                publishers['view'].publish(observation)
            if command:
                cmd = PositionCommand()
                cmd.header.stamp = now
                cmd.position.x, cmd.position.y, cmd.position.z = .8, .2, 4.
                cmd.velocity.x, cmd.velocity.y = .7, .2
                cmd.acceleration.x, cmd.acceleration.y = .12, -.04
                publishers['cmd'].publish(cmd)
            publishers['map'].publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=now, frame_id='world'), []))
            count += 1
            rate.sleep()
        end = time.monotonic()
        angle = initial + speed * (end - begin)
        phases.append(dict(name=name, begin=begin, end=end, initial_deg=initial,
                           speed_degps=speed, view=view, command=command, distance=distance))
        return begin, end

    def tail(records, interval, seconds=.3):
        return [value for stamp, value in records if interval[1] - seconds <= stamp <= interval[1]]

    def enabled(msg):
        return not (msg.type_mask & (PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE))

    def verify_rate_path(interval, required_mask, message):
        samples = [msg for msg in tail(raw, interval, .45) if msg.type_mask == required_mask]
        check(len(samples) >= 10 and all(enabled(msg) for msg in samples)
              and any(abs(msg.yaw_rate) > .08 for msg in samples), message)
        for msg in samples:
            matches = [diag for stamp, diag in diagnostics
                       if interval[0] <= stamp <= interval[1]
                       and abs(wrap(msg.yaw - math.radians(diag[1]))) < 2e-5]
            check_matches = matches and min(abs(msg.yaw_rate - math.radians(d[2]))
                                           for d in matches) < 2e-5
            if not check_matches:
                raise AssertionError('setpoint yaw/yaw_rate pair differs from controller diagnostics')

    try:
        warm = run('warmup', 1.5, view=None, mode=0)
        check(len(tail(raw, warm)) >= 8, 'private controller publishes a 50 Hz setpoint stream')
        static = run('visual_priority', 1.6)
        ds = tail(diagnostics, static)
        check(len(ds) >= 10 and all(d[3] == 1. for d in ds)
              and max(abs(d[1]) for d in ds) < 1.
              and all(abs(d[6] - 10.) < .05 for d in ds),
              'fresh assigned image bearing overrides a fusion LOS offset by 10 degrees')
        for kind in ('drone', 'assignment', 'stale'):
            interval = run('reject_' + kind, .9, view=kind)
            ds = tail(diagnostics, interval)
            check(len(ds) >= 8 and all(d[3] == 0. for d in ds)
                  and max(abs(d[1] - 10.) for d in ds) < 3.,
                  kind + ' mismatch cannot steer toward the injected bad image bearing')
        angle = 10.
        pva = run('pva_motion', 2., speed=20., command=True)
        verify_rate_path(pva, 0, 'EGO PVA uses the continuous nonzero tracking yaw_rate')
        ds = tail(diagnostics, pva, .45)
        check(all(d[3] == 1. for d in ds)
              and max(abs(d[2] - 20.) for d in ds) < 5.,
              '10 Hz image observations produce approximately 20 deg/s at 50 Hz control')
        steady = [(stamp, diag) for stamp, diag in diagnostics
                  if pva[1] - .3 <= stamp <= pva[1]]
        initial = phases[-1]['initial_deg']
        errors = [wrap(math.radians(diag[1] - initial - 20. * (stamp - pva[0])))
                  for stamp, diag in steady]
        check(len(errors) >= 10 and max(abs(error) for error in errors) < math.radians(1.),
              '10 Hz moving-image feedback has less than one degree steady reference error')
        metrics['twenty_degps_final_error_deg'] = math.degrees(errors[-1])
        metrics['twenty_degps_tail_max_error_deg'] = math.degrees(max(abs(e) for e in errors))
        print('20 deg/s reference error: final=%+.4f deg, tail_max=%.4f deg' % (
            metrics['twenty_degps_final_error_deg'], metrics['twenty_degps_tail_max_error_deg']),
            flush=True)
        samples = [msg for stamp, msg in raw if pva[0] + .4 < stamp < pva[1]
                   and msg.type_mask == 0]
        pairs = [(a, b) for a, b in zip(samples, samples[1:])
                 if .008 < (b.header.stamp - a.header.stamp).to_sec() < .06]
        derivative_errors = [abs(wrap(b.yaw - a.yaw) /
                                 (b.header.stamp - a.header.stamp).to_sec() - b.yaw_rate)
                             for a, b in pairs]
        check(len(derivative_errors) >= 40 and max(derivative_errors) < .08,
              'transmitted yaw_rate agrees with the timestamped yaw reference derivative')
        saturated = run('bounded_fast_turn', 1.2, speed=100., command=True)
        samples = tail(raw, saturated, 1.)
        check(samples and max(abs(m.yaw_rate) for m in samples) <= math.radians(60.) + 1e-5
              and max(abs(m.yaw_rate) for m in samples) > math.radians(58.),
              'demand above 60 deg/s is capped at the configured yaw rate')
        reverse = run('reverse', 1.6, speed=-20., command=True)
        check(any(m.yaw_rate < -.1 for m in tail(raw, reverse, .4)),
              'coherent reverse image motion reverses yaw_rate')
        bounded = [(stamp, msg) for stamp, msg in raw
                   if pva[0] + .1 < stamp < reverse[1] and msg.type_mask == 0]
        acceleration = [abs(b.yaw_rate - a.yaw_rate) /
                        (b.header.stamp - a.header.stamp).to_sec()
                        for (_, a), (_, b) in zip(bounded, bounded[1:])
                        if .008 < (b.header.stamp - a.header.stamp).to_sec() < .06]
        check(acceleration and max(acceleration) < math.radians(180.) + .3,
              'acceleration remains within the configured 180 deg/s^2 through reversal')
        hover = run('no_ego_hover', 1.7, speed=-20., command=False)
        ignore_acc = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        verify_rate_path(hover, ignore_acc,
                         'no-EGO hover preserves the same active yaw/yaw_rate pair')
        check(all(abs(m.velocity.x) + abs(m.velocity.y) < 1e-6
                  for m in tail(raw, hover, .4)), 'no-EGO hover sends zero horizontal velocity')
        near = run('near_velocity', 1.7, speed=-20., distance=6., command=True)
        verify_rate_path(near, ignore_acc | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY,
                         'near-distance velocity control preserves the same active yaw/yaw_rate pair')
        lost = run('view_loss', 1.4, view=None, distance=20., command=True)
        ds = tail(diagnostics, lost)
        check(len(ds) >= 10 and all(d[3] == 0. for d in ds)
              and max(abs(wrap(math.radians(d[1] - d[6]))) for d in ds) < math.radians(2.),
              'image loss smoothly returns yaw to the present fusion LOS')
        loss_samples = [m for stamp, m in raw if lost[0] < stamp < lost[1] and m.type_mask == 0]
        loss_acc = [abs(b.yaw_rate - a.yaw_rate) /
                    (b.header.stamp - a.header.stamp).to_sec()
                    for a, b in zip(loss_samples, loss_samples[1:])
                    if .008 < (b.header.stamp - a.header.stamp).to_sec() < .06]
        check(loss_acc and max(loss_acc) < math.radians(180.) + .3,
              'image-to-coordinate fallback obeys the same yaw acceleration bound')
        # Re-enable motion briefly, then keep publishing old-task images after release.
        run('before_release', .5, speed=-20., command=True)
        released = run('release', 1., speed=-20., command=True, mode=0)
        check(all(abs(m.yaw_rate) < 1e-6 for m in tail(raw, released, .5)),
              'OFF mode withdraws the old pedestrian yaw feedforward')
        check(not tail(diagnostics, released, .5),
              'continued old-person images cannot reactivate tracking after OFF mode')
        check(all(math.isfinite(v) for _, m in raw for v in (m.yaw, m.yaw_rate)),
              'all transmitted yaw and yaw_rate values remain finite')
        print('Tracking yaw ROS integration: %d checks passed' % len(checks), flush=True)
    finally:
        report = {'checks': checks, 'phases': phases, 'metrics': metrics,
                  'raw': [dict(receipt=t, stamp=m.header.stamp.to_sec(), mask=m.type_mask,
                               yaw=m.yaw, yaw_rate=m.yaw_rate, vx=m.velocity.x, vy=m.velocity.y)
                          for t, m in raw],
                  'diagnostics': [dict(receipt=t, data=d) for t, d in diagnostics]}
        (Path(os.environ['YAW_SMOKE_OUTPUT_DIR']) / 'observations.json').write_text(
            json.dumps(report, indent=2))


if __name__ == '__main__':
    if '--worker' in sys.argv:
        worker()
    else:
        sys.exit(run_isolated())
