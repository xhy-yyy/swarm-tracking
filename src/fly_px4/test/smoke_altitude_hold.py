#!/usr/bin/env python3
"""Exercise the real fly_px4_1 altitude output on a private loopback ROS master.
Inject inconsistent EGO Z PVA across patrol, tracking, STOP, release and fallback.
Requires built fly_px4 and a sourced workspace; no PX4 or Gazebo."""

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
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    output = Path(tempfile.mkdtemp(prefix='altitude_hold_smoke_'))
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', ROS_HOME=str(output / 'ros_home'),
               ROS_LOG_DIR=str(output / 'ros'), ALTITUDE_SMOKE_OUTPUT=str(output))
    env.pop('ROS_HOSTNAME', None)
    env['LD_LIBRARY_PATH'] = ':'.join(
        item for item in env.get('LD_LIBRARY_PATH', '').split(':') if 'racer_ws' not in item)
    processes, logs = [], []

    def start(command, name):
        log = (output / (name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    try:
        master_process = start(['roscore', '-p', str(port)], 'master')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic()+12.
        while time.monotonic() < deadline:
            if master_process.poll() is not None:
                raise RuntimeError('Private ROS master exited')
            try:
                if master.getUri('/altitude_smoke_runner')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('Private ROS master did not start')
        config = {'patrol': {'defaults': {
            'z': 4.5, 'track_z': 2.7, 'start_delay': 0., 'track_stuck_sec': 60.,
            'standoff_radius': 5., 'track_vel_loop_enter_dist': 9.,
            'track_vel_loop_exit_dist': 12.}, 'routes': {'drone_1': {
                'z': 4.5, 'start_delay': 0., 'waypoints': [
                    {'x': 0., 'y': 0., 'z': 4.5},
                    {'x': 40., 'y': 0., 'z': 4.5}]}}}}
        master.setParam('/altitude_smoke_runner', '/altitude_smoke_task', config)
        controller = start([str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'),
                            '__name:=altitude_smoke_task', '_drone_ns:=altitude_smoke_0',
                            '_drone_id:=1', '_drone_num:=1', '_init_x:=0', '_init_y:=0',
                            '_cmd_timeout:=0.15', '_stuck_yaw_sec:=0.35',
                            '_stuck_retreat_sec:=60', '_frozen_sec:=60',
                            '_pose_timeout:=0.2', '_track_release_sec:=0.5',
                            '_track_measurement_timeout:=0.2'], 'controller')
        worker_process = start([sys.executable, __file__, '--worker'], 'checks')
        if worker_process.wait(timeout=55) != 0:
            raise RuntimeError('Altitude control integration checks failed')
        if controller.poll() is not None:
            raise RuntimeError('Controller exited during altitude checks')
        print((output / 'checks.log').read_text())
        print('Altitude hold ROS regression PASSED. Logs:', output)
        return 0
    except Exception:
        for log in logs:
            log.flush()
        print('Altitude hold regression logs:', output)
        for name in ('checks.log', 'controller.log'):
            path = output / name
            if path.exists():
                print(name+':\n'+path.read_text(errors='replace')[-6000:])
        raise
    finally:
        cleanup = []
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=4)
            cleanup.append({'pid': process.pid, 'args': process.args,
                            'returncode': process.returncode})
        for log in logs:
            log.close()
        (output / 'processes.json').write_text(json.dumps(cleanup, indent=2))


def worker():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import PositionTarget, State
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, Int8
    from track_msgs.msg import TrackingTarget

    rospy.init_node('altitude_smoke_checks', disable_signals=True)
    namespace = '/altitude_smoke_0/mavros'
    raw, goals, phases, checks = [], [], [], []
    rospy.Subscriber(namespace+'/setpoint_raw/local', PositionTarget,
                     lambda message: raw.append((time.monotonic(), message)), queue_size=500)
    rospy.Subscriber('/ego_goal_1', PoseStamped,
                     lambda message: goals.append((time.monotonic(), message)), queue_size=100)
    publishers = {
        'state': rospy.Publisher(namespace+'/state', State, queue_size=1),
        'pose': rospy.Publisher(namespace+'/local_position/pose', PoseStamped, queue_size=1),
        'velocity': rospy.Publisher(namespace+'/local_position/velocity_local', TwistStamped, queue_size=1),
        'mode': rospy.Publisher('/tracking/mode_1', Int8, queue_size=1),
        'target': rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1),
        'cmd': rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1),
        'map': rospy.Publisher('/ego_planner_1/grid_map/occupancy_inflate', PointCloud2, queue_size=1)}
    deadline = time.monotonic()+8.
    while not publishers['pose'].get_num_connections():
        if time.monotonic() > deadline:
            raise RuntimeError('Controller did not subscribe to fake pose')
        time.sleep(.02)

    def check(condition, description):
        if not condition:
            raise AssertionError(description)
        checks.append(description)
        print('PASS:', description, flush=True)

    def feed(name, seconds, mode=0, distance=20., command=True, z='follow',
             xy=(0., 0.), observation=True, pose=True):
        begin = time.monotonic()
        frozen_observation = rospy.Time.now()
        count = 0
        rate = rospy.Rate(50)
        while time.monotonic()-begin < seconds and not rospy.is_shutdown():
            now = rospy.Time.now()
            publishers['state'].publish(State(connected=True, armed=True, mode='OFFBOARD'))
            current_z = raw[-1][1].position.z if z == 'follow' and raw else 4.5
            if z != 'follow':
                current_z = z
            if pose:
                position = PoseStamped()
                position.header = Header(stamp=now, frame_id='map')
                position.pose.position.x, position.pose.position.y = xy
                position.pose.position.z = current_z
                # Keep actual yaw consistent with horizontal control and return alignment.
                yaw = raw[-1][1].yaw if mode and raw else 0.
                position.pose.orientation.z = math.sin(yaw/2.)
                position.pose.orientation.w = math.cos(yaw/2.)
                publishers['pose'].publish(position)
            velocity = TwistStamped()
            velocity.header.stamp = now
            publishers['velocity'].publish(velocity)
            publishers['mode'].publish(Int8(data=mode))
            target = TrackingTarget()
            target.state.header = Header(stamp=now, frame_id='world')
            target.state.child_frame_id = target.assignment = 'altitude-smoke:person'
            target.state.pose.pose.position.x = xy[0]+distance
            target.state.pose.pose.position.y = xy[1]
            target.state.pose.pose.position.z = 1.
            target.state.pose.pose.orientation.w = 1.
            target.state.twist.covariance[0] = target.state.twist.covariance[7] = .01
            target.measurement_stamp = now if observation else frozen_observation
            target.depth_stamp = target.measurement_stamp
            publishers['target'].publish(target)
            if command:
                cmd = PositionCommand()
                cmd.header = Header(stamp=now, frame_id='world')
                cmd.position.x, cmd.position.y = .8, .2
                cmd.velocity.x, cmd.velocity.y = .7, -.2
                cmd.acceleration.x, cmd.acceleration.y = .12, -.04
                # Deliberately disagreeing Z PVA; flight output must ignore all three.
                cmd.position.z = (5.9, 1., 7.5, -2.)[count % 4]
                cmd.velocity.z = (3., -3.)[count % 2]
                cmd.acceleration.z = (8., -8.)[count % 2]
                publishers['cmd'].publish(cmd)
            publishers['map'].publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=now, frame_id='world'), []))
            count += 1
            rate.sleep()
        interval = (begin, time.monotonic())
        phases.append({'name': name, 'begin': interval[0], 'end': interval[1]})
        return interval

    def tail(interval, seconds=.25):
        return [message for stamp, message in raw
                if max(interval[0], interval[1]-seconds) <= stamp <= interval[1]]

    def steady(interval, z, description):
        samples = tail(interval)
        z_ignore = PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFZ
        check(len(samples) >= 8 and all(not (m.type_mask & z_ignore) and
              abs(m.position.z-z) < 1e-6 and abs(m.velocity.z) < 1e-6 and
              abs(m.acceleration_or_force.z) < 1e-6 for m in samples), description)

    def is_ego(message):
        return not (message.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                         PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY))

    def is_pid(message):
        return (message.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY)) == (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY)

    def profile(begin, end, direction, description):
        samples = [(stamp, message) for stamp, message in raw if begin <= stamp <= end]
        check(len(samples) > 150 and all(2.7-1e-6 <= m.position.z <= 4.5+1e-6 and
              abs(m.velocity.z) <= .5+1e-6 and abs(m.acceleration_or_force.z) <= 1.+1e-6
              for _, m in samples), description+' respects configured vertical speed/acceleration bounds')
        pairs = [(a.header.stamp.to_sec(), a, b.header.stamp.to_sec(), b)
                 for (_, a), (_, b) in zip(samples, samples[1:])
                 if .008 <= (b.header.stamp-a.header.stamp).to_sec() <= .07]
        check(pairs and all(direction*(b.position.z-a.position.z) >= -1e-7
              for _, a, _, b in pairs), description+' stays monotonic across horizontal mode changes')
        # Compare finite differences at actual publication timestamps to the mean
        # analytic derivative. Small tolerance covers sample spacing and preparation.
        moving = [(ta, a, tb, b) for ta, a, tb, b in pairs
                  if abs(a.velocity.z)+abs(b.velocity.z) > .1]
        p_errors = [abs((b.position.z-a.position.z)/(tb-ta) -
                        .5*(a.velocity.z+b.velocity.z)) for ta, a, tb, b in moving]
        v_errors = [abs((b.velocity.z-a.velocity.z)/(tb-ta) -
                        .5*(a.acceleration_or_force.z+b.acceleration_or_force.z))
                    for ta, a, tb, b in moving]
        check(len(moving) > 80 and max(p_errors) < .02 and max(v_errors) < .03,
              description+' transmitted Z position, velocity and acceleration have consistent derivatives')

    try:
        feed('initial_ground_pose', .2, z=0.)
        warm = feed('patrol', 1., z=4.5)
        steady(warm, 4.5, 'patrol holds YAML cruise altitude despite malicious EGO Z PVA')
        check(any(is_ego(message) for message in tail(warm)), 'patrol still forwards horizontal EGO PVA')
        check(all(abs(message.velocity.x-.7) < 1e-6 and abs(message.velocity.y+.2) < 1e-6
              and abs(message.acceleration_or_force.x-.12) < 1e-6 and
              abs(message.acceleration_or_force.y+.04) < 1e-6
              for message in tail(warm) if is_ego(message)), 'horizontal EGO velocity/acceleration remain unchanged')
        drift = feed('patrol_pose_drift', .5, z=4.42)
        steady(drift, 4.5, 'patrol pose error does not rewrite its altitude setpoint')
        fallback = feed('patrol_ego_loss', .85, command=False, z=4.35)
        steady(fallback, 4.5, 'patrol EGO loss and fallback spin keep cruise altitude')
        check(all(abs(m.velocity.x)+abs(m.velocity.y) < 1e-6 for m in tail(fallback)),
              'patrol fallback stops horizontal motion')
        feed('patrol_ego_recovery', .35, z=4.5)

        down = feed('track_far_descent', 1.5, mode=1)
        check(any(is_ego(m) for m in tail(down)), 'far tracking keeps horizontal EGO trajectory control')
        check(any(abs(g.pose.position.z-2.7) < 1e-6 for stamp, g in goals if stamp >= down[0]),
              'tracking EGO goals use YAML tracking altitude')
        near = feed('track_near_pid_descent', .8, mode=1, distance=6.)
        check(any(is_pid(m) for m in tail(near)), 'near tracking switches to existing horizontal PID velocity loop')
        far = feed('track_far_ego_descent', .8, mode=1)
        check(any(is_ego(m) for m in tail(far)), 'far distance hands horizontal control back to EGO')
        # Preserve the existing horizontal velocity LPF; allow its zero request to settle.
        stopped = feed('tracking_observation_stop', 1.4, mode=1, observation=False)
        check(all(abs(m.velocity.x)+abs(m.velocity.y) < 1e-3 for m in tail(stopped)),
              'observation STOP halts horizontal motion without releasing tracking height')
        release = feed('brief_release_hysteresis', .25, mode=0)
        check(all(abs(m.velocity.x)+abs(m.velocity.y) < 1e-6 for m in tail(release, .15)),
              'brief target release uses zero XY hover through hysteresis')
        feed('tracking_recaptured', 1.5, mode=1)
        settled = feed('tracking_pid_settled', 2., mode=1, distance=6.)
        profile(down[0], settled[1], -1., 'patrol-to-track descent')
        steady(settled, 2.7, 'tracking settles exactly at YAML altitude with zero vertical feedforward')
        drift = feed('track_pose_drift', .6, mode=1, distance=6., z=2.62)
        steady(drift, 2.7, 'tracking PID pose error does not rewrite its altitude setpoint')

        released = feed('target_released', .8, mode=0)
        check(any(abs(g.pose.position.z-4.5) < 1e-6 for stamp, g in goals if stamp >= released[0]),
              'completed target release publishes a cruise-height return goal')
        check(tail(released) and all(m.position.z < 3.0 for m in tail(released)),
              'target release begins a smooth climb without a cruise-height step')
        returned = feed('return_and_patrol', 7.1, mode=0)
        profile(released[0], returned[1], 1., 'track-to-patrol return climb')
        steady(returned, 4.5, 'TRANSIT/PATROL reaches and holds cruise altitude')
        fallback = feed('patrol_fallback_after_return', .85, command=False, z=4.2)
        steady(fallback, 4.5, 'return-to-patrol fallback holds cruise altitude despite measured descent')

        lost_pose = feed('pose_feedback_lost', .55, pose=False)
        check(tail(lost_pose) and all(m.type_mask & PositionTarget.IGNORE_PZ and
              abs(m.velocity.z) < 1e-6 for m in tail(lost_pose)),
              'pose loss clears altitude position reference and preserves zero-velocity heartbeat')
        recovered = feed('pose_feedback_recovered', 1.6, z=4.3)
        steady(recovered, 4.5, 'fresh pose recovery resumes the common cruise-height reference')
        check(any(4.3-1e-6 <= m.position.z < 4.45 for stamp, m in raw
                  if recovered[0]+.04 < stamp < recovered[0]+.3),
              'pose recovery starts altitude reference near measured height')
        print('Altitude hold integration: %d checks passed' % len(checks), flush=True)
        return 0
    finally:
        output = Path(os.environ['ALTITUDE_SMOKE_OUTPUT'])
        (output / 'samples.json').write_text(json.dumps({
            'phases': phases, 'checks': checks,
            'raw': [{'time': stamp, 'mask': m.type_mask, 'z': m.position.z,
                     'vz': m.velocity.z, 'az': m.acceleration_or_force.z,
                     'vx': m.velocity.x, 'vy': m.velocity.y}
                    for stamp, m in raw]}, indent=2))


if __name__ == '__main__':
    sys.exit(worker() if '--worker' in sys.argv else run_isolated())
