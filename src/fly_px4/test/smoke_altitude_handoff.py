#!/usr/bin/env python3
"""Regress staged altitude changes and atomic EGO tracking handoff in isolation.

Runs the actual controller and planner against fake MAVROS/odometry on a private
loopback ROS master. It never connects to the user's flight or simulation master.
"""

import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

PACKAGE = Path(__file__).resolve().parent.parent
WORKSPACE = PACKAGE.parents[1]


def run_isolated():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    logs = Path(tempfile.mkdtemp(prefix='altitude-handoff-ros-'))
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', ROS_HOME=str(logs / 'home'),
               ROS_LOG_DIR=str(logs / 'ros'))
    env.pop('ROS_HOSTNAME', None)
    env['LD_LIBRARY_PATH'] = ':'.join(
        part for part in env.get('LD_LIBRARY_PATH', '').split(':')
        if 'racer_ws' not in part)
    processes, files = [], []

    def start(args, name):
        handle = (logs / (name + '.log')).open('w')
        files.append(handle)
        process = subprocess.Popen(args, env=env, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    try:
        master_process = start(['roscore', '-p', str(port)], 'master')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        deadline = time.monotonic() + 15
        while True:
            try:
                if master.getUri('/altitude_handoff_test')[0] == 1:
                    break
            except OSError:
                pass
            if master_process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Private ROS master failed to start')
            time.sleep(.1)
        config = {'patrol': {
            'defaults': {'z': 4.5, 'start_delay': 0., 'climb_in_place': True,
                         'track_vel_loop_enable': True, 'track_stuck_sec': 60.,
                         'goal_repub_period': 2., 'patrol_yaw_rate_max': 90.},
            'routes': {'drone_1': {'waypoints': [
                {'x': -14., 'y': -3.}, {'x': 26., 'y': -3.},
                {'x': 26., 'y': 37.}, {'x': -14., 'y': 37.}]}}}}
        master.setParam('/altitude_handoff_test', '/altitude_handoff_task', config)
        master.setParam('/altitude_handoff_test', '/swarm_coordinates/local_xy_scale',
                        0.9971746978496311)
        controller = start([
            str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'),
            '__name:=altitude_handoff_task', '_drone_ns:=altitude_handoff_fake',
            '_drone_id:=1', '_init_x:=-14', '_init_y:=-3',
            '_track_release_sec:=0.25'], 'controller')
        worker = start([sys.executable, __file__, '--worker', 'controller'], 'controller_test')
        if worker.wait(timeout=110) != 0:
            raise RuntimeError('Altitude controller regression failed')
        stop(controller)
        planner = start([
            'roslaunch', 'fly_px4', 'single_relay_ego.launch',
            'drone_id:=1', 'xtdrone_idx:=0', 'init_x:=0', 'init_y:=0',
            'slab_bottom:=2.2', 'slab_thick:=3.0', 'map_size_x:=60',
            'map_size_y:=60', 'resolution:=0.3', 'max_vel:=3', 'max_acc:=3',
            'planning_horizon:=10', 'sensor_range:=20', 'lidar_skip:=1',
            'inflation:=0.8', 'cam_pitch_deg:=25', 'depth_topic:=unused',
            'plane_z:=4.5', 'lambda_smooth:=1', 'lambda_collision:=1.5',
            'dist0:=1.5', 'ns:=altitude_handoff_real'], 'planner')
        worker = start([sys.executable, __file__, '--worker', 'planner'], 'planner_test')
        if worker.wait(timeout=55) != 0:
            raise RuntimeError('Real EGO altitude regression failed')
        stop(planner)
        for name in ('controller_test', 'planner_test'):
            print((logs / (name + '.log')).read_text())
        print('Altitude handoff regression PASSED. Logs:', logs)
        return 0
    except Exception:
        for handle in files:
            handle.flush()
        print('Regression logs:', logs, file=sys.stderr)
        for name in ('controller_test', 'planner_test', 'controller', 'planner'):
            path = logs / (name + '.log')
            if path.exists():
                print(name + ':\n' + path.read_text(errors='replace')[-6000:], file=sys.stderr)
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in files:
            handle.close()


def controller_worker():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import PositionTarget, State
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from std_msgs.msg import Int8, UInt64MultiArray
    from track_msgs.msg import TrackingTarget

    rospy.init_node('altitude_handoff_controller_checks', disable_signals=True)
    origin, anchor = (-14., -3.), (6., 7.)
    scale = float(rospy.get_param('/swarm_coordinates/local_xy_scale'))
    ns = '/altitude_handoff_fake/mavros'
    raw, ordinary_goals, track_goals, return_goals = [], [], [], []
    rospy.Subscriber(ns + '/setpoint_raw/local', PositionTarget, raw.append, queue_size=500)
    rospy.Subscriber('/ego_goal_1', PoseStamped, ordinary_goals.append, queue_size=100)
    rospy.Subscriber('/ego_track_goal_1', Odometry, track_goals.append, queue_size=100)
    rospy.Subscriber('/ego_return_goal_1', PoseStamped, return_goals.append, queue_size=100)
    poses = rospy.Publisher(ns + '/local_position/pose', PoseStamped, queue_size=1)
    velocities = rospy.Publisher(ns + '/local_position/velocity_local', TwistStamped, queue_size=1)
    states = rospy.Publisher(ns + '/state', State, queue_size=1)
    modes = rospy.Publisher('/tracking/mode_1', Int8, queue_size=1)
    targets = rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1)
    commands = rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1)
    track_ready = rospy.Publisher('/ego_track_ready_1', UInt64MultiArray, queue_size=1)
    return_ready = rospy.Publisher('/ego_return_ready_1', UInt64MultiArray, queue_size=1)
    deadline = time.monotonic() + 10
    while not poses.get_num_connections():
        if time.monotonic() > deadline:
            raise RuntimeError('Controller did not subscribe to fake MAVROS')
        time.sleep(.02)

    def feed(seconds, point=anchor, yaw=0., track=True, z=4.5,
             speed=(0., 0., 0.), velocity=True, assignment='altitude:person-a',
             trajectory=10, acknowledgement=None, return_acknowledgement=None,
             command_z=4.5, offset=(.2, 0.), follow_z=False, target_distance=25.):
        end = time.monotonic() + seconds
        last_time = time.monotonic()
        actual_z = z
        while time.monotonic() < end and not rospy.is_shutdown():
            now = rospy.Time.now()
            current_time = time.monotonic()
            actual_speed = speed
            if follow_z and raw and not (raw[-1].type_mask & PositionTarget.IGNORE_PZ):
                next_z = raw[-1].position.z
                actual_speed = (0., 0., (next_z - actual_z) / max(current_time-last_time, .001))
                actual_z = next_z
            last_time = current_time
            pose = PoseStamped()
            pose.header.stamp = now
            pose.pose.position.x = (point[0]-origin[0]) / scale
            pose.pose.position.y = (point[1]-origin[1]) / scale
            pose.pose.position.z = actual_z
            pose.pose.orientation.z = math.sin(math.radians(yaw)/2)
            pose.pose.orientation.w = math.cos(math.radians(yaw)/2)
            poses.publish(pose)
            if velocity:
                local_velocity = TwistStamped()
                local_velocity.header.stamp = now
                local_velocity.twist.linear.x = actual_speed[0] / scale
                local_velocity.twist.linear.y = actual_speed[1] / scale
                local_velocity.twist.linear.z = actual_speed[2]
                velocities.publish(local_velocity)
            states.publish(State(connected=True, armed=True, mode='OFFBOARD'))
            modes.publish(Int8(data=1 if track else 0))
            if track:
                target = TrackingTarget()
                target.state.header.stamp = now
                target.state.header.frame_id = 'world'
                target.state.child_frame_id = assignment
                target.state.pose.pose.position.x = point[0]+target_distance
                target.state.pose.pose.position.y = point[1]
                target.state.pose.pose.position.z = 1.
                target.measurement_stamp = target.depth_stamp = now
                target.assignment = assignment
                targets.publish(target)
            if acknowledgement is not None:
                track_ready.publish(UInt64MultiArray(data=acknowledgement))
            if return_acknowledgement is not None:
                return_ready.publish(UInt64MultiArray(data=return_acknowledgement))
            command = PositionCommand()
            command.header.stamp = now
            command.trajectory_id = trajectory
            command.position.x, command.position.y = point[0]+offset[0], point[1]+offset[1]
            command.position.z = command_z
            command.velocity.x, command.velocity.y = (.8, .3)
            command.acceleration.x, command.acceleration.y = (.2, .1)
            commands.publish(command)
            time.sleep(.02)

    checks = 0
    def check(condition, description):
        nonlocal checks
        if not condition:
            raise AssertionError(description)
        checks += 1
        print('PASS:', description, flush=True)

    def held_xy(description, samples=None, point=anchor):
        expected = tuple((point[i]-origin[i])/scale for i in (0, 1))
        samples = raw[-8:] if samples is None else samples
        check(bool(samples) and all(
            not (m.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY))
            and math.hypot(m.position.x-expected[0], m.position.y-expected[1]) < 1e-6
            and abs(m.velocity.x)+abs(m.velocity.y) < 1e-9
            and abs(m.acceleration_or_force.x)+abs(m.acceleration_or_force.y) < 1e-9
            for m in samples), description)

    def at_height(height, description):
        check(all(abs(m.position.z-height) < 1e-6 for m in raw[-8:]), description)

    feed(.4, point=origin, track=False, z=1.5)
    feed(.5, point=origin, track=False)
    check(bool(ordinary_goals), 'initial climb enters normal patrol')
    goal_count = len(ordinary_goals)
    feed(.7, velocity=False)
    check(not track_goals and len(ordinary_goals) == goal_count,
          'missing local velocity cannot start tracking or publish a tracking goal')
    at_height(4.5, 'missing velocity retains current altitude')
    feed(.6, speed=(.7, 0., .3))
    held_xy('braking rejects fresh moving EGO commands')
    at_height(4.5, 'horizontal or vertical motion prevents descending')
    feed(.15)
    at_height(4.5, 'one brief zero-speed sample does not complete braking')
    feed(.5)
    start = len(raw)
    feed(8.)  # The reference finishes; the measured aircraft is deliberately frozen.
    vertical = raw[start:]
    held_xy('the complete descent reference locks the settled XY', vertical)
    check(all(2.7-1e-6 <= m.position.z <= 4.5+1e-6 for m in vertical)
          and all(abs(m.velocity.z) <= .5+1e-6 and
                  abs(m.acceleration_or_force.z) <= 1.+1e-6 for m in vertical),
          'quintic descent stays within altitude, vertical speed and acceleration limits')
    at_height(2.7, 'descent reference reaches the tracking altitude')
    check(not track_goals and not return_goals,
          'elapsed descent time alone cannot release horizontal navigation')
    feed(.5, z=2.7, speed=(0., 0., .3))
    check(not track_goals, 'correct altitude with nonzero measured vertical speed still waits')
    feed(.15, z=2.7)
    check(not track_goals, 'altitude stabilization requires consecutive samples')
    feed(.45, z=2.7)
    check(bool(track_goals) and len(ordinary_goals) == goal_count,
          'stable actual height publishes an atomic tracking request instead of an ordinary goal')
    request = track_goals[-1].header.stamp.to_nsec()
    check(abs(track_goals[-1].pose.pose.position.z-2.7) < 1e-6,
          'EGO tracking request uses the actual horizontal flight plane at 2.7m')
    held_xy('new tracking goal waits for its new trajectory acknowledgement')
    start_stamp = rospy.Time.now().to_nsec()
    feed(.25, z=2.7, acknowledgement=[request+1, 20, start_stamp])
    held_xy('ACK for another tracking request is rejected')
    feed(.25, z=2.7, acknowledgement=[request, 20, start_stamp], trajectory=10)
    held_xy('tracking ACK cannot admit an old patrol trajectory')
    feed(.25, z=2.7, acknowledgement=[request, 20, start_stamp], trajectory=20, command_z=4.5)
    held_xy('matching track ACK rejects a command on the old cruise-height plane')
    feed(.3, z=2.7, acknowledgement=[request, 20, start_stamp], trajectory=20, command_z=2.7)
    check(all(abs(m.velocity.x*scale-.8) < 1e-6 and abs(m.position.z-2.7) < 1e-6
              for m in raw[-8:]), 'matching fresh track ACK releases the new horizontal PVA')

    # A brief mode-0 pause shorter than release hysteresis must revoke the
    # previously accepted trajectory and restart from actual pose on recovery.
    previous_count = len(track_goals)
    feed(.1, track=False, z=2.7)
    feed(1.1, z=2.7, acknowledgement=[request, 20, start_stamp],
         trajectory=20, command_z=2.7)
    check(len(track_goals) > previous_count and track_goals[-1].header.stamp.to_nsec() != request,
          'a brief mode-0 pause revokes the old trajectory even before return release hysteresis')
    held_xy('resuming the same assignment cannot reuse the trajectory ACK from before the pause')

    # Reassignment at an already correct height still brakes and discards old ACKs.
    previous_count = len(track_goals)
    feed(1.1, z=2.7, assignment='altitude:person-b',
         acknowledgement=[request, 20, start_stamp], trajectory=20, command_z=2.7)
    check(len(track_goals) > previous_count and
          track_goals[-1].header.stamp.to_nsec() != request,
          'a new assignment issues its own request even when height is already correct')
    held_xy('previous assignment ACK cannot admit its trajectory into the new assignment')

    # Releasing TRACK must ascend first; no return goal can run at tracking height.
    count = len(return_goals)
    yaw = math.degrees(math.atan2(-3.-anchor[1], 26.-anchor[0]))
    start = len(raw)
    feed(1., track=False, z=2.7, yaw=yaw)
    climb = raw[start:]
    held_xy('TRACK release brakes then climbs with fixed XY', climb[20:])
    check(climb[20].position.z < 2.75 and
          all(m.position.z <= 4.5+1e-6 and abs(m.velocity.z) <= .5+1e-6 for m in climb[20:]),
          'return climb starts continuously at tracking height without minimum-height clamp jump')
    check(len(return_goals) == count, 'yaw alignment cannot bypass unfinished return climb')

    # Interrupt an ongoing ascent and return to the measured tracking height.
    previous_count = len(track_goals)
    feed(1.1, z=2.7, assignment='altitude:person-c', yaw=yaw)
    check(len(track_goals) > previous_count and len(return_goals) == count,
          'new TRACK during ascent cancels the pending return and starts a fresh tracking handoff')
    held_xy('ascent cancellation still waits for the new tracking ACK')
    request = track_goals[-1].header.stamp.to_nsec()
    start_stamp = rospy.Time.now().to_nsec()
    feed(.3, z=2.7, assignment='altitude:person-c', yaw=yaw,
         acknowledgement=[request, 30, start_stamp], trajectory=30, command_z=2.7)

    # Exercise actual ascent motion by following the generated Z setpoint.
    start = len(raw)
    feed(8., track=False, z=2.7, yaw=yaw, follow_z=True)
    climb = raw[start:]
    held_xy('actual climb keeps horizontal hold until a new return ACK', climb[20:])
    check(all(abs(m.velocity.z) <= .5+1e-6 and abs(m.acceleration_or_force.z) <= 1.+1e-6
              for m in climb[20:]), 'ascent reference respects vertical speed and acceleration limits')
    check(len(return_goals) > count and abs(return_goals[-1].pose.position.z-4.5) < 1e-6,
          'actual height stabilization and measured yaw alignment publish a 4.5m return goal')
    held_xy('completed ascent holds while fresh old tracking commands continue')
    return_request = return_goals[-1].header.stamp.to_nsec()
    start_stamp = rospy.Time.now().to_nsec()
    feed(.3, track=False, z=4.5, yaw=yaw, trajectory=40,
         return_acknowledgement=[return_request, 40, start_stamp])
    check(all(abs(m.velocity.x*scale-.8) < 1e-6 and abs(m.position.z-4.5) < 1e-6
              for m in raw[-8:]), 'only matching new return ACK permits horizontal flight at cruise height')
    # Cancel a descent before reaching 2.7m: return must stabilize the actual
    # current height and climb from there, instead of continuing the old ramp.
    previous_track_count = len(track_goals)
    feed(.8, z=4.5, assignment='altitude:person-d', yaw=yaw)
    feed(.3, z=4.3, assignment='altitude:person-d', yaw=yaw)
    check(len(track_goals) == previous_track_count,
          'a new descent cannot issue a tracking goal before actual height is reached')
    count = len(return_goals)
    start = len(raw)
    feed(1.7, track=False, z=4.3, yaw=yaw)
    canceled = raw[start:]
    held_xy('release during descent cancels tracking and keeps the current horizontal anchor', canceled[20:])
    check(len(return_goals) == count and all(4.3-1e-6 <= m.position.z <= 4.5+1e-6
                                           for m in canceled[20:]),
          'interrupted descent climbs from actual 4.3m and still waits for measured 4.5m')
    feed(.5, track=False, z=4.5, yaw=yaw)
    check(len(return_goals) > count, 'interrupted descent publishes return only after actual cruise height settles')
    previous_track_count = len(track_goals)
    feed(1.1, z=2.7, assignment='altitude:person-near', target_distance=7.)
    check(len(track_goals) > previous_track_count, 'near-distance tracking also requires a fresh EGO request')
    held_xy('near-distance PID cannot bypass altitude and initial trajectory acknowledgement')
    request = track_goals[-1].header.stamp.to_nsec()
    start_stamp = rospy.Time.now().to_nsec()
    feed(.4, z=2.7, assignment='altitude:person-near', target_distance=7.,
         acknowledgement=[request, 60, start_stamp], trajectory=60, command_z=2.7)
    check(all((m.type_mask & PositionTarget.IGNORE_PX) and
              (m.type_mask & PositionTarget.IGNORE_PY) and
              not (m.type_mask & PositionTarget.IGNORE_PZ) and abs(m.position.z-2.7) < 1e-6
              for m in raw[-8:]), 'PID enters only after matching ACK and keeps the same 2.7m flight plane')
    check(all(math.isfinite(v) for m in raw for v in
              (m.position.x, m.position.y, m.position.z, m.velocity.x,
               m.velocity.y, m.velocity.z, m.acceleration_or_force.z)),
          'all transition flight targets are finite')
    print('Controller altitude regression: %d checks passed' % checks, flush=True)


def planner_worker():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header, UInt64MultiArray
    from traj_utils.msg import Bspline

    rospy.init_node('altitude_handoff_planner_checks', disable_signals=True)
    track_acks, return_acks, trajectories, commands, planner_targets = [], [], [], [], []
    rospy.Subscriber('/ego_track_ready_1', UInt64MultiArray, track_acks.append, queue_size=100)
    rospy.Subscriber('/ego_return_ready_1', UInt64MultiArray, return_acks.append, queue_size=100)
    rospy.Subscriber('/drone_0_planning/bspline', Bspline, trajectories.append, queue_size=100)
    rospy.Subscriber('/ego/pos_cmd_1', PositionCommand, commands.append, queue_size=100)
    def on_planner_log(message):
        if message.name != '/ego_planner_1':
            return
        match = re.search(r'local_tgt=\(([-\d.]+),([-\d.]+)\) v_tgt=([-\d.]+) '
                          r'end=\(([-\d.]+),([-\d.]+)\).* success=1', message.msg)
        if match:
            planner_targets.append(tuple(float(value) for value in match.groups()))
    rospy.Subscriber('/rosout', Log, on_planner_log, queue_size=100)
    odometry = rospy.Publisher('/altitude_handoff_real/odom_world_relay', Odometry, queue_size=1)
    clouds = rospy.Publisher('/altitude_handoff_real/obstacle_cloud_world', PointCloud2, queue_size=1)
    track_goal = rospy.Publisher('/ego_track_goal_1', Odometry, queue_size=1)
    ordinary_goal = rospy.Publisher('/ego_goal_1', PoseStamped, queue_size=1)
    return_goal = rospy.Publisher('/ego_return_goal_1', PoseStamped, queue_size=1)
    goal_velocity = rospy.Publisher('/ego_goal_vel_1', TwistStamped, queue_size=1, latch=True)
    deadline = time.monotonic()+10
    while not odometry.get_num_connections() or not track_goal.get_num_connections():
        if time.monotonic() > deadline:
            raise RuntimeError('Real EGO did not subscribe to altitude test topics')
        time.sleep(.02)
    wall = [(-4., y*.3, 4.5) for y in range(-7, 8)]

    def feed(seconds, point=(-8., 0.), z=2.7):
        end = time.monotonic()+seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            now = rospy.Time.now()
            message = Odometry()
            message.header = Header(stamp=now, frame_id='world')
            message.pose.pose.position.x, message.pose.pose.position.y = point
            message.pose.pose.position.z = z
            message.pose.pose.orientation.w = 1.
            odometry.publish(message)
            clouds.publish(create_cloud_xyz32(message.header, wall))
            time.sleep(.02)

    def wait_for(condition, description, point=(-8., 0.), z=2.7, timeout=10):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            feed(.1, point, z)
            if condition():
                return
        raise AssertionError(description)

    checks = 0
    def check(condition, description):
        nonlocal checks
        if not condition:
            raise AssertionError(description)
        checks += 1
        print('PASS:', description, flush=True)

    def new_track(point, z):
        message = Odometry()
        message.header = Header(stamp=rospy.Time.now(), frame_id='world')
        message.child_frame_id = 'altitude:person'
        message.pose.pose.position.x, message.pose.pose.position.y = point
        message.pose.pose.position.z = z
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = .5
        return message

    def matching_ack(collection, stamp):
        return next((ack for ack in collection if len(ack.data) == 3 and ack.data[0] == stamp), None)

    def spline_start(t):
        return tuple((getattr(t.pos_pts[0], a)+4*getattr(t.pos_pts[1], a)+
                      getattr(t.pos_pts[2], a))/6 for a in ('x', 'y', 'z'))

    def spline_samples(t):
        samples = []
        for i in range(len(t.pos_pts)-3):
            for k in range(21):
                u = k/20.
                basis = ((1-u)**3/6, (3*u**3-6*u**2+4)/6,
                         (-3*u**3+3*u**2+3*u+1)/6, u**3/6)
                samples.append(tuple(sum(b*getattr(t.pos_pts[i+j], a)
                                         for j, b in enumerate(basis)) for a in ('x', 'y', 'z')))
        return samples

    feed(1.)
    stale_velocity = TwistStamped()
    stale_velocity.header.stamp = rospy.Time.now()
    stale_velocity.twist.linear.x, stale_velocity.twist.linear.y = (1.2, .8)
    goal_velocity.publish(stale_velocity)
    feed(.2)
    request = new_track((0., 0.), 2.7)
    track_goal.publish(request)
    stamp = request.header.stamp.to_nsec()
    wait_for(lambda: matching_ack(track_acks, stamp), 'Atomic track request never received an ACK')
    ack = matching_ack(track_acks, stamp)
    wait_for(lambda: any(t.traj_id == ack.data[1] for t in trajectories), 'ACK spline was not published')
    trajectory = next(t for t in trajectories if t.traj_id == ack.data[1])
    check(trajectory.start_time.to_nsec() == ack.data[2] and ack.data[2] >= stamp,
          'track ACK identifies the fresh B-spline and its start time')
    check(math.dist(spline_start(trajectory), (-8., 0., 2.7)) < .1,
          'tracking spline starts at current actual odometry and tracking height')
    samples = spline_samples(trajectory)
    check(all(abs(p[2]-2.7) < .03 for p in samples),
          'all tracking spline samples use 2.7m instead of launch cruise plane')
    crossings = [p for p in samples if abs(p[0]+4.) < .4]
    check(bool(crossings) and all(abs(p[1]) > 2.3 for p in crossings),
          'tracking plane at 2.7m still avoids wall samples acquired at 4.5m')
    wait_for(lambda: any(c.trajectory_id == trajectory.traj_id for c in commands),
             'traj_server did not emit acknowledged tracking commands')
    check(True, 'real traj_server emits commands belonging to acknowledged tracking trajectory')
    wait_for(lambda: any(math.hypot(*record[3:5]) < .05 and abs(record[2]-.5) < .06
                         for record in planner_targets),
             'atomic tracking request did not preserve its target velocity')
    check(True, 'atomic request binds its own terminal speed instead of retaining the old goal velocity')
    track_goal.publish(request)
    feed(.15)
    repeated = [a for a in track_acks if len(a.data) == 3 and a.data[0] == stamp]
    check(len(repeated) >= 2 and all(a.data == ack.data for a in repeated),
          'retrying identical atomic track request preserves ACK identity')

    relocated = (-8., 6.)
    feed(.3, relocated, 4.5)
    return_request = PoseStamped()
    return_request.header = Header(stamp=rospy.Time.now(), frame_id='world')
    return_request.pose.position.x, return_request.pose.position.y = (-2., 6.)
    return_request.pose.position.z = 4.5
    return_request.pose.orientation.w = 1.
    return_goal.publish(return_request)
    return_stamp = return_request.header.stamp.to_nsec()
    wait_for(lambda: matching_ack(return_acks, return_stamp), 'Return after track never ACKed', relocated, 4.5)
    return_ack = matching_ack(return_acks, return_stamp)
    wait_for(lambda: any(t.traj_id == return_ack.data[1] for t in trajectories),
             'Return spline was not published', relocated, 4.5)
    returning = next(t for t in trajectories if t.traj_id == return_ack.data[1])
    check(math.dist(spline_start(returning), (*relocated, 4.5)) < .1 and
          all(abs(p[2]-4.5) < .03 for p in spline_samples(returning)),
          'return switches back to cruise plane and restarts from actual elevated odometry')
    count = len(track_acks)
    track_goal.publish(request)
    feed(.3, relocated, 4.5)
    check(len(track_acks) == count, 'retained old tracking request cannot overwrite a newer return')
    invalid = new_track((-2., 6.), 100.)
    track_goal.publish(invalid)
    feed(.5, relocated, 4.5)
    check(not matching_ack(track_acks, invalid.header.stamp.to_nsec()),
          'tracking altitude outside the map is rejected without ACK')
    feed(.2, relocated, 2.7)
    ordinary = PoseStamped()
    ordinary.header = Header(stamp=rospy.Time.now(), frame_id='world')
    ordinary.pose.position.x, ordinary.pose.position.y = (-2., 6.)
    ordinary.pose.position.z = 2.7
    ordinary.pose.orientation.w = 1.
    ordinary_goal.publish(ordinary)
    wait_for(lambda: any(t.traj_id > returning.traj_id and
                        abs(spline_start(t)[2]-2.7) < .03 for t in trajectories),
             'ordinary tracking update did not restore its requested plane', relocated, 2.7)
    newest = next(t for t in trajectories if t.traj_id > returning.traj_id and
                  abs(spline_start(t)[2]-2.7) < .03)
    check(all(abs(p[2]-2.7) < .03 for p in spline_samples(newest)),
          'a newer ordinary tracking update also sets the requested horizontal plane')
    count = len(return_acks)
    return_goal.publish(return_request)
    feed(.3, relocated, 2.7)
    check(len(return_acks) == count, 'retained older return request cannot reset a newer tracking plane')
    print('Real EGO altitude regression: %d checks passed' % checks, flush=True)


if __name__ == '__main__':
    if '--worker' in sys.argv:
        controller_worker() if sys.argv[-1] == 'controller' else planner_worker()
    else:
        sys.exit(run_isolated())
