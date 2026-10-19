#!/usr/bin/env python3
"""Regress TRACK -> return yaw hold and EGO trajectory handoff in isolation.

Starts a private ROS master on a random loopback port. The controller worker
uses fake MAVROS/EGO; the planner worker exercises the actual EGO and traj_server.
No connection to the user's ROS master or flight hardware is made.
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
    logs = Path(tempfile.mkdtemp(prefix='return-handoff-ros-'))
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
                if master.getUri('/return_handoff_test')[0] == 1:
                    break
            except OSError:
                pass
            if master_process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Private ROS master failed to start')
            time.sleep(.1)

        # The initial LPF position is local (0,0); the return hold is 20m away.
        config = {'patrol': {
            'defaults': {'z': 4.5, 'start_delay': 0., 'climb_in_place': True,
                         'track_vel_loop_enable': False, 'track_stuck_sec': 60.,
                         'goal_repub_period': 2., 'patrol_yaw_rate_max': 90.},
            'routes': {'drone_1': {'waypoints': [
                {'x': -14., 'y': -3.}, {'x': 26., 'y': -3.},
                {'x': 26., 'y': 37.}, {'x': -14., 'y': 37.}]}}}}
        master.setParam('/return_handoff_test', '/return_handoff_task', config)
        master.setParam('/return_handoff_test', '/swarm_coordinates/local_xy_scale',
                        0.9971746978496311)
        controller = start([
            str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'),
            '__name:=return_handoff_task', '_drone_ns:=return_handoff_fake',
            '_drone_id:=1', '_init_x:=-14', '_init_y:=-3',
            '_track_release_sec:=0.25'], 'controller')
        worker = start([sys.executable, __file__, '--worker', 'controller'], 'controller_test')
        if worker.wait(timeout=45) != 0:
            raise RuntimeError('Return controller regression failed')
        stop(controller)

        planner = start([
            'roslaunch', 'fly_px4', 'single_relay_ego.launch',
            'drone_id:=1', 'xtdrone_idx:=0', 'init_x:=0', 'init_y:=0',
            'slab_bottom:=2.2', 'slab_thick:=3.0', 'map_size_x:=60',
            'map_size_y:=60', 'resolution:=0.3', 'max_vel:=3', 'max_acc:=3',
            'planning_horizon:=10', 'sensor_range:=20', 'lidar_skip:=1',
            'inflation:=0.8', 'cam_pitch_deg:=25', 'depth_topic:=unused',
            'plane_z:=4.5', 'lambda_smooth:=1', 'lambda_collision:=1.5',
            'dist0:=1.5', 'ns:=return_handoff_real'], 'planner')
        worker = start([sys.executable, __file__, '--worker', 'planner'], 'planner_test')
        if worker.wait(timeout=35) != 0:
            raise RuntimeError('Real EGO return regression failed')
        stop(planner)
        for name in ('controller_test', 'planner_test'):
            print((logs / (name + '.log')).read_text())
        print('Return handoff regression PASSED. Logs:', logs)
        return 0
    except Exception:
        for handle in files:
            handle.flush()
        print('Regression logs:', logs, file=sys.stderr)
        for name in ('controller_test', 'planner_test', 'controller', 'planner'):
            path = logs / (name + '.log')
            if path.exists():
                print(name + ':\n' + path.read_text(errors='replace')[-5000:], file=sys.stderr)
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        for handle in files:
            handle.close()


def controller_worker():
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import PositionTarget, State
    from quadrotor_msgs.msg import PositionCommand
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header, Int8, UInt64MultiArray
    from track_msgs.msg import TrackingTarget

    rospy.init_node('return_handoff_controller_checks', disable_signals=True)
    origin = (-14., -3.)
    scale = float(rospy.get_param('/swarm_coordinates/local_xy_scale'))
    ns = '/return_handoff_fake/mavros'
    raw, ordinary_goals, return_goals, track_goals = [], [], [], []
    rospy.Subscriber(ns + '/setpoint_raw/local', PositionTarget, raw.append, queue_size=200)
    rospy.Subscriber('/ego_goal_1', PoseStamped, ordinary_goals.append, queue_size=100)
    rospy.Subscriber('/ego_return_goal_1', PoseStamped, return_goals.append, queue_size=100)
    rospy.Subscriber('/ego_track_goal_1', Odometry, track_goals.append, queue_size=100)
    poses = rospy.Publisher(ns + '/local_position/pose', PoseStamped, queue_size=1)
    velocities = rospy.Publisher(ns + '/local_position/velocity_local', TwistStamped, queue_size=1)
    states = rospy.Publisher(ns + '/state', State, queue_size=1)
    modes = rospy.Publisher('/tracking/mode_1', Int8, queue_size=1)
    targets = rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1)
    commands = rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1)
    ready = rospy.Publisher('/ego_return_ready_1', UInt64MultiArray, queue_size=1)
    track_ready = rospy.Publisher('/ego_track_ready_1', UInt64MultiArray, queue_size=1)
    obstacle_map = rospy.Publisher('/ego_planner_1/grid_map/occupancy_inflate',
                                   PointCloud2, queue_size=1)

    deadline = time.monotonic() + 10
    while not poses.get_num_connections():
        if time.monotonic() > deadline:
            raise RuntimeError('Controller did not subscribe to fake MAVROS')
        time.sleep(.02)

    def feed(seconds, point, yaw=180., track=False, z=None, trajectory=10,
             acknowledgement=None, command_age=0., command_offset=(.2, 0.), blocked=()):
        # This regression isolates return/yaw/ACK behavior; altitude sequencing
        # is covered separately by smoke_altitude_handoff.py. Feed settled actual
        # altitude and speed for whichever mode is being exercised.
        if z is None:
            z = 2.7 if track else 4.5
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            now = rospy.Time.now()
            pose = PoseStamped()
            pose.header.stamp = now
            pose.pose.position.x = (point[0] - origin[0]) / scale
            pose.pose.position.y = (point[1] - origin[1]) / scale
            pose.pose.position.z = z
            pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
            pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
            poses.publish(pose)
            local_velocity = TwistStamped()
            local_velocity.header.stamp = now
            velocities.publish(local_velocity)
            states.publish(State(connected=True, armed=True, mode='OFFBOARD'))
            modes.publish(Int8(data=1 if track else 0))
            obstacle_map.publish(create_cloud_xyz32(
                Header(stamp=now, frame_id='world'), [(x, y, 4.5) for x, y in blocked]))
            if track:
                target = TrackingTarget()
                target.state.header.stamp = now
                target.state.header.frame_id = 'world'
                target.state.child_frame_id = 'return-handoff:person'
                target.state.pose.pose.position.x = point[0] + 25.
                target.state.pose.pose.position.y = point[1]
                target.state.pose.pose.position.z = 1.
                target.measurement_stamp = target.depth_stamp = now
                target.assignment = 'return-handoff:person'
                targets.publish(target)
            if acknowledgement is not None:
                ready.publish(UInt64MultiArray(data=acknowledgement))
            command = PositionCommand()
            command.header.stamp = now - rospy.Duration(command_age)
            command.trajectory_id = 70 if track else trajectory
            if track and track_goals:
                request = track_goals[-1].header.stamp.to_nsec()
                track_ready.publish(UInt64MultiArray(data=[request, 70, request]))
            command.position.x = point[0] + command_offset[0]
            command.position.y = point[1] + command_offset[1]
            command.position.z = 2.7 if track else 4.5
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

    def held_at(anchor, description):
        expected = tuple((anchor[i] - origin[i]) / scale for i in (0, 1))
        samples = raw[-8:]
        check(len(samples) == 8 and all(
            not (message.type_mask & (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY))
            and math.hypot(message.position.x - expected[0],
                           message.position.y - expected[1]) < 1e-6
            and abs(message.position.z - 4.5) < 1e-6
            and all(abs(getattr(message.velocity, axis)) < 1e-9 for axis in ('x', 'y', 'z'))
            and all(abs(getattr(message.acceleration_or_force, axis)) < 1e-9
                    for axis in ('x', 'y', 'z'))
            for message in samples), description)

    def return_yaw(point):
        # This location projects nearest to the first eastbound rectangle segment.
        return math.degrees(math.atan2(-3. - point[1], 26. - point[0]))

    feed(.4, origin, yaw=0., z=1.5)
    feed(.5, origin, yaw=0.)
    check(bool(ordinary_goals), 'initial climb enters patrol on ordinary EGO goal')
    anchor = (6., 7.)
    count = len(track_goals)
    feed(1.1, anchor, track=True)
    check(len(track_goals) > count and abs(raw[-1].position.z - 2.7) < 1e-6,
          'TRACK takes control before return')
    feed(.9, anchor)
    held_at(anchor, 'return yaw hold uses current XY despite LPF retaining the takeoff point')
    count = len(ordinary_goals)
    feed(2.3, anchor)  # Reference yaw has time to finish; measured yaw stays frozen.
    check(not return_goals and len(ordinary_goals) == count,
          'frozen measured yaw cannot start a return trajectory after reference yaw finishes')
    held_at(anchor, 'continuously fresh old EGO trajectory is ignored throughout the turn')
    drift = (anchor[0] + .2, anchor[1] + .2)
    feed(.3, drift)
    held_at(anchor, 'small odometry drift does not move the latched hold point')

    count = len(track_goals)
    feed(1.1, drift, track=True)
    check(len(track_goals) > count and abs(raw[-1].position.z - 2.7) < 1e-6,
          'new TRACK assignment interrupts return yaw alignment')
    anchor = (9., 6.)
    feed(.9, anchor)
    held_at(anchor, 'another release latches its own current XY')
    yaw = return_yaw(anchor)
    feed(.25, anchor, yaw=yaw + 21.)
    check(not return_goals, 'measured yaw error above the 20 degree threshold keeps holding')
    count = len(ordinary_goals)
    feed(.3, anchor, yaw=yaw)
    check(len(return_goals) == 1 and len(ordinary_goals) == count,
          'measured yaw alignment publishes one dedicated return request')
    held_at(anchor, 'alignment alone does not permit old PositionCommands to take over')
    request = return_goals[-1].header.stamp.to_nsec()
    trajectory_start = rospy.Time.now().to_nsec()
    feed(.25, anchor, yaw=yaw, acknowledgement=[request + 1, 20, trajectory_start])
    held_at(anchor, 'ready ACK for another request is rejected')
    feed(.25, anchor, yaw=yaw, acknowledgement=[request, 20, trajectory_start])
    held_at(anchor, 'matching ready ACK still rejects commands from the old trajectory ID')
    feed(.25, anchor, yaw=yaw, acknowledgement=[request, 20, trajectory_start], trajectory=99)
    held_at(anchor, 'old commands with a larger ID cannot impersonate a restarted planner')
    feed(.25, anchor, yaw=yaw, acknowledgement=[request, 20, trajectory_start],
         trajectory=20, command_age=3.)
    held_at(anchor, 'recently delivered commands with old source timestamps cannot take over')

    count = len(track_goals)
    feed(1.1, anchor, yaw=yaw, track=True,
         acknowledgement=[request, 20, trajectory_start])
    check(len(track_goals) > count and abs(raw[-1].position.z - 2.7) < 1e-6,
          'TRACK interrupts return while waiting for the new trajectory')
    anchor = (11., 5.)
    old_ack = [request, 20, trajectory_start]
    feed(.9, anchor, acknowledgement=old_ack, trajectory=20)
    held_at(anchor, 'releasing TRACK discards the interrupted return handoff')
    check(len(return_goals) == 1, 'second return request also waits for measured yaw')
    yaw = return_yaw(anchor)
    feed(.3, anchor, yaw=yaw, acknowledgement=old_ack, trajectory=20)
    check(len(return_goals) == 2 and return_goals[-1].header.stamp.to_nsec() != request,
          'second aligned return has a new request stamp')
    held_at(anchor, 'previous return ACK and commands cannot acknowledge the new request')
    request = return_goals[-1].header.stamp.to_nsec()
    trajectory_start = rospy.Time.now().to_nsec()
    feed(.25, anchor, yaw=yaw, acknowledgement=[request, 30, trajectory_start], trajectory=20)
    held_at(anchor, 'new return ACK waits for its matching new trajectory command')
    feed(.3, anchor, yaw=yaw, acknowledgement=[request, 30, trajectory_start],
         trajectory=30, command_offset=(4., 0.))
    held_at(anchor, 'acknowledged trajectory reference more than 1m away cannot pull aircraft')
    feed(1.8, anchor, yaw=yaw, acknowledgement=[request, 30, trajectory_start],
         trajectory=30, command_offset=(4., 0.))
    check(len(return_goals) == 3 and return_goals[-1].header.stamp.to_nsec() != request,
          'distant reference retries with a new request stamp from current odometry')
    held_at(anchor, 'waiting for the retry holds its current anchor')
    request = return_goals[-1].header.stamp.to_nsec()
    trajectory_start = rospy.Time.now().to_nsec()
    feed(.3, anchor, yaw=yaw, acknowledgement=[request, 40, trajectory_start], trajectory=40)
    check(all(abs(message.velocity.x * scale - .8) < 1e-6 and
              abs(message.velocity.y * scale - .3) < 1e-6 for message in raw[-8:]),
          'matching fresh return ACK and trajectory command resume EGO PVA')
    feed(.2, anchor, yaw=yaw, trajectory=41)
    check(all(abs(message.velocity.x * scale - .8) < 1e-6 for message in raw[-8:]),
          'normal later EGO replanning can advance the trajectory ID after handoff')
    anchor = (12., 4.)
    feed(1.1, anchor, track=True, trajectory=41)
    feed(.9, anchor, trajectory=41)
    yaw = return_yaw(anchor)
    feed(.3, anchor, yaw=yaw, trajectory=41)
    check(len(return_goals) == 4, 'another release enters return trajectory wait')
    request = return_goals[-1].header.stamp.to_nsec()
    count = len(ordinary_goals)
    blocked = ((26., -3.),)
    feed(.3, anchor, yaw=yaw, trajectory=50,
         acknowledgement=[request, 50, rospy.Time.now().to_nsec()], blocked=blocked)
    check(len(return_goals) == 4 and len(ordinary_goals) == count,
          'blocked return waypoint selects its successor without publishing a goal before turning')
    held_at(anchor, 'blocked waypoint invalidates its ACK and preserves the same hold anchor')
    next_yaw = math.degrees(math.atan2(37. - anchor[1], 26. - anchor[0]))
    feed(.3, anchor, yaw=next_yaw, trajectory=50, blocked=blocked)
    check(len(return_goals) == 5 and
          math.dist((return_goals[-1].pose.position.x, return_goals[-1].pose.position.y),
                    (26., 37.)) < 1e-6,
          're-aligned blocked-waypoint successor uses a fresh dedicated return goal')
    held_at(anchor, 'old blocked-waypoint trajectory cannot take over the successor')
    request = return_goals[-1].header.stamp.to_nsec()
    feed(.3, anchor, yaw=next_yaw, trajectory=60,
         acknowledgement=[request, 60, rospy.Time.now().to_nsec()], blocked=blocked)
    check(all(abs(message.velocity.x * scale - .8) < 1e-6 for message in raw[-8:]),
          'successor waypoint resumes only after its own new trajectory acknowledgement')
    check(all(math.isfinite(value) for message in raw for value in
              (message.position.x, message.position.y, message.position.z,
               message.velocity.x, message.velocity.y, message.yaw)),
          'all emitted flight targets remain finite')
    print('Controller return regression: %d checks passed' % checks, flush=True)


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

    rospy.init_node('return_handoff_planner_checks', disable_signals=True)
    acknowledgements, trajectories, commands, planner_targets = [], [], [], []
    rospy.Subscriber('/ego_return_ready_1', UInt64MultiArray, acknowledgements.append, queue_size=100)
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
    odometry = rospy.Publisher('/return_handoff_real/odom_world_relay', Odometry, queue_size=1)
    clouds = rospy.Publisher('/return_handoff_real/obstacle_cloud_world', PointCloud2, queue_size=1)
    ordinary_goal = rospy.Publisher('/ego_goal_1', PoseStamped, queue_size=1)
    return_goal = rospy.Publisher('/ego_return_goal_1', PoseStamped, queue_size=1)
    goal_velocity = rospy.Publisher('/ego_goal_vel_1', TwistStamped, queue_size=1, latch=True)

    deadline = time.monotonic() + 10
    while not odometry.get_num_connections() or not return_goal.get_num_connections():
        if time.monotonic() > deadline:
            raise RuntimeError('Real EGO did not subscribe to return test topics')
        time.sleep(.02)

    def feed(seconds, point):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            now = rospy.Time.now()
            message = Odometry()
            message.header = Header(stamp=now, frame_id='world')
            message.pose.pose.position.x, message.pose.pose.position.y = point
            message.pose.pose.position.z = 4.5
            message.pose.pose.orientation.w = 1.
            odometry.publish(message)
            clouds.publish(create_cloud_xyz32(message.header, [(18., 18., 4.5)]))
            time.sleep(.02)

    def goal_message(point):
        message = PoseStamped()
        message.header = Header(stamp=rospy.Time.now(), frame_id='world')
        message.pose.position.x, message.pose.position.y = point
        message.pose.position.z = 4.5
        message.pose.orientation.w = 1.
        return message

    def wait_for(condition, point, message, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            feed(.1, point)
            if condition():
                return
        raise AssertionError(message)

    def check(condition, message):
        if not condition:
            raise AssertionError(message)
        print('PASS:', message, flush=True)

    feed(.5, (0., 0.))
    ordinary_goal.publish(goal_message((12., 0.)))
    wait_for(lambda: len(trajectories) > 0 and len(commands) > 5, (0., 0.),
             'Real EGO failed to plan the ordinary goal')
    check(not acknowledgements, 'ordinary patrol goals do not acknowledge a return request')
    old_id = trajectories[-1].traj_id
    relocated = (-8., 6.)
    feed(.3, relocated)
    # A TRACK target's retained velocity must not become the return endpoint speed.
    retained_velocity = TwistStamped()
    retained_velocity.header.stamp = rospy.Time.now()
    retained_velocity.twist.linear.x, retained_velocity.twist.linear.y = (.7, -.2)
    goal_velocity.publish(retained_velocity)
    feed(.15, relocated)
    request = goal_message((-2., 6.))
    return_goal.publish(request)
    stamp = request.header.stamp.to_nsec()
    wait_for(lambda: any(len(ack.data) >= 3 and ack.data[0] == stamp
                         for ack in acknowledgements), relocated,
             'Dedicated return goal never received a real EGO ACK')
    ack = next(ack for ack in acknowledgements if len(ack.data) >= 3 and ack.data[0] == stamp)
    wait_for(lambda: any(trajectory.traj_id == ack.data[1] for trajectory in trajectories),
             relocated, 'ACK trajectory was not published')
    trajectory = next(t for t in trajectories if t.traj_id == ack.data[1])
    check(trajectory.traj_id != old_id and trajectory.start_time.to_nsec() == ack.data[2],
          'return ACK identifies the newly published B-spline ID and start time')
    # The production planner uses an unclamped uniform cubic B-spline.
    start = ((trajectory.pos_pts[0].x + 4 * trajectory.pos_pts[1].x + trajectory.pos_pts[2].x) / 6,
             (trajectory.pos_pts[0].y + 4 * trajectory.pos_pts[1].y + trajectory.pos_pts[2].y) / 6)
    check(math.dist(start, relocated) < .1,
          'new return spline starts at current odometry instead of the old moving reference')
    def endpoint(spline):
        return ((spline.pos_pts[-3].x + 4 * spline.pos_pts[-2].x + spline.pos_pts[-1].x) / 6,
                (spline.pos_pts[-3].y + 4 * spline.pos_pts[-2].y + spline.pos_pts[-1].y) / 6)

    def endpoint_velocity(spline):
        interval = spline.knots[4] - spline.knots[3]
        return ((spline.pos_pts[-1].x - spline.pos_pts[-3].x) / (2 * interval),
                (spline.pos_pts[-1].y - spline.pos_pts[-3].y) / (2 * interval))

    return_velocity = endpoint_velocity(trajectory)
    print('Return terminal diagnostics: end=%r target=(-2,6) velocity=%r retained=(.7,-.2)' %
          (endpoint(trajectory), return_velocity), flush=True)
    # Rebound optimization has a free endpoint and only a position penalty;
    # the final optimized derivative does not strictly equal the requested speed.
    wait_for(lambda: any(math.dist(record[3:5], (-2., 6.)) < .05 and record[2] == 0.
                         for record in planner_targets), relocated,
             'Return planning did not request zero target speed')
    check(abs(return_velocity[1]) < .05 and math.dist(return_velocity, (.7, -.2)) > .1,
          'return requests zero target speed and its spline does not inherit target lateral velocity')
    wait_for(lambda: any(command.trajectory_id == trajectory.traj_id for command in commands),
             relocated, 'traj_server never published the acknowledged return command')
    check(True, 'real traj_server emits commands with the acknowledged return trajectory ID')
    # A delivery retry must not regenerate trajectories or alter the ACK identity.
    count = len([a for a in acknowledgements if a.data[0] == stamp])
    return_goal.publish(request)
    feed(.2, relocated)
    matching = [a for a in acknowledgements if len(a.data) >= 3 and a.data[0] == stamp]
    check(len(matching) >= count and all(a.data[:3] == ack.data[:3] for a in matching),
          'retrying the same return request preserves its trajectory acknowledgement')
    next_goal = goal_message((-8., 12.))
    ordinary_goal.publish(next_goal)
    wait_for(lambda: any(t.traj_id > trajectory.traj_id for t in trajectories) and
             any(math.dist(record[3:5], (-8., 12.)) < .05 and
                 abs(record[2] - math.hypot(.7, -.2)) < .06 for record in planner_targets),
             relocated, 'New ordinary goal did not replace the return destination')
    next_trajectory = next(t for t in trajectories if t.traj_id > trajectory.traj_id)
    print('Ordinary terminal diagnostics: end=%r target=(-8,12) velocity=%r requested=(.7,-.2)' %
          (endpoint(next_trajectory), endpoint_velocity(next_trajectory)), flush=True)
    check(True, 'a newer ordinary tracking goal restores the requested target speed')
    count = len(acknowledgements)
    target_count = len(planner_targets)
    return_goal.publish(request)  # Simulate a retained old request on reconnection.
    feed(.4, relocated)
    check(len(acknowledgements) == count and all(
        math.dist(record[3:5], (-8., 12.)) < .05 for record in planner_targets[target_count:]),
        'an older retained return request cannot overwrite a newer ordinary tracking goal')
    print('Real EGO return regression: 8 checks passed', flush=True)


if __name__ == '__main__':
    if '--worker' in sys.argv:
        controller_worker() if sys.argv[-1] == 'controller' else planner_worker()
    else:
        sys.exit(run_isolated())
