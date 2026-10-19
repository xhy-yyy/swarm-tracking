#!/usr/bin/env python3
"""Check actual six-drone coordinate relays/controllers on an isolated master.

Uses fake MAVROS inputs and fake EGO commands. No Gazebo, PX4, actual EGO,
arming services, or connection to an existing ROS master.
"""

import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml

PACKAGE = Path(__file__).resolve().parent.parent
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from start_positions import load_start_positions


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='start-positions-smoke-') as tmp:
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                   ROS_IP='127.0.0.1', ROS_HOME=tmp)
        env.pop('ROS_HOSTNAME', None)
        os.environ.update(env)
        processes, handles = [], []

        def start(command, name):
            log = open(Path(tmp) / (name + '.log'), 'w')
            handles.append(log)
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        master = start(['roscore', '-p', str(port)], 'master')
        try:
            import rosgraph
            deadline = time.monotonic() + 15
            while True:
                try:
                    rosgraph.Master('/start_positions_smoke').getPid()
                    break
                except Exception:
                    if master.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Isolated master did not start')
                    time.sleep(.1)

            import roslaunch.config
            import roslaunch.xmlloader
            import rospy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import PositionTarget, State
            from nav_msgs.msg import Odometry
            from quadrotor_msgs.msg import PositionCommand

            rospy.init_node('start_positions_smoke', anonymous=True, disable_signals=True)
            # Exercise a 14m translation plus an independent edit of drone 6.
            data = {key: {'x': xy[0] + 14., 'y': xy[1]}
                    for key, xy in load_start_positions().items()}
            data['drone_6']['y'] += 2.25
            data['patrol_maps_dir'] = 'patrol_maps_2'
            starts_file = Path(tmp) / 'starts.yaml'
            starts_file.write_text(yaml.safe_dump(data))
            config = roslaunch.config.ROSLaunchConfig()
            loader = roslaunch.xmlloader.XmlLoader()
            override = ['start_positions_file:=' + str(starts_file)]
            loader.load(str(PACKAGE / 'launch/swarm_fly_px4_1.launch'), config,
                        argv=override + ['map:=base3'], verbose=False)
            loader.load(str(PACKAGE / 'launch/swarm_relay_ego.launch'), config,
                        argv=override, verbose=False)
            for key, param in config.params.items():
                if key.startswith(('/fly_px4_1_', '/odom_relay_', '/swarm_coordinates/')):
                    rospy.set_param(key, param.value)
            scale = rospy.get_param('/swarm_coordinates/local_xy_scale')
            cruise_z = config.params['/fly_px4_1_1/patrol/routes/drone_1/z'].value
            rospy.set_param('/use_sim_time', False)

            publishers, raw, world_odom, world_pose, goals, subscribers = {}, {}, {}, {}, {}, []
            for i in range(1, 7):
                ns = '/typhoon_h480_%d' % (i - 1)
                publishers[i] = (
                    rospy.Publisher(ns + '/mavros/local_position/pose', PoseStamped, queue_size=1),
                    rospy.Publisher(ns + '/mavros/local_position/odom', Odometry, queue_size=1),
                    rospy.Publisher(ns + '/mavros/state', State, queue_size=1),
                    rospy.Publisher('/ego/pos_cmd_%d' % i, PositionCommand, queue_size=1))
                subscribers.extend([
                    rospy.Subscriber(ns + '/mavros/setpoint_raw/local', PositionTarget,
                                     lambda m, idx=i: raw.__setitem__(idx, m)),
                    rospy.Subscriber(ns + '/odom_world_relay', Odometry,
                                     lambda m, idx=i: world_odom.__setitem__(idx, m)),
                    rospy.Subscriber(ns + '/pose_relay', PoseStamped,
                                     lambda m, idx=i: world_pose.__setitem__(idx, m)),
                    rospy.Subscriber('/ego_goal_%d' % i, PoseStamped,
                                     lambda m, idx=i: goals.__setitem__(idx, m))])
                start([str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'),
                       '__name:=fly_px4_1_%d' % i], 'fly_%d' % i)
                start([sys.executable, str(PACKAGE / 'scripts/odom_world_relay.py'),
                       '__name:=odom_relay_%d' % i], 'relay_%d' % i)
            deadline = time.monotonic() + 10
            while not all(pub.get_num_connections() for pubs in publishers.values() for pub in pubs):
                if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                    raise RuntimeError('Controllers/relays did not connect')
                time.sleep(.02)

            def feed(z, duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    for pose_pub, odom_pub, state_pub, ego_pub in publishers.values():
                        stamp = rospy.Time.now()
                        pose = PoseStamped()
                        pose.header.stamp = stamp
                        pose.pose.orientation.w = 1.
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (2., -1., z)
                        pose_pub.publish(pose)
                        odom = Odometry()
                        odom.header = pose.header
                        odom.pose.pose = pose.pose
                        odom_pub.publish(odom)
                        state = State(connected=True, armed=True, mode='OFFBOARD')
                        state.header.stamp = stamp
                        state_pub.publish(state)
                        command = PositionCommand()
                        command.header.stamp = stamp
                        command.position.x, command.position.y, command.position.z = (25., 12., cruise_z)
                        ego_pub.publish(command)
                    time.sleep(.02)

            feed(1.5, .4)
            feed(cruise_z, 6.)  # Allow command filtering and all six handoffs to settle.
            # The shared altitude reference can take longer than XY filtering.
            deadline = time.monotonic() + 20.
            while any(i not in raw or abs(raw[i].position.z - cruise_z) >= .02
                      for i in range(1, 7)):
                if time.monotonic() >= deadline:
                    raise RuntimeError('Altitude references did not settle')
                feed(cruise_z, .2)
            for i in range(1, 7):
                xy = data['drone_%d' % i]
                for position in (world_odom[i].pose.pose.position, world_pose[i].pose.position):
                    assert abs(position.x - (xy['x'] + 2. * scale)) < 1e-6
                    assert abs(position.y - (xy['y'] - scale)) < 1e-6
                expected = ((25. - xy['x']) / scale, (12. - xy['y']) / scale)
                actual = (raw[i].position.x, raw[i].position.y)
                assert all(abs(a - b) < .02 for a, b in zip(actual, expected)), (i, actual, expected)
                assert abs(raw[i].position.z-cruise_z) < .02, (i, raw[i].position.z, cruise_z)
                first = config.params['/fly_px4_1_%d/patrol/routes/drone_%d/entry_waypoints' % (i, i)].value[0]
                assert abs(goals[i].pose.position.x - first['x']) < 1e-6
                assert abs(goals[i].pose.position.y - first['y']) < 1e-6
            print('PASS: all six relays use the edited starts; position, camera pose and scale agree')
            print('PASS: all six controllers subtract the same starts and keep the original road goals')

            for process in processes[1:]:
                stop(process)
            # A bad coordinate belonging to another drone must fail before flight starts.
            for bad_value in (True, float('inf')):
                bad = {key: dict(xy) if isinstance(xy, dict) else xy
                       for key, xy in data.items()}
                bad['drone_6']['x'] = bad_value
                for name in ('invalid_fly', 'invalid_relay'):
                    rospy.set_param('/' + name + '/start_positions', bad)
                    rospy.set_param('/' + name + '/start_position_key', 'drone_1')
                for command, name in (
                    ([str(WORKSPACE / 'devel/lib/fly_px4/fly_px4_1'), '__name:=invalid_fly'], 'invalid_fly'),
                    ([sys.executable, str(PACKAGE / 'scripts/odom_world_relay.py'), '__name:=invalid_relay'], 'invalid_relay')):
                    process = start(command, name)
                    assert process.wait(timeout=5) != 0, 'Invalid starts must refuse startup'
            print('PASS: controllers and relays both refuse invalid coordinates before accepting flight inputs')
            return 0
        except Exception:
            for handle in handles:
                handle.flush()
            for name in ('fly_1', 'fly_6', 'relay_1', 'relay_6'):
                path = Path(tmp) / (name + '.log')
                if path.exists():
                    print(name + ':\n' + path.read_text(errors='replace')[-2200:], file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                stop(process)
            for handle in handles:
                handle.close()


if __name__ == '__main__':
    sys.exit(main())
