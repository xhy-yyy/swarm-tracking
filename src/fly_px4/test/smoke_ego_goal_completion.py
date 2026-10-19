#!/usr/bin/env python3
"""Exercise real EGO/traj_server with estimated odometry on an isolated master.

Reproduce a trajectory ending before the vehicle reaches its requested waypoint.
No Gazebo, PX4, arming services or flight-control nodes are started.
"""

import os
from pathlib import Path
import re
import shlex
import socket
import subprocess
import sys
import tempfile
import time
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
WORKSPACE = PACKAGE.parents[1]


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='ego-goal-smoke-') as tmp:
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                   ROS_IP='127.0.0.1', ROS_HOME=tmp)
        env.pop('ROS_HOSTNAME', None)
        os.environ.update(env)
        handles, processes = [], []

        def start(command, name):
            log = open(Path(tmp) / (name+'.log'), 'w')
            handles.append(log)
            process = subprocess.Popen(command, env=env, stdout=log,
                                       stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        master = start(['roscore', '-p', str(port)], 'master')
        try:
            import rosgraph
            deadline = time.monotonic()+15
            while True:
                try:
                    rosgraph.Master('/ego_goal_smoke').getPid()
                    break
                except Exception:
                    if master.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Isolated ROS master failed to start')
                    time.sleep(.1)

            import roslaunch.config
            import roslaunch.xmlloader
            import rospy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from quadrotor_msgs.msg import PositionCommand
            from rosgraph_msgs.msg import Log
            from sensor_msgs.msg import PointCloud2
            from sensor_msgs.point_cloud2 import create_cloud_xyz32
            from std_msgs.msg import Float32MultiArray, Header
            from traj_utils.msg import Bspline

            rospy.init_node('ego_goal_smoke', anonymous=True, disable_signals=True)
            config = roslaunch.config.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader().load(
                str(PACKAGE / 'launch/swarm_relay_ego.launch'), config, verbose=False)
            cruise_z = config.params['/ego_planner_3/manager/plane_z'].value
            track_z = yaml.safe_load((PACKAGE / 'config/patrol_routes.yaml').read_text())['patrol']['defaults']['track_z']
            names = ('ego_planner_3', 'traj_server_3')
            for key, param in config.params.items():
                if any(key.startswith('/'+name+'/') for name in names):
                    rospy.set_param(key, param.value)

            # Keep all production avoidance settings; provide an empty sensor map.
            odom = rospy.Publisher('/typhoon_h480_2/odom_world_relay', Odometry, queue_size=1)
            cloud = rospy.Publisher('/typhoon_h480_2/obstacle_cloud_world', PointCloud2, queue_size=1)
            goal = rospy.Publisher('/ego_goal_3', PoseStamped, queue_size=1, latch=True)
            speed = rospy.Publisher('/ego_speed_3', Float32MultiArray, queue_size=1, latch=True)
            trajectories, commands, states = [], [], []
            rospy.Subscriber('/drone_2_planning/bspline', Bspline,
                             lambda m: trajectories.append(m))
            rospy.Subscriber('/ego/pos_cmd_3', PositionCommand,
                             lambda m: commands.append(m))

            def log_cb(message):
                if message.name == '/ego_planner_3':
                    match = re.search(r'\b([A-Z_]+) -> ([A-Z_]+)\b', message.msg)
                    if match:
                        states.append(match.group(2))
            rospy.Subscriber('/rosout', Log, log_cb)

            for node in config.nodes:
                if node.name in names:
                    command = shlex.split(node.launch_prefix or '')
                    command += [str(WORKSPACE / 'devel/lib/ego_planner' / node.type),
                                '__name:='+node.name]
                    command += [source+':='+target for source, target in node.remap_args]
                    start(command, node.name)

            deadline = time.monotonic()+10
            while odom.get_num_connections() == 0 or goal.get_num_connections() == 0:
                if any(p.poll() is not None for p in processes) or time.monotonic() > deadline:
                    raise RuntimeError('Planner failed to connect to test inputs')
                time.sleep(.02)

            position = (-13.9, 4.3, cruise_z)
            velocity = (0., 0., 0.)

            def feed(duration, follow=False, actual_z=None):
                nonlocal position, velocity
                deadline = time.monotonic()+duration
                while time.monotonic() < deadline:
                    if follow and commands:
                        command = commands[-1]
                        position = (command.position.x, command.position.y, command.position.z)
                        velocity = (command.velocity.x, command.velocity.y, command.velocity.z)
                    if actual_z is not None:
                        position = (position[0], position[1], actual_z)
                        velocity = (velocity[0], velocity[1], 0.)
                    stamp = rospy.Time.now()
                    message = Odometry()
                    message.header = Header(stamp=stamp, frame_id='world')
                    message.pose.pose.orientation.w = 1.
                    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
                    message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = velocity
                    odom.publish(message)
                    cloud.publish(create_cloud_xyz32(message.header, []))
                    time.sleep(.02)

            feed(.3)
            speed.publish(Float32MultiArray(data=[8., 6.]))
            target = PoseStamped()
            target.header = Header(stamp=rospy.Time.now(), frame_id='world')
            target.pose.orientation.w = 1.
            target.pose.position.x, target.pose.position.y, target.pose.position.z = (-13.5, 12., cruise_z)
            goal.publish(target)  # Send once; recovery must not depend on resending.

            # Keep odometry behind the waypoint while the first trajectory expires.
            # traj_server still publishes fresh endpoint commands in this situation.
            feed(3.)
            assert len(commands) > 20, 'traj_server did not publish fresh commands'
            assert len(trajectories) >= 2, (
                'Trajectory expired with vehicle still away from goal, but EGO stopped planning',
                len(trajectories), states[-8:])
            assert 'WAIT_TARGET' not in states[1:], (
                'Planner declared arrival from planned time instead of odometry', states)
            print('PASS: finished trajectory with distant odometry triggers another plan despite fresh commands')

            # Now let estimated odometry follow the real generated commands.
            deadline = time.monotonic()+12
            while time.monotonic() < deadline:
                feed(.1, follow=True)
                distance = sum((a-b)**2 for a,b in zip(position, (-13.5,12.,cruise_z)))**.5
                if distance <= 1.0 and states[-1:] == ['WAIT_TARGET']:
                    break
            else:
                raise AssertionError(('Planner did not finish at the actual goal', position, states[-8:]))
            count = len(trajectories)
            feed(1.2, follow=True)
            assert len(trajectories) == count, 'Reached goal should remain idle'
            print('PASS: arrival within 1m completes the goal and stops further replanning')

            target.header.stamp = rospy.Time.now()
            target.pose.position.x = -8.
            goal.publish(target)
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                feed(.1, follow=True)
                distance = sum((a-b)**2 for a,b in zip(position, (-8.,12.,cruise_z)))**.5
                if len(trajectories) > count and distance <= 1.0 and states[-1:] == ['WAIT_TARGET']:
                    break
            else:
                raise AssertionError(('Next waypoint did not complete', position, states[-8:]))
            print('PASS: next waypoint is accepted and reached after the recovered first goal')

            # Fly control owns tracking altitude while EGO plans horizontal motion
            # on the cruise plane. Their deliberate Z difference is not an XY miss.
            count = len(trajectories)
            feed(.3, actual_z=track_z)
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = -3.
            goal.publish(target)
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                feed(.1, follow=True, actual_z=track_z)
                xy_distance = ((position[0]+3.)**2+(position[1]-12.)**2)**.5
                if len(trajectories) > count and xy_distance <= 1.0 and states[-1:] == ['WAIT_TARGET']:
                    break
            else:
                raise AssertionError(('Plane planner confused tracking altitude with a missed horizontal goal',
                                      position, states[-8:]))
            count = len(trajectories)
            feed(1.2, follow=True, actual_z=track_z)
            assert len(trajectories) == count, 'Tracking height must not cause endless goal replanning'
            assert position[2] == track_z
            print('PASS: planar goals finish at the actual XY goal while tracking altitude stays %.1fm' % track_z)
            return 0
        except Exception:
            for handle in handles:
                handle.flush()
            for name in ('ego_planner_3', 'traj_server_3'):
                path = Path(tmp) / (name+'.log')
                if path.exists():
                    print(path.read_text(errors='replace')[-2500:], file=sys.stderr)
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            for handle in handles:
                handle.close()


if __name__ == '__main__':
    sys.exit(main())
