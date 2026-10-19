#!/usr/bin/env python3
"""Run real fly_px4_1 with fake MAVROS/EGO on a private ROS master.
Check waypoint forwarding, one-time entry, loop closure, TRACK takeover and return
for seven original and three competition maps; no Gazebo or PX4."""

import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

import yaml

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / 'scripts'))
from check_patrol_maps import read_config
from patrol_world_geometry import MAPS


def stop(process):
    if process is not None and process.poll() is None:
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
    starts = yaml.safe_load((PACKAGE / 'config/start_positions.yaml').read_text())
    origin = tuple(starts['drone_4'][axis] for axis in ('x', 'y'))
    scale = yaml.safe_load((PACKAGE / 'config/world_frame.yaml').read_text())[
        'swarm_coordinates']['local_xy_scale']
    with tempfile.TemporaryDirectory(prefix='patrol-yaw-ros-') as tmp:
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                   ROS_IP='127.0.0.1', ROS_HOME=tmp)
        env.pop('ROS_HOSTNAME', None)
        os.environ.update(env)
        with open(Path(tmp) / 'master.log', 'w') as master_log, \
                open(Path(tmp) / 'controller.log', 'w') as node_log:
            master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                      stdout=master_log, stderr=subprocess.STDOUT)
            node = None
            try:
                import rosgraph
                import rosparam
                import rospy
                from geometry_msgs.msg import PoseStamped
                from mavros_msgs.msg import PositionTarget, State
                from quadrotor_msgs.msg import PositionCommand
                from std_msgs.msg import Int8
                from track_msgs.msg import TrackingTarget
                deadline = time.monotonic() + 15
                while True:
                    try:
                        rosgraph.Master('/patrol_yaw_checks').getPid()
                        break
                    except Exception:
                        if time.monotonic() > deadline:
                            raise RuntimeError('Isolated ROS master failed to start')
                        time.sleep(.1)
                rospy.init_node('patrol_yaw_checks', anonymous=True, disable_signals=True)
                goals, raw = [], []

                def on_goal(message):
                    p = message.pose.position
                    point = (p.x, p.y)
                    if not goals or goals[-1] != point:
                        goals.append(point)

                rospy.Subscriber('/ego_goal_4', PoseStamped, on_goal)
                rospy.Subscriber('/patrol_yaw_fake/mavros/setpoint_raw/local',
                                 PositionTarget, lambda message: raw.append(message))
                poses = rospy.Publisher('/patrol_yaw_fake/mavros/local_position/pose',
                                        PoseStamped, queue_size=1)
                states = rospy.Publisher('/patrol_yaw_fake/mavros/state', State, queue_size=1)
                modes = rospy.Publisher('/tracking/mode_4', Int8, queue_size=1)
                targets = rospy.Publisher('/tracking/state_4', TrackingTarget, queue_size=1)
                ego = rospy.Publisher('/ego/pos_cmd_4', PositionCommand, queue_size=1)

                def feed(xy, yaw, seconds=.2, z=4.5, track=False):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        stamp = rospy.Time.now()
                        p = PoseStamped()
                        p.header.stamp = stamp
                        p.pose.position.x = (xy[0]-origin[0])/scale
                        p.pose.position.y = (xy[1]-origin[1])/scale
                        p.pose.position.z = z
                        p.pose.orientation.z = math.sin(math.radians(yaw)/2)
                        p.pose.orientation.w = math.cos(math.radians(yaw)/2)
                        poses.publish(p)
                        state = State(connected=True, armed=True, mode='OFFBOARD')
                        state.header.stamp = stamp
                        states.publish(state)
                        modes.publish(Int8(data=1 if track else 0))
                        # Moving EGO commands must continue after a corner is reached.
                        command = PositionCommand()
                        command.header.stamp = stamp
                        command.position.x, command.position.y, command.position.z = (80., 40., 4.5)
                        command.velocity.x, command.velocity.y = (6., -1.)
                        command.acceleration.x = 1.
                        ego.publish(command)
                        if track:
                            target = TrackingTarget()
                            target.state.header.stamp = stamp
                            target.state.header.frame_id = 'world'
                            target.state.child_frame_id = 'patrol-yaw:person'
                            target.state.pose.pose.position.x = xy[0]+25.
                            target.state.pose.pose.position.y = xy[1]
                            target.state.pose.pose.position.z = 1.
                            target.measurement_stamp = target.depth_stamp = stamp
                            target.assignment = 'patrol-yaw:person'
                            targets.publish(target)
                        time.sleep(.02)

                def moving_ego_forwarded():
                    assert len(raw) >= 5, 'No PX4 commands'
                    for message in raw[-3:]:
                        assert abs(message.velocity.x-6./scale) < 1e-6
                        assert abs(message.velocity.y+1./scale) < 1e-6
                        assert abs(message.acceleration_or_force.x-1./scale) < 1e-6

                maps = list(MAPS)+['competition_base1', 'competition_base2', 'competition_base3']
                for map_name in maps:
                    if rospy.has_param('/patrol_yaw_test'):
                        rospy.delete_param('/patrol_yaw_test')
                    cfg = read_config(map_name)
                    rosparam.upload_params('/patrol_yaw_test/patrol', cfg)
                    rospy.set_param('/patrol_yaw_test/start_positions', starts)
                    rospy.set_param('/patrol_yaw_test/start_position_key', 'drone_4')
                    rospy.set_param('/swarm_coordinates/local_xy_scale', scale)
                    route = cfg['routes']['drone_4']
                    entries = [tuple(w[axis] for axis in ('x', 'y')) for w in route['entry_waypoints']]
                    loop = [tuple(w[axis] for axis in ('x', 'y')) for w in route['waypoints']]
                    area2 = sum(a[0]*b[1]-b[0]*a[1]
                                for a, b in zip(loop, loop[1:]+loop[:1]))
                    assert area2 > 0, (map_name, 'drone_4 must patrol counterclockwise')
                    # The last entry is also the loop's first point; on_goal records
                    # consecutive identical coordinates only once.
                    prefix = entries[:-1] if entries[-1:] == loop[:1] else entries
                    goals.clear()
                    raw.clear()
                    node = subprocess.Popen([
                        str(PACKAGE.parents[1] / 'devel/lib/fly_px4/fly_px4_1'),
                        '__name:=patrol_yaw_test', '_drone_ns:=patrol_yaw_fake', '_drone_id:=4',
                        '_patrol_map:='+map_name, '_track_release_sec:=0.5'],
                        env=env, stdout=node_log, stderr=subprocess.STDOUT)
                    deadline = time.monotonic()+5
                    while not poses.get_num_connections():
                        if node.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError('Controller subscription timeout: '+map_name)
                        time.sleep(.02)
                    feed(origin, 0., z=0., seconds=.1)
                    feed(origin, 0., z=1.5)
                    feed(origin, 0.)
                    assert goals == entries[:1], (map_name, goals)
                    for point in prefix:
                        feed(point, 0.)
                    assert goals == prefix+loop[:1], (map_name, goals)

                    # Hold actual yaw 90 degrees away from every outgoing segment.
                    # A corner must publish its successor immediately, without
                    # waiting for the fake vehicle to turn to the requested yaw.
                    expected = prefix+loop[:1]
                    for point in loop*2:
                        index = loop.index(point)
                        following = loop[(index+1) % len(loop)]
                        yaw = math.degrees(math.atan2(following[1]-point[1], following[0]-point[0]))
                        feed(point, yaw+90.)
                        expected.append(following)
                        assert goals == expected, (map_name, goals, expected)
                        moving_ego_forwarded()
                    assert goals == prefix+loop*2+loop[:1], (map_name, goals)

                    if map_name == 'base3':
                        count = len(goals)
                        feed(loop[-1], 0., seconds=.4, track=True)
                        assert len(goals) > count and goals[-1] not in loop, 'TRACK must take over during patrol'
                        count = len(goals)
                        # 归航目标在机头对准后才发 (对准期间 EGO 轨迹时钟不能先走):
                        # 慢慢转机头, 目标出现时机头必须已对准它。
                        yaw = 0.
                        for _ in range(72):
                            feed(loop[-1], yaw, seconds=.1)
                            if len(goals) > count:
                                break
                            yaw += 10.
                        assert len(goals) > count and goals[-1] in loop, 'Release must resume the loop'
                        want = math.degrees(math.atan2(goals[-1][1]-loop[-1][1], goals[-1][0]-loop[-1][0]))
                        assert abs((want-yaw+180.) % 360.-180.) <= 30., ('Return goal only after yaw alignment', want, yaw)
                        assert goals[-1] not in set(entries)-set(loop), 'Return must skip entry-only waypoints'
                        return_point = goals[-1]
                        # Keep the existing return alignment, then verify that
                        # reaching the loop restores immediate corner handoff.
                        approach_yaw = math.degrees(math.atan2(return_point[1]-loop[-1][1],
                                                              return_point[0]-loop[-1][0]))
                        feed(loop[-1], approach_yaw)
                        following = loop[(loop.index(return_point)+1) % len(loop)]
                        count = len(goals)
                        feed(return_point, approach_yaw)
                        assert len(goals) == count+1 and goals[-1] == following, 'Return arrival must publish the next patrol point'
                        print('PASS: TRACK takeover and return skip one-time entry and resume direct handoff', flush=True)
                    print('PASS '+map_name+': entry once, immediate corners with unaligned yaw, EGO forwarding, two loop closures', flush=True)
                    stop(node)
                    node = None
                    deadline = time.monotonic()+3
                    while poses.get_num_connections():
                        if time.monotonic() > deadline:
                            raise RuntimeError('Previous controller did not disconnect')
                        time.sleep(.02)
                return 0
            except Exception:
                node_log.flush()
                print((Path(tmp) / 'controller.log').read_text(errors='replace')[-3500:], file=sys.stderr)
                raise
            finally:
                stop(node)
                stop(master)


if __name__ == '__main__':
    sys.exit(main())
