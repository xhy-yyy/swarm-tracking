#!/usr/bin/env python3
"""Real fly_px4_1 ROS smoke test with fake MAVROS, on an isolated ROS master.

No Gazebo/PX4 or actual vehicles: verify vertical lift followed by moving climb, one-time approach,
transit/cruise speed switching, two loop closures, and invalid-map refusal.
Run after building: source scripts/env.sh; python3 test/smoke_patrol_maps.py
"""

import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / 'scripts'))


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='patrol-ros-smoke-') as tmp:
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                   ROS_IP='127.0.0.1', ROS_HOME=tmp)
        env.pop('ROS_HOSTNAME', None)
        os.environ.update(env)
        master_log = open(Path(tmp) / 'master.log', 'w')
        node_log = open(Path(tmp) / 'node.log', 'w')
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=master_log, stderr=subprocess.STDOUT)
        node = None
        try:
            import rosgraph
            deadline = time.monotonic() + 15
            while True:
                try:
                    rosgraph.Master('/patrol_smoke').getPid()
                    break
                except Exception:
                    if time.monotonic() > deadline:
                        raise RuntimeError('Isolated ROS master did not start')
                    time.sleep(.1)
            import rosparam
            import rospy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import PositionTarget, State
            from mavros_msgs.srv import ParamSet, ParamSetResponse
            from std_msgs.msg import Float32MultiArray, Int8
            from track_msgs.msg import TrackingTarget
            from check_patrol_maps import read_config
            rospy.init_node('patrol_map_smoke', anonymous=True, disable_signals=True)
            cfg = read_config('base3')
            cruise_z = cfg['routes']['drone_3']['z']
            track_z = cfg['defaults']['track_z']
            cfg['routes']['drone_3']['start_delay'] = 1.5
            rosparam.upload_params('/patrol_map_test/patrol', cfg)
            rospy.set_param('/swarm_coordinates/local_xy_scale', 1.0)
            goals, speeds, raw = [], [], []
            parameter_requests = []

            def set_parameter(request):
                parameter_requests.append(request.param_id)
                return ParamSetResponse(success=True, value=request.value)

            parameter_service = rospy.Service('/patrol_fake/mavros/param/set',
                                               ParamSet, set_parameter)

            def goal_cb(message):
                p = message.pose.position
                point = (round(p.x, 2), round(p.y, 2))
                if not goals or goals[-1] != point:
                    goals.append(point)
            rospy.Subscriber('/ego_goal_3', PoseStamped, goal_cb)
            rospy.Subscriber('/ego_speed_3', Float32MultiArray,
                             lambda m: speeds.append(tuple(m.data)))
            rospy.Subscriber('/patrol_fake/mavros/setpoint_raw/local', PositionTarget,
                             lambda m: raw.append(m))
            poses = rospy.Publisher('/patrol_fake/mavros/local_position/pose', PoseStamped, queue_size=1)
            states = rospy.Publisher('/patrol_fake/mavros/state', State, queue_size=1)
            modes = rospy.Publisher('/tracking/mode_3', Int8, queue_size=1)
            targets = rospy.Publisher('/tracking/state_3', TrackingTarget, queue_size=1)
            binary = PACKAGE.parents[1] / 'devel/lib/fly_px4/fly_px4_1'
            command = [str(binary), '__name:=patrol_map_test', '_drone_ns:=patrol_fake',
                       '_drone_id:=3', '_init_x:=-14', '_init_y:=3', '_patrol_map:=base3']
            node = subprocess.Popen(command, env=env, stdout=node_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while poses.get_num_connections() == 0:
                if node.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('fly_px4_1 failed to subscribe')
                time.sleep(.02)

            def feed(xy, z, duration, track=False, yaw_deg=0.):
                deadline = time.monotonic()+duration
                while time.monotonic() < deadline:
                    stamp = rospy.Time.now()
                    state = State(connected=True, armed=True, mode='OFFBOARD')
                    state.header.stamp = stamp
                    states.publish(state)
                    p = PoseStamped(); p.header.stamp = stamp
                    p.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
                    p.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
                    p.pose.position.x, p.pose.position.y = xy[0]+14, xy[1]-3
                    p.pose.position.z = z
                    poses.publish(p); modes.publish(Int8(data=1 if track else 0))
                    if track:
                        target = TrackingTarget()
                        target.state.header.stamp = stamp
                        target.state.header.frame_id = 'world'
                        target.state.child_frame_id = 'patrol-smoke:1'
                        target.state.pose.pose.position.x = xy[0]+25
                        target.state.pose.pose.position.y = xy[1]
                        target.state.pose.pose.position.z = 1
                        target.measurement_stamp = target.depth_stamp = stamp
                        target.assignment = 'patrol-smoke:1'
                        targets.publish(target)
                    time.sleep(.02)

            feed((-14, 3), 1.5, .2)
            assert raw and all(abs(m.position.x) < 1e-6 and abs(m.position.y) < 1e-6 for m in raw), \
                'Below 2m CLIMB must keep the fixed spawn XY'
            raw.clear()
            feed((-14, 3), 2.5, .3)
            # ControlPx4 smooths position commands. Check movement toward the
            # local entry (.5, 0), rather than expecting an instantaneous jump.
            assert len(raw) >= 5 and all(.1 < m.position.x <= .5 and abs(m.position.y) < 1e-6 for m in raw[-5:]), \
                ('Above 2m CLIMB must aim at the entry point while start_delay is pending',
                 [(m.position.x, m.position.y, m.type_mask) for m in raw[-5:]])
            assert not goals, 'Must not enter before reaching min_expl_z'
            feed((-14, 3), cruise_z, .2)
            assert not goals, 'start_delay must gate EGO entry'
            feed((-14, 3), cruise_z, 1.3)
            entries = [tuple(w[k] for k in ('x', 'y')) for w in cfg['routes']['drone_3']['entry_waypoints']]
            # The first entry is already within the arrival radius of the spawn,
            # so TRANSIT immediately advances to the following street entry.
            assert goals == entries, goals
            entry = entries[-1]
            assert abs(speeds[-1][0]-8.) < .01, speeds[-1]
            feed(entry, cruise_z, .2)
            loop = [tuple(w[k] for k in ('x', 'y')) for w in cfg['routes']['drone_3']['waypoints']]
            assert goals[-1] == loop[0], goals
            assert abs(speeds[-1][0]-8.) < .01, 'Approach must keep transit speed'
            for _ in range(2):
                for i, point in enumerate(loop):
                    next_point = loop[(i+1) % len(loop)]
                    yaw = math.degrees(math.atan2(next_point[1]-point[1], next_point[0]-point[0]))
                    feed(point, cruise_z, .16, yaw_deg=yaw)
                assert goals[-1] == loop[0], 'Closed loop must skip the approach points'
            assert goals == entries + loop*2 + loop[:1], goals
            assert abs(speeds[-1][0]-cfg['defaults']['cruise_speed']) < .01, 'Must use configured cruise speed on entering the loop'
            print('PASS: vertical lift below 2m, moving climb above 2m, EGO delay, approach once, transit/cruise switch, two loop closures')
            count = len(goals)
            feed(entry, cruise_z, 2., track=True)
            assert len(goals) > count and goals[-1] not in loop, 'Must enter TRACK for a fresh target'
            assert abs(raw[-1].position.z-track_z) < .02, ('Tracking command height', raw[-1].position.z, track_z)
            feed(entry, cruise_z, 1.8)
            assert goals[-1] == loop[0], 'Return from tracking must ignore one-time approach legs'
            assert abs(raw[-1].position.z-cruise_z) < .02, ('Return command height', raw[-1].position.z, cruise_z)
            assert parameter_requests == ['COM_RCL_EXCEPT'], parameter_requests
            print('PASS: controller leaves PX4 speed/acceleration limits untouched')
            print('PASS: tracking release returns to the patrol loop, skipping the approach')
            print('PASS: tracking commands use %.1fm and return commands use %.1fm' % (track_z, cruise_z))
            node.terminate(); node.wait(timeout=5); node = None

            # A route override is authoritative even without a shared track_z.
            defaults_key = '/patrol_map_test/patrol/defaults/track_z'
            override_key = '/patrol_map_test/patrol/routes/drone_3/track_z'
            rospy.delete_param(defaults_key)
            override_z = 2.3
            rospy.set_param(override_key, override_z)
            rospy.set_param('/patrol_map_test/patrol/routes/drone_3/start_delay', 0.)
            raw.clear()
            node = subprocess.Popen(command, env=env, stdout=node_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+5
            while poses.get_num_connections() == 0:
                if node.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('fly_px4_1 failed to subscribe with a route height override')
                time.sleep(.02)
            feed(entry, cruise_z, .5)
            feed(entry, cruise_z, 2., track=True)
            assert node.poll() is None and raw and abs(raw[-1].position.z-override_z) < .02, \
                ('Route tracking height override', raw[-1].position.z if raw else None, override_z)
            print('PASS: route track_z override below 2.7m is used without a shared height default')
            node.terminate(); node.wait(timeout=5); node = None

            # Reject malformed explicit overrides instead of hiding them behind a default.
            rospy.set_param(defaults_key, track_z)
            for invalid in (0., -1., float('nan'), 'invalid'):
                rospy.set_param(override_key, invalid)
                node = subprocess.Popen(command, env=env, stdout=node_log, stderr=subprocess.STDOUT)
                assert node.wait(timeout=5) != 0, ('Invalid track_z must refuse startup', invalid)
                node = None
            rospy.delete_param(override_key)
            rospy.delete_param(defaults_key)
            node = subprocess.Popen(command, env=env, stdout=node_log, stderr=subprocess.STDOUT)
            assert node.wait(timeout=5) != 0, 'Missing YAML track_z must refuse a compiled height fallback'
            node = None
            rospy.set_param(defaults_key, track_z)
            print('PASS: missing, non-positive, non-finite, and malformed YAML heights refuse startup')

            rospy.delete_param('/patrol_map_test/patrol/routes/drone_3/waypoints')
            node = subprocess.Popen(command, env=env, stdout=node_log, stderr=subprocess.STDOUT)
            assert node.wait(timeout=5) != 0, 'Invalid non-base0 route must refuse startup'
            assert 'refusing the base0 fallback' in (Path(tmp) / 'node.log').read_text(errors='replace')
            print('PASS: damaged non-base0 route refuses the base0 fallback')
            return 0
        except Exception:
            node_log.flush()
            print((Path(tmp) / 'node.log').read_text(errors='replace')[-5000:], file=sys.stderr)
            raise
        finally:
            for process in (node, master):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait()
            node_log.close(); master_log.close()


if __name__ == '__main__':
    sys.exit(main())
