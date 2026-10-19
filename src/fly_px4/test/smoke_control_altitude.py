#!/usr/bin/env python3
"""Run the real ControlPx4 publisher regression on a private ROS master."""

import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    with tempfile.TemporaryDirectory(prefix='control-altitude-ros-') as temporary:
        logs = Path(temporary)
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                   ROS_IP='127.0.0.1', ROS_HOME=str(logs / 'home'),
                   ROS_LOG_DIR=str(logs / 'ros'))
        env.pop('ROS_HOSTNAME', None)
        with (logs / 'master.log').open('w') as log:
            master_process = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
                deadline = time.monotonic() + 15
                while True:
                    try:
                        if master.getUri('/control_altitude_test')[0] == 1:
                            break
                    except OSError:
                        pass
                    if master_process.poll() is not None or time.monotonic() > deadline:
                        log.flush()
                        raise RuntimeError('Private ROS master failed:\n' +
                                           (logs / 'master.log').read_text())
                    time.sleep(.05)
                result = subprocess.run([args.executable], env=env, timeout=25)
                return result.returncode
            finally:
                if master_process.poll() is None:
                    os.killpg(master_process.pid, signal.SIGINT)
                    try:
                        master_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(master_process.pid, signal.SIGKILL)
                        master_process.wait()


if __name__ == '__main__':
    raise SystemExit(main())
