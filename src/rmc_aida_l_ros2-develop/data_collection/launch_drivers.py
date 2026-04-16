#!/usr/bin/env python3
"""
Launch ROS2 arm drivers and RealSense cameras.

Run this with system Python 3.8 (has ROS2):
    python3.8 launch_drivers.py

Then run collect.py / deploy.py with conda environment (has LeRobot/PyTorch).
"""

import os
import signal
import subprocess
import sys

ROS_WS   = os.path.expanduser("~/ros2_ws/install/setup.bash")

# ── Config ───────────────────────────────────────────────────────
ARM_MODEL = "rm65"   # "rm65" or "rm75"
ARM_ONLY  = False    # True = skip cameras
# ─────────────────────────────────────────────────────────────────


def run(cmd: list, env=None) -> subprocess.Popen:
    return subprocess.Popen(cmd, env=env)


def source_ros():
    """Return env dict with ROS2 sourced (reads from shell)."""
    import shlex
    result = subprocess.run(
        f"bash -c 'source /opt/ros/foxy/setup.bash && source {ROS_WS} 2>/dev/null; env'",
        shell=True, capture_output=True, text=True
    )
    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    return env


def main():
    ros_env = source_ros()
    procs = []

    driver_launch = f"dual_{ARM_MODEL}_driver.launch.py"
    print(f"Launching arm driver: {driver_launch}")
    procs.append(run(["ros2", "launch", "rm_driver", driver_launch], env=ros_env))

    if not ARM_ONLY:
        print("Launching cameras (3x RealSense)...")
        for i in range(3):
            procs.append(run(
                ["ros2", "launch", "realsense2_camera", "rs_launch.py",
                 f"camera_name:=camera{i}"],
                env=ros_env
            ))

    print(f"Drivers running ({len(procs)} processes). Ctrl+C to stop.\n")

    def shutdown(sig, _):
        print("Stopping drivers...")
        for p in procs:
            p.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    for p in procs:
        p.wait()


if __name__ == "__main__":
    main()
