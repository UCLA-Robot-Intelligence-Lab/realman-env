#!/usr/bin/env python3
"""
Data collection entry point.

Waits for hardware to be ready, then records episodes of teleop data
in LeRobot v3 format (Parquet + MP4).

Usage:
    python collect.py \\
        --repo-id  user/my-dataset \\
        --output   ./collected_data \\
        --task     "pick and place" \\
        --fps      10 \\
        --episodes 20 \\
        [--one-arm left|right] [--base] [--platform]

Press  Enter   to start/stop an episode.
Press  Ctrl+C  to finish and finalize the dataset.
"""

import os
import sys
import threading
import time

import rclpy

# allow imports from core/ when running from this directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "core"))

from hardware_bridge import RealmanHardwareBridge
from lerobot_recorder import LeRobotRecorder
from constants import DEFAULT_FPS, DEFAULT_DATA_DIR, CAMERA_SERIALS

# ── Config — edit these before running ───────────────────────────
REPO_ID    = "your-username/my-dataset"
OUTPUT_DIR = DEFAULT_DATA_DIR
TASK       = "pick and place"
FPS        = DEFAULT_FPS
EPISODES   = 0       # 0 = record until Ctrl+C
ONE_ARM    = None    # None = both arms, "left", or "right"
BASE       = False   # include AGV base velocity
PLATFORM   = False   # include platform height
# ─────────────────────────────────────────────────────────────────


def wait_for_ready(bridge, timeout=30.0):
    print("Waiting for robot state...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(bridge, timeout_sec=0.1)
        if bridge.is_ready() and bridge.hands_ready():
            print("Robot ready.")
            return True
    print("Timeout: robot not ready.")
    return False


def spin_background(bridge, stop_event):
    """Keep ROS spinning in a background thread during recording."""
    while not stop_event.is_set():
        rclpy.spin_once(bridge, timeout_sec=0.05)


def record_episode(bridge, recorder, fps, task):
    """Record one episode until the user presses Enter to stop."""
    frame_interval = 1.0 / fps
    stop_event = threading.Event()

    spin_thread = threading.Thread(target=spin_background, args=(bridge, stop_event), daemon=True)
    spin_thread.start()

    recorder.start_episode()

    # record in a thread so input() can stop it
    recording = threading.Event()
    recording.set()
    frames = [0]

    def record_loop():
        while recording.is_set():
            t0 = time.time()
            state  = bridge.get_state()
            images = bridge.get_images()
            # during data collection, action = current state (kinesthetic teaching)
            recorder.add_frame(obs=state, action=state, images=images)
            frames[0] += 1
            elapsed = time.time() - t0
            wait = frame_interval - elapsed
            if wait > 0:
                time.sleep(wait)

    rec_thread = threading.Thread(target=record_loop, daemon=True)
    rec_thread.start()

    print("  Recording... press Enter to stop.")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        pass

    recording.clear()
    rec_thread.join(timeout=2.0)
    stop_event.set()

    n = recorder.end_episode(task=task)
    print(f"  Saved {n} frames at {fps} FPS  ({n/fps:.1f}s)")
    return n


def main():
    rclpy.init()
    bridge = RealmanHardwareBridge(
        one_arm=ONE_ARM,
        base=BASE,
        platform=PLATFORM,
    )

    if not wait_for_ready(bridge):
        bridge.destroy_node()
        rclpy.shutdown()
        return 1

    recorder = LeRobotRecorder(
        repo_id=REPO_ID,
        root=OUTPUT_DIR,
        fps=FPS,
        state_names=bridge.state_names,
        camera_names=list(CAMERA_SERIALS.keys()),
    )

    print(f"\nDataset: {REPO_ID}  |  task: '{TASK}'  |  {bridge.state_dim}D state")
    print("Press Enter to start an episode, Ctrl+C to finish.\n")

    ep = 0
    try:
        while True:
            if EPISODES and ep >= EPISODES:
                break
            try:
                input(f"[Episode {ep}] Press Enter to start...")
            except (KeyboardInterrupt, EOFError):
                break
            record_episode(bridge, recorder, FPS, TASK)
            ep += 1

    except KeyboardInterrupt:
        pass
    finally:
        recorder.finalize()
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()
        print(f"\nDone. {recorder.num_episodes} episodes saved to {OUTPUT_DIR}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
