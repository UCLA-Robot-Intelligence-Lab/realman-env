#!/usr/bin/env python3
"""
Deploy a trained LeRobot policy on the RealMan robot.

Usage:
    python deploy.py \\
        --policy-path  ./outputs/train/act/checkpoints/last \\
        --fps          10 \\
        [--one-arm left|right] [--base] [--platform]

Press  Enter   to start/stop an episode.
Press  Ctrl+C  to exit.
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
import torch
from lerobot.policies.factory import make_policy

# import hardware bridge from data_collection/core
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "data_collection", "core"))
from hardware_bridge import RealmanHardwareBridge
from constants import DEFAULT_FPS, CAMERA_SERIALS

# ── Config — edit these before running ───────────────────────────
POLICY_PATH = "./train/imitation/models/<run>/final"
FPS         = DEFAULT_FPS
ONE_ARM     = None    # None = both arms, "left", or "right"
BASE        = False   # include AGV base
PLATFORM    = False   # include platform height
# ─────────────────────────────────────────────────────────────────


def load_policy(policy_path: str):
    policy = make_policy(hydra_cfg=None, policy_path=policy_path)
    policy.train(False)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    policy.to(device)
    print(f"Policy loaded from {policy_path} on {device}")
    return policy, device


def build_obs_batch(state: np.ndarray, images: dict, camera_names: list, device: str):
    """Convert state + images into a policy input dict."""
    batch = {}
    batch["observation.state"] = torch.from_numpy(state).float().unsqueeze(0).to(device)
    for cam in camera_names:
        img = images.get(cam)
        if img is not None:
            img_rgb = img[:, :, ::-1].copy()   # BGR → RGB
            t = torch.from_numpy(img_rgb).permute(2, 0, 1).float().div(255.0)
            batch[f"observation.images.{cam}"] = t.unsqueeze(0).to(device)
    return batch


def run_episode(bridge, policy, device, fps, camera_names):
    """Run policy in a loop until user presses Enter."""
    frame_interval = 1.0 / fps
    stop = threading.Event()
    running = threading.Event()
    running.set()

    def spin():
        while not stop.is_set():
            rclpy.spin_once(bridge, timeout_sec=0.02)

    def control_loop():
        policy.reset()
        while running.is_set():
            t0 = time.time()
            state  = bridge.get_state()
            images = bridge.get_images()
            batch  = build_obs_batch(state, images, camera_names, device)
            with torch.no_grad():
                action = policy.select_action(batch)
            bridge.send_action(action.squeeze(0).cpu().numpy())
            wait = frame_interval - (time.time() - t0)
            if wait > 0:
                time.sleep(wait)

    threading.Thread(target=spin, daemon=True).start()
    threading.Thread(target=control_loop, daemon=True).start()

    print("  Policy running... press Enter to stop.")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        pass

    running.clear()
    stop.set()
    time.sleep(0.2)


def main():
    rclpy.init()
    bridge = RealmanHardwareBridge(
        one_arm=ONE_ARM, base=BASE, platform=PLATFORM
    )

    print("Waiting for robot state...")
    deadline = time.time() + 30
    while time.time() < deadline:
        rclpy.spin_once(bridge, timeout_sec=0.1)
        if bridge.is_ready():
            break
    print("Robot ready.")

    bridge.connect_arms()
    policy, device = load_policy(POLICY_PATH)
    camera_names = list(CAMERA_SERIALS.keys())

    print("\nPress Enter to run an episode, Ctrl+C to exit.\n")
    ep = 0
    try:
        while True:
            try:
                input(f"[Episode {ep}] Press Enter to start...")
            except (KeyboardInterrupt, EOFError):
                break
            run_episode(bridge, policy, device, FPS, camera_names)
            ep += 1
    finally:
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
