#!/usr/bin/env python3
"""
RealMan Hardware Bridge — unified ROS2 state reader + command sender.

State reading:  ROS2 subscriptions (joint_states, hand_status, arm_pose)
Command sending: Robotic_Arm Python API (rm_robot_interface) via TCP

Default mode: both arms + both hands → 24D state/action
Stackable flags:
  --one-arm left|right   only one arm+hand → 12D
  --base                 add AGV base velocity (+3D)
  --platform             add platform height (+1D)

Usage (data collection, read-only):
    bridge = RealmanHardwareBridge(one_arm=None, base=False, platform=False)

Usage (deployment, read+write):
    bridge = RealmanHardwareBridge(...)
    bridge.connect_arms()   # opens TCP to arm API
    bridge.send_action(action_vector)
"""

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rm_ros_interfaces.msg import Handstatus
from sensor_msgs.msg import JointState

from constants import (
    IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS,
    ARM_DOF, HAND_DOF,
    LEFT_ARM_IP, RIGHT_ARM_IP, ARM_PORT, ARM_SPEED,
    HAND_SERVO_MAX,
    JOINT_STATES_TOPIC,
    LEFT_HAND_TOPIC, RIGHT_HAND_TOPIC,
    LEFT_ARM_POSE_TOPIC, RIGHT_ARM_POSE_TOPIC,
)
from camera_utils import AsyncCameraReader


class RealmanHardwareBridge(Node):
    """
    ROS2 node that reads robot state and (optionally) sends commands.

    Args:
        one_arm:  None (both arms), 'left', or 'right'
        base:     subscribe to base odometry and enable base commands
        platform: subscribe to platform height and enable platform commands
    """

    def __init__(self, one_arm=None, base=False, platform=False):
        super().__init__("realman_hardware_bridge")

        self.one_arm  = one_arm   # None | 'left' | 'right'
        self.use_base = base
        self.use_platform = platform

        # --- arm state ---
        self._l_arm = None          # (ARM_DOF,) radians
        self._r_arm = None
        self._l_arm_pose = Pose()
        self._r_arm_pose = Pose()

        # --- hand state (positions normalized 0→1) ---
        self._l_hand = [0.0] * HAND_DOF
        self._r_hand = [0.0] * HAND_DOF

        # --- base state [vx, vy, omega] m/s, rad/s ---
        self._base_vel = [0.0, 0.0, 0.0]

        # --- platform height (meters) ---
        self._platform_height = 0.0

        # ready flags
        self._got_l_arm   = False
        self._got_r_arm   = False
        self._got_l_hand  = False
        self._got_r_hand  = False

        # arm API objects (only created when connect_arms() is called)
        self._l_arm_api = None
        self._r_arm_api = None

        self._setup_subscriptions()
        self._camera = AsyncCameraReader(timeout_ms=100)
        self._default_img = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8)

        self.get_logger().info(
            f"Bridge init — one_arm={one_arm}, base={base}, platform={platform}, state_dim={self.state_dim}"
        )

    # ──────────────────────────────────────────────────────────────
    # Setup
    # ──────────────────────────────────────────────────────────────

    def _setup_subscriptions(self):
        self.create_subscription(JointState, JOINT_STATES_TOPIC,   self._joint_cb,   10)
        self.create_subscription(Handstatus, LEFT_HAND_TOPIC,      self._l_hand_cb,  10)
        self.create_subscription(Handstatus, RIGHT_HAND_TOPIC,     self._r_hand_cb,  10)
        self.create_subscription(Pose, LEFT_ARM_POSE_TOPIC,        self._l_pose_cb,  10)
        self.create_subscription(Pose, RIGHT_ARM_POSE_TOPIC,       self._r_pose_cb,  10)

        if self.use_base:
            self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

    def connect_arms(self):
        """Open TCP connections to arm API (required for deployment / send_action)."""
        try:
            from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
            mode = rm_thread_mode_e.RM_TRIPLE_MODE_E

            if self.one_arm != "right":
                self._l_arm_api = RoboticArm(mode)
                ret = self._l_arm_api.rm_create_robot_arm(ip=LEFT_ARM_IP, port=ARM_PORT)
                if ret.id == -1:
                    self.get_logger().error("Left arm API connection failed")
                else:
                    self.get_logger().info("Left arm API connected")

            if self.one_arm != "left":
                self._r_arm_api = RoboticArm(mode)
                ret = self._r_arm_api.rm_create_robot_arm(ip=RIGHT_ARM_IP, port=ARM_PORT)
                if ret.id == -1:
                    self.get_logger().error("Right arm API connection failed")
                else:
                    self.get_logger().info("Right arm API connected")

        except ImportError:
            self.get_logger().error("Robotic_Arm library not found — arm commands unavailable")

    def disconnect_arms(self):
        """Close arm API connections."""
        try:
            if self._l_arm_api:
                self._l_arm_api.rm_delete_robot_arm()
            if self._r_arm_api:
                self._r_arm_api.rm_delete_robot_arm()
        except Exception as e:
            self.get_logger().warn(f"Disconnect error: {e}")

    # ──────────────────────────────────────────────────────────────
    # ROS2 callbacks
    # ──────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        if len(msg.position) < ARM_DOF:
            return
        positions = list(msg.position[:ARM_DOF])
        if all(abs(p) < 1e-4 for p in positions):
            return  # skip all-zeros (driver not ready yet)
        if msg.name and msg.name[0].startswith("l_"):
            self._l_arm = positions
            self._got_l_arm = True
        elif msg.name and msg.name[0].startswith("r_"):
            self._r_arm = positions
            self._got_r_arm = True

    def _l_hand_cb(self, msg: Handstatus):
        # hand_pos range is 0–1000; normalize to 0–1
        self._l_hand = [p / 1000.0 for p in msg.hand_pos]
        self._got_l_hand = True

    def _r_hand_cb(self, msg: Handstatus):
        self._r_hand = [p / 1000.0 for p in msg.hand_pos]
        self._got_r_hand = True

    def _l_pose_cb(self, msg: Pose):
        self._l_arm_pose = msg

    def _r_pose_cb(self, msg: Pose):
        self._r_arm_pose = msg

    def _odom_cb(self, msg):
        v = msg.twist.twist
        self._base_vel = [v.linear.x, v.linear.y, v.angular.z]

    # ──────────────────────────────────────────────────────────────
    # State
    # ──────────────────────────────────────────────────────────────

    @property
    def state_dim(self) -> int:
        """Total dimension of the state vector."""
        if self.one_arm:
            dim = ARM_DOF + HAND_DOF   # 12
        else:
            dim = 2 * (ARM_DOF + HAND_DOF)  # 24
        if self.use_base:
            dim += 3
        if self.use_platform:
            dim += 1
        return dim

    @property
    def state_names(self) -> list:
        """Human-readable names matching the state vector order."""
        names = []
        arms = ["left"] if self.one_arm == "left" else (["right"] if self.one_arm == "right" else ["left", "right"])
        for side in arms:
            names += [f"{side}_arm_{i}" for i in range(ARM_DOF)]
            names += [f"{side}_hand_{i}" for i in range(HAND_DOF)]
        if self.use_base:
            names += ["base_vx", "base_vy", "base_omega"]
        if self.use_platform:
            names += ["platform_height"]
        return names

    def get_state(self) -> np.ndarray:
        """Return the current state vector (float32, shape=(state_dim,))."""
        parts = []

        if self.one_arm != "right":
            l_arm = self._l_arm if self._l_arm is not None else [0.0] * ARM_DOF
            parts += l_arm + self._l_hand

        if self.one_arm != "left":
            r_arm = self._r_arm if self._r_arm is not None else [0.0] * ARM_DOF
            parts += r_arm + self._r_hand

        if self.use_base:
            parts += self._base_vel

        if self.use_platform:
            parts += [self._platform_height]

        return np.array(parts, dtype=np.float32)

    def get_images(self) -> dict:
        """Return latest camera frames as {camera_name: (H,W,C) uint8 BGR}."""
        frames = self._camera.get_camera_data()  # list ordered by CAMERA_SERIALS
        names = list(CAMERA_SERIALS.keys())
        result = {}
        for i, name in enumerate(names):
            result[name] = frames[i] if i < len(frames) else self._default_img
        return result

    # ──────────────────────────────────────────────────────────────
    # Commands (deployment only — requires connect_arms())
    # ──────────────────────────────────────────────────────────────

    def send_action(self, action: np.ndarray):
        """
        Send an action vector to the robot.

        The action vector must have the same layout as get_state():
          [l_arm(6), l_hand(6), r_arm(6), r_hand(6)]  + optional base/platform

        Arm angles are in radians (converted to degrees for the API).
        Hand values are in 0–1 range (converted to servo units internally).
        """
        idx = 0

        if self.one_arm != "right" and self._l_arm_api:
            l_arm_rad  = action[idx : idx + ARM_DOF];  idx += ARM_DOF
            l_hand_01  = action[idx : idx + HAND_DOF]; idx += HAND_DOF
            self._send_arm(self._l_arm_api, l_arm_rad)
            self._send_hand(self._l_arm_api, l_hand_01)
        else:
            idx += ARM_DOF + HAND_DOF  # skip

        if self.one_arm != "left" and self._r_arm_api:
            r_arm_rad  = action[idx : idx + ARM_DOF];  idx += ARM_DOF
            r_hand_01  = action[idx : idx + HAND_DOF]; idx += HAND_DOF
            self._send_arm(self._r_arm_api, r_arm_rad)
            self._send_hand(self._r_arm_api, r_hand_01)
        else:
            idx += ARM_DOF + HAND_DOF

        if self.use_base:
            base = action[idx : idx + 3]; idx += 3
            self._send_base(base)

        # platform not yet implemented — requires extension joint command
        if self.use_platform:
            idx += 1

    def _send_arm(self, arm_api, joints_rad):
        """Send joint position command (rad → deg for API)."""
        joints_deg = [math.degrees(j) for j in joints_rad]
        ret = arm_api.rm_movej(joints_deg, ARM_SPEED, 0, 0, 0)
        if ret != 0:
            self.get_logger().warn(f"rm_movej returned {ret}")

    def _send_hand(self, arm_api, positions_01):
        """Send hand finger positions (0–1 → servo units)."""
        # TODO: verify HAND_SERVO_MAX with hardware — currently 1000
        servo_vals = [int(p * HAND_SERVO_MAX) for p in positions_01]
        ret = arm_api.rm_set_hand_follow_pos(servo_vals, block=0)
        if ret != 0:
            self.get_logger().warn(f"rm_set_hand_follow_pos returned {ret}")

    def _send_base(self, vel):
        """Send base velocity command [vx, vy, omega]."""
        # The Woosh base uses a ROS2 Action (StepControl), not a simple topic.
        # For deployment, publish a geometry_msgs/Twist to /cmd_vel as a simpler interface.
        # TODO: switch to StepControl action if precise distance control is needed.
        if not hasattr(self, "_cmd_vel_pub"):
            self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        msg = Twist()
        msg.linear.x  = float(vel[0])
        msg.linear.y  = float(vel[1])
        msg.angular.z = float(vel[2])
        self._cmd_vel_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────
    # Ready checks
    # ──────────────────────────────────────────────────────────────

    def is_ready(self) -> bool:
        """True once joint state received from all active arms."""
        if self.one_arm == "left":
            return self._got_l_arm
        if self.one_arm == "right":
            return self._got_r_arm
        return self._got_l_arm and self._got_r_arm

    def hands_ready(self) -> bool:
        if self.one_arm == "left":
            return self._got_l_hand
        if self.one_arm == "right":
            return self._got_r_hand
        return self._got_l_hand and self._got_r_hand

    def cleanup(self):
        self._camera.stop()
        self.disconnect_arms()


# ──────────────────────────────────────────────────────────────────
# CLI test
# ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--one-arm", choices=["left", "right"], default=None)
    parser.add_argument("--base",     action="store_true")
    parser.add_argument("--platform", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    bridge = RealmanHardwareBridge(one_arm=args.one_arm, base=args.base, platform=args.platform)

    try:
        rate = bridge.create_rate(5)
        while rclpy.ok():
            rclpy.spin_once(bridge, timeout_sec=0.1)
            state = bridge.get_state()
            print(f"state({bridge.state_dim}D): {state.round(3)}")
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
