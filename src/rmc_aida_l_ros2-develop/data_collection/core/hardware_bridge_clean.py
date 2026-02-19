#!/usr/bin/env python3
"""
Clean ROS2 Hardware Bridge for RealMan Dual Arms with 6-DOF Hands
Following the structure of the reference hardware bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import Handstatus
import numpy as np
import time
import threading

from constants import IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS, ARM_SPEED, ARM_DOF, HAND_DOF
from camera_utils import AsyncCameraReader


class RealmanHardwareBridge(Node):
    """Clean ROS2 Hardware bridge for dual Realman arms with 6-DOF hand state feedback"""

    def __init__(self):
        super().__init__('realman_hardware_bridge')

        # Joint state storage
        self.l_arm_current_angles = None
        self.r_arm_current_angles = None
        self._received_l_arm = False
        self._received_r_arm = False

        # Hand state storage (6-DOF hands, not grippers)
        self.l_hand_angles = [0.0] * HAND_DOF
        self.r_hand_angles = [0.0] * HAND_DOF
        self.l_hand_positions = [0.0] * HAND_DOF
        self.r_hand_positions = [0.0] * HAND_DOF
        self.l_hand_states = [0] * HAND_DOF
        self.r_hand_states = [0] * HAND_DOF
        self.l_hand_forces = [0.0] * HAND_DOF
        self.r_hand_forces = [0.0] * HAND_DOF
        self._received_l_hand = False
        self._received_r_hand = False

        # Arm pose storage
        self.l_arm_pose = Pose()
        self.r_arm_pose = Pose()
        self._received_l_pose = False
        self._received_r_pose = False

        # Subscriber for unified joint states (both arms on same topic)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Subscribers for hand state feedback (6-DOF hands)
        self.l_hand_state_sub = self.create_subscription(
            Handstatus,
            '/left_arm_controller/rm_driver/udp_hand_status',
            self.l_hand_state_callback,
            10
        )
        self.r_hand_state_sub = self.create_subscription(
            Handstatus,
            '/right_arm_controller/rm_driver/udp_hand_status',
            self.r_hand_state_callback,
            10
        )

        # Subscribers for arm poses
        self.l_arm_pose_sub = self.create_subscription(
            Pose,
            '/left_arm_controller/rm_driver/udp_arm_position',
            self.l_arm_pose_callback,
            10
        )
        self.r_arm_pose_sub = self.create_subscription(
            Pose,
            '/right_arm_controller/rm_driver/udp_arm_position',
            self.r_arm_pose_callback,
            10
        )

        # Camera setup using AsyncCameraReader (standalone RealSense)
        self.camera_reader = AsyncCameraReader(
            timeout_ms=100, slow_speed_threshold=15.0
        )
        self._default_image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8)

        self.get_logger().info('Hardware bridge initialized')
        self.get_logger().info('  Hand state: Subscribing to RM driver feedback (6-DOF hands)')

    def joint_states_callback(self, msg):
        """Unified joint states callback - filters by joint name prefix"""
        if len(msg.name) > 0 and len(msg.position) >= 6:
            positions = list(msg.position[:6])

            # Check if we're receiving all zeros (might indicate a problem)
            if all(abs(p) < 0.001 for p in positions):
                return

            if msg.name[0].startswith('l_'):
                self.l_arm_current_angles = positions
                if not self._received_l_arm:
                    self._received_l_arm = True
                    self.get_logger().info(f'✓ Receiving left arm joint states')
            elif msg.name[0].startswith('r_'):
                self.r_arm_current_angles = positions
                if not self._received_r_arm:
                    self._received_r_arm = True
                    self.get_logger().info(f'✓ Receiving right arm joint states')

    def l_hand_state_callback(self, msg: Handstatus):
        """Left hand state callback - reads actual 6-DOF hand data from hardware"""
        self.l_hand_angles = list(msg.hand_angle)
        self.l_hand_positions = list(msg.hand_pos)
        self.l_hand_states = list(msg.hand_state)
        self.l_hand_forces = list(msg.hand_force)
        
        if not self._received_l_hand:
            self._received_l_hand = True
            self.get_logger().info(f'✓ Receiving left hand state (6-DOF)')

    def r_hand_state_callback(self, msg: Handstatus):
        """Right hand state callback - reads actual 6-DOF hand data from hardware"""
        self.r_hand_angles = list(msg.hand_angle)
        self.r_hand_positions = list(msg.hand_pos)
        self.r_hand_states = list(msg.hand_state)
        self.r_hand_forces = list(msg.hand_force)
        
        if not self._received_r_hand:
            self._received_r_hand = True
            self.get_logger().info(f'✓ Receiving right hand state (6-DOF)')

    def l_arm_pose_callback(self, msg: Pose):
        """Left arm pose callback"""
        self.l_arm_pose = msg
        if not self._received_l_pose:
            self._received_l_pose = True
            self.get_logger().info(f'✓ Receiving left arm pose')

    def r_arm_pose_callback(self, msg: Pose):
        """Right arm pose callback"""
        self.r_arm_pose = msg
        if not self._received_r_pose:
            self._received_r_pose = True
            self.get_logger().info(f'✓ Receiving right arm pose')

    def get_state(self):
        """Get current robot state (26D: left arm + left hand + right arm + right hand)

        Returns ACTUAL hand positions from hardware via RM driver feedback!
        """
        l_arm = self.l_arm_current_angles if self.l_arm_current_angles is not None else [0.0] * ARM_DOF
        r_arm = self.r_arm_current_angles if self.r_arm_current_angles is not None else [0.0] * ARM_DOF

        # Normalize hand positions to 0-1 range (0=closed, 1000=open)
        l_hand_norm = [max(0.0, min(1.0, pos / 1000.0)) for pos in self.l_hand_positions]
        r_hand_norm = [max(0.0, min(1.0, pos / 1000.0)) for pos in self.r_hand_positions]

        # State: [l_arm(6), l_hand(6), r_arm(6), r_hand(6)] = 24D
        return np.concatenate([l_arm, l_hand_norm, r_arm, r_hand_norm])

    def get_full_state(self):
        """Get full robot state including poses and forces"""
        state = {
            'timestamp': time.time(),
            'joint_state': self.get_state(),
            'left_arm_pose': self.l_arm_pose,
            'right_arm_pose': self.r_arm_pose,
            'left_hand_angles': self.l_hand_angles,
            'right_hand_angles': self.r_hand_angles,
            'left_hand_positions': self.l_hand_positions,
            'right_hand_positions': self.r_hand_positions,
            'left_hand_states': self.l_hand_states,
            'right_hand_states': self.r_hand_states,
            'left_hand_forces': self.l_hand_forces,
            'right_hand_forces': self.r_hand_forces
        }
        return state

    def is_ready(self):
        """Check if we've received joint states from both arms"""
        return self._received_l_arm and self._received_r_arm

    def hands_ready(self):
        """Check if we've received hand state from both arms"""
        return self._received_l_hand and self._received_r_hand

    def poses_ready(self):
        """Check if we've received arm poses from both arms"""
        return self._received_l_pose and self._received_r_pose

    def get_camera_data(self, wait_for_fresh=True):
        """Get camera frames [top, left_wrist, right_wrist]"""
        try:
            frames = self.camera_reader.get_camera_data()
        except Exception as e:
            self.get_logger().warn(f"Camera read error: {e}")
            frames = [
                self._default_image,
                self._default_image,
                self._default_image
            ]

        return frames

    def move_to_initial_pose(self):
        """Open hands for kinesthetic teaching"""
        self.get_logger().info('Opening hands for kinesthetic teaching')
        # Note: Hand control would require publishers for hand commands
        time.sleep(1.0)

    def cleanup(self):
        """Release resources"""
        self.camera_reader.stop()


def main():
    rclpy.init()
    bridge = RealmanHardwareBridge()

    try:
        rate = bridge.create_rate(10)
        while rclpy.ok():
            state = bridge.get_state()
            print(f'State: L_arm={state[:3].round(2)} L_hand={state[6:9].round(2)} | R_arm={state[12:15].round(2)} R_hand={state[18:21].round(2)}')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()