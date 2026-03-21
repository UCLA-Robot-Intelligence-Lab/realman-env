#!/usr/bin/env python3
# Constants for RealMan dual-arm data collection and deployment

# --- Camera ---
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_CHANNELS = 3

CAMERA_SERIALS = {
    "top":        "027322070274",
    "left_wrist": "216322071814",
    # add right_wrist serial here when available
}

# --- Robot arms ---
ARM_DOF = 6          # joints per arm (RM65-B). Change to 7 for RM75-B
HAND_DOF = 6         # fingers per hand

LEFT_ARM_IP  = "169.254.128.18"
RIGHT_ARM_IP = "169.254.128.19"
ARM_PORT     = 8080

ARM_SPEED    = 20    # joint speed for deployment (deg/s)

# Hand servo range for command (rm_set_hand_follow_pos)
HAND_SERVO_MAX = 1000   # TODO: verify actual max with hardware

# --- ROS2 topics ---
JOINT_STATES_TOPIC      = "/joint_states"
LEFT_HAND_TOPIC         = "/left_arm_controller/rm_driver/udp_hand_status"
RIGHT_HAND_TOPIC        = "/right_arm_controller/rm_driver/udp_hand_status"
LEFT_ARM_POSE_TOPIC     = "/left_arm_controller/rm_driver/udp_arm_position"
RIGHT_ARM_POSE_TOPIC    = "/right_arm_controller/rm_driver/udp_arm_position"

# Base (Woosh) — state from odometry, commands via StepControl action
BASE_ODOM_TOPIC         = "/woosh_robot/ros/odom"   # TODO: verify topic name
BASE_ACTION_SERVER      = "woosh_robot/ros/StepControl"

# Platform (connected as extension joint on left arm — joint index 6 if ARM_DOF=7)
PLATFORM_JOINT_IDX      = 6    # 0-indexed; only used when --platform flag active
PLATFORM_TOPIC          = "/left_arm_controller/rm_driver/udp_arm_position"  # shares with left arm pose

# --- Data collection ---
DEFAULT_FPS       = 10
DEFAULT_DATA_DIR  = "collected_data"
