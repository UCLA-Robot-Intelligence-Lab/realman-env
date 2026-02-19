#!/usr/bin/env python3
"""
Data Collection Constants for RealMan Dual-Arm System
"""

# Camera configuration
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_CHANNELS = 3

# Robot configuration
ARM_SPEED = 0.5  # rad/s
ARM_DOF = 6  # degrees of freedom per arm

# Hand configuration (6-DOF hands, not grippers)
HAND_DOF = 6  # fingers per hand
HAND_ANGLE_RANGE = (0, 2000)  # angle sensor range
HAND_POS_RANGE = (0, 1000)    # position sensor range

# Data collection
DEFAULT_COLLECTION_RATE = 10  # Hz
MAX_BUFFER_SIZE = 1000

# Topics
LEFT_ARM_POSE_TOPIC = '/left_arm_controller/rm_driver/udp_arm_position'
RIGHT_ARM_POSE_TOPIC = '/right_arm_controller/rm_driver/udp_arm_position'
LEFT_HAND_STATUS_TOPIC = '/left_arm_controller/rm_driver/udp_hand_status'
RIGHT_HAND_STATUS_TOPIC = '/right_arm_controller/rm_driver/udp_hand_status'
JOINT_STATES_TOPIC = '/joint_states'

# File paths
DEFAULT_DATA_DIR = 'collected_data'
META_FILENAME = 'metadata.json'