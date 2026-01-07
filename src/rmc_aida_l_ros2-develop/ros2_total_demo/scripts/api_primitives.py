#!/usr/bin/env python3
"""
Dual-Hand Robot Piano Player Library - robot_pianist_api_library.py (FIXED VERSION 5)
Provides a class with high-level primitives to control the robot pianist using both hands.
This library uses the Python API instead of ROS and is intended to be imported by execution scripts.

FIXED FEATURES (Version 5):
1.  Collision avoidance uses LATERAL-ONLY "sidestep".
2.  REMOVED over-eager proactive collision check in 'move_wrist_to_note'.
3.  Collision avoidance is now purely PREDICTIVE, triggering a sidestep
    only if the arm's *destination* is a collision risk.
    This fixes the "back-and-forth" shuffling motion.
"""
import time
import numpy as np
import signal
import sys

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


class PianoKeyboard:
    """
    Internal helper class for piano layout and dynamic calculations.
    Supports mirrored configurations and pre-calculates note offsets
    relative to each hand's unique calibration anchor.
    """
    def __init__(self):
        self.key_spacing = 0.023  # meters

        # As notes go "right" (higher pitch), the offset value increases
        _absolute_note_offsets = {
            'A2': -19, 'B2': -18, 'C3': -17, 'D3': -16, 'E3': -15, 'F3': -14, 'G3': -13, 'A3': -12,
            'B3': -11, 'C4': -10, 'D4': -9, 'E4': -8, 'F4': -7, 'G4': -6, 'A4': -5, 'B4': -4,
            'C5': -3, 'D5': -2, 'E5': -1, 'F5': 0, 'G5': 1, 'A5': 2, 'B5': 3, 'C6': 4, 'D6': 5,
            'E6': 6, 'F6': 7, 'G6': 8,
        }

        self.left_calibration_anchor = 'E4'
        self.right_calibration_anchor = 'F5'

        left_anchor_value = _absolute_note_offsets[self.left_calibration_anchor]
        self.left_note_offsets = {
            note: val - left_anchor_value for note, val in _absolute_note_offsets.items()
        }

        right_anchor_value = _absolute_note_offsets[self.right_calibration_anchor]
        self.right_note_offsets = {
            note: val - right_anchor_value for note, val in _absolute_note_offsets.items()
        }

        self.left_exclusive_range = [
            'A2', 'B2', 'C3', 'D3', 'E3', 'F3', 'G3', 'A3', 'B3'
        ]
        self.right_exclusive_range = [
            'E5', 'F5', 'G5', 'A5', 'B5',
            'C6', 'D6', 'E6', 'F6', 'G6'
        ]
        self.overlap_notes = [
            'C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5', 'D5', 
        ]

        self.left_hand_range = self.left_exclusive_range + self.overlap_notes
        self.right_hand_range = self.overlap_notes + self.right_exclusive_range

        self.right_finger_assignments = {'index': 0, 'middle': 1, 'ring': 2, 'pinky': 3}
        self.left_finger_assignments = {'pinky': 0, 'ring': 1, 'middle': 2, 'index': 3}

        # Servo values for finger movements
        self.right_servo_values = {
            'open': [0, 0, 0, 0, 0, 30000], 
            'index_press': [0, 8500, 0, 0, 0, 30000],
            'middle_press': [0, 0, 7800, 0, 0, 30000],
            'ring_press': [0, 0, 0, 7200, 0, 30000],
            'pinky_press': [0, 0, 0, 0, 9800, 30000],
            'index_lifted': [0, 1500, 0, 0, 0, 30000],
            'middle_lifted': [0, 0, 1500, 0, 0, 30000],
            'ring_lifted': [0, 0, 0, 1500, 0, 30000],
            'pinky_lifted': [0, 0, 0, 0, 1500, 30000],
        }
        self.left_servo_values = {
            'open': [0, 0, 0, 0, 0, 30000], 
            'index_press': [0, 7500, 0, 0, 0, 30000],
            'middle_press': [0, 0, 9000, 0, 0, 30000],
            'ring_press': [0, 0, 0, 9000, 0, 30000],
            'pinky_press': [0, 0, 0, 0, 4500, 30000],
            'index_lifted': [0, 1500, 0, 0, 0, 30000],
            'middle_lifted': [0, 0, 1500, 0, 0, 30000],
            'ring_lifted': [0, 0, 0, 1500, 0, 30000],
            'pinky_lifted': [0, 0, 0, 0, 1500, 30000],
        }


class RobotPianoPlayer:
    """Provides the motion primitive API to control the dual-hand robot pianist using Python API."""

    # Connection Constants
    LEFT_ARM_IP = "169.254.128.18"
    RIGHT_ARM_IP = "169.254.128.19"
    PORT = 8080

    # Home position joint angles (in degrees) - modify these based on your setup
    LEFT_HOME_JOINTS = [-98.23, -77.51, -107.24, -19.66, 84.58, -279.46]
    RIGHT_HOME_JOINTS = [91.45, 78.77, 109.33, -3.53, -95.01, 159.59]

    def __init__(self, finger_speed=200, wrist_speed=100, collision_threshold=0.10):
        """
        Initialize robot piano player with configurable speeds.
        Collision checking is now always enabled.
        
        Args:
            finger_speed (int): Speed for finger movements (50-500, default 200)
            wrist_speed (int): Speed for wrist/arm movements (10-200, default 100)
            collision_threshold (float): Minimum safe distance between arms in meters (default 0.12)
        """
        self.left_arm = None
        self.right_arm = None
        self.piano = PianoKeyboard()
        self.hands = {
            'left': {
                'base_position': None, 
                'base_orientation': None, 
                'current_pose': None, 
                'calibrated': False, 
                'current_wrist_target_note': None, 
                'last_played_note': None,
                'is_hovering': False  # --- NOTE: 'is_hovering' now means "is in safe-lane"
            },
            'right': {
                'base_position': None, 
                'base_orientation': None, 
                'current_pose': None, 
                'calibrated': False, 
                'current_wrist_target_note': None, 
                'last_played_note': None,
                'is_hovering': False # --- NOTE: 'is_hovering' now means "is in safe-lane"
            }
        }

        # Set speed parameters
        self.finger_speed = max(50, min(1000, finger_speed))
        self.wrist_speed = max(10, min(100, wrist_speed))
        
        self.COLLISION_AVOIDANCE_DISTANCE = collision_threshold
        
        self.collision_check_enabled = True # Always on
        self.auto_collision_check = True  # Always on
        
        self.collision_log = []  # Store collision events for analysis
        self._last_collision_check_time = 0  # Prevent spam checking
        self._collision_check_cooldown = 0.2  # Minimum seconds between checks

        signal.signal(signal.SIGINT, self._emergency_stop_handler)
        print("Dual-Hand Robot Piano Player API Library Initialized.")
        print(f"Finger Speed: {self.finger_speed}, Wrist Speed: {self.wrist_speed}")
        print(f"Collision Threshold: {self.COLLISION_AVOIDANCE_DISTANCE}m")
        # --- MODIFIED ---
        print("Auto Collision Check: ALWAYS ENABLED (Predictive Lateral Sidestep Strategy)")
        print("Press Ctrl+C at any time for emergency stop.")

    def set_collision_threshold(self, threshold):
        """
        Update the collision avoidance distance threshold.
        
        Args:
            threshold (float): Minimum safe distance in meters
        """
        self.COLLISION_AVOIDANCE_DISTANCE = max(0.02, threshold)
        print(f"Collision threshold updated to: {self.COLLISION_AVOIDANCE_DISTANCE}m")

    def get_arm_distance(self):
        """
        Calculate the current distance between both arm end-effectors.
        
        Returns:
            float: Distance in meters, or None if positions unavailable
        """
        if not (self.hands['left']['calibrated'] and self.hands['right']['calibrated']):
            return None
        
        # Update both arm states
        self._get_arm_state('left')
        self._get_arm_state('right')
        
        left_pos = self.hands['left']['current_pose']
        right_pos = self.hands['right']['current_pose']
        
        if left_pos is None or right_pos is None:
            return None
        
        distance = np.linalg.norm(
            left_pos['position'] - right_pos['position']
        )
        return distance

    def check_and_prevent_collision(self, timestamp=None, min_distance=None):
        """
        Enhanced collision checking with cooldown to prevent spam.
        This function is now primarily for *monitoring* or for non-move-related checks.
        The main avoidance logic is now inside 'move_wrist_to_note'.
        """
        threshold = min_distance if min_distance is not None else self.COLLISION_AVOIDANCE_DISTANCE
        current_time = timestamp or time.time()
        distance = self.get_arm_distance()
        
        result = {
            'collision_risk': False,
            'distance': distance,
            'timestamp': current_time,
            'action_taken': 'none'
        }
        
        if distance is None:
            result['action_taken'] = 'no_data'
            return result
        
        if distance < threshold:
            result['collision_risk'] = True
            print(f"\n!!! COLLISION RISK DETECTED at t={current_time:.3f}s !!!")
            print(f"    Distance: {distance:.4f}m (threshold: {threshold:.4f}m)")
            
            left_hovering = self.hands['left']['is_hovering']
            right_hovering = self.hands['right']['is_hovering']
            
            # If both are in safe lane and still too close, apply emergency separation
            if left_hovering and right_hovering:
                print("    Both hands already in safe-lane but STILL TOO CLOSE!")
                print("    Applying additional lateral separation...")
                self._apply_emergency_separation()
                result['action_taken'] = 'emergency_separation_while_in_lane'
                return result

            # If a risk is detected, and at least one arm isn't in a safe-lane,
            # move the non-safe-lane arm (or the 'left' one if both are unsafe)
            if not left_hovering and not right_hovering:
                 print(f"    Moving LEFT hand to safe-lane for safety...")
                 self.move_to_hover('left')
                 result['action_taken'] = 'moved_left_to_safe_lane'
            elif not left_hovering:
                 print(f"    Moving LEFT hand to safe-lane for safety...")
                 self.move_to_hover('left')
                 result['action_taken'] = 'moved_left_to_safe_lane'
            elif not right_hovering:
                 print(f"    Moving RIGHT hand to safe-lane for safety...")
                 self.move_to_hover('right')
                 result['action_taken'] = 'moved_right_to_safe_lane'
                 
            # Verify distance increased
            time.sleep(0.3)
            new_distance = self.get_arm_distance()
            if new_distance:
                print(f"    New distance: {new_distance:.4f}m")
                result['new_distance'] = new_distance
        
        # Log the collision check
        self.collision_log.append(result)
        
        return result

    def clear_collision_log(self):
        """Clear the collision event log."""
        self.collision_log = []
        print("Collision log cleared")

    def set_speeds(self, finger_speed=None, wrist_speed=None):
        """
        Update speed parameters.
        
        Args:
            finger_speed (int): Speed for finger movements (50-500)
            wrist_speed (int): Speed for wrist/arm movements (10-200)
        """
        if finger_speed is not None:
            self.finger_speed = max(50, min(1000, finger_speed))
            print(f"Finger speed updated to: {self.finger_speed}")
        
        if wrist_speed is not None:
            self.wrist_speed = max(10, min(100, wrist_speed))
            print(f"Wrist speed updated to: {self.wrist_speed}")

    def _emergency_stop_handler(self, signum, frame):
        print("\n\n>>> Ctrl+C detected - Emergency stopping all arms! <<<")
        self.emergency_stop()
        print(">>> Emergency stop complete. Exiting program. <<<")
        sys.exit(0)

    def connect(self):
        print("--- Connecting to Arms ---")
        try:
            self.left_arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self.right_arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            if self.left_arm.rm_create_robot_arm(ip=self.LEFT_ARM_IP, port=self.PORT).id == -1:
                print("ERROR: Left arm connection failed.")
                return False
            print("Left arm connected.")
            if self.right_arm.rm_create_robot_arm(ip=self.RIGHT_ARM_IP, port=self.PORT).id == -1:
                print("ERROR: Right arm connection failed.")
                return False
            print("Right arm connected.")
            return True
        except Exception as e:
            print(f"ERROR: Failed to connect to arms. Is the library imported? Error: {e}")
            return False

    def _prepare_single_arm(self, arm_instance, arm_name):
        """Prepares a single arm for motion and sets hand speed."""
        if arm_instance is None:
            print(f"ERROR: {arm_name} arm is not connected.")
            return False
        
        print(f"Preparing {arm_name} arm...")
        if arm_instance.rm_set_arm_power(1) != 0:
            print(f"ERROR: Failed to power on {arm_name} arm.")
            return False
        time.sleep(2)
        for i in range(1, 7):
            if arm_instance.rm_set_joint_en_state(i, 1) != 0:
                print(f"ERROR: Failed to enable joint {i} on {arm_name} arm.")
                return False

        if arm_instance.rm_set_hand_speed(self.finger_speed) != 0:
            print(f"WARNING: Failed to set hand speed for {arm_name} arm.")
        else:
            print(f"  -> {arm_name.capitalize()} hand speed set to {self.finger_speed}.")

        print(f"  -> {arm_name.capitalize()} arm prepared.")
        return True

    def prepare_arms(self):
        if not self._prepare_single_arm(self.left_arm, "left"): return False
        if not self._prepare_single_arm(self.right_arm, "right"): return False
        return True

    def move_to_home_position(self, hand='both'):
        """
        Move specified arm(s) to predefined home position using joint angles.
        
        Args:
            hand (str): 'left', 'right', or 'both'
        """
        hands_to_move = ['left', 'right'] if hand == 'both' else [hand]
        
        for hand_name in hands_to_move:
            arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
            if arm_instance is None:
                print(f"ERROR: {hand_name} arm not connected.")
                continue
                
            home_joints = self.LEFT_HOME_JOINTS if hand_name == 'left' else self.RIGHT_HOME_JOINTS
            
            print(f"Moving {hand_name} arm to home position: {home_joints}")
            
            result = arm_instance.rm_movej(home_joints, 5, 0, 0, 0)  
            
            if result == 0:
                print(f"  -> {hand_name.capitalize()} arm reached home position.")
                time.sleep(0.5)
            else:
                print(f"ERROR: Failed to move {hand_name} arm to home position. Code: {result}")
                return False
        
        return True

    def debug_joint_positions(self, hand='both'):
        """Print current and target joint positions for debugging."""
        hands_to_check = ['left', 'right'] if hand == 'both' else [hand]
        
        for hand_name in hands_to_check:
            arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
            if arm_instance is None:
                print(f"ERROR: {hand_name} arm not connected.")
                continue
                
            home_joints = self.LEFT_HOME_JOINTS if hand_name == 'left' else self.RIGHT_HOME_JOINTS
            
            ret, state = arm_instance.rm_get_current_arm_state()
            if ret == 0 and 'joint' in state:
                current = state['joint']
                print(f"\n{hand_name.upper()} ARM:")
                print(f"  Current joints: {current}")
                print(f"  Target joints:  {home_joints}")
                
                diffs = [abs(home_joints[i] - current[i]) for i in range(len(current))]
                print(f"  Differences:    {[f'{d:.2f}' for d in diffs]}")
                print(f"  Total movement: {sum(diffs):.2f}°")
            else:
                print(f"ERROR: Could not read {hand_name} arm state")

    def _get_finger_assignments(self, hand_name: str):
        return self.piano.right_finger_assignments if hand_name == 'right' else self.piano.left_finger_assignments

    def _get_servo_values(self, hand_name: str, command: str):
        servo_map = self.piano.right_servo_values if hand_name == 'right' else self.piano.left_servo_values
        return servo_map.get(command, servo_map['open'])

    def _get_note_offsets(self, hand_name: str):
        return self.piano.right_note_offsets if hand_name == 'right' else self.piano.left_note_offsets

    def _get_arm_state(self, hand_name: str):
        arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
        if arm_instance is None:
            return None
            
        ret_state, state_dict = arm_instance.rm_get_current_arm_state()
        if ret_state == 0 and state_dict.get('pose'):
            pose = state_dict['pose']
            self.hands[hand_name]['current_pose'] = {'position': np.array(pose[:3]), 'orientation': np.array(pose[3:])}
            return pose
        return None

    def calibrate_piano_base_position(self, hand='both', wait_for_input=False):
        """
        Calibrate piano base position. If wait_for_input is False, assumes arms are already
        at home position with index fingers over calibration notes.
        
        Args:
            hand (str): 'left', 'right', or 'both'
            wait_for_input (bool): If True, wait for user to manually position arms
        """
        hands_to_calibrate = ['left', 'right'] if hand == 'both' else [hand]
        
        for hand_name in hands_to_calibrate:
            print(f"Calibrating {hand_name} hand...")
            if not self._get_arm_state(hand_name):
                print(f"ERROR: Cannot get {hand_name} arm state for calibration.")
                return False
            
            anchor_note = self.piano.left_calibration_anchor if hand_name == 'left' else self.piano.right_calibration_anchor
            
            if wait_for_input:
                print(f"Position the {hand_name} hand's INDEX finger over the '{anchor_note}' key.")
                input(f"Press Enter when {hand_name} hand is ready...")
                if not self._get_arm_state(hand_name):
                    print(f"ERROR: Cannot get {hand_name} arm state after positioning.")
                    return False
            else:
                print(f"  -> Using current position as calibration for '{anchor_note}'.")
            
            current_pose = self.hands[hand_name]['current_pose']
            self.hands[hand_name]['base_position'] = current_pose['position'].copy()
            self.hands[hand_name]['base_orientation'] = current_pose['orientation'].copy()
            self.hands[hand_name]['calibrated'] = True
            self.hands[hand_name]['current_wrist_target_note'] = anchor_note
            self.hands[hand_name]['last_played_note'] = None
            self.hands[hand_name]['is_hovering'] = False
            self._send_finger_command(hand_name, self._get_servo_values(hand_name, 'open'))
            print(f"{hand_name.capitalize()} hand calibrated. Anchor '{anchor_note}' at: {self.hands[hand_name]['base_position']}")
        
        return True

    def choose_hand_for_note(self, note: str, prefer_hand=None):
        if prefer_hand and self._can_hand_play_note(prefer_hand, note):
            return prefer_hand
        if note in self.piano.left_exclusive_range and self.hands['left']['calibrated']:
            return 'left'
        if note in self.piano.right_exclusive_range and self.hands['right']['calibrated']:
            return 'right'
        if note in self.piano.overlap_notes:
            left_ok = self.hands['left']['calibrated']
            right_ok = self.hands['right']['calibrated']
            if left_ok and right_ok:
                # Simple heuristic: if left is in safe-lane, use right, and vice-versa
                if self.hands['left']['is_hovering']: return 'right'
                if self.hands['right']['is_hovering']: return 'left'
                # Default to closest if neither is in safe-lane
                return 'left' if self._calculate_movement_distance('left', note) <= self._calculate_movement_distance('right', note) else 'right'
            elif left_ok: return 'left'
            elif right_ok: return 'right'
        if self._can_hand_play_note('left', note): return 'left'
        if self._can_hand_play_note('right', note): return 'right'
        print(f"WARNING: Could not assign note '{note}' to any calibrated hand.")
        return None

    def _can_hand_play_note(self, hand_name: str, note: str):
        full_range = self.piano.left_hand_range if hand_name == 'left' else self.piano.right_hand_range
        return self.hands[hand_name]['calibrated'] and note in full_range

    def _calculate_movement_distance(self, hand_name: str, note: str):
        current_note = self.hands[hand_name]['current_wrist_target_note']
        if not current_note: return float('inf')
        
        # If in safe-lane, calculate distance from base position, not current pose
        if self.hands[hand_name]['is_hovering']:
            anchor_note = self.piano.left_calibration_anchor if hand_name == 'left' else self.piano.right_calibration_anchor
            current_note = anchor_note # Pretend we are at the base
            
        note_map = self._get_note_offsets(hand_name)
        return abs(note_map.get(note, 0) - note_map.get(current_note, 0))

    def move_wrist_to_note(self, hand_name: str, wrist_target_note: str, timestamp=None):
        """
        Move wrist to target note with simple collision avoidance.
        If target would cause collision, move other hand 10cm away first.
        """
        if not self.hands[hand_name]['calibrated']:
            print(f"ERROR: {hand_name.capitalize()} hand is not calibrated!")
            return False
        
        is_hovering = self.hands[hand_name]['is_hovering']
        
        # Check if arm is already at the target AND not in safe-lane
        if (wrist_target_note == self.hands[hand_name]['current_wrist_target_note'] 
            and not is_hovering):
            print(f"  -> {hand_name.capitalize()} hand already at '{wrist_target_note}'")
            return True

        # Calculate target position
        note_offsets = self._get_note_offsets(hand_name)
        if wrist_target_note not in note_offsets:
            print(f"ERROR: Target note '{wrist_target_note}' not in offset map for {hand_name} hand.")
            return False
        x_offset = note_offsets[wrist_target_note] * self.piano.key_spacing
        
        # We subtract the offset because -X is RIGHT (higher notes, higher offset)
        target_position = self.hands[hand_name]['base_position'].copy()
        target_position[0] -= x_offset
        
        other_hand = 'right' if hand_name == 'left' else 'left'

        if is_hovering:
            print(f"Moving {hand_name} wrist from safe-lane to target '{wrist_target_note}'...")
        else:
            print(f"Moving {hand_name} wrist to target '{wrist_target_note}'...")
        
        # === SIMPLE COLLISION CHECK ===
        # If target position would be too close to other hand, move other hand away
        if self.collision_check_enabled and self._check_collision_risk(hand_name, target_position):
            print(f"⚠️  Collision risk detected - moving {other_hand} hand 10cm away...")
            self.move_to_hover(other_hand, lateral_offset=0.10)  # 10cm
            time.sleep(0.3)

        # Open fingers before moving
        self._send_finger_command(hand_name, self._get_servo_values(hand_name, 'open'))
        
        # Move to target position
        if self._send_wrist_position_and_wait(hand_name, target_position):
            self.hands[hand_name]['current_wrist_target_note'] = wrist_target_note
            self.hands[hand_name]['last_played_note'] = None
            self.hands[hand_name]['is_hovering'] = False  # We've arrived at a playing position
            
            time.sleep(0.1) 
            return True
        return False

    def _check_collision_risk(self, moving_hand: str, target_position: np.ndarray):
        other_hand = 'right' if moving_hand == 'left' else 'left'
        if not self.hands[other_hand]['calibrated'] or not self.hands[other_hand]['current_pose']:
            return False
        other_position = self.hands[other_hand]['current_pose']['position']
        return np.linalg.norm(target_position - other_position) < self.COLLISION_AVOIDANCE_DISTANCE

    def move_to_hover(self, hand_name: str, lateral_offset=0.15):
        """
        Moves the specified hand LATERALLY (sideways) to a safe-lane to avoid collision.
        Uses LATERAL-ONLY motion. The 'is_hovering' state will be set to True.
        
        Args:
            hand_name (str): 'left' or 'right'
            lateral_offset (float): Lateral offset for safe lane (default 0.15m / 15cm)
        """
        if not self.hands[hand_name]['calibrated']:
            print(f"ERROR: Cannot move {hand_name} hand, not calibrated.")
            return False
        
        # Don't move if already in the safe-lane
        if self.hands[hand_name]['is_hovering']:
            print(f"  -> {hand_name.capitalize()} hand already in safe-lane, skipping.")
            return True
            
        print(f"  -> Moving {hand_name} hand to safe-lane (LATERAL-ONLY motion)...")
        
        # Get the arm's current state
        current_pose_raw = self._get_arm_state(hand_name)
        if current_pose_raw is None:
            print(f"ERROR: Could not get {hand_name} arm pose for safe-lane move.")
            return False
        
        arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
        
        # Move LATERALLY (horizontal only) to hover lane
        lateral_position = self.hands[hand_name]['current_pose']['position'].copy()
        lateral_orientation = self.hands[hand_name]['current_pose']['orientation'].copy()
        
        if hand_name == 'left':
            # Left hand moves LEFT (+X)
            lateral_position[0] += lateral_offset
            print(f"     Moving LATERALLY left (+X) by {lateral_offset}m...")
        else:
            # Right hand moves RIGHT (-X)
            lateral_position[0] -= lateral_offset
            print(f"     Moving LATERALLY right (-X) by {lateral_offset}m...")
        
        # Check if lateral position would still cause collision
        other_hand = 'right' if hand_name == 'left' else 'left'
        if self.hands[other_hand]['calibrated'] and self.hands[other_hand]['current_pose']:
            other_pos = self.hands[other_hand]['current_pose']['position']
            hover_distance = np.linalg.norm(lateral_position - other_pos)
            
            if hover_distance < self.COLLISION_AVOIDANCE_DISTANCE:
                print(f"     WARNING: Lateral position still too close ({hover_distance:.3f}m)!")
                print(f"     Doubling lateral offset...")
                if hand_name == 'left':
                    lateral_position[0] += lateral_offset # Move further left (+X)
                else:
                    lateral_position[0] -= lateral_offset # Move further right (-X)
        
        lateral_pose = lateral_position.tolist() + lateral_orientation.tolist()
        result = arm_instance.rm_movel(lateral_pose, self.wrist_speed, 0, 0, 1)
        
        if result != 0:
            print(f"ERROR: {hand_name.capitalize()} lateral move failed. Code: {result}")
            return False
        
        time.sleep(0.3)  # Wait for lateral movement to complete
        self._get_arm_state(hand_name)  # Update final position
        
        # Mark as being in the safe lane
        self.hands[hand_name]['is_hovering'] = True
        
        # Verify we're actually safe now
        final_distance = self.get_arm_distance()
        if final_distance and final_distance < self.COLLISION_AVOIDANCE_DISTANCE:
            print(f"     WARNING: Still close after sidestep! Distance: {final_distance:.3f}m")
        else:
            print(f"  -> {hand_name.capitalize()} hand safely in lane. Distance: {final_distance:.3f}m")
        
        return True

    def play_key(self, note: str, duration: float = 0.5, prefer_hand=None, prefer_finger=None):
        """
        Enhanced play_key with predictive collision checking.
        """
        if prefer_finger not in ('index', 'middle', 'ring', 'pinky'):
            prefer_finger=None
        chosen_hand = self.choose_hand_for_note(note, prefer_hand)
        if not chosen_hand: 
            print(f"ERROR: No hand can play note '{note}'")
            return False

        note_map = self._get_note_offsets(chosen_hand)
        finger_assignments = self._get_finger_assignments(chosen_hand)
        target_note_val = note_map.get(note)
        
        if target_note_val is None:
            print(f"ERROR: Note '{note}' not found in {chosen_hand} hand's range")
            return False
        
        current_wrist_val_note = self.hands[chosen_hand]['current_wrist_target_note']
        if self.hands[chosen_hand]['is_hovering']:
            # If in safe-lane, base the calculation off the *base* note, not the current one
            current_wrist_val_note = self.piano.left_calibration_anchor if chosen_hand == 'left' else self.piano.right_calibration_anchor

        current_wrist_val = note_map.get(current_wrist_val_note, -1000)
        best_wrist_note, min_move_distance = None, float('inf')

        # Find the best wrist position to play the target note
        if prefer_finger is None:
            for finger, offset in finger_assignments.items():
                potential_wrist_val = target_note_val - offset
                potential_wrist_note = next((k for k, v in note_map.items() if v == potential_wrist_val), None)
            
                if potential_wrist_note:
                    move_dist = abs(potential_wrist_val - current_wrist_val)
                if move_dist <= min_move_distance:
                    min_move_distance, best_wrist_note = move_dist, potential_wrist_note
            if not best_wrist_note:
                print(f"ERROR: Could not calculate a valid wrist position for note '{note}'.")
                return False

        # Find the finger that corresponds to this best wrist position
            finger_to_use = next((f for f, o in finger_assignments.items() 
                              if o == target_note_val - note_map[best_wrist_note]), None)
            if not finger_to_use:
                print(f"ERROR: Logic error: Could not find finger for note '{note}' with wrist at '{best_wrist_note}'.")
                return False
        else:
            potential_wrist_val = target_note_val-finger_assignments[prefer_finger]
            potential_wrist_note = next((k for k, v in note_map.items() if v == potential_wrist_val), None)

            if potential_wrist_note:
                move_dist = abs(potential_wrist_val - current_wrist_val)
                if move_dist <= min_move_distance:
                    min_move_distance, best_wrist_note = move_dist, potential_wrist_note
            if not best_wrist_note:
                print(f"ERROR: Could not calculate a valid wrist position for note '{note}'.")
                return False
            finger_to_use=prefer_finger

        # Move wrist (this includes collision checks and return from safe-lane)
        if not self.move_wrist_to_note(chosen_hand, best_wrist_note):
            print(f"ERROR: Failed to move wrist for note '{note}'")
            return False

        # Handle consecutive key presses
        if self.hands[chosen_hand]['last_played_note'] == note:
            print(f"Consecutive key: {note}. Lifting for separation.")
            self._send_finger_command(chosen_hand, self._get_servo_values(chosen_hand, f"{finger_to_use}_lifted"))
            time.sleep(0.05)
        
        print(f"Playing {note} with {chosen_hand} {finger_to_use} finger (wrist @ {best_wrist_note}) for {duration:.2f}s")

        # Press the key down
        self._send_finger_command(chosen_hand, self._get_servo_values(chosen_hand, f"{finger_to_use}_press"))
        
        # Hold for duration
        time.sleep(duration)

        # Release the key
        self._send_finger_command(chosen_hand, self._get_servo_values(chosen_hand, 'open'))

        self.hands[chosen_hand]['last_played_note'] = note
        return True

    def return_to_base(self, hand='both'):
        """
        Returns arm(s) to their calibrated base position.
        """
        hands_to_move = ['left', 'right'] if hand == 'both' else [hand]
        for hand_name in hands_to_move:
            if not self.hands[hand_name]['calibrated']:
                print(f"Cannot return {hand_name} hand to base, not calibrated.")
                continue

            print(f"Returning {hand_name} hand to base position...")
            
            # Open fingers first
            self._send_finger_command(hand_name, self._get_servo_values(hand_name, 'open'))
            time.sleep(0.2)
            
            target_base_position = self.hands[hand_name]['base_position']
            is_hovering = self.hands[hand_name]['is_hovering']

            if is_hovering:
                print(f"  -> {hand_name.capitalize()} hand moving from safe-lane to base...")
            else:
                print(f"  -> {hand_name.capitalize()} hand moving laterally to base...")
            
            # Check for collision on the way back to base
            other_hand = 'right' if hand_name == 'left' else 'left'
            if self._check_collision_risk(hand_name, target_base_position):
                print(f"WARNING: Returning to base would collide with {other_hand}!")
                if not self.hands[other_hand]['is_hovering']:
                    print(f"  -> Moving {other_hand} hand to safe-lane for safety.")
                    self.move_to_hover(other_hand)
                    time.sleep(0.5)

            if not self._send_wrist_position_and_wait(hand_name, target_base_position):
                print(f"ERROR: {hand_name.capitalize()} hand failed to return to base.")
                continue

            # Set final state
            self.hands[hand_name]['current_wrist_target_note'] = self.piano.left_calibration_anchor if hand_name == 'left' else self.piano.right_calibration_anchor
            self.hands[hand_name]['last_played_note'] = None
            self.hands[hand_name]['is_hovering'] = False  # Clear safe-lane state
            print(f"  -> {hand_name.capitalize()} hand is at base.")
            
        print(f"Finished returning {hand} hand(s) to base.")


    def _apply_emergency_separation(self):
        """
        Apply additional lateral separation when both hands are in safe-lane but still too close.
        Uses LATERAL-ONLY motion for safety.
        """
        print("    >>> EMERGENCY SEPARATION PROTOCOL (LATERAL-ONLY) <<<")
        
        for hand_name in ['left', 'right']:
            if not self.hands[hand_name]['is_hovering']:
                # If one hand isn't in a safe-lane, move it there first.
                print(f"    {hand_name} hand not in safe-lane, moving it there...")
                self.move_to_hover(hand_name, lateral_offset=0.10) # Use smaller offset for first move
                continue
                
            print(f"    Applying extra separation to {hand_name} hand...")
            
            current_pose_raw = self._get_arm_state(hand_name)
            if current_pose_raw is None:
                continue
            
            arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
            
            # Move LATERALLY (aggressive offset)
            lateral_position = self.hands[hand_name]['current_pose']['position'].copy()
            lateral_orientation = self.hands[hand_name]['current_pose']['orientation'].copy()
            
            emergency_offset = 0.10  # 10cm emergency offset
            
            if hand_name == 'left':
                # Left hand moves LEFT (+X)
                lateral_position[0] += emergency_offset
                print(f"       Moving LATERALLY left (+X) by {emergency_offset}m...")
            else:
                # Right hand moves RIGHT (-X)
                lateral_position[0] -= emergency_offset
                print(f"       Moving LATERALLY right (-X) by {emergency_offset}m...")
            
            lateral_pose = lateral_position.tolist() + lateral_orientation.tolist()
            result = arm_instance.rm_movel(lateral_pose, self.wrist_speed, 0, 0, 1)
            
            if result == 0:
                time.sleep(0.3)
                self._get_arm_state(hand_name)
                print(f"       {hand_name.capitalize()} emergency separation complete")
            else:
                print(f"       ERROR: {hand_name} lateral emergency move failed!")
        
        # Verify the separation worked
        time.sleep(0.5)
        final_distance = self.get_arm_distance()
        if final_distance:
            print(f"    >>> Final distance after emergency separation: {final_distance:.3f}m <<<")
            if final_distance < self.COLLISION_AVOIDANCE_DISTANCE:
                print(f"    >>> WARNING: STILL TOO CLOSE! Consider stopping! <<<")
            else:
                print(f"    >>> Emergency separation SUCCESSFUL <<<")

    def emergency_stop(self):
        print("--- EMERGENCY STOP ---")
        if self.left_arm:
            print("Emergency stopping LEFT arm...")
            self.left_arm.rm_set_arm_stop()
            time.sleep(0.1) # Give time for stop command
            self._send_finger_command('left', self._get_servo_values('left', 'open'))
            
        if self.right_arm:
            print("Emergency stopping RIGHT arm...")
            self.right_arm.rm_set_arm_stop()
            time.sleep(0.1) # Give time for stop command
            self._send_finger_command('right', self._get_servo_values('right', 'open'))
            
        time.sleep(0.5)
        print("Moving arms to safe base positions...")
        self.return_to_base('both')

    def _send_wrist_position_and_wait(self, hand_name: str, position: np.ndarray):
        hand = self.hands[hand_name]
        arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
        if arm_instance is None:
            print(f"ERROR: {hand_name} arm not connected.")
            return False
            
        target_pose = position.tolist() + hand['base_orientation'].tolist()
        result = arm_instance.rm_movel(target_pose, self.wrist_speed, 0, 0, 1)

        if result == 0:
            self._get_arm_state(hand_name)
            return True
        else:
            print(f"ERROR: {hand_name.capitalize()} hand move command failed. Code: {result}")
            return False

    def _send_finger_command(self, hand_name: str, servo_values):
        try:
            arm_instance = self.left_arm if hand_name == 'left' else self.right_arm
            if arm_instance is None:
                print(f"ERROR: {hand_name} arm not connected.")
                return
                
            result = arm_instance.rm_set_hand_follow_pos(servo_values, block=200)
            
            if result != 0:
                print(f"WARNING: {hand_name} hand finger command returned code: {result}")
            
        except Exception as e:
            print(f"ERROR: Sending {hand_name} hand finger command: {e}")

    def disconnect(self):
        print("--- Disconnecting Arms ---")
        if self.left_arm: 
            self._send_finger_command('left', self._get_servo_values('left', 'open'))
            self.left_arm.rm_delete_robot_arm()
        if self.right_arm: 
            self._send_finger_command('right', self._get_servo_values('right', 'open'))
            self.right_arm.rm_delete_robot_arm()
        
        try:
            RoboticArm.rm_destroy()
        except Exception as e:
            print(f"MOCK/ERROR: rm_destroy failed: {e}")
            
        print("All resources cleaned up.")
