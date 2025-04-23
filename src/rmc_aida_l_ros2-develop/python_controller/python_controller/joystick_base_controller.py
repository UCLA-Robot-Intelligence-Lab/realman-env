import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep, State
from inputs import get_gamepad
import threading
import sys
import tty
import termios
import select
import time


class KeyboardController(Node):
    """Control the Woosh Robot using keyboard inputs with continuous movement."""

    # Action values
    ACTION_CANCEL = 0
    ACTION_EXECUTE = 1
    ACTION_PAUSE = 2
    ACTION_CONTINUE = 3

    # Movement modes
    MODE_STRAIGHT = 1  # Direct movement
    MODE_ROTATE = 2  # Rotation
    MODE_LATERAL = 3  # Lateral movement
    MODE_DIAGONAL = 4  # Diagonal movement

    def __init__(self, node_name="woosh_keyboard_controller"):
        super().__init__(node_name)

        # Movement parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.movement_distance = 0.2  # meters for each keypress
        self.rotation_angle = 0.2  # radians for each rotation keypress

        # Create the action client
        self.action_client = ActionClient(
            self,
            StepControl,
            "woosh_robot/ros/StepControl",
            callback_group=ReentrantCallbackGroup(),
        )

        # Track if we're currently processing a movement
        self.is_moving = False
        self.is_cancelling = False  # Add flag to prevent recursion
        self.movement_complete = threading.Event()
        self.movement_complete.set()  # Initially not moving

        # Track key states (pressed or released)
        self.key_states = {
            "w": False,  # Forward
            "s": False,  # Backward
            "a": False,  # Rotate left
            "d": False,  # Rotate right
            "q": False,  # Strafe left
            "e": False,  # Strafe right
        }

        # Flag to indicate if continuous movement thread should run
        self.running = True

        # Wait for the action server to be available
        self.get_logger().info("Waiting for robot action server...")
        self.action_server_ready = self.action_client.wait_for_server(timeout_sec=20.0)

        if not self.action_server_ready:
            self.get_logger().error("Action server not available after waiting!")
            return

        self.get_logger().info("Robot action server ready!")
        self.print_controls()

        # Start the continuous movement thread
        self.movement_thread = threading.Thread(target=self.continuous_movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()

    def print_controls(self):
        """Print the keyboard control instructions."""
        print("\n=== WOOSH ROBOT KEYBOARD CONTROLLER ===")
        print("Control the robot using the following keys:")
        print("  W - Hold to move forward")
        print("  S - Hold to move backward")
        print("  A - Hold to rotate counterclockwise")
        print("  D - Hold to rotate clockwise")
        print("  Q - Hold to strafe left")
        print("  E - Hold to strafe right")
        print("  + - Increase speed")
        print("  - - Decrease speed")
        print("  X - Stop/Cancel movement")
        print("  ESC/Ctrl+C - Quit")
        print("=====================================\n")

    def continuous_movement_loop(self):
        """Thread function that continuously sends movement commands based on key states."""
        while self.running:
            # Check which keys are pressed and send appropriate commands
            if any(self.key_states.values()):
                if self.key_states["w"]:
                    self.move_forward()
                elif self.key_states["s"]:
                    self.move_backward()
                elif self.key_states["a"]:
                    self.rotate_left()
                elif self.key_states["d"]:
                    self.rotate_right()
                elif self.key_states["q"]:
                    self.strafe_left()
                elif self.key_states["e"]:
                    self.strafe_right()

                # Wait for the movement to complete
                self.movement_complete.wait(timeout=0.05)

            # Small sleep to avoid CPU hogging
            time.sleep(0.05)

    def send_movement_command(self, mode, speed, value, angle=0.0, action=1):
        """Send a movement command to the robot."""
        if not self.action_server_ready:
            self.get_logger().error("Action server not available!")
            return

        # FIXED: Avoid recursive calls between send_movement_command and cancel_movement
        if self.is_moving and action != self.ACTION_CANCEL and not self.is_cancelling:
            # Set the cancellation flag to prevent recursion
            self.is_cancelling = True

            # Send a direct cancel command instead of calling cancel_movement()
            cancel_goal_msg = StepControl.Goal()
            cancel_step = StepControlStep()
            cancel_goal_msg.arg.action.value = self.ACTION_CANCEL
            cancel_step.mode.value = self.MODE_STRAIGHT
            cancel_step.speed = 0.0
            cancel_step.value = 0.0
            cancel_step.angle = 0.0
            cancel_goal_msg.arg.steps.append(cancel_step)

            # Send the cancellation goal directly
            self.action_client.send_goal_async(cancel_goal_msg)

            # Wait briefly for the cancellation to take effect
            time.sleep(0.1)

            # Reset the cancellation flag
            self.is_cancelling = False

            # Wait for previous movement to complete
            self.movement_complete.wait(timeout=0.2)

        # Mark that we're starting a movement
        self.is_moving = True
        self.movement_complete.clear()

        # Create the goal message - use EXACTLY the same approach as the working controller
        goal_msg = StepControl.Goal()
        step = StepControlStep()

        # Set the action type
        goal_msg.arg.action.value = action

        # Set the movement parameters
        step.mode.value = mode
        step.speed = speed
        step.value = value
        step.angle = angle
        goal_msg.arg.steps.append(step)

        # Only log for movement commands (not cancellations)
        if action == self.ACTION_EXECUTE:
            self.get_logger().info(
                f"Moving - Mode: {mode}, Speed: {speed:.2f}, Value: {value:.2f}"
            )

        # Send the goal
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback from the robot during movement."""
        feedback = feedback_msg.feedback.fb
        self.get_logger().debug(
            f"Movement status - State: {feedback.state.value}, "
            f"Code: {feedback.code}, Message: {feedback.msg}"
        )

        # If we get a completion or failure status, mark movement as complete
        if (
            feedback.state.value == State.K_ROS_SUCCESS
            or feedback.state.value == State.K_ROS_CANCEL
            or feedback.state.value == State.K_ROS_FAILURE
        ):
            self.is_moving = False
            self.movement_complete.set()

    def goal_response_callback(self, future):
        """Process the response from sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.is_moving = False
            self.movement_complete.set()
            return

        self.get_logger().debug("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Process the final result of the movement."""
        result = future.result().result
        status = result.ret.state.value

        if status == State.K_ROS_SUCCESS:
            self.get_logger().debug("Movement completed successfully")
        elif status == State.K_ROS_CANCEL:
            self.get_logger().debug("Movement was cancelled")
        elif status == State.K_ROS_FAILURE:
            self.get_logger().error("Movement failed")
        else:
            self.get_logger().warning(f"Unknown result status: {status}")

        # Mark that we're done moving
        self.is_moving = False
        self.movement_complete.set()

    def move_forward(self):
        """Move the robot forward."""
        self.send_movement_command(
            self.MODE_STRAIGHT, self.linear_speed, self.movement_distance
        )

    def move_backward(self):
        """Move the robot backward."""
        self.send_movement_command(
            self.MODE_STRAIGHT, self.linear_speed, -self.movement_distance
        )

    def rotate_left(self):
        """Rotate the robot counterclockwise."""
        self.send_movement_command(
            self.MODE_ROTATE, self.angular_speed, self.rotation_angle
        )

    def rotate_right(self):
        """Rotate the robot clockwise."""
        self.send_movement_command(
            self.MODE_ROTATE, self.angular_speed, -self.rotation_angle
        )

    def strafe_left(self):
        """Move the robot laterally to the left."""
        self.send_movement_command(
            self.MODE_LATERAL, self.linear_speed, self.movement_distance
        )

    def strafe_right(self):
        """Move the robot laterally to the right."""
        self.send_movement_command(
            self.MODE_LATERAL, self.linear_speed, -self.movement_distance
        )

    def increase_speed(self):
        """Increase the movement speed."""
        self.linear_speed += 0.05
        self.angular_speed += 0.1
        self.get_logger().info(f"Speed increased to {self.linear_speed:.2f} m/s")

    def decrease_speed(self):
        """Decrease the movement speed."""
        self.linear_speed = max(0.05, self.linear_speed - 0.05)
        self.angular_speed = max(0.1, self.angular_speed - 0.1)
        self.get_logger().info(f"Speed decreased to {self.linear_speed:.2f} m/s")

    def cancel_movement(self):
        """Cancel the current movement."""
        # Only send cancel command if we're actually moving
        if self.is_moving and not self.is_cancelling:
            self.send_movement_command(
                self.MODE_STRAIGHT, 0.0, 0.0, action=self.ACTION_CANCEL
            )

    def stop(self):
        """Stop the continuous movement thread and clean up."""
        self.running = False
        # Wait for the thread to finish
        if self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
        # Cancel any ongoing movement
        self.cancel_movement()

    def send_combined_movement(self, xvector, yvector):
        """Send a movement command with both X (strafe) and Y (forward) components."""
        if not self.action_server_ready:
            self.get_logger().error("Action server not ready.")
            return

        magnitude = (xvector**2 + yvector**2) ** 0.5
        if magnitude > 1.0:
            xvector /= magnitude
            yvector /= magnitude

        self.is_moving = True
        self.movement_complete.clear()

        goal_msg = StepControl.Goal()
        step = StepControlStep()
        step.mode.value = self.MODE_DIAGONAL
        step.speed = self.linear_speed
        step.value = magnitude * self.movement_distance
        step.angle = 0.0
        goal_msg.arg.action.value = self.ACTION_EXECUTE
        step.vector.x = xvector
        step.vector.y = yvector
        goal_msg.arg.steps.append(step)

        self.get_logger().info(
            f"Joystick Move → x: {xvector:.2f}, y: {yvector:.2f}, mag: {magnitude:.2f}"
        )
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)


def get_key_nonblocking(timeout=0.1):
    """Get a single keypress from the terminal without blocking."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


# New Code added, will eventually need to clean up the old code, trying to get a base working version first


def normalize_joystick(value, deadzone=5000):
    """Normalize the joystick input."""
    MAX = 32767
    MIN = -32768

    if abs(value) < deadzone:
        return 0.0
    return float(value) / MAX if value > 0 else float(value) / -MIN


def main(args=None):
    """Main function for the keyboard controller."""
    rclpy.init(args=args)
    controller = KeyboardController()

    # Check if the controller is ready
    if not controller.action_server_ready:
        print("\nERROR: Cannot connect to robot action server.")
        print("Make sure the robot control agent is running.")
        controller.destroy_node()
        rclpy.shutdown()
        return

    # Create a thread for ROS spinning
    spin_thread = threading.Thread(target=lambda: rclpy.spin(controller))
    spin_thread.daemon = True
    spin_thread.start()

    xvector, yvector, prev_x, prev_y = 0.0, 0.0, 0.0, 0.0
    try:
        while True:
            events = get_gamepad()
            for event in events:
                if event.ev_type == "Absolute":
                    if event.code == "ABS_X":
                        xvector = normalize_joystick(event.state)
                    elif event.code == "ABS_Y":
                        yvector = normalize_joystick(
                            event.state
                        )  # invert Y axis (up = +1)

            # Only send movement if there's change
            if (xvector, yvector) != (prev_x, prev_y):
                prev_x, prev_y = xvector, yvector

                if abs(xvector) < 1e-2 and abs(yvector) < 1e-2:
                    controller.cancel_movement()
                    continue

                # Decide direction based on both x and y together
                controller.send_combined_movement(xvector, yvector)

            time.sleep(0.05)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.stop()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
