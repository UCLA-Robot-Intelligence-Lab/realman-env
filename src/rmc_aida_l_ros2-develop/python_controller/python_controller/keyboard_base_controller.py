import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep, State
import threading
import sys
import tty
import termios
import select

class KeyboardController(Node):
    """Control the Woosh Robot using keyboard inputs."""
   
    # Action values
    ACTION_CANCEL = 0
    ACTION_EXECUTE = 1
    ACTION_PAUSE = 2
    ACTION_CONTINUE = 3
   
    # Movement modes
    MODE_STRAIGHT = 1  # Direct movement
    MODE_ROTATE = 2    # Rotation
    MODE_LATERAL = 3   # Lateral movement
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
            'woosh_robot/ros/StepControl',
            callback_group=ReentrantCallbackGroup()
        )
       
        # Track if we're currently processing a movement
        self.is_moving = False
        self.movement_complete = threading.Event()
        self.movement_complete.set()  # Initially not moving
       
        # Wait for the action server to be available
        self.get_logger().info("Waiting for robot action server...")
        self.action_server_ready = self.action_client.wait_for_server(timeout_sec=20.0)
       
        if not self.action_server_ready:
            self.get_logger().error("Action server not available after waiting!")
            return
           
        self.get_logger().info("Robot action server ready!")
        self.print_controls()
   
    def print_controls(self):
        """Print the keyboard control instructions."""
        print("\n=== WOOSH ROBOT KEYBOARD CONTROLLER ===")
        print("Control the robot using the following keys:")
        print("  W - Move forward")
        print("  S - Move backward")
        print("  A - Rotate counterclockwise")
        print("  D - Rotate clockwise")
        print("  Q - Strafe left")
        print("  E - Strafe right")
        print("  + - Increase speed")
        print("  - - Decrease speed")
        print("  X - Stop/Cancel movement")
        print("  ESC/Ctrl+C - Quit")
        print("=====================================\n")
   
    def send_movement_command(self, mode, speed, value, angle=0.0, action=1):
        """Send a movement command to the robot."""
        if not self.action_server_ready:
            self.get_logger().error("Action server not available!")
            return
           
        if self.is_moving:
            # Cancel the current movement first
            self.cancel_movement()
            # Wait for the cancellation to complete
            self.movement_complete.wait(timeout=1.0)
       
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
       
        self.get_logger().info(
            f"Moving - Mode: {mode}, Speed: {speed:.2f}, Value: {value:.2f}"
        )
       
        # Send the goal
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
       
        send_goal_future.add_done_callback(self.goal_response_callback)
   
    def feedback_callback(self, feedback_msg):
        """Process feedback from the robot during movement."""
        feedback = feedback_msg.feedback.fb
        self.get_logger().info(
            f"Movement status update - State: {feedback.state.value}, "
            f"Code: {feedback.code}, Message: {feedback.msg}"
        )
       
        # If we get a completion or failure status, mark movement as complete
        if feedback.state.value == State.K_ROS_SUCCESS or \
           feedback.state.value == State.K_ROS_CANCEL or \
           feedback.state.value == State.K_ROS_FAILURE:
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
       
        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
   
    def result_callback(self, future):
        """Process the final result of the movement."""
        result = future.result().result
        status = result.ret.state.value
       
        if status == State.K_ROS_SUCCESS:
            self.get_logger().info("Movement completed successfully")
        elif status == State.K_ROS_CANCEL:
            self.get_logger().info("Movement was cancelled")
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
            self.MODE_STRAIGHT,
            self.linear_speed,
            self.movement_distance
        )
   
    def move_backward(self):
        """Move the robot backward."""
        self.send_movement_command(
            self.MODE_STRAIGHT,
            self.linear_speed,
            -self.movement_distance
        )
   
    def rotate_left(self):
        """Rotate the robot counterclockwise."""
        self.send_movement_command(
            self.MODE_ROTATE,
            self.angular_speed,
            self.rotation_angle
        )
   
    def rotate_right(self):
        """Rotate the robot clockwise."""
        self.send_movement_command(
            self.MODE_ROTATE,
            self.angular_speed,
            -self.rotation_angle
        )
   
    def strafe_left(self):
        """Move the robot laterally to the left."""
        self.send_movement_command(
            self.MODE_LATERAL,
            self.linear_speed,
            self.movement_distance
        )
   
    def strafe_right(self):
        """Move the robot laterally to the right."""
        self.send_movement_command(
            self.MODE_LATERAL,
            self.linear_speed,
            -self.movement_distance
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
        return self.send_movement_command(
            self.MODE_STRAIGHT, 0.0, 0.0, action=self.ACTION_CANCEL
        )


def get_key(timeout=0.1):
    """Get a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


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
   
    try:
        while True:
            key = get_key()
           
            # Check for quit keys
            if key == '\x03' or key == '\x1b':  # Ctrl+C or ESC
                break
               
            # Process the key
            if key == 'w':
                controller.move_forward()
            elif key == 's':
                controller.move_backward()
            elif key == 'a':
                controller.rotate_left()
            elif key == 'd':
                controller.rotate_right()
            elif key == 'q':
                controller.strafe_left()
            elif key == 'e':
                controller.strafe_right()
            elif key == '+':
                controller.increase_speed()
            elif key == '-':
                controller.decrease_speed()
            elif key == 'x':
                controller.cancel_movement()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        controller.cancel_movement()
        controller.destroy_node()
        rclpy.shutdown()
        print("\nKeyboard controller terminated")


if __name__ == '__main__':
    main()
