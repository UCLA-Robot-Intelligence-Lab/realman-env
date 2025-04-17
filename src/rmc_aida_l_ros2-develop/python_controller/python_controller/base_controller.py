import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from woosh_ros_msgs.action import StepControl
from woosh_ros_msgs.msg import StepControlStep, State

class WooshRobotController(Node):
    """A Python controller for the Woosh Robot."""
   
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
   
    def __init__(self, node_name="python_controller"):
        super().__init__(node_name)
       
        # Create the action client
        self.action_client = ActionClient(
            self,
            StepControl,
            'woosh_robot/ros/StepControl',
            callback_group=ReentrantCallbackGroup()
        )
       
        # Wait for the action server to be available
        self.get_logger().info("Waiting for action server...")
        self.action_server_ready = self.action_client.wait_for_server(timeout_sec=20.0)
        if not self.action_server_ready:
            self.get_logger().error("Action server not available after waiting!")
        else:
            self.get_logger().info("Action server ready!")
   
    def send_movement_command(self, mode, speed, value, angle=0.0, action=1):
        """
        Send a movement command to the robot.
       
        Args:
            mode (int): Movement mode (1=straight, 2=rotate, 3=lateral, 4=diagonal)
            speed (float): Speed value (m/s for linear, rad/s for rotation)
            value (float): Movement value (distance in meters or angle in radians)
            angle (float, optional): Angle for diagonal movement. Defaults to 0.0.
            action (int, optional): Action type. Defaults to 1 (execute).
       
        Returns:
            Future: A future representing the result of the action
        """
        if not self.action_server_ready:
            self.get_logger().error("Action server not available!")
            return None
       
        # Create the goal message
        goal_msg = StepControl.Goal()
        step = StepControlStep()
       
        # Set the action type (execute, cancel, pause, continue)
        goal_msg.arg.action.value = action
       
        # Set the movement parameters
        step.mode.value = mode
        step.speed = speed
        step.value = value
        step.angle = angle
        goal_msg.arg.steps.append(step)
       
        self.get_logger().info(
            f"Sending command - Mode: {mode}, Action: {action}, "
            f"Speed: {speed}, Value: {value}, Angle: {angle}"
        )
       
        # Set up feedback callback
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
       
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future
   
    def feedback_callback(self, feedback_msg):
        """Process feedback from the robot during movement."""
        feedback = feedback_msg.feedback.fb
        self.get_logger().info(
            f"Movement status update - State: {feedback.state.value}, "
            f"Code: {feedback.code}, Message: {feedback.msg}"
        )
   
    def goal_response_callback(self, future):
        """Process the response from sending a goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
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

    # Convenience methods for common movements
    def move_forward(self, distance, speed=0.2):
        """Move the robot forward by the specified distance (in meters)."""
        return self.send_movement_command(
            self.MODE_STRAIGHT, speed, distance
        )
   
    def rotate(self, angle, speed=0.2):
        """Rotate the robot by the specified angle (in radians)."""
        return self.send_movement_command(
            self.MODE_ROTATE, speed, angle
        )
   
    def move_lateral(self, distance, speed=0.2):
        """Move the robot laterally by the specified distance (in meters)."""
        return self.send_movement_command(
            self.MODE_LATERAL, speed, distance
        )
   
    def move_diagonal(self, distance, angle, speed=0.2):
        """Move the robot diagonally."""
        return self.send_movement_command(
            self.MODE_DIAGONAL, speed, distance, angle
        )
   
    def cancel_movement(self):
        """Cancel the current movement."""
        return self.send_movement_command(
            self.MODE_STRAIGHT, 0.0, 0.0, action=self.ACTION_CANCEL
        )


def main(args=None):
    """Example of using the controller."""
    rclpy.init(args=args)
    controller = WooshRobotController()
   
    # Example: Move forward 0.5 meters at 0.2 m/s
    controller.move_forward(0.5, 0.2)
   
    # Spin until the action completes
    rclpy.spin(controller)
   
    # Clean up
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
