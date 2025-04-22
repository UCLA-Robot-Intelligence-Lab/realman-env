#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from dual_rm_ros2_interfaces.msg import Liftheight, Liftspeed, Liftstate
import keyboard  # pip install keyboard
import threading
import time

# ros2 topic pub /right_arm_controller/rm_driver/set_lift_speed_cmd ros2_rm_robot/dual_rm_ros2_interfaces/rm_ros_interfaces/msg/Liftspeed "{speed: 30.0}"
# ros2 topic pub /right_arm_controller/rm_driver/set_lift_height_cmd ros2_rm_robot/dual_rm_ros2_interfaces/rm_ros_interfaces/msg/Liftheight "{height: 1500, speed: 30, block: false}"
# ros2 topic list

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_keyboard_controller')

        # Publishers to control right arm lift
        self.height_pub = self.create_publisher(Liftheight, '/right_arm_controller/rm_driver/set_lift_height_cmd', 10)
        self.speed_pub = self.create_publisher(Liftspeed, '/right_arm_controller/rm_driver/set_lift_speed_cmd', 10)

        # Subscribers to get result and state
        self.create_subscription(Bool, '/left_arm_controller/rm_driver/set_lift_height_result', self.height_result_callback, 10)
        self.create_subscription(Bool, '/left_arm_controller/rm_driver/set_lift_speed_result', self.speed_result_callback, 10)
        self.create_subscription(Empty, '/left_arm_controller/rm_driver/get_lift_state_cmd', self.request_state_callback, 10)

        self.current_height = 0.0
        self.height_step = 20
        self.min_height = 0.0
        self.max_height = 2600

        self.current_speed = 0
        self.set_lift_speed(self.current_speed)

        # Keyboard Thread
        self.keyboard_thread = threading.Thread(target=self.listen_keyboard)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        self.get_logger().info("Use UP/DOWN arrows to control lift.")

    def height_result_callback(self, msg: Bool):
        self.get_logger().info(f"[Height Result] {'Success' if msg.data else 'Failed'}")

    def speed_result_callback(self, msg: Bool):
        self.get_logger().info(f"[Speed Result] {'Success' if msg.data else 'Failed'}")

    '''
    def request_state_callback(self, msg: Empty):
        pass
    '''

    def set_lift_height(self, height):
        msg = Liftheight()
        msg.height = height
        msg.speed = self.current_speed
        msg.block = False
        self.height_pub.publish(msg)
        self.get_logger().info(f"Published lift height: {height:.2f}")

    def set_lift_speed(self, speed):
        msg = Liftspeed()
        msg.speed = speed
        self.speed_pub.publish(msg)
        self.get_logger().info(f"Published lift speed: {speed:.2f}")

    def listen_keyboard(self):
        while True:
            # Up for increasing height, down for decreasing
            if keyboard.is_pressed('up'):
                self.current_speed = 30
                self.set_lift_speed(self.current_speed)
                time.sleep(0.3)
            elif keyboard.is_pressed('down'):
                self.current_speed = -30
                self.set_lift_speed(self.current_speed)
                time.sleep(0.3)
            else:
                self.current_speed = 0
                self.set_lift_speed(self.current_speed)
                time.sleep(0.3)
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
