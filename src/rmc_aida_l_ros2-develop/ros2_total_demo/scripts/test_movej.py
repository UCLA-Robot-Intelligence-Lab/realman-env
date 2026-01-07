#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_total_demo.msg import ObjectPose
import time

class TestCmdJ(Node):
    def __init__(self):
        super().__init__('test_cmd_j')

        self.object_pose_pub = self.create_publisher(ObjectPose, 'object_pose_bottle', 10)
        self.agv_result_pub = self.create_publisher(String, 'goto_mark/result', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if not self.sent:
            # Simulate AGV result immediately so CatchNode continues
            agv_msg = String()
            agv_msg.data = "任务完成"
            self.agv_result_pub.publish(agv_msg)
            self.get_logger().info("Sent fake AGV result")

            # Send known object coordinates (you can change these)
            obj_msg = ObjectPose()
            obj_msg.x = 0.1
            obj_msg.y = 0.05
            obj_msg.z = 0.2
            self.object_pose_pub.publish(obj_msg)
            self.get_logger().info(f"Published object at ({obj_msg.x}, {obj_msg.y}, {obj_msg.z})")

            self.sent = True  # Only send once

def main(args=None):
    rclpy.init(args=args)
    node = TestCmdJ()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
