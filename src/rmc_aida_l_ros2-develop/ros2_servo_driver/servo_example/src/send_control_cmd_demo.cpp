// Copyright RealMan
// License(BSD/GPL/...)
// Author: Starry

// 通过话题/servo_control/move控制头部舵机转动响应的角度  舵机1可设置范围400-600 舵机2可设置的范围200-800

#include "rclcpp/rclcpp.hpp"
#include "servo_interfaces/msg/servo_move.hpp"

// 自定义节点类
class SendControlCmd : public rclcpp::Node
{
public:
  SendControlCmd() : Node("send_control_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "send control servo cmd  demo!");
    // 创建舵机控制发布方
    publisher_ = this->create_publisher<servo_interfaces::msg::ServoMove>("/servo_control/move", 1);
  }

  void publish_msg()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    auto msg = servo_interfaces::msg::ServoMove();
    msg.servo_id = 2;     // 舵机ID
    msg.angle = 400;      // 舵机角度
    RCLCPP_INFO(this->get_logger(), "send msg: servo_id = %d,angle = %d", msg.servo_id, msg.angle);
    // 发布消息
    publisher_->publish(msg);

    msg.servo_id = 1;     // 舵机ID
    msg.angle = 500;      // 舵机角度
    RCLCPP_INFO(this->get_logger(), "send msg: servo_id = %d,angle = %d", msg.servo_id, msg.angle);
    // 发布消息
    publisher_->publish(msg);
  }

private:
  // 声明舵机控制发布器
  rclcpp::Publisher<servo_interfaces::msg::ServoMove>::SharedPtr publisher_; 
};

int main(int argc, char *argv[])
{
  // 初始化 ROS2 客户端
  rclcpp::init(argc, argv);
  // 调用spin函数，并传入节点对象指针
  auto node = std::make_shared<SendControlCmd>();
  node->publish_msg();
  // rclcpp::spin(node);
  // 释放资源
  rclcpp::shutdown();
  return 0;
}
