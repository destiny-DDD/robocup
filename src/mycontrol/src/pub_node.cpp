#include "pub_node.hpp"

namespace control_pub {
MyPub::MyPub(const std::string &name,
             std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->send_data_callback(msg);
      });
}

void MyPub::send_data_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg_data) {
  data.speed_x = msg_data->linear.x;
  data.speed_y = msg_data->linear.y;
  data.ang_z = msg_data->angular.z;
  RCLCPP_INFO(this->get_logger(), "发送: x=%.2f y=%.2f z=%.2f", data.speed_x,
              data.speed_y, data.ang_z);
  serial_->send(data);
}
} // namespace control_pub
