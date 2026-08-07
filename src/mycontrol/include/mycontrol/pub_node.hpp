#ifndef __PUB_NODE_H_
#define __PUB_NODE_H_

#include "my_serial.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_pub {
class MyPub : public rclcpp::Node {
private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<control_ser::MySer> serial_;
  control_ser::WheelMsg data = {{0xAA, 0xBB}, 0.0f, 0.0f, 0.0f, 0xCC};

public:
  MyPub(const std::string &name, std::shared_ptr<control_ser::MySer> serial);
  void send_data_callback(const geometry_msgs::msg::Twist::SharedPtr msg_data);
};
} // namespace control_pub

#endif
