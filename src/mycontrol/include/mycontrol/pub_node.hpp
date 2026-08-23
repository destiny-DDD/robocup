#ifndef __PUB_NODE_H_
#define __PUB_NODE_H_

#include "myserial/my_serial.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace control_pub {
class MyPub : public rclcpp::Node {
public:
  MyPub(const std::string &name, std::shared_ptr<ser::MySer> serial);
  void send_data_callback(const geometry_msgs::msg::Twist::SharedPtr msg_data);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  std::shared_ptr<ser::MySer> serial_;
  ser::WheelMsg data = {{0xAA, 0xBB}, 0.0f, 0.0f, 0.0f, 0xCC};
};
} // namespace control_pub

#endif
