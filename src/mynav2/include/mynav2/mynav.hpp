#ifndef __MYNAV_HPP_
#define __MYNAV_HPP_

#include "mynav_config.hpp"
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>

namespace mynav {
class MyNav : public rclcpp::Node {
public:
  MyNav(const std::string &name);
  bool run();
  // bt_navigator 是生命周期节点：action server 存在 ≠ 已激活，
  // 未激活时会拒绝 goal（"Action server is inactive. Rejecting the goal."）。
  // 发第一个 goal 前先等它进入 ACTIVE，30s 超时按失败处理。
  bool wait_for_navigator_active();
  geometry_msgs::msg::Quaternion yaw_to_q(double yaw);
  void resume_signal();

private:
  // 配置文件
  mynav_config::nav2_config config_;

  // ros2
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      action_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr server_client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_sub_;
};
} // namespace mynav

#endif