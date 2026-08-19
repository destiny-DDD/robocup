#include "mynav.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace mynav {
MyNav::MyNav(const std::string &name) : Node(name) {
  const std::string file =
      declare_parameter<std::string>("waypoint_file"); // 相当于传空字符
  config_ = mynav_config::my_nav2_config(file);

  action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "/navigate_to_pose");
  server_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
      "/bt_navigator/get_state");
  resume_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      config_.resume_topic, 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { resume_signal(); });
}

bool MyNav::run() {
  wait_for_navigator_active();
  for (size_t i = 0; i < config_.waypoints.size(); i++) {
    const auto &point_ = config_.waypoints[i];
    RCLCPP_INFO(get_logger(), "[%zu/%zu] 前往航点 %s (%.2f, %.2f, yaw=%.2f)",
                i + 1, config_.waypoints.size(), point_.name.c_str(), point_.x,
                point_.y, point_.yaw);

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = config_.frame_id;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = point_.x;
    goal.pose.pose.position.y = point_.y;
    goal.pose.pose.orientation = yaw_to_q(point_.yaw);

    // 异步发送动作，循环等待服务器接收
    auto goal_handle_future = action_client_->async_send_goal(
        goal, rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::
                  SendGoalOptions()); // 结构体，里面藏着三个回调
    // 等待服务端响应
    while (rclcpp::ok() &&
           goal_handle_future.wait_for(std::chrono::milliseconds(100)) !=
               std::future_status::ready) {
    }
    if (!rclcpp::ok()) {
      return false; // 被 Ctrl+C 中断，不再等服务器应答
    }

    const auto goal_handle = goal_handle_future.get();
    // 异步等待action结果
    auto result_future = action_client_->async_get_result(goal_handle);
    // 真正卡住程序直到动作完成的循环
    while (rclcpp::ok() && result_future.wait_for(std::chrono::milliseconds(
                               100)) != std::future_status::ready) {
    }
    if (!rclcpp::ok()) {
      return false; // 被 Ctrl+C 中断：导航进行中，干净退出
    }
    RCLCPP_INFO(get_logger(), "导航完成");
    wait_for_resume();
  }
  return true;
}

geometry_msgs::msg::Quaternion MyNav::yaw_to_q(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  return q;
}

void MyNav::resume_signal() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resume_received_ = true;
  }
  cv_.notify_all();
}

void MyNav::wait_for_resume() {
  std::unique_lock<std::mutex> lock(mutex_);
  // 边到边等：只有"开始等待之后"到达的信号才算数（早到信号在此被清零忽略）。
  // 用 wait_for 轮询 rclcpp::ok()，保证 Ctrl+C 能干净退出。
  resume_received_ = false;
  while (!resume_received_ && rclcpp::ok()) {
    cv_.wait_for(lock, std::chrono::milliseconds(200));
  }
}

bool MyNav::wait_for_navigator_active() {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (server_client_->service_is_ready()) {
      auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = server_client_->async_send_request(req);
      if (future.wait_for(std::chrono::milliseconds(500)) ==
              std::future_status::ready &&
          future.get()->current_state.id ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return true;
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }
  RCLCPP_ERROR(get_logger(), "等待 bt_navigator 激活超时");
  return false;
}

} // namespace mynav

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mynav::MyNav>("mynav2");
  std::thread spin_thread([node] { rclcpp::spin(node); });
  const bool completed = node->run();
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return completed ? 0 : 1; // 失败/被中止退出码 1，便于上层感知
}