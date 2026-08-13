#include "mynav/waypoint_config.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace mynav {

class WaypointRunner : public rclcpp::Node {
public:
  WaypointRunner() : Node("waypoint_runner") {
    const std::string file = declare_parameter<std::string>("waypoint_file");
    try {
      cfg_ = parse_waypoint_config(file);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "加载航点配置失败: %s", e.what());
      ok_ = false;
      return;
    }

    client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    state_client_ =
        create_client<lifecycle_msgs::srv::GetState>("/bt_navigator/get_state");
    resume_sub_ = create_subscription<std_msgs::msg::Empty>(
        cfg_.resume_topic, rclcpp::QoS(10),
        [this](std_msgs::msg::Empty::ConstSharedPtr) { resume_signal(); });

    RCLCPP_INFO(get_logger(), "已加载 %zu 个航点 (frame=%s, resume=%s)",
                cfg_.waypoints.size(), cfg_.frame_id.c_str(),
                cfg_.resume_topic.c_str());
  }

  bool is_ok() const { return ok_; }

  bool run() {
    if (!ok_) {
      return false;
    }
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "/navigate_to_pose action server 不可用");
      return false;
    }

    // bt_navigator 是生命周期节点，action server 存在 ≠ 已激活，未激活时会拒绝
    // goal。 先等它进入 ACTIVE 再发第一个 goal，避免冷启动竞态导致首个 goal
    // 被拒。
    if (!wait_for_navigator_active()) {
      return false; // 等待 bt_navigator 激活超时，按失败处理
    }

    for (size_t i = 0; i < cfg_.waypoints.size(); ++i) {
      if (!rclcpp::ok()) {
        return false; // 被 Ctrl+C 中断
      }
      const auto &wp = cfg_.waypoints[i];
      RCLCPP_INFO(get_logger(), "[%zu/%zu] 前往航点 %s (%.2f, %.2f, yaw=%.2f)",
                  i + 1, cfg_.waypoints.size(), wp.name.c_str(), wp.x, wp.y,
                  wp.yaw);

      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = cfg_.frame_id;
      goal.pose.header.stamp = now();
      goal.pose.pose.position.x = wp.x;
      goal.pose.pose.position.y = wp.y;
      goal.pose.pose.orientation = yaw_to_quat(wp.yaw);

      auto goal_handle_future = client_->async_send_goal(
          goal, rclcpp_action::Client<NavigateToPose>::SendGoalOptions());
      while (rclcpp::ok() &&
             goal_handle_future.wait_for(std::chrono::milliseconds(100)) !=
                 std::future_status::ready) {
      }
      if (!rclcpp::ok()) {
        return false; // 被 Ctrl+C 中断，不再等服务器应答
      }
      const auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "航点 %s 目标被服务器拒绝，中止序列",
                     wp.name.c_str());
        return false;
      }

      auto result_future = client_->async_get_result(goal_handle);
      while (rclcpp::ok() && result_future.wait_for(std::chrono::milliseconds(
                                 100)) != std::future_status::ready) {
      }
      if (!rclcpp::ok()) {
        return false; // 被 Ctrl+C 中断：导航进行中，干净退出
      }
      const auto result = result_future.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        const char *code = (result.code == rclcpp_action::ResultCode::ABORTED)
                               ? "ABORTED"
                               : "CANCELED";
        RCLCPP_ERROR(get_logger(), "航点 %s 导航失败 (%s)，中止序列",
                     wp.name.c_str(), code);
        return false;
      }

      // 已到达：完全停稳，等待外部 resume 信号再继续
      RCLCPP_INFO(get_logger(), "已到达 %s，等待 %s 信号", wp.name.c_str(),
                  cfg_.resume_topic.c_str());
      wait_for_resume();
    }
    RCLCPP_INFO(get_logger(), "多点导航完成");
    return true;
  }

private:
  geometry_msgs::msg::Quaternion yaw_to_quat(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.z = std::sin(yaw / 2.0);
    return q;
  }

  void resume_signal() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      resume_received_ = true;
    }
    cv_.notify_all();
  }

  void wait_for_resume() {
    std::unique_lock<std::mutex> lock(mutex_);
    // 边到边等：只有"开始等待之后"到达的信号才算数（早到信号在此被清零忽略）。
    // 用 wait_for 轮询 rclcpp::ok()，保证 Ctrl+C 能干净退出。
    resume_received_ = false;
    while (!resume_received_ && rclcpp::ok()) {
      cv_.wait_for(lock, std::chrono::milliseconds(200));
    }
  }

  // bt_navigator 是生命周期节点：action server 存在 ≠ 已激活，
  // 未激活时会拒绝 goal（"Action server is inactive. Rejecting the goal."）。
  // 发第一个 goal 前先等它进入 ACTIVE，30s 超时按失败处理。
  bool wait_for_navigator_active() {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (state_client_->service_is_ready()) {
        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = state_client_->async_send_request(req);
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

  WaypointConfig cfg_;
  bool ok_ = true;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_sub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool resume_received_ = false;
};

} // namespace mynav

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mynav::WaypointRunner>();
  if (!node->is_ok()) {
    return 1; // 配置加载失败
  }
  std::thread spin_thread([node] { rclcpp::spin(node); });
  const bool completed = node->run();
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return completed ? 0 : 1; // 失败/被中止退出码 1，便于上层感知
}
