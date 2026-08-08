#ifndef __SUB_NODE_H_
#define __SUB_NODE_H_

#include "my_serial.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace control_sub {

struct TFdata {
  float x_;
  float y_;
  float yaw_;
};

class MySub : public rclcpp::Node {
private:
  // 串口
  std::shared_ptr<control_ser::MySer> serial_;
  control_ser::WheelMsg msg;
  // timer
  rclcpp::Time last_time_;
  rclcpp::Time now_time_;
  float dt;
  rclcpp::TimerBase::SharedPtr timer_;
  // tf
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  geometry_msgs::msg::TransformStamped transform{}; // 结构体初始化
  TFdata change{0.0f, 0.0f, 0.0f};
  // 发布odom话题，用于ekf融合
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // 创建odom
  // 参数
  std::string odom_frame_;
  std::string child_frame_;
  bool tf_yn;

public:
  MySub(const std::string &name, std::shared_ptr<control_ser::MySer> serial);
  void TimeCallback();
  void Init();
};
} // namespace control_sub

#endif