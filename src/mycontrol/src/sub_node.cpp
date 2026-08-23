#include "sub_node.hpp"

namespace control_sub {
MySub::MySub(const std::string &name, std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  Init();
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  serial_->start_receive([this](const std::vector<uint8_t> &buffer) {
    if (buffer.size() < sizeof(ser::WheelMsg)) {
      RCLCPP_WARN(this->get_logger(), "收到不完整数据: %zu 字节",
                  buffer.size());
      return;
    }
    std::memcpy(&msg, buffer.data(), sizeof(msg)); // 把数据存到msg里
    RCLCPP_INFO(this->get_logger(), "收到: x=%.2f y=%.2f z=%.2f", msg.speed_x,
                msg.speed_y, msg.ang_z);
  });
  last_time_ = this->now();
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&MySub::TimeCallback, this));
}
void MySub::TimeCallback() {
  now_time_ = this->now();
  dt = (now_time_ - last_time_).seconds();
  if (dt <= 0.0 || dt > 0.5) // 25倍时间间隔
    dt = 0.02;
  last_time_ = now_time_;
  change.yaw_ += msg.ang_z * dt;
  change.x_ += (msg.speed_x * std::cos(change.yaw_) -
                msg.speed_y * std::sin(change.yaw_)) *
               dt;
  change.y_ += (msg.speed_x * std::sin(change.yaw_) +
                msg.speed_y * std::cos(change.yaw_)) *
               dt;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, change.yaw_);

  odom_msg.header.stamp = now_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = child_frame_;
  odom_msg.pose.pose.position.x = change.x_;
  odom_msg.pose.pose.position.y = change.y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_msg.twist.twist.linear.x = msg.speed_x;
  odom_msg.twist.twist.linear.y = msg.speed_y;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = msg.ang_z;

  odom_pub_->publish(odom_msg);

  if (tf_yn) {
    transform.header.stamp = now_time_;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = change.x_;
    transform.transform.translation.y = change.y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_->sendTransform(transform);
  }
}

void MySub::Init() {
  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  child_frame_ = this->declare_parameter("child_frame", "base_footprint");
  tf_yn = this->declare_parameter<bool>("tf_yn", true);
}
} // namespace control_sub
