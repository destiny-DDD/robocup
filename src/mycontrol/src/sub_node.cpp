#include "sub_node.hpp"

namespace control_sub {
MySub::MySub(const std::string &name,
             std::shared_ptr<control_ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  serial_->start_receive([this](const std::vector<uint8_t> &buffer) {
    if (buffer.size() < sizeof(control_ser::WheelMsg)) {
      RCLCPP_WARN(this->get_logger(), "收到不完整数据: %zu 字节",
                  buffer.size());
      return;
    }
    std::memcpy(&msg, buffer.data(), sizeof(msg));
    RCLCPP_INFO(this->get_logger(), "收到: x=%.2f y=%.2f z=%.2f", msg.speed_x,
                msg.speed_y, msg.ang_z);
  });
}
} // namespace control_sub
