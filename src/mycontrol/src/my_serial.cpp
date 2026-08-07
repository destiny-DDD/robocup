#include "my_serial.hpp"
#include <rclcpp/rclcpp.hpp>

namespace control_ser {

MySer::MySer(const std::string &device, uint32_t baud_rate, size_t threads)
    : io_context_(threads) {
  serial_ = std::make_shared<drivers::serial_driver::SerialDriver>(io_context_);

  drivers::serial_driver::SerialPortConfig config(
      baud_rate, drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);

  try {
    serial_->init_port(device, config);
    serial_->port()->open();
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("serial"), "Failed to open %s: %s",
                 device.c_str(), ex.what());
    serial_.reset();
  }
}

size_t MySer::send(const WheelMsg &msg) {
  if (!serial_ || !serial_->port() || !serial_->port()->is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("serial"), "Serial port not ready");
    return 0;
  }
  std::vector<uint8_t> buffer(sizeof(WheelMsg));
  std::memcpy(buffer.data(), &msg, sizeof(WheelMsg));
  return serial_->port()->send(buffer);
}

void MySer::start_receive(RecvCallback callback) {
  if (!serial_ || !serial_->port() || !serial_->port()->is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("serial"),
                 "Serial port not ready for receive");
    return;
  }
  recv_callback_ = std::move(callback);
  do_async_receive();
}

void MySer::do_async_receive() {
  serial_->port()->async_receive(
      [this](std::vector<uint8_t> &buffer, const size_t &bytes) {
        if (bytes > 0 && recv_callback_) {
          recv_callback_(
              std::vector<uint8_t>(buffer.begin(), buffer.begin() + bytes));
        }
        // 重新注册，持续接收
        do_async_receive();
      });
}

} // namespace control_ser
