#include "my_serial.hpp"

#include <rclcpp/rclcpp.hpp>

namespace control_ser {

MySer::MySer(const std::string &device, uint32_t baud_rate, size_t threads)
    : io_context_(threads) {
  serial_ =
      std::make_shared<drivers::serial_driver::SerialDriver>(io_context_);

  drivers::serial_driver::SerialPortConfig config(
      baud_rate,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE);

  try {
    serial_->init_port(device, config);
    serial_->port()->open();
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("serial"),
                 "Failed to open %s: %s", device.c_str(), ex.what());
    serial_.reset();
  }
}

drivers::serial_driver::SerialDriver *MySer::operator->() {
  return serial_.get();
}

} // namespace control_ser
