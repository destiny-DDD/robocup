#include "my_serial.hpp"
#include <cstddef> // offsetof
#include <rclcpp/rclcpp.hpp>

namespace ser {

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
        if (bytes > 0) {
          // 来多少攒多少，先把这次到的字节全塞进累积缓冲
          rx_buffer_.insert(rx_buffer_.end(), buffer.begin(),
                            buffer.begin() + bytes);
          // 再从缓冲里拆出完整帧
          parse_frames();
        }
        // 重新注册，持续接收
        do_async_receive();
      });
}

// 滑动窗口拆包：只适配固定 20 字节的 WheelMsg 帧
// 线上帧格式:
//   [AA][BB][2字节pad][speed_x 4B][speed_y 4B][ang_z 4B][CC][3字节pad]
//   = sizeof(WheelMsg) = 20 字节
void MySer::parse_frames() {
  static constexpr uint8_t H1 = 0xAA, H2 = 0xBB, TAIL = 0xCC;
  static constexpr size_t LEN = sizeof(WheelMsg);                // 20
  static constexpr size_t TAIL_OFF = offsetof(WheelMsg, header2); // 16

  size_t start = 0; // 窗口起点游标
  while (rx_buffer_.size() - start >= 2) {
    // ① 不是帧头：窗口滑 1 字节（丢掉这个坏字节，重新对齐）
    if (rx_buffer_[start] != H1 || rx_buffer_[start + 1] != H2) {
      ++start;
      continue;
    }
    // ② 帧头对了，但一帧还没攒够：留在缓冲里，等下一包拼上
    if (rx_buffer_.size() - start < LEN) {
      break;
    }
    // ③ 帧头对但帧尾不对：这是藏在数据里的假帧头，滑 1 字节继续找
    if (rx_buffer_[start + TAIL_OFF] != TAIL) {
      ++start;
      continue;
    }
    // ④ 头尾都对的完整一帧：取出来交给回调
    if (recv_callback_) {
      recv_callback_(std::vector<uint8_t>(
          rx_buffer_.begin() + start, rx_buffer_.begin() + start + LEN));
    }
    start += LEN; // 消费掉这一帧，继续看后面还有没有帧
  }
  // 把已消费的字节删掉；半帧和未匹配的垃圾留在缓冲里等下包
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + start);
}

} // namespace ser
