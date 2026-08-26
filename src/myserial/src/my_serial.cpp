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

size_t MySer::send(const VideoMsg &msg) {
  if (!serial_ || !serial_->port() || !serial_->port()->is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("serial"), "Serial port not ready");
    return 0;
  }
  std::vector<uint8_t> buffer(sizeof(VideoMsg));
  std::memcpy(buffer.data(), &msg, sizeof(VideoMsg));
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

// 滑动窗口拆包：支持固定长度的 WheelMsg 和 VideoMsg 帧
// 线上帧格式:
//   [AA][BB][2字节pad][speed_x 4B][speed_y 4B][ang_z 4B][CC][3字节pad]
//   = sizeof(WheelMsg) = 20 字节
void MySer::parse_frames() {
  static constexpr uint8_t H1 = 0xAA, H2 = 0xBB, TAIL = 0xCC;
  static constexpr uint8_t VIDEO_H1 = 0x11, VIDEO_H2 = 0x22, VIDEO_TAIL = 0x33;
  static constexpr size_t WHEEL_LEN = sizeof(WheelMsg);
  static constexpr size_t WHEEL_TAIL_OFF = offsetof(WheelMsg, header2);
  static constexpr size_t VIDEO_LEN = sizeof(VideoMsg);
  static constexpr size_t VIDEO_TAIL_OFF = offsetof(VideoMsg, header2);

  size_t start = 0; // 窗口起点游标
  while (rx_buffer_.size() - start >= 2) {
    // ① 不是帧头：窗口滑 1 字节（丢掉这个坏字节，重新对齐）
    const bool is_wheel =
        rx_buffer_[start] == H1 && rx_buffer_[start + 1] == H2;
    const bool is_video =
        rx_buffer_[start] == VIDEO_H1 && rx_buffer_[start + 1] == VIDEO_H2;
    if (!is_wheel && !is_video) {
      ++start;
      continue;
    }
    // ② 帧头对了，但一帧还没攒够：留在缓冲里，等下一包拼上
    const size_t frame_len = is_wheel ? WHEEL_LEN : VIDEO_LEN;
    const size_t tail_offset = is_wheel ? WHEEL_TAIL_OFF : VIDEO_TAIL_OFF;
    const uint8_t expected_tail = is_wheel ? TAIL : VIDEO_TAIL;
    if (rx_buffer_.size() - start < frame_len) {
      break;
    }
    // ③ 帧头对但帧尾不对：这是藏在数据里的假帧头，滑 1 字节继续找
    if (rx_buffer_[start + tail_offset] != expected_tail) {
      ++start;
      continue;
    }
    // ④ 头尾都对的完整一帧：取出来交给回调
    if (recv_callback_) {
      recv_callback_(std::vector<uint8_t>(
          rx_buffer_.begin() + start, rx_buffer_.begin() + start + frame_len));
    }
    start += frame_len;
  }
  // 把已消费的字节删掉；半帧和未匹配的垃圾留在缓冲里等下包
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + start);
}

} // namespace ser
