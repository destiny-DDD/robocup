#ifndef __MY_SERIAL_H_
#define __MY_SERIAL_H_

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <serial_driver/serial_driver.hpp>

namespace control_ser {

struct WheelMsg {
  uint8_t header[2] = {0xAA, 0xBB};
  float speed_x;
  float speed_y;
  float ang_z;
  uint8_t header2 = 0xCC;
};

class MySer {
public:
  /// @brief 接收回调：收到数据时调用，buffer 为收到的原始字节
  using RecvCallback = std::function<void(const std::vector<uint8_t> &buffer)>;

private:
  drivers::common::IoContext io_context_;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_;
  RecvCallback recv_callback_;

  void do_async_receive();

public:
  /// @param device 串口设备路径
  /// @param baud_rate 波特率
  /// @param threads IO 线程数，0=仅发送，1+=支持异步接收
  explicit MySer(const std::string &device, uint32_t baud_rate = 115200,
                 size_t threads = 0);

  /// @brief 阻塞发送 WheelMsg
  size_t send(const WheelMsg &msg);

  /// @brief 启动异步接收，收到数据时回调
  void start_receive(RecvCallback callback);
};

} // namespace control_ser

#endif
