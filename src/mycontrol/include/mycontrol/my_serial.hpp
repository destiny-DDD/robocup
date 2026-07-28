#ifndef __MY_SERIAL_H_
#define __MY_SERIAL_H_

#include <serial_driver/serial_driver.hpp>

#include <memory>
#include <string>

namespace control_ser {

class MySer {
private:
  drivers::common::IoContext io_context_;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_;

public:
  explicit MySer(const std::string &device, uint32_t baud_rate = 115200,
                 size_t threads = 1);

  drivers::serial_driver::SerialDriver *operator->();
};

} // namespace control_ser

#endif
