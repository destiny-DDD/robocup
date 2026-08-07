#ifndef __SUB_NODE_H_
#define __SUB_NODE_H_

#include "my_serial.hpp"
#include <rclcpp/rclcpp.hpp>

namespace control_sub {
class MySub : public rclcpp::Node {
private:
  std::shared_ptr<control_ser::MySer> serial_;
  control_ser::WheelMsg msg;

public:
  MySub(const std::string &name, std::shared_ptr<control_ser::MySer> serial);
};
} // namespace control_sub

#endif