#ifndef __SUB_NODE_H_
#define __SUB_NODE_H_

#include <rclcpp/rclcpp.hpp>

namespace control_sub {
class MySub : public rclcpp::Node {
private:
public:
  explicit MySub(const std::string &name);
};
} // namespace control_sub

#endif