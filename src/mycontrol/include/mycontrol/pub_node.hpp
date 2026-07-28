#ifndef __PUB_NODE_H_
#define __PUB_NODE_H_

#include <rclcpp/rclcpp.hpp>

namespace control_pub {
class MyPub : public rclcpp::Node {
private:
public:
  explicit MyPub(const std::string &name);
};
} // namespace control_pub

#endif
