#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "rclcpp/rclcpp.hpp"

namespace myvideo {

class MyVideo : public rclcpp::Node {
public:
  MyVideo(const std::string &name);

private:
};

} // namespace myvideo

#endif