#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

namespace myvideo {

class MyVideo : public rclcpp::Node {
public:
  MyVideo(const std::string &name);
  void TimeCallback();

private:
  cv::VideoCapture cap;
  cv::Mat image_origin;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace myvideo

#endif