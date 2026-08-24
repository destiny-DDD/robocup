#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "myserial/my_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace myvideo {
class MyVideo : public rclcpp::Node {
public:
  MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial);
  ~MyVideo();
  void TimeCallback();

private:
  struct TemplateImage {
    std::string name;
    cv::Mat image;
  };

  struct MatchResult {
    std::string name;
    double score = -1.0;
    cv::Point location;
    cv::Size size;
  };

  void LoadTemplates();
  bool FindBestTemplate(const cv::Mat &gray, MatchResult &best) const;

  cv::VideoCapture cap;
  cv::Mat image_origin;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<ser::MySer> serial_;

  std::vector<TemplateImage> templates_;

  // TM_CCOEFF_NORMED 的分数越接近 1 越相似，实际阈值需按现场画面调整。
  static constexpr double kMatchThreshold = 0.80;
  static constexpr int kStableFrames = 3;
  std::string candidate_name_;
  int candidate_count_ = 0;
  std::string confirmed_name_;
};

} // namespace myvideo

#endif
