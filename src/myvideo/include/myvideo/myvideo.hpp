#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "myserial/my_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
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
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ser::MySer> serial_;

  // 帧率
  std::chrono::steady_clock::time_point fps_start_time_ =
      std::chrono::steady_clock::now();
  int fps_frame_count_ = 0;
  double fps_ = 0.0;
  int match_frame_count_ = 0;

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

  std::vector<TemplateImage> templates_; // 模板

  // 在多个缩放比例下匹配模板，以适应摄像头与目标距离的变化。
  double template_scale_min_ = 0.8;
  double template_scale_max_ = 1.2;
  double template_scale_step_ = 0.2;
  int match_max_width_ = 640;

  // TM_CCOEFF_NORMED 的分数越接近 1 越相似，实际阈值需按现场画面调整。
  static constexpr double kMatchThreshold = 0.80; // 阈值
  static constexpr int kMatchInterval = 5;
  static constexpr int kStableFrames = 3;
  std::string candidate_name_;
  int candidate_count_ = 0;
  std::string confirmed_name_;
};

} // namespace myvideo

#endif
