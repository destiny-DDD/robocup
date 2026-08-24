#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "myserial/my_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>

namespace myvideo {
class MyVideo : public rclcpp::Node {
public:
  MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial);
  ~MyVideo();
  void TimeCallback();

private:
  std::string Recognize(const cv::Mat &roi);  // 对一块区域做 OCR，返回识别到的文字

  cv::VideoCapture cap;
  cv::Mat image_origin;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<ser::MySer> serial_;

  // ---- OCR ----
  tesseract::TessBaseAPI tess_;      // tesseract 引擎，构造时初始化一次
  bool tess_ready_ = false;          // Init 成功后才允许识别/End
  cv::Rect roi_;                     // 识别区域，默认取画面中间 60%（后续可加鼠标框选）
  int frame_count_ = 0;              // 帧计数，用来隔 N 帧才识别一次
  static constexpr int kOcrEveryNFrames = 20;  // 每 20 帧识别一次(约0.6s)，tesseract 慢，不能每帧跑
  std::string ocr_result_;           // 最近一次识别结果，画到画面上
};

} // namespace myvideo

#endif
