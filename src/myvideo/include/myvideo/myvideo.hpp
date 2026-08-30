#ifndef __MYVIDEO_HPP_
#define __MYVIDEO_HPP_

#include "myserial/my_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tesseract/baseapi.h>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace myvideo {
class MyVideo : public rclcpp::Node {
public:
  std::atomic<int> num{3};
  MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial);
  ~MyVideo();
  void TimeCallback();
  bool run1();
  bool run2();
  bool run3();
  bool run4();

private:
  std::shared_ptr<ser::MySer> serial_;

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
  bool InitTesseract();
  bool InitHogSvm();
  bool NormalizeCharacter(const cv::Mat &character_mask,
                          cv::Mat &normalized) const;
  bool ExtractHog(const cv::Mat &normalized, cv::Mat &features) const;

  cv::VideoCapture cap;
  cv::Mat image_origin;

  // 有 X 显示时才弹窗口（OpenCV Qt 走 xcb 后端，需要 DISPLAY）。
  // 无显示环境（如 SSH 运行）跳过 imshow/waitKey，避免节点崩溃。
  bool show_window_ = false;
  static bool HasDisplay();
  void ShowWindow(const std::string &name, const cv::Mat &image);

  std::vector<TemplateImage> templates_; // 模板

  // 在多个缩放比例下匹配模板，以适应摄像头与目标距离的变化。
  double template_scale_min_ = 0.2;
  double template_scale_max_ = 1.8;
  double template_scale_step_ = 0.2;
  int match_max_width_ = 640;

  // TM_CCOEFF_NORMED 的分数越接近 1 越相似，实际阈值需按现场画面调整。
  static constexpr double kMatchThreshold = 0.80; // 阈值
  static constexpr int kMatchInterval = 5;
  int match_frame_count_ = 0;
  static constexpr int kStableFrames = 3;
  std::string candidate_name_;
  int candidate_count_ = 0;
  std::string confirmed_name_;

  std::unique_ptr<tesseract::TessBaseAPI> tesseract_;
  std::string tesseract_language_ = "eng";
  // PSM 10 适合当前每帧识别一个 A/B/C/D 字符的场景。
  int tesseract_psm_ = 10;
  bool tesseract_initialized_ = false;
  std::string last_ocr_text_;

  cv::HOGDescriptor hog_{cv::Size(64, 64), cv::Size(16, 16), cv::Size(8, 8),
                         cv::Size(8, 8), 9};
  cv::Ptr<cv::ml::SVM> hog_svm_;
  bool hog_svm_initialized_ = false;
  std::string svm_model_path_;
  int white_s_min_ = 60;
  int white_v_min_ = 150;
  int white_v_max_ = 255;
  int white_min_area_ = 1500;
  double white_min_fill_ = 0.35;
  int black_threshold_ = 180;
  int black_min_area_ = 40;
  int letter_stable_count_ = 0;
  std::string letter_candidate_;
  std::string letter_confirmed_;
  static constexpr int kLetterStableFrames = 3;
};

} // namespace myvideo

#endif
