#include "myvideo.hpp"

#include <algorithm>
#include <cctype>

namespace myvideo {
MyVideo::MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  cap.open(0);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 /dev/video0");
    return;
  }

  // 初始化 tesseract，语言用 eng（数字+英文字母），中文需要另下 chi_sim 数据
  if (tess_.Init(nullptr, "eng") != 0) {
    RCLCPP_ERROR(this->get_logger(), "tesseract 初始化失败，请检查 TESSDATA_PREFIX");
  } else {
    // 只认数字和大写字母，能大幅减少误识别；要小写再加 abcdefghijklmnopqrstuvwxyz
    tess_.SetVariable("tessedit_char_whitelist",
                      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ");
    // 7 = 单行文本模式，对这种场景又快又准
    tess_.SetPageSegMode(tesseract::PSM_SINGLE_LINE);
    tess_ready_ = true;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                   [this]() { TimeCallback(); });
}

MyVideo::~MyVideo() {
  if (tess_ready_) tess_.End();
  cv::destroyAllWindows();
}

std::string MyVideo::Recognize(const cv::Mat &roi) {
  cv::Mat gray;
  if (roi.channels() == 3)
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  else
    gray = roi;

  // 文字太小 tesseract 认不出，放大 2 倍再识别
  cv::Mat big;
  cv::resize(gray, big, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);

  tess_.SetImage(big.data, big.cols, big.rows, 1, big.step);
  tess_.Recognize(nullptr);
  char *out = tess_.GetUTF8Text();
  std::string text = out ? out : "";
  delete[] out;  // GetUTF8Text 返回的 char* 需要手动释放

  // 去掉首尾空白/换行，避免画到画面上出现怪字符
  auto is_space = [](unsigned char c) { return std::isspace(c); };
  text.erase(text.begin(),
             std::find_if(text.begin(), text.end(),
                          [](unsigned char c) { return !std::isspace(c); }));
  text.erase(std::find_if(text.rbegin(), text.rend(),
                          [](unsigned char c) { return !std::isspace(c); })
                 .base(),
             text.end());
  return text;
}

void MyVideo::TimeCallback() {
  cap.read(image_origin);
  if (image_origin.empty()) return;

  // 第一帧时确定识别区域：取画面中间 60%
  if (roi_.width == 0) {
    roi_ = cv::Rect(cv::Point(image_origin.cols / 5, image_origin.rows / 5),
                    cv::Point(image_origin.cols * 4 / 5,
                              image_origin.rows * 4 / 5));
  }

  // 隔 N 帧识别一次，tesseract 在树莓派上跑一次要几百毫秒到 2 秒
  if (tess_ready_ && ++frame_count_ % kOcrEveryNFrames == 0) {
    cv::Mat roi_img = image_origin(roi_);
    ocr_result_ = Recognize(roi_img);
    RCLCPP_INFO(this->get_logger(), "识别结果: [%s]", ocr_result_.c_str());
  }

  // 画出识别区域和结果
  cv::rectangle(image_origin, roi_, cv::Scalar(0, 255, 0), 2);
  if (!ocr_result_.empty()) {
    cv::putText(image_origin, ocr_result_, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
  }

  cv::imshow("img", image_origin);
  cv::waitKey(1);
}
} // namespace myvideo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto video_tx = std::make_shared<ser::MySer>("/dev/ttyUSB0", 115200, 0);
  auto node = std::make_shared<myvideo::MyVideo>("myvideo", video_tx);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
