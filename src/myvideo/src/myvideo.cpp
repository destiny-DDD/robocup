#include "myvideo.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <thread>

namespace myvideo {
MyVideo::MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  template_scale_min_ = this->declare_parameter<double>("template_scale_min",
                                                        template_scale_min_);
  template_scale_max_ = this->declare_parameter<double>("template_scale_max",
                                                        template_scale_max_);
  template_scale_step_ = this->declare_parameter<double>("template_scale_step",
                                                         template_scale_step_);
  match_max_width_ =
      this->declare_parameter<int>("match_max_width", match_max_width_);
  tesseract_language_ = this->declare_parameter<std::string>(
      "tesseract_language", tesseract_language_);
  tesseract_psm_ =
      this->declare_parameter<int>("tesseract_psm", tesseract_psm_);
  show_window_ = this->declare_parameter<bool>("show_window", HasDisplay());

  LoadTemplates();

  cap.open(2);

  serial_->start_receive([this](const std::vector<uint8_t> &buffer) {
    if (buffer.size() != sizeof(ser::VideoMsg)) {
      return; // MySer 也可能收到其他类型的串口帧
    }

    ser::VideoMsg command;
    std::memcpy(&command, buffer.data(), sizeof(command));
    if (command.num < 1 || command.num > 3) {
      RCLCPP_WARN(this->get_logger(), "忽略无效的video模式: %u",
                  static_cast<unsigned>(command.num));
      return;
    }

    num.store(static_cast<int>(command.num),
              std::memory_order_relaxed); // 原子写入
    RCLCPP_INFO(this->get_logger(), "串口切换video模式: %d",
                num.load(std::memory_order_relaxed));
  });
}

MyVideo::~MyVideo() { cv::destroyAllWindows(); }

bool MyVideo::HasDisplay() {
  const char *const display = std::getenv("DISPLAY");
  return display != nullptr && *display != '\0';
}

void MyVideo::ShowWindow(const std::string &name, const cv::Mat &image) {
  if (!show_window_) {
    return;
  }
  cv::imshow(name, image);
  cv::waitKey(1);
}

/*
S 小：接近灰色、白色、黑色，颜色不明显；
S 大：颜色更纯、更鲜艳。
V 小：暗处、阴影、黑色；
V 大：明亮区域。
*/

bool MyVideo::run1() {
  if (!cap.read(image_origin) || image_origin.empty()) {
    return false;
  }

  cv::Mat hsv;
  cv::Mat mask; // 输出掩码，黑白图，白色为符合的地方
  cv::cvtColor(image_origin, hsv, cv::COLOR_BGR2HSV);

  struct ColorRange {
    const char *name;
    cv::Scalar lower;      // HSV 下限
    cv::Scalar upper;      // HSV 上限
    cv::Scalar draw_color; // BGR 绘图颜色
  };

  // OpenCV 的 HSV 色调范围为 0 到 179；红色跨过 0，需要合并两段范围。
  const std::array<ColorRange, 3> colors = {
      ColorRange{"RED", cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255),
                 cv::Scalar(0, 0, 255)},
      ColorRange{"BLUE", cv::Scalar(100, 120, 70), cv::Scalar(130, 255, 255),
                 cv::Scalar(255, 0, 0)},
      ColorRange{"YELLOW", cv::Scalar(20, 50, 50), cv::Scalar(35, 255, 255),
                 cv::Scalar(0, 255, 255)}};

  // 椭圆形
  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21));
  constexpr double kMinArea = 1000.0;
  constexpr double kMinRadius = 50.0;
  constexpr double kMinCircularity = 0.5;
  std::array<int, colors.size()> counts{};

  for (size_t color_index = 0; color_index < colors.size(); ++color_index) {
    const auto &color = colors[color_index];
    cv::inRange(hsv, color.lower, color.upper, mask);

    if (color_index == 0) {
      cv::Mat red_wrap;
      cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255),
                  red_wrap);
      cv::bitwise_or(mask, red_wrap, mask);
    }

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>>
        contours; // 可储存多个轮廓，轮廓里面有多个点
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < kMinArea) {
        continue;
      }

      const double perimeter =
          cv::arcLength(contour, true); // 计算轮廓周长，true代表闭合曲线
      if (perimeter <= 0.0) {
        continue;
      }
      const double circularity =
          4.0 * CV_PI * area /
          (perimeter * perimeter); // 圆形度 4π × 面积 / 周长²
      if (circularity < kMinCircularity) {
        continue;
      }

      cv::Point2f center;
      float radius = 0.0F;
      cv::minEnclosingCircle(contour, center, radius); // 计算最小外接圆
      if (radius < kMinRadius) {
        continue;
      }

      cv::circle(image_origin, center, static_cast<int>(radius),
                 color.draw_color, 2); // 画圆
      cv::putText(image_origin, color.name,
                  cv::Point(static_cast<int>(center.x - radius),
                            static_cast<int>(center.y - radius - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color.draw_color, 2);
      ++counts[color_index]; // 每检测到颜色，计数+1
    }
  }

  const std::string count_text = cv::format("RED: %d  BLUE: %d  YELLOW: %d",
                                            counts[0], counts[1], counts[2]);
  cv::putText(image_origin, count_text, cv::Point(20, 35),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  ShowWindow("img", image_origin);
  ShowWindow("image", mask);
  return true;
}

void MyVideo::LoadTemplates() {
  std::string image_dir;
  try {
    image_dir =
        ament_index_cpp::get_package_share_directory("myvideo"); // myvideo
    image_dir =
        (std::filesystem::path(image_dir) / "image").string(); // myvideo/image
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(), "找不到 myvideo 安装目录: %s",
                 error.what());
    return;
  }

  const std::array<const char *, 4> names = {"A", "B", "C", "D"};
  for (const char *name : names) {
    const auto path =
        (std::filesystem::path(image_dir) / (std::string(name) + ".png"))
            .string(); // myvideo/image/A.png
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE); // 灰度读取
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "模板加载失败: %s", path.c_str());
      continue;
    }

    templates_.push_back({name, image});
    RCLCPP_INFO(this->get_logger(), "已加载模板 %s (%dx%d)", name, image.cols,
                image.rows);
  }

  if (templates_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "没有可用的模板，模板匹配将被跳过");
  }
}

/*
cv::resize
INTER_NEAREST：最近邻，最快，但容易出现锯齿
INTER_LINEAR：线性插值，速度和质量平衡
INTER_CUBIC：质量较高，但速度较慢
INTER_AREA：缩小图像时通常效果较好
*/
bool MyVideo::FindBestTemplate(const cv::Mat &gray, MatchResult &best) const {
  bool found = false;

  // 处理图像
  const double image_scale =
      gray.cols > match_max_width_ // gray.cols为列数，即灰度图宽度
          ? static_cast<double>(match_max_width_) / gray.cols
          : 1.0;
  cv::Mat search_image;
  if (image_scale < 1.0) {
    cv::resize(gray, search_image, cv::Size(), image_scale, image_scale,
               cv::INTER_AREA);
  } else {
    search_image = gray;
  }

  for (const auto &templ : templates_) {
    // 处理模板，一个倍率再乘图像缩小的倍率
    for (double scale = template_scale_min_;
         scale <= template_scale_max_ + template_scale_step_ * 0.5;
         scale += template_scale_step_) {
      const double effective_scale = scale * image_scale;
      const cv::Size scaled_size( // lround为四舍五入
          std::max(1, static_cast<int>(
                          std::lround(templ.image.cols * effective_scale))),
          std::max(1, static_cast<int>(
                          std::lround(templ.image.rows * effective_scale))));
      if (search_image.cols < scaled_size.width ||
          search_image.rows < scaled_size.height) {
        continue;
      }

      cv::Mat scaled_template;
      cv::resize(templ.image, scaled_template, scaled_size, 0.0, 0.0,
                 cv::INTER_LINEAR);

      cv::Mat result;
      cv::matchTemplate(search_image, scaled_template, result,
                        cv::TM_CCOEFF_NORMED);

      double min_value = 0.0;
      double max_value = 0.0;
      cv::Point min_location;
      cv::Point max_location;
      cv::minMaxLoc(result, &min_value, &max_value, &min_location,
                    &max_location);

      // 用的 || 所以只要满足后面的条件就能进来
      if (!found || max_value > best.score) {
        found = true;
        best.name = templ.name;
        best.score = max_value;
        // 还原原倍率
        best.location = cv::Point(
            static_cast<int>(std::lround(max_location.x / image_scale)),
            static_cast<int>(std::lround(max_location.y / image_scale)));
        best.size = cv::Size(
            static_cast<int>(std::lround(scaled_size.width / image_scale)),
            static_cast<int>(std::lround(scaled_size.height / image_scale)));
      }
    }
  }

  return found;
}

bool MyVideo::run2() {
  if (!cap.read(image_origin) || image_origin.empty()) {
    return false;
  }

  // 矩形
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(27, 27));

  cv::Mat gray, binary, opened;
  cv::cvtColor(image_origin, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
  cv::morphologyEx(binary, opened, cv::MORPH_OPEN, kernel);

  ++match_frame_count_;
  if (match_frame_count_ >= kMatchInterval) {
    match_frame_count_ = 0;

    MatchResult best;
    if (FindBestTemplate(binary, best) && best.score >= kMatchThreshold) {
      if (best.name == candidate_name_) {
        ++candidate_count_;
      } else {
        candidate_name_ = best.name;
        candidate_count_ = 1;
      }

      if (candidate_count_ >= kStableFrames && confirmed_name_ != best.name) {
        confirmed_name_ = best.name;
        RCLCPP_INFO(this->get_logger(), "模板匹配确认: %s (score=%.3f)",
                    confirmed_name_.c_str(), best.score);
      }
    } else {
      candidate_name_.clear();
      candidate_count_ = 0;
      confirmed_name_.clear();
    }
  }

  ShowWindow("img", image_origin);
  return true;
}

bool MyVideo::InitTesseract() {
  if (tesseract_initialized_) {
    return tesseract_ != nullptr;
  }

  tesseract_initialized_ = true;
  tesseract_ = std::make_unique<tesseract::TessBaseAPI>();
  // 初始化tesseract，0为成功
  if (tesseract_->Init(nullptr, tesseract_language_.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Tesseract 初始化失败 (language=%s)，请安装对应的 traineddata",
                 tesseract_language_.c_str());
    tesseract_.reset();
    return false;
  }

  if (tesseract_psm_ < static_cast<int>(tesseract::PSM_OSD_ONLY) ||
      tesseract_psm_ > static_cast<int>(tesseract::PSM_RAW_LINE)) {
    RCLCPP_WARN(this->get_logger(), "无效的 tesseract_psm=%d，使用 PSM 10",
                tesseract_psm_);
    tesseract_psm_ = 10;
  }
  tesseract_->SetPageSegMode(
      static_cast<tesseract::PageSegMode>(tesseract_psm_));   // 设置模式
  tesseract_->SetVariable("user_defined_dpi", "600");         // 设置300dpi
  tesseract_->SetVariable("tessedit_char_whitelist", "ABCD"); // 设置只识别ABCD
  tesseract_->SetVariable("load_system_dawg", "F");
  tesseract_->SetVariable("load_freq_dawg",
                          "F"); // 关闭词典，更适合识别单个字母
  RCLCPP_INFO(this->get_logger(), "Tesseract 已初始化 (language=%s, psm=%d)",
              tesseract_language_.c_str(), tesseract_psm_);
  return true;
}

bool MyVideo::run3() {
  if (!cap.read(image_origin) || image_origin.empty()) {
    return false;
  }
  if (!InitTesseract()) {
    cv::putText(image_origin, "Tesseract unavailable", cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    ShowWindow("img", image_origin);
    return true;
  }

  cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7));
  cv::Mat gray;
  cv::cvtColor(image_origin, gray, cv::COLOR_BGR2GRAY);
  cv::Mat prepared;
  cv::resize(gray, prepared, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
  cv::threshold(prepared, prepared, 200, 255, cv::THRESH_BINARY);
  cv::morphologyEx(prepared, prepared, cv::MORPH_OPEN, kernel_open);
  cv::morphologyEx(prepared, prepared, cv::MORPH_CLOSE, kernel_close);

  tesseract_->SetImage(prepared.data, prepared.cols, prepared.rows, 1,
                       static_cast<int>(prepared.step)); // 提供图像
  char *raw_text = tesseract_->GetUTF8Text();            // 识别字母
  std::string text = raw_text != nullptr ? raw_text : "";
  delete[] raw_text;

  // 只保留目标字符，避免 OCR 偶尔返回标点、数字或小写字母。
  std::string normalized;
  for (const char character : text) {
    if (character == 'A' || character == 'B' || character == 'C' ||
        character == 'D') {
      normalized += character;
    }
  }
  if (normalized != last_ocr_text_) {
    last_ocr_text_ = normalized;
    if (normalized.empty()) {
      RCLCPP_INFO(this->get_logger(), "Tesseract 未识别到文本");
    } else {
      RCLCPP_INFO(this->get_logger(), "Tesseract: %s", normalized.c_str());
    }
  }

  // Tesseract 的字符坐标对应 prepared（放大后的图像），绘制到原图前需要缩回去。
  const double scale_x = static_cast<double>(prepared.cols) /
                         static_cast<double>(image_origin.cols);
  const double scale_y = static_cast<double>(prepared.rows) /
                         static_cast<double>(image_origin.rows);
  auto clamp_coordinate = [](int value, int upper_bound) {
    return std::max(0, std::min(value, upper_bound - 1));
  };
  std::unique_ptr<tesseract::ResultIterator> iterator(
      tesseract_->GetIterator()); // 结果的迭代器
  if (iterator != nullptr) {
    iterator->Begin();
    do {
      char *raw_symbol = iterator->GetUTF8Text(tesseract::RIL_SYMBOL);
      const std::string symbol = raw_symbol != nullptr ? raw_symbol : "";
      delete[] raw_symbol;

      const bool is_target =
          symbol.size() == 1 && (symbol[0] == 'A' || symbol[0] == 'B' ||
                                 symbol[0] == 'C' || symbol[0] == 'D');
      if (!is_target) {
        continue;
      }

      int left = 0;
      int top = 0;
      int right = 0;
      int bottom = 0;
      if (!iterator->BoundingBox(tesseract::RIL_SYMBOL, &left, &top, &right,
                                 &bottom)) {
        continue;
      }

      const int x1 = clamp_coordinate(
          static_cast<int>(std::lround(left / scale_x)), image_origin.cols);
      const int y1 = clamp_coordinate(
          static_cast<int>(std::lround(top / scale_y)), image_origin.rows);
      const int x2 = clamp_coordinate(
          static_cast<int>(std::lround(right / scale_x)), image_origin.cols);
      const int y2 = clamp_coordinate(
          static_cast<int>(std::lround(bottom / scale_y)), image_origin.rows);
      if (x2 > x1 && y2 > y1) {
        cv::rectangle(image_origin, cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(0, 0, 255), 2);
      }
    } while (iterator->Next(tesseract::RIL_SYMBOL));
  }

  const std::string display_text =
      normalized.empty() ? "(no text)" : normalized.substr(0, 120);
  cv::putText(image_origin, display_text, cv::Point(20, 35),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  ShowWindow("img", image_origin);
  ShowWindow("ocr", prepared);
  return true;
}

} // namespace myvideo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto video_tx = std::make_shared<ser::MySer>("/dev/ttyUSB0", 115200, 1);
  auto node = std::make_shared<myvideo::MyVideo>("myvideo", video_tx);
  auto fps_start = std::chrono::steady_clock::now();
  std::size_t frame_count = 0;
  while (rclcpp::ok()) {
    bool frame_processed = false;
    switch (node->num.load(std::memory_order_relaxed)) {
    case 1:
      frame_processed = node->run1();
      break;
    case 2:
      frame_processed = node->run2();
      break;
    case 3:
      frame_processed = node->run3();
      break;
    default:
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      break;
    }

    if (frame_processed) {
      ++frame_count;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(now - fps_start).count();
    if (elapsed >= 1.0) {
      RCLCPP_INFO(node->get_logger(), "处理帧率: %.2f FPS",
                  static_cast<double>(frame_count) / elapsed);
      frame_count = 0;
      fps_start = now;
    }
  }
  rclcpp::shutdown();
  return 0;
}
