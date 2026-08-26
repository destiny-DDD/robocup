#include "myvideo.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <thread>

/*
S 小：接近灰色、白色、黑色，颜色不明显；
S 大：颜色更纯、更鲜艳。
V 小：暗处、阴影、黑色；
V 大：明亮区域。
*/

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

  LoadTemplates();

  cap.open(2);

  serial_->start_receive([this](const std::vector<uint8_t> &buffer) {
    if (buffer.size() != sizeof(ser::VideoMsg)) {
      return; // MySer 也可能收到其他类型的串口帧
    }

    ser::VideoMsg command;
    std::memcpy(&command, buffer.data(), sizeof(command));
    if (command.num != 1 && command.num != 2) {
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

bool MyVideo::run1() {
  if (!cap.read(image_origin) || image_origin.empty()) {
    return false;
  }

  cv::Mat hsv;
  cv::cvtColor(image_origin, hsv, cv::COLOR_BGR2HSV);

  struct ColorRange {
    const char *name;
    cv::Scalar lower;      // HSV 下限
    cv::Scalar upper;      // HSV 上限
    cv::Scalar draw_color; // BGR 绘图颜色
  };

  // OpenCV 的 HSV 色调范围为 0 到 179；红色跨过 0，需要合并两段范围。
  const std::array<ColorRange, 3> colors = {
      ColorRange{"RED", cv::Scalar(0, 100, 80), cv::Scalar(10, 255, 255),
                 cv::Scalar(0, 0, 255)},
      ColorRange{"BLUE", cv::Scalar(90, 100, 60), cv::Scalar(130, 255, 255),
                 cv::Scalar(255, 0, 0)},
      ColorRange{"YELLOW", cv::Scalar(18, 100, 80), cv::Scalar(38, 255, 255),
                 cv::Scalar(0, 255, 255)}};

  // 椭圆形
  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  constexpr double kMinArea = 150.0;
  constexpr double kMinRadius = 6.0;
  constexpr double kMinCircularity = 0.7;
  std::array<int, colors.size()> counts{};

  for (size_t color_index = 0; color_index < colors.size(); ++color_index) {
    const auto &color = colors[color_index];
    cv::Mat mask; // 输出掩码，黑白图，白色为符合的地方
    cv::inRange(hsv, color.lower, color.upper, mask);

    if (color_index == 0) {
      cv::Mat red_wrap;
      cv::inRange(hsv, cv::Scalar(170, 100, 80), cv::Scalar(179, 255, 255),
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
  cv::imshow("img", image_origin);
  cv::waitKey(1);
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

  cv::imshow("img", image_origin);
  cv::waitKey(1);
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
