#include "myvideo.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <filesystem>

namespace myvideo {
MyVideo::MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  LoadTemplates();

  cap.open(0);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 /dev/video0");
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                   [this]() { TimeCallback(); });
}

MyVideo::~MyVideo() { cv::destroyAllWindows(); }

void MyVideo::LoadTemplates() {
  std::string image_dir;
  try {
    image_dir = ament_index_cpp::get_package_share_directory("myvideo");
    image_dir = (std::filesystem::path(image_dir) / "image").string();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(), "找不到 myvideo 安装目录: %s",
                 error.what());
    return;
  }

  const std::array<const char *, 4> names = {"A", "B", "C", "D"};
  for (const char *name : names) {
    const auto path = (std::filesystem::path(image_dir) /
                       (std::string(name) + ".png"))
                          .string();
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "模板加载失败: %s", path.c_str());
      continue;
    }

    templates_.push_back({name, image});
    RCLCPP_INFO(this->get_logger(), "已加载模板 %s (%dx%d)", name,
                image.cols, image.rows);
  }

  if (templates_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "没有可用的模板，模板匹配将被跳过");
  }
}

bool MyVideo::FindBestTemplate(const cv::Mat &gray,
                               MatchResult &best) const {
  bool found = false;

  for (const auto &templ : templates_) {
    if (gray.cols < templ.image.cols || gray.rows < templ.image.rows) {
      continue;
    }

    cv::Mat result;
    cv::matchTemplate(gray, templ.image, result, cv::TM_CCOEFF_NORMED);

    double min_value = 0.0;
    double max_value = 0.0;
    cv::Point min_location;
    cv::Point max_location;
    cv::minMaxLoc(result, &min_value, &max_value,
                  &min_location, &max_location);

    if (!found || max_value > best.score) {
      found = true;
      best.name = templ.name;
      best.score = max_value;
      best.location = max_location;
      best.size = templ.image.size();
    }
  }

  return found;
}

void MyVideo::TimeCallback() {
  if (!cap.read(image_origin) || image_origin.empty()) {
    return;
  }

  cv::Mat gray;
  cv::cvtColor(image_origin, gray, cv::COLOR_BGR2GRAY);

  MatchResult best;
  if (FindBestTemplate(gray, best) &&
      best.score >= kMatchThreshold) {
    // best.location 是整张图中的左上角坐标，不需要再加 ROI 偏移。
    const cv::Rect box(best.location, best.size);
    cv::rectangle(image_origin, box, cv::Scalar(0, 255, 0), 2);

    const std::string text = best.name + " " +
                             cv::format("%.2f", best.score);
    cv::putText(image_origin, text,
                best.location + cv::Point(0, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 0), 2);

    if (best.name == candidate_name_) {
      ++candidate_count_;
    } else {
      candidate_name_ = best.name;
      candidate_count_ = 1;
    }

    if (candidate_count_ >= kStableFrames &&
        confirmed_name_ != best.name) {
      confirmed_name_ = best.name;
      RCLCPP_INFO(this->get_logger(), "模板匹配确认: %s (score=%.3f)",
                  confirmed_name_.c_str(), best.score);
    }
  } else {
    candidate_name_.clear();
    candidate_count_ = 0;
    confirmed_name_.clear();
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
