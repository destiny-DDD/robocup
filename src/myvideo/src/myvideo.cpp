#include "myvideo.hpp"

namespace myvideo {
MyVideo::MyVideo(const std::string &name, std::shared_ptr<ser::MySer> serial)
    : Node(name), serial_(std::move(serial)) {
  cap.open(0);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开摄像头 /dev/video0");
    return;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
                                   [this]() { TimeCallback(); });
}

void MyVideo::TimeCallback() {
  cap.read(image_origin);

  cv::imshow("img", image_origin);
  cv::waitKey(1);
}
} // namespace myvideo

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto video_tx = std::make_shared<ser::MySer>("/dev/ttyUSB0", 115200, 0);
  auto node = std::make_shared<myvideo::MyVideo>("myvideo",video_tx);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}