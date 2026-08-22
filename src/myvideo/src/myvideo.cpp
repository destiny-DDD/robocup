#include "myvideo.hpp"
#include <opencv2/highgui.hpp>

namespace myvideo {
MyVideo::MyVideo(const std::string &name) : Node(name) {
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

int main(int argc, char **argv) {}