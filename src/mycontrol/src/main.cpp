#include "pub_node.hpp"
#include "sub_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto pub_node = std::make_shared<control_pub::MyPub>("pub_node");
  auto sub_node = std::make_shared<control_sub::MySub>("sub_node");
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}