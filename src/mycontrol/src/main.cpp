#include "pub_node.hpp"
#include "sub_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // auto serial_tx = std::make_shared<ser::MySer>("/tmp/vtx", 115200, 0);
  // auto serial_rx = std::make_shared<ser::MySer>("/tmp/vrx", 115200, 1);
  auto serial = std::make_shared<ser::MySer>("/dev/ttyAMA0", 115200, 1);
  auto pub_node = std::make_shared<control_pub::MyPub>("pub_node", serial);
  auto sub_node = std::make_shared<control_sub::MySub>("sub_node", serial);
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}