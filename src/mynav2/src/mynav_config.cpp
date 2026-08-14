#include "mynav_config.hpp"

namespace mynav_config {
nav2_config my_nav2_config(const std::string &file_path) {
  YAML::Node read;
  nav2_config points;
  read = YAML::LoadFile(file_path);
  points.frame_id = read["frame_id"].as<std::string>();
  points.resume_topic = read["resume_topic"].as<std::string>();
  for (const auto &node : read["waypoints"]) {
    nav2_point wp;
    wp.name = node["name"].as<std::string>();
    wp.x = node["x"].as<double>();
    wp.y = node["y"].as<double>();
    wp.yaw = node["yaw"].as<double>();
    points.waypoints.push_back(wp);
  }
  if (points.waypoints.empty())
    throw std::runtime_error("waypoints 列表为空");
  return points;
}
} // namespace mynav_config