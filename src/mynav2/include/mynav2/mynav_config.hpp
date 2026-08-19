#ifndef __MYNAV_CONFIG_HPP_
#define __MYNAV_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

namespace mynav_config {

struct nav2_point {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct nav2_config {
  std::string frame_id = "map";
  std::string resume_topic = "/nav_resume";
  std::vector<nav2_point> waypoints;
};

nav2_config mynav2_config(const std::string &file_path);

} // namespace mynav_config

#endif