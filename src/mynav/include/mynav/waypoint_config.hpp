#ifndef MYNAV__WAYPOINT_CONFIG_HPP_
#define MYNAV__WAYPOINT_CONFIG_HPP_

#include <string>
#include <vector>

namespace mynav {

struct Waypoint {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct WaypointConfig {
  std::string frame_id = "map";
  std::string resume_topic = "/nav_resume";
  std::vector<Waypoint> waypoints;
};

// 解析失败抛 std::runtime_error。
WaypointConfig parse_waypoint_config(const std::string &file_path);

} // namespace mynav

#endif // MYNAV__WAYPOINT_CONFIG_HPP_
