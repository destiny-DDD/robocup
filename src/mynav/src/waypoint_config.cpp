#include "mynav/waypoint_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace mynav {

WaypointConfig parse_waypoint_config(const std::string &file_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(file_path);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("无法读取航点文件 " + file_path + ": " + e.what());
  }

  WaypointConfig cfg;
  try {
    if (root["frame_id"]) {
      cfg.frame_id = root["frame_id"].as<std::string>();
    }
    if (root["resume_topic"]) {
      cfg.resume_topic = root["resume_topic"].as<std::string>();
    }
    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
      throw std::runtime_error("waypoints.yaml 缺少 waypoints 列表");
    }
    for (const auto &node : root["waypoints"]) {
      Waypoint wp;
      wp.name = node["name"] ? node["name"].as<std::string>() : "";
      wp.x = node["x"] ? node["x"].as<double>() : 0.0;
      wp.y = node["y"] ? node["y"].as<double>() : 0.0;
      wp.yaw = node["yaw"] ? node["yaw"].as<double>() : 0.0;
      cfg.waypoints.push_back(wp);
    }
    if (cfg.waypoints.empty()) {
      throw std::runtime_error("waypoints 列表为空");
    }
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("waypoints.yaml 解析失败: " +
                             std::string(e.what()));
  }
  return cfg;
}

} // namespace mynav
