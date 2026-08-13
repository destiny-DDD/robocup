#include "mynav/waypoint_config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace mynav
{

class WaypointConfigTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    path_ = (std::filesystem::temp_directory_path() / "mynav_wpt_test.yaml").string();
  }
  void TearDown() override
  {
    std::remove(path_.c_str());
  }
  void write(const std::string & content)
  {
    std::ofstream f(path_);
    ASSERT_TRUE(f.is_open());
    f << content;
  }
  std::string path_;
};

TEST_F(WaypointConfigTest, ParsesWaypoints)
{
  write(
      R"(
frame_id: "map"
resume_topic: "/nav_resume"
waypoints:
  - name: "point1"
    x: 1.0
    y: 0.0
    yaw: 0.0
  - name: "point2"
    x: 2.0
    y: 1.0
    yaw: 1.5708
)");
  WaypointConfig cfg = parse_waypoint_config(path_);
  ASSERT_EQ(cfg.waypoints.size(), 2u);
  EXPECT_EQ(cfg.frame_id, "map");
  EXPECT_EQ(cfg.resume_topic, "/nav_resume");
  EXPECT_EQ(cfg.waypoints[0].name, "point1");
  EXPECT_DOUBLE_EQ(cfg.waypoints[0].x, 1.0);
  EXPECT_DOUBLE_EQ(cfg.waypoints[0].y, 0.0);
  EXPECT_DOUBLE_EQ(cfg.waypoints[0].yaw, 0.0);
  EXPECT_EQ(cfg.waypoints[1].name, "point2");
  EXPECT_DOUBLE_EQ(cfg.waypoints[1].x, 2.0);
  EXPECT_DOUBLE_EQ(cfg.waypoints[1].yaw, 1.5708);
}

TEST_F(WaypointConfigTest, DefaultsAreFilled)
{
  write(R"(
waypoints:
  - name: "a"
    x: 0.5
    y: -1.0
)");
  WaypointConfig cfg = parse_waypoint_config(path_);
  EXPECT_EQ(cfg.frame_id, "map");
  EXPECT_EQ(cfg.resume_topic, "/nav_resume");
  EXPECT_EQ(cfg.waypoints[0].name, "a");
  EXPECT_DOUBLE_EQ(cfg.waypoints[0].yaw, 0.0);
}

TEST_F(WaypointConfigTest, ThrowsOnMissingWaypointsKey)
{
  write("frame_id: \"map\"\n");
  EXPECT_THROW(parse_waypoint_config(path_), std::runtime_error);
}

TEST_F(WaypointConfigTest, ThrowsOnEmptyWaypoints)
{
  write("waypoints: []\n");
  EXPECT_THROW(parse_waypoint_config(path_), std::runtime_error);
}

TEST_F(WaypointConfigTest, ThrowsOnMissingFile)
{
  EXPECT_THROW(parse_waypoint_config("/tmp/definitely_not_a_waypoint_file.yaml"),
    std::runtime_error);
}

TEST_F(WaypointConfigTest, ThrowsOnWrongTypes)
{
  write("waypoints:\n  - name: \"a\"\n    x: \"abc\"\n    y: 0.0\n");
  EXPECT_THROW(parse_waypoint_config(path_), std::runtime_error);
}

}  // namespace mynav
