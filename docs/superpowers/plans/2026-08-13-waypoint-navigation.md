# 多点导航（到点停车等信号）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 小车按 yaml 里的多个航点依次导航；每到达一个点完全停稳、等待外部话题信号后才继续；任一航点失败则中止整条序列。

**Architecture:** 在 `mynav` 包新增一个纯客户端 C++ 节点 `waypoint_runner`：启动时用 yaml-cpp 解析航点文件，循环调用 Nav2 标准的 `/navigate_to_pose` action 逐个发目标；每个目标成功后订阅 resume 话题（默认 `/nav_resume`，`std_msgs/Empty`）阻塞等待信号；失败（ABORTED/CANCELED）即中止序列。新增 `waypoint_run.launch.py` 复用现有 `mynav.launch.py`（完美定位导航栈）并启动本节点。不动 Nav2 内部、不动 `mynav.launch.py`。

**Tech Stack:** ROS 2 Jazzy (rclcpp / rclcpp_action / nav2_msgs / std_msgs / geometry_msgs)、yaml-cpp、ament_cmake、gtest。

## Global Constraints

- 包：`mynav`（现有 C++ ament_cmake 包）。节点语言 C++。
- 航点来源：yaml 文件，字段 `name`/`x`/`y`/`yaw`（yaw 缺省 0），`name` 用于日志与报错。
- 目标帧 `frame_id` 默认 `map`；resume 话题默认 `/nav_resume`，消息类型 `std_msgs/Empty`。
- resume 语义：**边到边等** —— 只在到达某点后才监听该点的信号；信号早到（还在路上）被忽略，不跳过停车。
- 失败处理：任一航点导航失败（ABORTED/CANCELED）→ **立即中止整条序列**，车停原地，不自动重试。
- 不能修改 `src/mynav/launch/mynav.launch.py`；`src/mynav/config/nav2_params.yaml` 不能改参数。
- 每个航点的 `name`、`x`、`y`、`yaw` 均可能缺省，缺省值分别是空串、0.0、0.0、0.0。
- `waypoints` 为空或缺 `waypoints` 键时，解析必须抛 `std::runtime_error`。

---

### Task 1: 航点配置解析库（TDD）

**Files:**
- Create: `src/mynav/include/mynav/waypoint_config.hpp`
- Create: `src/mynav/src/waypoint_config.cpp`
- Test: `src/mynav/test/test_waypoint_config.cpp`
- Modify: `src/mynav/package.xml`
- Modify: `src/mynav/CMakeLists.txt`

**Interfaces:**
- Produces: 纯逻辑库，后续 Task 2 直接复用：
  - `struct mynav::Waypoint { std::string name; double x; double y; double yaw; };`
  - `struct mynav::WaypointConfig { std::string frame_id; std::string resume_topic; std::vector<Waypoint> waypoints; };`
  - `mynav::WaypointConfig mynav::parse_waypoint_config(const std::string & file_path);` —— 解析失败抛 `std::runtime_error`。
- 注意：`frame_id` 与 `resume_topic` 在 yaml 里可缺省，缺省分别 `"map"` 与 `"/nav_resume"`。

- [ ] **Step 1: 给 package.xml 加依赖**

在 `src/mynav/package.xml` 的 `<depend>nav2_bringup</depend>` 之后、`<test_depend>` 之前插入：

```xml
  <depend>rclcpp_action</depend>
  <depend>nav2_msgs</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>yaml_cpp_vendor</depend>
```

并把测试依赖块改为：

```xml
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

- [ ] **Step 2: 写失败测试**（先建 `src/mynav/test/test_waypoint_config.cpp`）

```cpp
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
  write(R"(
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
```

- [ ] **Step 3: 加 CMake 测试目标**（`src/mynav/CMakeLists.txt`）

把 `find_package(ament_cmake_auto REQUIRED)` 与 `ament_auto_find_build_dependencies()` 之后的那段，替换/补充为：

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(waypoint_config src/waypoint_config.cpp)
```

`if(BUILD_TESTING)` 块内、`ament_lint_auto_find_test_dependencies()` 之前插入：

```cmake
  ament_add_gtest(test_waypoint_config test/test_waypoint_config.cpp)
  target_link_libraries(test_waypoint_config waypoint_config)
```

- [ ] **Step 4: 编译并跑测试，确认失败（红）**

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav
source install/setup.bash
ros2 run mynav test_waypoint_config
```

Expected: 编译报错 `fatal error: mynav/waypoint_config.hpp: No such file or directory`（红）。如果只是链接/运行失败也算红，总之**必须失败**。

- [ ] **Step 5: 实现解析库**（绿）

创建 `src/mynav/include/mynav/waypoint_config.hpp`：

```cpp
#ifndef MYNAV__WAYPOINT_CONFIG_HPP_
#define MYNAV__WAYPOINT_CONFIG_HPP_

#include <string>
#include <vector>

namespace mynav
{

struct Waypoint
{
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct WaypointConfig
{
  std::string frame_id = "map";
  std::string resume_topic = "/nav_resume";
  std::vector<Waypoint> waypoints;
};

// 解析失败抛 std::runtime_error。
WaypointConfig parse_waypoint_config(const std::string & file_path);

}  // namespace mynav

#endif  // MYNAV__WAYPOINT_CONFIG_HPP_
```

创建 `src/mynav/src/waypoint_config.cpp`：

```cpp
#include "mynav/waypoint_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace mynav
{

WaypointConfig parse_waypoint_config(const std::string & file_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(file_path);
  } catch (const YAML::Exception & e) {
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
    for (const auto & node : root["waypoints"]) {
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
  } catch (const YAML::Exception & e) {
    // 字段类型错误(如 x: "abc")会抛 YAML::BadConversion，统一转成 std::runtime_error
    throw std::runtime_error("waypoints.yaml 解析失败: " + std::string(e.what()));
  }
  return cfg;
}

}  // namespace mynav
```

- [ ] **Step 6: 编译并跑测试，确认通过（绿）**

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav
source install/setup.bash
ros2 run mynav test_waypoint_config
```

Expected: 5 个测试全部 PASS，退出码 0。

- [ ] **Step 7: 提交**

```bash
git add src/mynav/package.xml src/mynav/CMakeLists.txt src/mynav/include/mynav/waypoint_config.hpp src/mynav/src/waypoint_config.cpp src/mynav/test/test_waypoint_config.cpp
git commit -m "添加航点配置解析库及单元测试"
```

---

### Task 2: `waypoint_runner` 节点 + 示例配置

**Files:**
- Create: `src/mynav/src/waypoint_runner.cpp`
- Create: `src/mynav/config/waypoints.yaml`
- Modify: `src/mynav/CMakeLists.txt`（加可执行目标）

**Interfaces:**
- Consumes: `mynav::parse_waypoint_config` / `mynav::WaypointConfig`（Task 1 产出）。
- Consumes: `/navigate_to_pose`（`nav2_msgs::action::NavigateToPose`，bt_navigator 提供）。
- Produces: 可执行文件 `waypoint_runner`；参数 `waypoint_file`（字符串，必填）；订阅话题 `resume_topic`（来自配置，默认 `/nav_resume`）。
- 行为契约：按顺序逐个发目标；每个 `SUCCEEDED` 后打印"已到达 <name>"并阻塞等待 resume 消息；`ABORTED`/`CANCELED` 立即 return（中止序列）；全部完成打印"多点导航完成"；配置加载失败进程退出码 1。

- [ ] **Step 1: 写示例配置** `src/mynav/config/waypoints.yaml`

```yaml
# 多点导航航点列表 —— 按实际地图(room.pgm)调整 x/y/yaw
frame_id: "map"
resume_topic: "/nav_resume"
waypoints:
  - name: "point1"
    x: 1.0
    y: 0.0
    yaw: 0.0
  - name: "point2"
    x: 2.0
    y: 0.0
    yaw: 0.0
```

- [ ] **Step 2: 写节点** `src/mynav/src/waypoint_runner.cpp`

```cpp
#include "mynav/waypoint_config.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>

#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace mynav
{

class WaypointRunner : public rclcpp::Node
{
public:
  WaypointRunner() : Node("waypoint_runner")
  {
    const std::string file = declare_parameter<std::string>("waypoint_file");
    try {
      cfg_ = parse_waypoint_config(file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "加载航点配置失败: %s", e.what());
      ok_ = false;
      return;
    }

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    resume_sub_ = create_subscription<std_msgs::msg::Empty>(
      cfg_.resume_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::ConstSharedPtr) { resume_signal(); });

    RCLCPP_INFO(get_logger(), "已加载 %zu 个航点 (frame=%s, resume=%s)",
      cfg_.waypoints.size(), cfg_.frame_id.c_str(), cfg_.resume_topic.c_str());
  }

  bool is_ok() const { return ok_; }

  bool run()
  {
    if (!ok_) {
      return false;
    }
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "/navigate_to_pose action server 不可用");
      return false;
    }

    for (size_t i = 0; i < cfg_.waypoints.size(); ++i) {
      if (!rclcpp::ok()) {
        return false;  // 被 Ctrl+C 中断
      }
      const auto & wp = cfg_.waypoints[i];
      RCLCPP_INFO(get_logger(), "[%zu/%zu] 前往航点 %s (%.2f, %.2f, yaw=%.2f)",
        i + 1, cfg_.waypoints.size(), wp.name.c_str(), wp.x, wp.y, wp.yaw);

      NavigateToPose::Goal goal;
      goal.pose.header.frame_id = cfg_.frame_id;
      goal.pose.header.stamp = now();
      goal.pose.pose.position.x = wp.x;
      goal.pose.pose.position.y = wp.y;
      goal.pose.pose.orientation = yaw_to_quat(wp.yaw);

      const auto goal_handle_future =
        client_->async_send_goal(goal, rclcpp_action::Client<NavigateToPose>::SendGoalOptions());
      const auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "航点 %s 目标被服务器拒绝，中止序列", wp.name.c_str());
        return false;
      }

      const auto result = client_->async_get_result(goal_handle).get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        const char * code = (result.code == rclcpp_action::ResultCode::ABORTED) ?
          "ABORTED" : "CANCELED";
        RCLCPP_ERROR(get_logger(), "航点 %s 导航失败 (%s)，中止序列",
          wp.name.c_str(), code);
        return false;
      }

      // 已到达：完全停稳，等待外部 resume 信号再继续
      RCLCPP_INFO(get_logger(), "已到达 %s，等待 %s 信号",
        wp.name.c_str(), cfg_.resume_topic.c_str());
      wait_for_resume();
    }
    RCLCPP_INFO(get_logger(), "多点导航完成");
    return true;
  }

private:
  geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.z = std::sin(yaw / 2.0);
    return q;
  }

  void resume_signal()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      resume_received_ = true;
    }
    cv_.notify_all();
  }

  void wait_for_resume()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // 边到边等：只有"开始等待之后"到达的信号才算数（早到信号在此被清零忽略）。
    // 用 wait_for 轮询 rclcpp::ok()，保证 Ctrl+C 能干净退出。
    resume_received_ = false;
    while (!resume_received_ && rclcpp::ok()) {
      cv_.wait_for(lock, std::chrono::milliseconds(200));
    }
  }

  WaypointConfig cfg_;
  bool ok_ = true;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resume_sub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool resume_received_ = false;
};

}  // namespace mynav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mynav::WaypointRunner>();
  if (!node->is_ok()) {
    return 1;  // 配置加载失败
  }
  std::thread spin_thread([node] { rclcpp::spin(node); });
  const bool completed = node->run();
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return completed ? 0 : 1;  // 失败/被中止退出码 1，便于上层感知
}
```

- [ ] **Step 3: 加可执行目标**（`src/mynav/CMakeLists.txt`，在 Task 1 加的 `ament_auto_add_library` 之后）

```cmake
ament_auto_add_executable(waypoint_runner src/waypoint_runner.cpp)
target_link_libraries(waypoint_runner waypoint_config)
```

- [ ] **Step 4: 编译**

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav
```

Expected: 编译成功，无 warning（`-Wall -Wextra -Wpedantic`）。

- [ ] **Step 5: 冒烟运行（独立验证节点构造 + 解析 + action 客户端）**

```bash
cd ~/robocup
source install/setup.bash
timeout 15 ros2 run mynav waypoint_runner --ros-args \
  -p waypoint_file:=$PWD/src/mynav/config/waypoints.yaml
```

Expected: 打印 `已加载 2 个航点 (frame=map, resume=/nav_resume)`，约 10 秒后打印 `「/navigate_to_pose action server 不可用」` 并退出（未启动导航栈，这是预期）。退出码 1（run() 失败路径返回 false）。

- [ ] **Step 6: 提交**

```bash
git add src/mynav/src/waypoint_runner.cpp src/mynav/config/waypoints.yaml src/mynav/CMakeLists.txt
git commit -m "添加 waypoint_runner 多点导航节点及示例航点配置"
```

---

### Task 3: launch 集成 + 端到端验证

**Files:**
- Create: `src/mynav/launch/waypoint_run.launch.py`
- Modify: `src/mynav/config/waypoints.yaml`（仅临时改动做失败测试，测完还原）

**Interfaces:**
- Consumes: `mynav.launch.py`（现有导航栈）、`waypoint_runner` 可执行（Task 2）、`config/waypoints.yaml`。
- Produces: `ros2 launch mynav waypoint_run.launch.py` 一键启动多点导航。

- [ ] **Step 1: 写 launch 文件** `src/mynav/launch/waypoint_run.launch.py`

```python
import os

import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 现有完美定位导航栈（map_server + 静态 map→odom + nav2_bringup）
    nav_launch = os.path.join(
        get_package_share_directory('mynav'),
        'launch',
        'mynav.launch.py',
    )
    # 多点航点配置
    waypoint_file = os.path.join(
        get_package_share_directory('mynav'),
        'config',
        'waypoints.yaml',
    )

    action_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch)
    )
    action_runner = launch_ros.actions.Node(
        package='mynav',
        executable='waypoint_runner',
        name='waypoint_runner',
        output='screen',
        parameters=[{'waypoint_file': waypoint_file}],
    )

    return launch.LaunchDescription([
        action_nav,
        action_runner,
    ])
```

- [ ] **Step 2: 编译（让新 launch 文件安装到位）**

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav
source install/setup.bash
```

- [ ] **Step 3: 正常流程端到端验证**

终端 A 启动：

```bash
cd ~/robocup
source install/setup.bash
ros2 launch mynav waypoint_run.launch.py
```

Expected 日志（终端 A）：
```
[waypoint_runner]: 已加载 2 个航点 (frame=map, resume=/nav_resume)
[waypoint_runner]: [1/2] 前往航点 point1 (1.00, 0.00, yaw=0.00)
... （车开到 point1 停稳）
[waypoint_runner]: 已到达 point1，等待 /nav_resume 信号
```

此时终端 B 发一次信号：

```bash
ros2 topic pub /nav_resume std_msgs/msg/Empty "{}" --once
```

Expected（终端 A 继续）：
```
[waypoint_runner]: [2/2] 前往航点 point2 (2.00, 0.00, yaw=0.00)
...
[waypoint_runner]: 已到达 point2，等待 /nav_resume 信号
```

终端 B 再发一次信号 → Expected：
```
[waypoint_runner]: 多点导航完成
```

RViz 侧应看到小车点 point1 → 停 → point2 → 停。

- [ ] **Step 4: 信号早到测试**

Ctrl+C 停掉 launch，重新 `ros2 launch mynav waypoint_run.launch.py`。车还在去 point1 的路上时，终端 B 立刻发一次：

```bash
ros2 topic pub /nav_resume std_msgs/msg/Empty "{}" --once
```

Expected：信号被忽略——车到达 point1 后**仍打印** `已到达 point1，等待 /nav_resume 信号` 并继续等待，不会直接去 point2。再发一次信号才继续。

- [ ] **Step 5: 失败中止测试**

临时把 `src/mynav/config/waypoints.yaml` 的 point2 改成地图外的点（全局规划必然失败）：

```yaml
  - name: "point2"
    x: 100.0
    y: 100.0
    yaw: 0.0
```

Ctrl+C 停掉上一步的 launch，重新 launch（不用重新编译，配置文件直接生效）。

Expected：
```
[waypoint_runner]: [1/2] 前往航点 point1 ...
[waypoint_runner]: 已到达 point1，等待 /nav_resume 信号
```
发一次信号 → Expected：
```
[waypoint_runner]: [2/2] 前往航点 point2 (100.00, 100.00, yaw=0.00)
[waypoint_runner]: 航点 point2 导航失败 (ABORTED)，中止序列
```
车停在原地，序列中止，进程退出。

测试完把 `waypoints.yaml` 的 point2 还原为 `(2.0, 0.0)`。

- [ ] **Step 6: 提交**

```bash
git add src/mynav/launch/waypoint_run.launch.py src/mynav/config/waypoints.yaml
git commit -m "添加多点导航 launch 集成并完成端到端验证"
```
