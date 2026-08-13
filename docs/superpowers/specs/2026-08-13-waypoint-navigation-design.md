# 多点导航（到点停车等信号）设计文档

日期：2026-08-13
状态：已确认设计，待实现

## 一句话

小车按 yaml 里写好的多个航点依次导航；每到达一个点**完全停下，等待外部话题信号**后才去下一个点；任一航点导航失败则**中止整条序列**。

## 背景与现状

当前工作区已有可跑的完美定位导航栈：

- `mynav.launch.py`：map_server + 静态 `map→odom`（`use_localization: 'False'`，不用 AMCL、不依赖 `/scan`）+ nav2_bringup（bt_navigator / planner / MPPI / velocity_smoother / collision_monitor 等）。
- `nav2_params.yaml`：真车参数；`bt_navigator` 已注册 `navigate_to_pose` 和 `navigate_through_poses`；内置 `waypoint_follower` 节点存在但默认 `wait_at_waypoint` 只支持**固定时长**等待，不支持"等外部话题信号"。

因此：单点导航（RViz 手点 → `/navigate_to_pose`）已可用；缺少的是"按列表依次执行 + 每点到点停车等信号"的编排层。

## 需求（来自澄清问答）

1. **多点导航**：按 yaml 航点列表依次执行。
2. **每点停车**：到达某点 → 完全停稳 → 等外部话题信号 → 才继续下一个点。
3. **航点来源**：yaml 配置文件（以 ROS 参数形式加载）。
4. **外部信号形式**：话题消息（`std_msgs/Empty`），默认话题 `/nav_resume`。
5. **失败处理**：任一航点失败 → 中止整条序列，停在原地报错，不自动重试。

## 方案选择

自写一个 C++ 客户端节点 `waypoint_runner`（方案 A），而非复用 Nav2 内置 `/follow_waypoints`。

理由：内置 waypoint_follower 的 `wait_at_waypoint` 任务执行器只支持固定时长等待，要支持"等话题"仍需自写 pluginlib 插件（更繁琐、更深的 Nav2 耦合）；独立节点逻辑直接、可独立调试、对"失败即中止"控制力最强，且完全复用 Nav2 的规划/控制机制。

## 架构与组件

**包**：`mynav`（现有 C++ ament_cmake 包，新增一个可执行）。

**节点 `waypoint_runner`**（纯客户端，不动 Nav2 内部）：

| 接口 | 方向 | 说明 |
|---|---|---|
| `/navigate_to_pose` | action client | 逐个向 bt_navigator 发送目标（`nav2_msgs::action::NavigateToPose`） |
| resume 话题（参数配置，默认 `/nav_resume`） | subscriber | `std_msgs/Empty`；收到即去下一个点 |
| 参数 | — | 航点列表、frame_id、resume 话题名 |

### 配置格式（新文件 `src/mynav/config/waypoints.yaml`）

以标准 ROS 参数形式加载，与 `nav2_params.yaml` 方式一致，无需额外 yaml 解析依赖：

```yaml
waypoint_runner:
  ros__parameters:
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
```

- 每个航点：`name`（日志/报错用）、`x`、`y`、`yaw`（到达朝向，转四元数；缺省 0）。
- 目标帧为 `map`（当前静态 map→odom 已提供 `map→odom→base_footprint→base_link` TF 链）。

### 行为循环

```
对每个航点 wp：
  1. 发 /navigate_to_pose goal(wp)，等 SUCCEEDED（车已自然停稳）
  2. 打印"到达 <name>，等待 <resume_topic>"
  3. 阻塞等待 resume 话题消息（订阅回调用条件变量唤醒）
  4. 收到 → 下一个航点
全部完成 → 打印"多点导航完成"，节点保持运行但不再动作
```

**resume 信号语义**：**边到边等** —— 只在到达某点后开始监听该点的信号；信号提前到达（车还在路上）会被忽略，不跳过停车。符合"每到一个点停一下"的语义。

### 数据流

```
waypoints.yaml ──参数──▶ waypoint_runner
waypoint_runner ──action──▶ /navigate_to_pose (bt_navigator)
                          → planner → MPPI → cmd_vel_nav → velocity_smoother
                          → cmd_vel_smoothed → collision_monitor → /cmd_vel
                          → mycontrol 闭环 → /odom + TF
外部节点/人 ──/nav_resume(Empty)──▶ waypoint_runner（到达后等待中，收到才继续）
```

## 失败处理

| 场景 | 行为 |
|---|---|
| 某航点导航失败（goal `ABORTED`，如卡住/规划不出来） | 记录失败航点 `name` + 错误码，**立即中止整条序列**；车停在原地（Nav2 失败时自身已停），不自动重试 |
| 中途被取消（goal `CANCELED`） | 同样中止序列并打印原因 |
| resume 话题一直没消息 | 在当前点一直等（设计意图），Ctrl+C 终止 |
| 全部完成 | 打印完成信息，节点空闲 |

可选扩展（当前不做，YAGNI）：`waypoint_runner/status` 状态话题、起跑前等待信号、每个点独立等待时长、失败重试/跳过。

## Launch 集成

- **不动** `mynav.launch.py`（保留单点 RViz 手点模式）。
- 新建 `src/mynav/launch/waypoint_run.launch.py`：include `mynav.launch.py`（现有完美定位导航栈）+ 启动 `waypoint_runner`（加载 `waypoints.yaml`）。
- 跑法：`ros2 launch mynav waypoint_run.launch.py`。

## 测试计划（无需雷达，当前栈即可验证）

1. **正常流程**：launch → 车自动去 point1 停住 → `ros2 topic pub /nav_resume std_msgs/msg/Empty "{}" --once` → 去 point2 停住 → 再发一次 → 完成。
2. **信号早到**：在车到达 point1 前发 resume → 被忽略，仍等到达后的信号。
3. **失败中止**：临时把某航点写进墙里 → 跑到该点时序列中止、车停、日志报错。
4. **干净收尾**：Ctrl+C 无残留进程（沿用现有 socat/进程清理习惯）。

## 范围外（非目标）

- 不做 AMCL 真车定位接入（雷达到货后另行处理）。
- 不做运行时动态增删航点、不做重复循环、不做航点间全局路径合并。
