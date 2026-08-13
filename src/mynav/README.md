# mynav — 多点导航

ROS 2 Jazzy 包，提供：

- **完美定位导航栈**：`map_server` + 静态 `map→odom` + nav2_bringup（不用 AMCL、不依赖 `/scan`）。
- **多点导航编排** `waypoint_runner`：按 `config/waypoints.yaml` 里的航点依次导航；每到达一个点**完全停住**，等外部 `/nav_resume` 信号（`std_msgs/Empty`）才去下一个点；任一航点失败立即**中止整条序列**。

## 编译

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav
source install/setup.bash
```

`--symlink-install` 下，改 `config/waypoints.yaml` 后**无需重新编译**，重启 launch 即生效。

## 启动多点导航（真车）

> **前置**：车体系统要先跑起来，提供 `/odom` 和 `odom→base_link` 的 TF（并消费 `/cmd_vel`）。否则 nav 栈激活会卡住（`等待 bt_navigator 激活超时`）。

按顺序开 4 个终端：

| 终端 | 命令 | 作用 |
|---|---|---|
| ① | `ros2 launch mysystem mysystem.launch.py` | 车体描述 + **RViz**（已配好地图/costmap/全局局部路径/车模显示，Navigation 2 面板，Fixed Frame=odom） |
| ② | `ros2 launch myekf myekf.launch.py` | 串口控制 + IMU + EKF → 发布 `/odom` 和 `odom→base_link` |
| ③ | `ros2 launch mynav waypoint_run.launch.py` | 导航栈 + 多点 runner（一条命令） |
| ④ | `ros2 topic pub /nav_resume std_msgs/msg/Empty "{}" --once` | 每次车到点停住后，发一次信号让它继续 |

### 预期日志（终端 ③）

```
[waypoint_runner]: 已加载 2 个航点 (frame=map, resume=/nav_resume)
[waypoint_runner]: [1/2] 前往航点 point1 (1.00, 0.00, yaw=0.00)
...（车开到 point1 停稳）
[waypoint_runner]: 已到达 point1，等待 /nav_resume 信号   ← 此时终端 ④ 发信号
[waypoint_runner]: [2/2] 前往航点 point2 (2.00, 0.00, yaw=0.00)
...
[waypoint_runner]: 多点导航完成
```

### 发信号时机

**边到边等**语义：只有**到达后**（看到"已到达 xxx，等待信号"）收到的信号才算数。车还在路上时发会被忽略，不跳过停车。

## 单点导航（RViz 手点）

```bash
ros2 launch mynav mynav.launch.py
```

RViz 用 **Navigation 2** 面板点 Goal 即可。

## 修改航点

编辑 `config/waypoints.yaml`，每个航点字段：`name`（日志/报错用）、`x`、`y`、`yaw`（到达朝向，缺省 0）：

```yaml
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

## 常见问题

- **`等待 bt_navigator 激活超时` 退出（退出码 1）**：车体系统没起（无 TF），或 nav 栈激活超 30s。先确认终端 ①② 正常。
- **信号发了没反应**：发早了被忽略（见"发信号时机"）；或确认话题名是 `/nav_resume`。
- **无车/无仿真环境**：`waypoint_run.launch.py` 能拉起整个栈做冒烟，但导航不会动（没有 `/odom` 和 TF）。
- **导航中 Ctrl+C**：runner 会干净退出（退出码 1），不会卡死；对正在进行的 goal 不主动取消。
