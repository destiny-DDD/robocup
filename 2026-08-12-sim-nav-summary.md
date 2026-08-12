# 无 scan 无 IMU 纯仿真导航 —— 实现总结（2026-08-12）

## 一句话

在 RViz 里点一个目标点，小车自动规划路径并开过去。不依赖实车、激光雷达(`/scan`)、IMU，障碍物全部来自静态地图 `room.pgm` 生成的代价地图。

## 为什么做这个

- 为将来雷达到货后的**真车导航**（AMCL + scan）做前期验证
- **不破坏现有实车链路**：`mynav.launch.py` + `nav2_params.yaml` 保持原样，雷达到货后直接用

## 关键设计（没有雷达怎么让 Nav2 动起来）

| 缺失的东西 | 替代方案 |
|---|---|
| 定位 `map→odom` | 不启动 AMCL，`sim_nav.launch.py` 发**静态 `map→odom` 单位变换**（"完美定位"） |
| 激光障碍 | 代价地图只用 `static_layer` + `inflation_layer`，**障碍 = 地图里的墙** |
| 里程计 `odom` | 复用已有的 mycontrol 闭环：`/cmd_vel` → socat 假串口 → `sub_node` 读回积分 → `/odom` + TF |
| RViz 给点 | `nav2_rviz_plugins/GoalTool`（工具栏）+ **"Navigation 2" 面板** → `/navigate_to_pose` |

### 数据流

```
RViz 给点 → /navigate_to_pose → bt_navigator → planner(全局路径)
→ MPPI 控制器 → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
→ collision_monitor → /cmd_vel → pub_node → socat 假串口 → sub_node → /odom + TF → 闭环
```

## 改了什么（全部在工作区，**未提交**）

| 文件 | 说明 |
|---|---|
| `src/mynav/config/nav2_params_sim.yaml`（新建） | 从 `nav2_params.yaml` 复制，仅 4 处改动：local costmap 改用 `static_layer`、global costmap 去掉 `obstacle_layer`、删除 voxel/obstacle 块、collision_monitor scan 禁用 |
| `src/mynav/launch/sim_nav.launch.py`（新建） | 编排 mycontrol(odom 闭环) + mysystem(rviz) + map_server + navigation_launch(不含定位) + 静态 map→odom |
| `src/mysystem/config/rviz2.rviz`（修改） | 工具栏 SetGoal → **Nav2 Goal**；加 Global/Local Costmap、Global/Local Plan 显示；补上 **"Navigation 2" 面板** |
| `src/mynav/config/nav2_params.yaml` | **只加了注释，参数原样未动**（真车模式保留） |
| `src/mynav/config/nav2_params_sim.yaml` | 加了注释 |
| `.gitignore` | 曾加 `__pycache__/`（后被移除，请自行决定） |
| `docs/superpowers/plans/2026-08-12-sim-nav-no-scan.md` | 实现计划文档 |
| `docs/superpowers/specs/2026-08-12-sim-nav-no-scan-design.md` | 设计文档（已提交 eeef83d） |

## 验证结果（端到端真实跑通）

在真实桌面启动 `ros2 launch mynav sim_nav.launch.py` 验证：

- **3 个导航目标全部 `SUCCEEDED`**：(1.0, 0) → (2.0, 0) → 返回 (0, 0)
- odom 位移 **0 → 0.871 → 1.869 → 0.014**，与目标一致（闭环积分生效）
- 飞行中 `/cmd_vel` 非零（linear.x=-0.045, angular.z=-0.459），车真的被指令驱动
- TF 链完整：`map → odom → base_footprint → base_link`
- 全局代价地图加载了 `room.pgm` 的墙（376×223 @ 0.05 m/cell）
- 干净收尾（SIGINT），无残留进程

## 怎么跑

```bash
cd ~/robocup
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mynav sim_nav.launch.py
```

**操作**：等 rviz 打开、左侧出现 **"Navigation 2"** 面板 → 点工具栏 **Nav2 Goal** → 在地图空地**点一下并按住拖一下设朝向** → 小车规划并开过去。

**停止/收尾**：launch 终端 `Ctrl+C`，然后 `pkill -x socat && rm -f /tmp/vtx /tmp/vrx`。

**注意**：别和 `myekf` / `rf2o` / `yesense` 同时跑（会抢 `/odom`）。

## 两个 yaml 的区别

| | `nav2_params.yaml`（真车） | `nav2_params_sim.yaml`（仿真） |
|---|---|---|
| 用途 | `mynav.launch.py` + AMCL | `sim_nav.launch.py` + 静态定位 |
| 定位 | AMCL（需要 `/scan`） | 不启动 AMCL |
| local costmap | `voxel_layer`(激光) + inflation | `static_layer`(地图) + inflation |
| global costmap | static + `obstacle_layer`(激光) + inflation | static + inflation |
| collision_monitor scan | 启用（实车避障） | `enabled: False`（无 /scan） |

## 踩过的坑

1. **rviz 给点没反应（车纹丝不动），但 CLI 发目标正常** —— 根因：rviz 配置里缺 **"Navigation 2" 面板**。`Nav2 Goal` 工具只是发一个 Qt 信号，必须由这个面板接住并发成 `/navigate_to_pose` action。没有面板，点击被静默丢弃。**已修复**（加了面板，需重启 launch 生效）。
2. **`/cmd_vel` 链有四层**：controller → `cmd_vel_nav` → velocity_smoother → `cmd_vel_smoothed` → collision_monitor → `/cmd_vel`。查"车不动"要逐层确认。
3. **起点在墙里会规划必败**：地图 (0,0) 已验证为空地。若改地图 origin 记得复查。
4. **socat 残留**：上次异常退出后 `/tmp/vtx /tmp/vrx` 可能残留，重启前先 `pkill -x socat; rm -f /tmp/vtx /tmp/vrx`。

## 当前状态

- 所有改动**未提交**（按你的要求），自行审阅后再 commit
- 真车模式原样保留：`ros2 launch mynav mynav.launch.py`
- 两个 yaml 已加中文注释（只加注释，没动参数）
