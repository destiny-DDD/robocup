# 无 scan 无 IMU 纯仿真导航（RViz 给点 → 小车自动过去）

日期：2026-08-12
状态：已批准设计

## 目标

不依赖实车、激光雷达（scan）、IMU，纯在 RViz 中验证 NAV2 导航：

- 在 RViz 里给一个目标点，小车自动规划路径并开过去
- 障碍物全部来自静态地图（`room.pgm`），即"障碍物就是代价地图"
- 为将来雷达到货后的真车导航（AMCL + scan）做前期验证，**不破坏现有实车链路**

## 现状盘点

已有资产（无需改动）：

- **odom 循环已跑通**：`/cmd_vel → pub_node → 写 /tmp/vtx → socat 转发 → /tmp/vrx → sub_node 读 → 积分 → /odom + TF(odom→base_footprint) → nav 栈`
  - socat 双向交叉连接已验证：写 `/tmp/t_vtx` 可在 `/tmp/t_vrx` 读到，反之亦然
  - 无实车时 `sub_node` 读回 `pub_node` 自己发的 cmd_vel 数据，软件自闭环，odom 跟随指令速度积分
- **帧链完整**：URDF 提供 `base_footprint → base_link`，costmap/controller 用 `base_link` 也能查到机器人位姿
- **NAV2 全家桶** 已安装（Jazzy），`room.pgm` 地图已加载
- **cmd_vel 链**（Jazzy 标准）：`controller_server / behavior_server → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → /cmd_vel`

本设计**不写任何新 C++ 代码**。

## 缺口与方案

### 1. 定位：静态 map→odom（"完美定位"）

- 问题：AMCL 订阅 `/scan`，无 scan 时不发布 `map→odom`，planner 无法确定机器人在 map 中的位姿
- 方案：sim 启动中发布静态变换 `static_transform_publisher 0 0 0 0 0 0 map odom`，**不启动 AMCL**；map_server 由 sim 启动自己起
- 效果：机器人 map 位姿 = odom 位姿，起点 = 地图原点 (0,0)

### 2. 代价地图：去 scan，静态层为唯一障碍来源

新建 `src/mynav/config/nav2_params_sim.yaml`（从 `nav2_params.yaml` 复制），三处改动：

- **global_costmap**：plugins 由 `["static_layer", "obstacle_layer", "inflation_layer"]` 改为 `["static_layer", "inflation_layer"]`，删除 `obstacle_layer` 块
- **local_costmap**：plugins 由 `["voxel_layer", "inflation_layer"]` 改为 `["static_layer", "inflation_layer"]`，新增 `static_layer` 块（`map_subscribe_transient_local: True`），删除 `voxel_layer` 块。**原配置无 static_layer，无 scan 时 local costmap 会空**
- **collision_monitor**：`scan` 观测源 `enabled: False`，保留 `FootprintApproach`（读 local costmap 静态障碍，可检测贴墙）

`amcl`、`loopback_simulator` 两段在 sim 中不会启动，保留（等真车模式用回原参数文件）。

### 3. RViz：Nav2 Goal 工具 + 观察显示

改 `src/mysystem/config/rviz2.rviz`：

- 工具条加入 **Nav2 Goal**（`nav2_rviz_plugins/GoalTool` → 发 `/navigate_to_pose`），移除旧 `SetGoal`（2D Nav Goal，发 `/move_base/goal`，NAV2 不收）
- 加入观察显示：Global Costmap（`/global_costmap/costmap_raw`）、Local Costmap（`/local_costmap/costmap_raw`）、Global Plan（`/plan`）、Local Plan（`/local_plan`）

### 4. 启动编排：新增 `src/mynav/launch/sim_nav.launch.py`

顺序组装：

1. include `mycontrol.launch.py` → socat 虚拟串口 + mycontrol（odom 回环）
2. include `mysystem.launch.py` → URDF + rviz2（用改过的配置）
3. Node `map_server`（`room.yaml`）
4. include `navigation_launch.py`（nav2_bringup，**不含 localization**），params_file 指向 `nav2_params_sim.yaml`
5. Node `static_transform_publisher`：`0 0 0 0 0 0 map odom`

`mynav.launch.py`（真车模式，带 AMCL）**保持原样不动**，雷达到货后直接用。

## 数据流

```
rviz 给点 → /navigate_to_pose → bt_navigator → planner(全局路径)
→ controller(MPPI) → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
→ collision_monitor → /cmd_vel → pub_node → socat → sub_node → /odom + TF
→ costmap / controller / planner 闭环
```

## 验收清单

1. 地图 (0,0) 附近是空地（否则初始位姿在墙里，规划必败）
2. `tf` 完整：`map → odom → base_footprint → base_link`
3. `/odom` 有数据且随 `/cmd_vel` 变化
4. RViz 中 Nav2 Goal 给点 → 出全局路径 → 车开过去 → 到达

## 常见报错对照

| 现象 | 原因 |
|------|------|
| 日志 "Transform from map to base_link not available" | 静态 map→odom 未启动 |
| 代价地图显示为空 | local costmap 缺 static_layer |
| 车不动、目标一直未到达 | `/cmd_vel` 没产生或 socat 回环未通 |
| 规划一直失败 | 起点在墙里，调整地图 origin 或起点 |

## 约定

- sim 模式不启动 `myekf.launch.py` / `rf2o` / `yesense`（避免与 mycontrol 抢 `/odom`）
- `socat` 虚拟串口链路 `/tmp/vtx`、`/tmp/vrx` 由 `mycontrol.launch.py` 创建，勿与其他 launch 同时跑
- 全链路真实时间（`use_sim_time: false`），不引入 Gazebo
