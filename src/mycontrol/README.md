# MyControl — 串口通信控制

## 架构

```
ros2 topic pub /cmd_vel → MyPub → 串口 TX → STM32  → 串口 RX → MySub

                        ┌──────────┐
                        │ socat    │  （无硬件调试用）
                        │ vt0 ←→ vt1 │
                        └──────────┘
```

## 构建

```bash
cd robocup
colcon build --packages-select mycontrol
source install/setup.bash
```

## 无硬件调试（socat 虚拟串口）

### 1. 创建虚拟串口对

```bash
socat -d -d pty,raw,echo=0,link=/tmp/vtx pty,raw,echo=0,link=/tmp/vrx &
```

`/tmp/vtx` ←→ `/tmp/vrx` 互为收发：写 tx 的数据从 rx 读出，反之亦然。

### 2. 运行节点

```bash
source install/setup.bash
ros2 run mycontrol mycontrol
```

### 3. 发送指令

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}' -1
```

### 4. 观察输出

```bash
# 两个节点的日志都会输出，确认收发闭环
```

## 连接硬件

```bash
# 杀掉 socat
kill %1

# 修改 main.cpp 中的设备路径为实际串口：
# /tmp/vtx → /dev/ttyUSB0
# /tmp/vrx → /dev/ttyUSB0

# 重新编译运行
colcon build --packages-select mycontrol && source install/setup.bash
ros2 run mycontrol mycontrol
```
