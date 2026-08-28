```
robocup/
├── .gitignore                # 隐藏文件
│
├── README.md                 # ← 你正在看的这份
│
└── src/
    ├── mycontrol/                          # 机器人控制逻辑
    ├── myekf/         　　　　　　　　　      # ekf融合
    ├── mysystem/                           # 启动rviz2和发布机器人静态TF
    ├── rf2o_laser_odometry/                # rf2o(处理激光雷达数据)的launch文件
    └── yesense_ros2/                       # imu的launch文件
```
### RF2O
在工作目录src下
```
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ../
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rf2o_laser_odometry
source install/setup.bash
```
### STM32如何接收
```
uint8_t rx_buf[15];  // DMA 收完 15 字节
WheelMsg msg;
memcpy(&msg, rx_buf, sizeof(WheelMsg));  // 搬回去

// msg.speed_x 现在就是发端那个 float，值不变
```

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

ros2 topic pub /nav_resume std_msgs/msg/Empty '{}' --once

MAKEFLAGS="-j1" colcon build --symlink-install

驱动读的：                    /dev/wheeltec_lidar  ──┐
                            /dev/wheeltec_IMU   ──┐├─▶ 这两个名字是固定的
                                                 │
软链接指向（每次插拔/开机按序列号重新算）：            │
  雷达(serial 5B8E672279) → wheeltec_lidar → 当前的某个 ttyACM×
  IMU(serial 0003)        → wheeltec_IMU  → 当前的某个 ttyACM×

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false

ros2 run nav2_map_server map_saver_cli -f ~/robocup/src/mynav2/maps