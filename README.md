

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