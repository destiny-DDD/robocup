
### STM32如何接收
uint8_t rx_buf[15];  // DMA 收完 15 字节
WheelMsg msg;
memcpy(&msg, rx_buf, sizeof(WheelMsg));  // 搬回去

// msg.speed_x 现在就是发端那个 float，值不变