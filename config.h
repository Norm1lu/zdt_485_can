#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 将频率定义为全局变量，默认值为200Hz
extern unsigned long modbusReadFreqHz;  
extern unsigned long canSendFreqHz;  

// 其他固定参数依然可以放在这里
#define MODBUS_RX 18
#define MODBUS_TX 19
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5

#endif  // CONFIG_H
