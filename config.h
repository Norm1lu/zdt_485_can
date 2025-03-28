#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// 将频率定义为全局变量，默认值为200Hz
extern unsigned long modbusReadFreqHz;  
extern unsigned long canSendFreqHz;  

// 其他固定参数依然可以放在这里
#define MODBUS_RX 23
#define MODBUS_TX 22
#define MODBUS2_RX 17
#define MODBUS2_TX 16
#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5


#define MODBUS3_RX 1
#define MODBUS3_TX 3
#endif  // CONFIG_H
