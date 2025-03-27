#ifndef MODBUS_HANDLER_H
#define MODBUS_HANDLER_H

#include <ModbusMaster.h>
#include <HardwareSerial.h>

// 由 main.cpp 中定义，extern 引用
extern HardwareSerial modbusSerial;
extern ModbusMaster node;

// 全局状态变量（部分在 CAN 发送中使用）
extern uint16_t param_count;
extern uint16_t bus_voltage;
extern uint16_t phase_current;
extern uint16_t encoder_value;

extern float final_target_angle;
extern float real_time_speed;
extern float real_time_angle;
extern float angle_error;

extern uint8_t homing_status;
extern uint8_t motor_status;

extern bool homing_fail;
extern bool homing_in_progress;
extern bool enabled;
extern bool in_position;
extern bool stall;
extern bool stall_protect;

extern int16_t speed_rpm;

// Modbus 相关接口
bool readAllMotorStatus();
void sendModbusData(int16_t direction, int16_t speed, uint32_t pulseCount,u_char accel);
void sendImmediateStop();
void setMotorEnable(uint8_t control_byte);
void sendHomingTrigger(uint8_t mode) ;
bool readMotorSpeed();
#endif  // MODBUS_HANDLER_H
