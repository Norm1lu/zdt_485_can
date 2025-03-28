#ifndef MODBUS_HANDLER_H
#define MODBUS_HANDLER_H
#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>

// 由 main.cpp 中定义，extern 引用
extern ModbusMaster node;
extern ModbusMaster node2;

extern ModbusMaster node3;

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

// 第二电机状态变量
extern uint16_t param_count2;
extern uint16_t bus_voltage2;
extern uint16_t phase_current2;
extern uint16_t encoder_value2;

extern float final_target_angle2;
extern float real_time_speed2;
extern float real_time_angle2;
extern float angle_error2;

extern uint8_t homing_status2;
extern uint8_t motor_status2;

extern bool homing_fail2;
extern bool homing_in_progress2;
extern bool enabled2;
extern bool in_position2;
extern bool stall2;
extern bool stall_protect2;

extern int16_t speed_rpm2;


// 第三电机状态变量
extern uint16_t param_count3;
extern uint16_t bus_voltage3;
extern uint16_t phase_current3;
extern uint16_t encoder_value3;

extern float final_target_angle3;
extern float real_time_speed3;
extern float real_time_angle3;
extern float angle_error3;

extern uint8_t homing_status3;
extern uint8_t motor_status3;

extern bool homing_fail3;
extern bool homing_in_progress3;
extern bool enabled3;
extern bool in_position3;
extern bool stall3;
extern bool stall_protect3;

extern int16_t speed_rpm3;


// Modbus 相关接口
bool readAllMotorStatus();
void sendModbusData(int16_t direction, int16_t speed, uint32_t pulseCount,u_char accel);
void sendImmediateStop();
void setMotorEnable(uint8_t control_byte);
void sendHomingTrigger(uint8_t mode) ;
bool readMotorSpeed();






bool readAllMotorStatus2();
void sendModbusData2(int16_t direction, int16_t speed, uint32_t pulseCount, u_char accel);
void sendImmediateStop2();
void setMotorEnable2(uint8_t control_byte);
void sendHomingTrigger2(uint8_t mode);

bool readAllMotorStatus3();
void sendModbusData3(int16_t direction, int16_t speed, uint32_t pulseCount, u_char accel);
void sendImmediateStop3();
void setMotorEnable3(uint8_t control_byte);
void sendHomingTrigger3(uint8_t mode);



#endif  // MODBUS_HANDLER_H
