#include <sys/types.h>
#include "modbus_handler.h"
#include "Arduino.h"

// 全局变量定义
uint16_t param_count = 0;
uint16_t bus_voltage = 0;
uint16_t phase_current = 0;
uint16_t encoder_value = 0;

float final_target_angle = 0.0f;
float real_time_speed = 0.0f;
float real_time_angle = 0.0f;
float angle_error = 0.0f;

uint8_t homing_status = 0;
uint8_t motor_status = 0;

uint16_t sign=0;
uint16_t value=0;
int speed=0;

bool homing_fail = false;
bool homing_in_progress = false;
bool enabled = false;
bool in_position = false;
bool stall = false;
bool stall_protect = false;

int16_t speed_rpm = 0;


uint16_t param_count2 = 0;
uint16_t bus_voltage2 = 0;
uint16_t phase_current2 = 0;
uint16_t encoder_value2 = 0;

float final_target_angle2 = 0.0f;
float real_time_speed2 = 0.0f;
float real_time_angle2 = 0.0f;
float angle_error2 = 0.0f;

uint8_t homing_status2 = 0;
uint8_t motor_status2 = 0;

bool homing_fail2 = false;
bool homing_in_progress2 = false;
bool enabled2 = false;
bool in_position2 = false;
bool stall2 = false;
bool stall_protect2 = false;

int16_t speed_rpm2 = 0;



uint16_t param_count3 = 0;
uint16_t bus_voltage3 = 0;
uint16_t phase_current3 = 0;
uint16_t encoder_value3 = 0;

float final_target_angle3 = 0.0f;
float real_time_speed3 = 0.0f;
float real_time_angle3 = 0.0f;
float angle_error3 = 0.0f;

uint8_t homing_status3 = 0;
uint8_t motor_status3 = 0;

bool homing_fail3 = false;
bool homing_in_progress3 = false;
bool enabled3 = false;
bool in_position3 = false;
bool stall3 = false;
bool stall_protect3 = false;

int16_t speed_rpm3 = 3;



// 读取系统状态参数（根据设备寄存器映射）
// ===================== 电机3 的 Modbus 读取和控制函数 =====================

bool readAllMotorStatus3() {
  // 读取 16 个寄存器
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t result = node3.readInputRegisters(0x43, 16);
    if (result == node3.ku8MBSuccess) {
      // 读取数据到电机3的专用变量
      param_count3      = node3.getResponseBuffer(0);
      bus_voltage3      = node3.getResponseBuffer(1);
      phase_current3    = node3.getResponseBuffer(2);
      encoder_value3    = node3.getResponseBuffer(3);

      // 最终目标位置角度（单位：deg）
      uint16_t final_angle_sign = node3.getResponseBuffer(4);
      uint16_t final_angle_high = node3.getResponseBuffer(5);
      uint16_t final_angle_low  = node3.getResponseBuffer(6);
      int32_t final_angle_raw   = ((uint32_t)final_angle_high << 16) | final_angle_low;
      if (final_angle_sign == 1) final_angle_raw = -final_angle_raw;
      final_target_angle3 = final_angle_raw * 360.0f / 65536.0f;

      // 实时转速（单位：rpm）
      uint16_t rt_speed_sign = node3.getResponseBuffer(7);
      uint16_t rt_speed_val  = node3.getResponseBuffer(8);
      speed_rpm3 = (rt_speed_sign == 0) ? rt_speed_val : -rt_speed_val;

      // 实时角速度（单位：deg/s）
      real_time_speed3 = speed_rpm3 * 6.0;

      // 实时位置角度（单位：deg）
      uint16_t rt_angle_sign = node3.getResponseBuffer(9);
      uint16_t rt_angle_high = node3.getResponseBuffer(10);
      uint16_t rt_angle_low  = node3.getResponseBuffer(11);
      int32_t rt_angle_raw   = ((uint32_t)rt_angle_high << 16) | rt_angle_low;
      if (rt_angle_sign == 1) rt_angle_raw = -rt_angle_raw;
      real_time_angle3 = rt_angle_raw * 360.0f / 65536.0f;

      // 位置角度误差（单位：deg）
      uint16_t error_sign  = node3.getResponseBuffer(12);
      uint16_t error_high  = node3.getResponseBuffer(13);
      uint16_t error_low   = node3.getResponseBuffer(14);
      int32_t error_raw    = ((uint32_t)error_high << 16) | error_low;
      if (error_sign == 1) error_raw = -error_raw;
      angle_error3 = error_raw * 360.0f / 65536.0f;

      // 回零状态 & 电机状态
      uint16_t flags = node3.getResponseBuffer(15);
      homing_status3   = flags >> 8;
      motor_status3    = flags & 0x0F;

      homing_fail3       = (homing_status3 & 0x04);
      homing_in_progress3 = ((homing_status3 & 0x03) == 0x03);

      enabled3       = (motor_status3 & 0x08);
      in_position3   = (motor_status3 & 0x04);
      stall3         = (motor_status3 & 0x02);
      stall_protect3 = (motor_status3 & 0x01);

      return true;
    }
    // Serial.print("❌ 电机3读取失败，错误码: ");
    // Serial.println(result, HEX);
  }
  return false;
}

void sendModbusData3(int16_t direction, int16_t speed, uint32_t pulseCount, u_char accel) {
  uint16_t regs[5];
  regs[0] = ((uint16_t)(direction & 0x0F) << 12) | (accel & 0x0F);
  regs[1] = (uint16_t)speed;
  regs[2] = (uint16_t)(pulseCount >> 16);
  regs[3] = (uint16_t)(pulseCount & 0xFFFF);
  regs[4] = 0x0100;

  uint16_t startAddress = 0x00FD;
  node3.beginTransmission(0x03); // 电机3的从站地址为 0x03
  for (int i = 0; i < 5; i++) {
    node3.setTransmitBuffer(i, regs[i]);
  }

  uint8_t result = node3.writeMultipleRegisters(startAddress, 5);
  if (result != node3.ku8MBSuccess) {
    // Serial.print("❌ Modbus 数据发送失败（电机3），错误码: ");
    // Serial.println(result);
  }
}

void sendImmediateStop3() {
  uint16_t stopRegister = 0x9800;
  uint16_t startAddress = 0x00FE;

  node3.beginTransmission(0x03);
  node3.setTransmitBuffer(0, stopRegister);
  uint8_t result = node3.writeMultipleRegisters(startAddress, 1);

  if (result != node3.ku8MBSuccess) {
    // Serial.print("❌ 电机3立即停止失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("🛑 电机3已发送立即停止指令");
  }
}

void setMotorEnable3(uint8_t control_byte) {
  uint16_t regs[2];
  regs[0] = ((uint16_t)0xAB << 8) | control_byte;
  regs[1] = 0x0000;
  uint16_t startAddress = 0x00F3;

  node3.beginTransmission(0x03);
  node3.setTransmitBuffer(0, regs[0]);
  node3.setTransmitBuffer(1, regs[1]);

  uint8_t result = node3.writeMultipleRegisters(startAddress, 2);
  if (result != node3.ku8MBSuccess) {
    // Serial.print("❌ 电机3使能指令发送失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("✅ 电机3使能指令发送成功");
  }
}

void sendHomingTrigger3(uint8_t mode) {
  uint16_t homing_register_value = ((uint16_t)mode << 8) | 0x00;
  const uint16_t startAddress = 0x009A;

  node3.beginTransmission(0x03);
  node3.setTransmitBuffer(0, homing_register_value);

  uint8_t result = node3.writeMultipleRegisters(startAddress, 1);
  if (result != node3.ku8MBSuccess) {
    // Serial.print("❌ 电机3回零触发失败，错误码: ");
    // Serial.println(result, HEX);
  } else {
    // Serial.println("✅ 电机3回零触发成功");
  }
}



bool readAllMotorStatus2() {
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t result = node2.readInputRegisters(0x43, 16);
    if (result == node2.ku8MBSuccess) {
      param_count2 = node2.getResponseBuffer(0);
      bus_voltage2 = node2.getResponseBuffer(1);
      phase_current2 = node2.getResponseBuffer(2);
      encoder_value2 = node2.getResponseBuffer(3);

      // 最终目标位置角度（单位：deg）
      uint16_t final_angle_sign = node2.getResponseBuffer(4);
      uint16_t final_angle_high = node2.getResponseBuffer(5);
      uint16_t final_angle_low = node2.getResponseBuffer(6);
      int32_t final_angle_raw = ((uint32_t)final_angle_high << 16) | final_angle_low;
      if (final_angle_sign == 1) final_angle_raw = -final_angle_raw;
      final_target_angle2 = final_angle_raw * 360.0f / 65536.0f;

      // 实时转速（单位：rpm）
      uint16_t rt_speed_sign = node2.getResponseBuffer(7);
      uint16_t rt_speed_val = node2.getResponseBuffer(8);
      speed_rpm2 = (rt_speed_sign == 0) ? rt_speed_val : -rt_speed_val;

      // 实时角速度（单位：度/秒）
      real_time_speed2 = speed_rpm2 * 6.0;

      // 实时位置角度（单位：deg）
      uint16_t rt_angle_sign = node2.getResponseBuffer(9);
      uint16_t rt_angle_high = node2.getResponseBuffer(10);
      uint16_t rt_angle_low = node2.getResponseBuffer(11);
      int32_t rt_angle_raw = ((uint32_t)rt_angle_high << 16) | rt_angle_low;
      if (rt_angle_sign == 1) rt_angle_raw = -rt_angle_raw;
      real_time_angle2 = rt_angle_raw * 360.0f / 65536.0f;
      // Serial.print("📍 电机2角度: ");
      // Serial.println(real_time_angle2, 2);

      // 位置角度误差（单位：deg）
      uint16_t error_sign = node2.getResponseBuffer(12);
      uint16_t error_high = node2.getResponseBuffer(13);
      uint16_t error_low = node2.getResponseBuffer(14);
      int32_t error_raw = ((uint32_t)error_high << 16) | error_low;
      if (error_sign == 1) error_raw = -error_raw;
      angle_error2 = error_raw * 360.0f / 65536.0f;

      // 回零状态 & 电机状态
      uint16_t flags = node2.getResponseBuffer(15);
      homing_status2 = flags >> 8;
      motor_status2 = flags & 0x0F;

      homing_fail2 = (homing_status2 & 0x04);
      homing_in_progress2 = ((homing_status2 & 0x03) == 0x03);

      enabled2 = (motor_status2 & 0x08);
      in_position2 = (motor_status2 & 0x04);
      stall2 = (motor_status2 & 0x02);
      stall_protect2 = (motor_status2 & 0x01);

      return true;
    }
    // Serial.print("❌ 电机2读取失败，错误码: ");
    // Serial.println(result, HEX);
  }
  return false;
}

void sendModbusData2(int16_t direction, int16_t speed, uint32_t pulseCount, u_char accel) {
  uint16_t regs[5];
  regs[0] = ((uint16_t)(direction & 0x0F) << 12) | (accel & 0x0F);
  regs[1] = (uint16_t)speed;
  regs[2] = (uint16_t)(pulseCount >> 16);
  regs[3] = (uint16_t)(pulseCount & 0xFFFF);
  regs[4] = 0x0100;

  // Serial.println("发送寄存器数据到电机2：");
  // for (int i = 0; i < 5; i++) {
  //   Serial.print("寄存器");
  //   Serial.print(i + 1);
  //   Serial.print(": 0x");
  //   Serial.println(regs[i], HEX);
  // }

  uint16_t startAddress = 0x00FD;
  node2.beginTransmission(0x02);
  for (int i = 0; i < 5; i++) {
    node2.setTransmitBuffer(i, regs[i]);
  }

  uint8_t result = node2.writeMultipleRegisters(startAddress, 5);
  if (result != node2.ku8MBSuccess) {
    // Serial.print("❌ Modbus 数据发送失败（电机2），错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("✅ Modbus 数据发送成功（电机2）");
    // Serial.print("响应地址: 0x");
    // Serial.println(node2.getResponseBuffer(0), HEX);
    // Serial.print("响应数量: ");
    // Serial.println(node2.getResponseBuffer(1));
  }
}

void sendImmediateStop2() {
  uint16_t stopRegister = 0x9800;
  uint16_t startAddress = 0x00FE;

  node2.beginTransmission(0x02);
  node2.setTransmitBuffer(0, stopRegister);
  uint8_t result = node2.writeMultipleRegisters(startAddress, 1);

  if (result != node2.ku8MBSuccess) {
    // Serial.print("❌ 电机2立即停止失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("🛑 电机2已发送立即停止指令");
  }
}

void setMotorEnable2(uint8_t control_byte) {
  uint16_t regs[2];
  regs[0] = ((uint16_t)0xAB << 8) | control_byte;
  regs[1] = 0x0000;
  uint16_t startAddress = 0x00F3;

  node2.beginTransmission(0x02);
  node2.setTransmitBuffer(0, regs[0]);
  node2.setTransmitBuffer(1, regs[1]);

  uint8_t result = node2.writeMultipleRegisters(startAddress, 2);
  if (result != node2.ku8MBSuccess) {
    // Serial.print("❌ 电机2使能指令失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("✅ 电机2使能成功");
  }
}

void sendHomingTrigger2(uint8_t mode) {
  uint16_t val = ((uint16_t)mode << 8) | 0x00;
  const uint16_t startAddress = 0x009A;

  node2.beginTransmission(0x02);
  node2.setTransmitBuffer(0, val);

  uint8_t result = node2.writeMultipleRegisters(startAddress, 1);
  if (result != node2.ku8MBSuccess) {
    // Serial.print("❌ 电机2回零触发失败，错误码: ");
    // Serial.println(result, HEX);
  } else {
    // Serial.println("✅ 电机2回零触发成功");
  }
}





bool readAllMotorStatus() {
  // 此处参考你原来的实现，错误处理等可根据需要调整
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t result = node.readInputRegisters(0x43, 16);
    if (result == node.ku8MBSuccess) {
      param_count = node.getResponseBuffer(0);
      bus_voltage = node.getResponseBuffer(1);
      phase_current = node.getResponseBuffer(2);
      encoder_value = node.getResponseBuffer(3);

      // 最终目标位置角度（单位：deg）
      uint16_t final_angle_sign = node.getResponseBuffer(4);
      uint16_t final_angle_high = node.getResponseBuffer(5);
      uint16_t final_angle_low = node.getResponseBuffer(6);
      int32_t final_angle_raw = ((uint32_t)final_angle_high << 16) | final_angle_low;
      if (final_angle_sign == 1) final_angle_raw = -final_angle_raw;
      final_target_angle = final_angle_raw * 360.0f / 65536.0f;

      // 实时转速（单位：rpm）
      uint16_t rt_speed_sign = node.getResponseBuffer(7);
      uint16_t rt_speed_val = node.getResponseBuffer(8);
      speed_rpm = (rt_speed_sign == 0) ? rt_speed_val : -rt_speed_val;
      // Serial.println(speed_rpm);
      // 实时角速度（单位：度/秒）
      real_time_speed = speed_rpm * 6.0;

      // 实时位置角度（单位：deg）
      uint16_t rt_angle_sign = node.getResponseBuffer(9);
      uint16_t rt_angle_high = node.getResponseBuffer(10);
      uint16_t rt_angle_low = node.getResponseBuffer(11);
      int32_t rt_angle_raw = ((uint32_t)rt_angle_high << 16) | rt_angle_low;
      if (rt_angle_sign == 1) rt_angle_raw = -rt_angle_raw;
      real_time_angle = rt_angle_raw * 360.0f / 65536.0f;

      // 位置角度误差（单位：deg）
      uint16_t error_sign = node.getResponseBuffer(12);
      uint16_t error_high = node.getResponseBuffer(13);
      uint16_t error_low = node.getResponseBuffer(14);
      int32_t error_raw = ((uint32_t)error_high << 16) | error_low;
      if (error_sign == 1) error_raw = -error_raw;
      angle_error = error_raw * 360.0f / 65536.0f;

      // 回零状态 & 电机状态
      uint16_t flags = node.getResponseBuffer(15);
      homing_status = flags >> 8;
      motor_status = flags & 0x0F;

      homing_fail = (homing_status & 0x04);
      homing_in_progress = ((homing_status & 0x03) == 0x03);

      enabled = (motor_status & 0x08);
      in_position = (motor_status & 0x04);
      stall = (motor_status & 0x02);
      stall_protect = (motor_status & 0x01);
      return true;
    }
    // Serial.print("❌ 读取失败，错误码: ");
    // Serial.println(node.getResponseBuffer(0), HEX);
  }
  return false;
}
void sendImmediateStop() {
  uint16_t stopRegister = 0x9800;
  uint16_t startAddress = 0x00FE;

  node.beginTransmission(0x01);  // 从机地址 0x01
  node.setTransmitBuffer(0, stopRegister);
  uint8_t result = node.writeMultipleRegisters(startAddress, 1);  // 写 1 个寄存器

  if (result != node.ku8MBSuccess) {
    // Serial.print("❌ 立即停止指令发送失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("🛑 已发送立即停止指令");
    // uint16_t respAddr = node.getResponseBuffer(0);
    // uint16_t respCount = node.getResponseBuffer(1);
    // Serial.print("响应起始地址: 0x");
    // Serial.println(respAddr, HEX);
    // Serial.print("响应寄存器数量: ");
    // Serial.println(respCount);
  }
}
//
// 电机使能控制：写寄存器 0x00F3，2 个寄存器
//   - 寄存器1：高位 0xAB，低位为控制字节（如 0x23 表示上使能）
//   - 寄存器2：固定为 0x0000（多机同步标志）
//
void setMotorEnable(uint8_t control_byte) {
  uint16_t regs[2];

  // 构造寄存器1：高字节 0xAB，低字节为控制字节
  regs[0] = ((uint16_t)0xAB << 8) | control_byte;
  regs[1] = 0x0000;  // 多机同步标志

  uint16_t startAddress = 0x00F3;

  node.beginTransmission(0x01);  // 从机地址 0x01
  node.setTransmitBuffer(0, regs[0]);
  node.setTransmitBuffer(1, regs[1]);

  uint8_t result = node.writeMultipleRegisters(startAddress, 2);
  if (result != node.ku8MBSuccess) {
    // Serial.print("❌ 电机使能指令发送失败，错误码: ");
    // Serial.println(result);
  } else {
    // Serial.println("✅ 电机使能指令发送成功:%d",control_byte);
    // uint16_t respAddr = node.getResponseBuffer(0);
    // uint16_t respCount = node.getResponseBuffer(1);
    // Serial.print("响应起始地址: 0x");
    // Serial.println(respAddr, HEX);
    // Serial.print("响应寄存器数量: ");
    // Serial.println(respCount);
  }
}
//
// 触发回零函数
// 参数：mode - 回零模式（如 0x01）
//
void sendHomingTrigger(uint8_t mode) {
  // 寄存器值：高 8 位为回零模式，低 8 位为多机同步标志（固定 0x00）
  uint16_t homing_register_value = ((uint16_t)mode << 8) | 0x00;

  // 起始寄存器地址（根据你提供的协议）为 0x009A
  const uint16_t startAddress = 0x009A;

  node.beginTransmission(0x01);  // 从机地址为 0x01
  node.setTransmitBuffer(0, homing_register_value);

  uint8_t result = node.writeMultipleRegisters(startAddress, 1);
  if (result != node.ku8MBSuccess) {
    // Serial.print("❌ 回零触发指令发送失败，错误码: ");
    // Serial.println(result, HEX);
  } else {
    // Serial.println("✅ 回零触发指令发送成功");
    // uint16_t respAddr = node.getResponseBuffer(0);
    // uint16_t respCount = node.getResponseBuffer(1);
    // Serial.print("响应起始地址: 0x");
    // Serial.println(respAddr, HEX);
    // Serial.print("响应寄存器数量: ");
    // Serial.println(respCount);
  }
}

//
// 发送 Modbus 数据（写多个寄存器）
//
void sendModbusData(int16_t direction, int16_t speed, uint32_t pulseCount,u_char accel) {
  uint16_t regs[5]; // 默认加速度为0

  // 寄存器0：高4位控制方向，低4位控制加速度
  regs[0] = ((uint16_t)(direction & 0x0F) << 12) | (accel & 0x0F);
  regs[1] = (uint16_t)speed;
  regs[2] = (uint16_t)(pulseCount >> 16);
  regs[3] = (uint16_t)(pulseCount & 0xFFFF);
  regs[4] = 0x0100;  // 触发/标志

  // Serial.println("发送的寄存器数据：");
  // for (int i = 0; i < 5; i++) {
  //   Serial.print("寄存器");
  //   Serial.print(i + 1);
  //   Serial.print(": 0x");
  //   Serial.println(regs[i], HEX);
  // }

  uint16_t startAddress = 0x00FD;
  node.beginTransmission(0x01);
  for (int i = 0; i < 5; i++) {
    node.setTransmitBuffer(i, regs[i]);
  }
  uint8_t result = node.writeMultipleRegisters(startAddress, 5);
  if (result != node.ku8MBSuccess) {
    // Serial.print("❌ Modbus 数据发送失败，错误码: ");
    // Serial.println(result);
  }
   else {
    // Serial.println("✅ Modbus 数据发送成功");
    // uint16_t respAddr = node.getResponseBuffer(0);
    // uint16_t respCount = node.getResponseBuffer(1);
    // Serial.print("响应起始地址: 0x");
    // Serial.println(respAddr, HEX);
    // Serial.print("响应寄存器数量: ");
    // Serial.println(respCount);
  }
}
