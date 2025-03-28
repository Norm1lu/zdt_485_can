#include "Arduino.h"
#include "config.h"
#include "modbus_handler.h"
#include "can_handler.h"
#include <ESP32-TWAI-CAN.hpp>
#include <SoftwareSerial.h>

// 定义 Modbus 串口和 ModbusMaster 节点（全局变量供各模块使用）
ModbusMaster node;
ModbusMaster node2;
// 定义 Modbus 软件串口（电机3）

// SoftwareSerial modbusSerial3(MODBUS3_RX, MODBUS3_TX);  // 请将 RX_PIN 和 TX_PIN 替换为实际的引脚编号
// ModbusMaster node3;
ModbusMaster node3;

unsigned long lastModbusTime = 0;   // 上次 Modbus 操作时间
unsigned long lastCanSendTime = 0;  // 上次 CAN 发送时间
uint16_t feedbackHz = 50;
uint8_t feedbackEnable = 1;
float diff_speed_deg_per_sec = 0;
float filtered_speed = 0.0;  // 滤波后的速度（°/s）
void setup() {
  // Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, SERIAL_RX, SERIAL_TX)
  Serial2.begin(256000, SERIAL_8N1, MODBUS_RX, MODBUS_TX);  // TX/RX: 你自定义的引脚
  Serial2.setTimeout(10);
  node.begin(1, Serial2);

  // ✅ 电机2 - UART1
  Serial1.begin(256000, SERIAL_8N1, MODBUS2_RX, MODBUS2_TX);  // TX/RX: 你自定义的引脚
  Serial1.setTimeout(10);
  node2.begin(1, Serial1);
  Serial.begin(256000);
  // communicate with Modbus slave ID 2 over Serial (port 0)
  // node3.begin(1, Serial);
  // modbusSerial3.begin(115200, SWSERIAL_8N1, MODBUS3_RX, MODBUS3_TX);  // false: 不反转逻辑电平
  // node3.begin(5, modbusSerial3);                                             // Modbus地址3，对应第三路电机


  // 初始化 CAN
  twai_general_config_t config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = CAN_TX_PIN,
    .rx_io = CAN_RX_PIN,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 200,
    .rx_queue_len = 200,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1
  };
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&config, &timing, &filter) != ESP_OK || twai_start() != ESP_OK) {
    // Serial.println("❌ CAN 初始化失败");
    while (1)
      ;
  }
  // Serial.println("✅ CAN 初始化成功...");
}

void loop() {
  static unsigned long lastCycleTime_us = 0;
  unsigned long now_us = micros();
  unsigned long cycleInterval_us = 1e6 / canSendFreqHz;

  if (now_us - lastCycleTime_us >= cycleInterval_us) {
    lastCycleTime_us += cycleInterval_us;  // 精确周期补偿

    // ===== 处理 CAN 指令接收 =====
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
      if (message.extd && message.identifier == 0x00DF01fc && message.data_length_code == 8) {
        feedbackHz = (uint16_t)(message.data[1]) | ((uint16_t)(message.data[0]) << 8);
        feedbackEnable = message.data[2];
        uint8_t triggerHoming = message.data[3];
        uint8_t motorEnable = message.data[4];
        uint8_t emergencyStop = message.data[5];

        if (feedbackHz >= 1 && feedbackHz <= 1000) {
          canSendFreqHz = feedbackHz;
          cycleInterval_us = 1e6 / canSendFreqHz;
        }

        if (emergencyStop == 1) sendImmediateStop();
        if (triggerHoming == 1) sendHomingTrigger(2);
        if (motorEnable == 1) setMotorEnable(1);
        else setMotorEnable(0);
        // 打印接收到的电机1控制参数
        // Serial.println("=== 电机1控制参数接收 ===");
        // Serial.print("反馈频率: "); Serial.println(feedbackHz);
        // Serial.print("反馈使能: "); Serial.println(feedbackEnable);
        // Serial.print("触发回零: "); Serial.println(triggerHoming);
        // Serial.print("电机使能: "); Serial.println(motorEnable);
        // Serial.print("急停: "); Serial.println(emergencyStop);
      } else if (message.identifier == 0x00df01fd && message.data_length_code == 8) {
        union {
          uint8_t b[4];
          float f;
        } angleUnion;
        angleUnion.b[3] = message.data[0];
        angleUnion.b[2] = message.data[1];
        angleUnion.b[1] = message.data[2];
        angleUnion.b[0] = message.data[3];
        float angle_deg = angleUnion.f;
        float factor = 3200.0f / 360.0f;
        int32_t pulse = (int32_t)(angle_deg * factor);

        uint16_t speed_deg_per_sec = (message.data[4] << 8) | message.data[5];
        float speed_rpm = speed_deg_per_sec / 6.0;

        uint8_t direction = message.data[6];
        if (pulse < 0) {
          pulse = -pulse;
          direction = 1;
        } else direction = 0;

        u_char accel = message.data[7];
        // 打印接收到的电机1角度控制数据
        // Serial.println("=== 电机1角度控制接收 ===");
        // Serial.print("角度 (deg): "); Serial.println(angle_deg);
        // Serial.print("脉冲: "); Serial.println(pulse);
        // Serial.print("速度 (rpm): "); Serial.println(speed_rpm);
        // Serial.print("方向: "); Serial.println(direction);
        // Serial.print("加速度: "); Serial.println(accel);
        sendModbusData(direction, uint16_t(speed_rpm), pulse, accel);
      } else if (message.identifier == 0x00df02fc && message.data_length_code == 8) {
        feedbackHz = (uint16_t)(message.data[1]) | ((uint16_t)(message.data[0]) << 8);
        feedbackEnable = message.data[2];
        uint8_t triggerHoming = message.data[3];
        uint8_t motorEnable = message.data[4];
        uint8_t emergencyStop = message.data[5];

        if (emergencyStop == 1) sendImmediateStop2();
        if (triggerHoming == 1) sendHomingTrigger2(2);
        if (motorEnable == 1) setMotorEnable2(1);
        else setMotorEnable2(0);
        // 打印接收到的电机2控制参数
        // Serial.println("=== 电机2控制参数接收 ===");
        // Serial.print("反馈频率: "); Serial.println(feedbackHz);
        // Serial.print("反馈使能: "); Serial.println(feedbackEnable);
        // Serial.print("触发回零: "); Serial.println(triggerHoming);
        // Serial.print("电机使能: "); Serial.println(motorEnable);
        // Serial.print("急停: "); Serial.println(emergencyStop);
      } else if (message.identifier == 0x00df02fd && message.data_length_code == 8) {
        union {
          uint8_t b[4];
          float f;
        } angleUnion;
        angleUnion.b[3] = message.data[0];
        angleUnion.b[2] = message.data[1];
        angleUnion.b[1] = message.data[2];
        angleUnion.b[0] = message.data[3];
        float angle_deg = angleUnion.f;
        float factor = 3200.0f / 360.0f;
        int32_t pulse = (int32_t)(angle_deg * factor);

        uint16_t speed_deg_per_sec = (message.data[4] << 8) | message.data[5];
        float speed_rpm = speed_deg_per_sec / 6.0;

        uint8_t direction = message.data[6];
        if (pulse < 0) {
          pulse = -pulse;
          direction = 1;
        } else direction = 0;

        u_char accel = message.data[7];
        // 打印接收到的电机2角度控制数据
        // Serial.println("=== 电机2角度控制接收 ===");
        // Serial.print("角度 (deg): "); Serial.println(angle_deg);
        // Serial.print("脉冲: "); Serial.println(pulse);
        // Serial.print("速度 (rpm): "); Serial.println(speed_rpm);
        // Serial.print("方向: "); Serial.println(direction);
        // Serial.print("加速度: "); Serial.println(accel);
        sendModbusData2(direction, uint16_t(speed_rpm), pulse, accel);
      } 
      // else if (message.identifier == 0x00df03fc && message.data_length_code == 8) {
      //   feedbackHz = (uint16_t)(message.data[1]) | ((uint16_t)(message.data[0]) << 8);
      //   feedbackEnable = message.data[2];
      //   uint8_t triggerHoming = message.data[3];
      //   uint8_t motorEnable = message.data[4];
      //   uint8_t emergencyStop = message.data[5];

      //   if (emergencyStop == 1) sendImmediateStop3();
      //   if (triggerHoming == 1) sendHomingTrigger3(2);
      //   if (motorEnable == 1) setMotorEnable3(1);
      //   else setMotorEnable3(0);

      //   // 可选调试打印
      //   // Serial.println("=== 电机3控制参数接收 ===");
      //   // Serial.print("反馈频率: "); Serial.println(feedbackHz);
      //   // Serial.print("反馈使能: "); Serial.println(feedbackEnable);
      //   // Serial.print("触发回零: "); Serial.println(triggerHoming);
      //   // Serial.print("电机使能: "); Serial.println(motorEnable);
      //   // Serial.print("急停: "); Serial.println(emergencyStop);
      // } else if (message.identifier == 0x00df03fd && message.data_length_code == 8) {
      //   union {
      //     uint8_t b[4];
      //     float f;
      //   } angleUnion;
      //   angleUnion.b[3] = message.data[0];
      //   angleUnion.b[2] = message.data[1];
      //   angleUnion.b[1] = message.data[2];
      //   angleUnion.b[0] = message.data[3];
      //   float angle_deg = angleUnion.f;
      //   float factor = 3200.0f / 360.0f;
      //   int32_t pulse = (int32_t)(angle_deg * factor);

      //   uint16_t speed_deg_per_sec = (message.data[4] << 8) | message.data[5];
      //   float speed_rpm = speed_deg_per_sec / 6.0;

      //   uint8_t direction = message.data[6];
      //   if (pulse < 0) {
      //     pulse = -pulse;
      //     direction = 1;
      //   } else direction = 0;

      //   u_char accel = message.data[7];

      //   // 可选调试打印
      //   // Serial.println("=== 电机3角度控制接收 ===");
      //   // Serial.print("角度 (deg): "); Serial.println(angle_deg);
      //   // Serial.print("脉冲: "); Serial.println(pulse);
      //   // Serial.print("速度 (rpm): "); Serial.println(speed_rpm);
      //   // Serial.print("方向: "); Serial.println(direction);
      //   // Serial.print("加速度: "); Serial.println(accel);

      //   sendModbusData3(direction, uint16_t(speed_rpm), pulse, accel);
      // }
    }

    // ===== 电机状态读取 + 速度估算（滤波） =====
    static float last_angle_1 = 0.0f;
    static float last_angle_2 = 0.0f;
    static float last_angle_3 = 0.0f;
    static unsigned long last_time_1_us = 0;
    static unsigned long last_time_2_us = 0;
    static unsigned long last_time_3_us = 0;
    static float filtered_speed1 = 0.0f;
    static float filtered_speed2 = 0.0f;
    static float filtered_speed3 = 0.0f;
    delayMicroseconds(10);
    if (readAllMotorStatus()) {
      float dt = (now_us - last_time_1_us) / 1000000.0f;
      if (dt > 0.0005f) {
        float raw_speed = (real_time_angle - last_angle_1) / dt;
        filtered_speed1 = 0.1f * raw_speed + 0.9f * filtered_speed1;
        real_time_speed = filtered_speed1;
        last_angle_1 = real_time_angle;
        last_time_1_us = now_us;
      }
    }
    delayMicroseconds(10);
    if (readAllMotorStatus2()) {
      float dt = (now_us - last_time_2_us) / 1000000.0f;
      if (dt > 0.0005f) {
        float raw_speed2 = (real_time_angle2 - last_angle_2) / dt;
        filtered_speed2 = 0.1f * raw_speed2 + 0.9f * filtered_speed2;
        real_time_speed2 = filtered_speed2;
        last_angle_2 = real_time_angle2;
        last_time_2_us = now_us;
      }
    }
    //  delayMicroseconds(10);
    // if (readAllMotorStatus3()) {
    //   float dt = (now_us - last_time_3_us) / 1000000.0f;
    //   if (dt > 0.0005f) {
    //     float raw_speed3 = (real_time_angle3 - last_angle_3) / dt;
    //     filtered_speed3 = 0.1f * raw_speed3 + 0.9f * filtered_speed3;
    //     real_time_speed3 = filtered_speed3;
    //     last_angle_3 = real_time_angle3;
    //     last_time_3_us = now_us;
    //   }
    // }


    // ===== 发送 CAN 状态帧 =====
    if (feedbackEnable == 1) {
      // 电机1
      uint8_t canData1[8];
      union {
        float f;
        uint8_t b[4];
      } angleUnion1;
      angleUnion1.f = real_time_angle;
      canData1[0] = angleUnion1.b[3];
      canData1[1] = angleUnion1.b[2];
      canData1[2] = angleUnion1.b[1];
      canData1[3] = angleUnion1.b[0];
      int16_t speed1 = (int16_t)(real_time_speed * 10);
      canData1[4] = speed1 >> 8;
      canData1[5] = speed1 & 0xFF;
      canData1[6] = phase_current >> 8;
      canData1[7] = phase_current & 0xFF;
      sendCAN(0x00df0104, canData1, 8);

      // 电机2
      uint8_t canData2[8];
      union {
        float f;
        uint8_t b[4];
      } angleUnion2;
      angleUnion2.f = real_time_angle2;
      canData2[0] = angleUnion2.b[3];
      canData2[1] = angleUnion2.b[2];
      canData2[2] = angleUnion2.b[1];
      canData2[3] = angleUnion2.b[0];
      int16_t speed2 = (int16_t)(real_time_speed2 * 10);
      canData2[4] = speed2 >> 8;
      canData2[5] = speed2 & 0xFF;
      canData2[6] = phase_current2 >> 8;
      canData2[7] = phase_current2 & 0xFF;
      sendCAN(0x00df0204, canData2, 8);
      // 电机3
      // uint8_t canData3[8];
      // union {
      //   float f;
      //   uint8_t b[4];
      // } angleUnion3;
      // angleUnion3.f = real_time_angle3;
      // canData3[0] = angleUnion3.b[3];
      // canData3[1] = angleUnion3.b[2];
      // canData3[2] = angleUnion3.b[1];
      // canData3[3] = angleUnion3.b[0];
      // int16_t speed3 = (int16_t)(real_time_speed3 * 10);
      // canData3[4] = speed3 >> 8;
      // canData3[5] = speed3 & 0xFF;
      // canData3[6] = phase_current3 >> 8;
      // canData3[7] = phase_current3 & 0xFF;
      // sendCAN(0x00df0304, canData3, 8);
    }
  }
}
