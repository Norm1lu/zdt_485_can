#include "Arduino.h"
#include "config.h"
#include "modbus_handler.h"
#include "can_handler.h"
#include <ESP32-TWAI-CAN.hpp>

// 定义 Modbus 串口和 ModbusMaster 节点（全局变量供各模块使用）
HardwareSerial modbusSerial(2);
ModbusMaster node;


unsigned long lastModbusTime = 0; // 上次 Modbus 操作时间
unsigned long lastCanSendTime = 0; // 上次 CAN 发送时间
uint16_t feedbackHz = 50;
uint8_t feedbackEnable =1;
float diff_speed_deg_per_sec=0;
float filtered_speed = 0.0; // 滤波后的速度（°/s）
void setup() {
  Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, SERIAL_RX, SERIAL_TX)
  modbusSerial.begin(512000, SERIAL_8N1, MODBUS_RX, MODBUS_TX);
  // modbusSerial.setTimeout(100);
  node.begin(1, modbusSerial);

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

  if (twai_driver_install(&config, &timing, &filter) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("❌ CAN 初始化失败");
    while (1);
  }
  Serial.println("✅ CAN 初始化成功...");
}

void loop() {
  unsigned long now = millis();

  // 接收 CAN 控制帧
  twai_message_t message;
  if (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd && message.identifier == 0x00DF01fc && message.data_length_code == 8) {
      feedbackHz = (uint16_t)(message.data[1]) | ((uint16_t)(message.data[0]) << 8);
      feedbackEnable = message.data[2];
      uint8_t triggerHoming = message.data[3];     // d4：触发回零
      uint8_t motorEnable   = message.data[4];     // d5：电机使能
      uint8_t emergencyStop = message.data[5];     // d6：立即停止

      Serial.println("📥 收到控制反馈速度指令");
      Serial.printf("反馈频率：%d Hz，反馈使能: %d，触发回零: %d，使能: %d，立即停止: %d\n",
                    feedbackHz, feedbackEnable, triggerHoming, motorEnable, emergencyStop);

      if (feedbackHz >= 1 && feedbackHz <= 1000) {
        canSendFreqHz = feedbackHz;
      }

      if (emergencyStop == 1) {
        sendImmediateStop();
      }

      if (triggerHoming == 1) {
        sendHomingTrigger(2);
        Serial.println("⚠️ 触发回零动作");
      }

      if (motorEnable == 1) {
        setMotorEnable(1);
        Serial.println("✅ 电机使能");
      } else {
        setMotorEnable(0);
        Serial.println("⛔ 电机停止");
      }
    }
    else if (message.identifier == 0x00df01fd && message.data_length_code == 8) {
union { uint8_t b[4]; float f; } angleUnion;
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
      if(pulse<0)
      {
        pulse=-pulse;
        direction=1;
      }
      else direction=0;

      u_char accel = message.data[7];

      Serial.printf("📥 控制角度: %.2f°, 脉冲: %ld, 速度: %.2f rpm, 方向: %d, 加速度: %d\n",
                    angle_deg, pulse, speed_rpm, direction, accel);

      sendModbusData(direction, uint16_t(speed_rpm), pulse,accel);
      // delayMicroseconds(3000);  // 延迟 3000 微秒（3ms）

    }
  }

  // ------ Modbus 定时读取 ------
  static float last_angle = 0.0f;
  static unsigned long last_time = 0;

  unsigned long modbusInterval = 1000 / modbusReadFreqHz;
  if (now - lastModbusTime >= modbusInterval) {
    if (!readAllMotorStatus()) {
      Serial.println("❌ readAllMotorStatus 失败");
      // delayMicroseconds(3000);  // 延迟 3000 微秒（3ms）
    }

    // ✅ 微分估算速度（度/s）+ 滤波
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt > 0.001) {
      float raw_speed = (real_time_angle - last_angle) / dt;

      const float alpha = 0.1f;  // 滤波系数（越小越平稳）
      filtered_speed = alpha * raw_speed + (1 - alpha) * filtered_speed;

      // Serial.printf("📈 微分角速度估算: %.2f °/s，滤波后: %.2f °/s\n",
      //               raw_speed, filtered_speed);

      last_angle = real_time_angle;
      last_time = current_time;

      // ✅ 将滤波值覆盖实时速度（用于后续 CAN 发送）
      real_time_speed = filtered_speed;
    }

    lastModbusTime = now;
  }

  // ------ 定时发送 CAN 状态帧 ------
  unsigned long canInterval = 1000 / canSendFreqHz;
  if (feedbackEnable == 1 && now - lastCanSendTime >= canInterval) {
    uint8_t canData[8];

    // 实时角度 float 转字节（大端）
    union { float f; uint8_t b[4]; } angleUnion;
    angleUnion.f = real_time_angle;
    canData[0] = angleUnion.b[3];
    canData[1] = angleUnion.b[2];
    canData[2] = angleUnion.b[1];
    canData[3] = angleUnion.b[0];

    // 实时速度（已滤波）度/s -> int16 -> 发送
    int16_t speed_deg_per_sec = (int16_t)(real_time_speed);
    canData[4] = (uint8_t)(speed_deg_per_sec >> 8);
    canData[5] = (uint8_t)(speed_deg_per_sec & 0xFF);

    // 电流反馈（大端）
    canData[6] = (uint8_t)(phase_current >> 8);
    canData[7] = (uint8_t)(phase_current & 0xFF);

    sendCAN(0x00df0104, canData, 8);
    lastCanSendTime = now;
  }
}


