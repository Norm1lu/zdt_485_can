#include "can_handler.h"
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

void sendCAN(uint32_t  id, uint8_t *data, uint8_t len) {
  twai_message_t message;
  message.identifier = id;
  message.extd = true;
  message.rtr  = false;
  message.data_length_code = len;
  memcpy(message.data, data, len);

  twai_status_info_t status_info;
  if (twai_get_status_info(&status_info) == ESP_OK) {
    if (status_info.msgs_to_tx < 20) {
      if (twai_transmit(&message, 0) != ESP_OK) {
        Serial.println("❌ 发送 CAN 数据失败");
      }
    }
  }
}
