#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <ESP32-TWAI-CAN.hpp>

void sendCAN(uint32_t id, uint8_t *data, uint8_t len);

#endif  // CAN_HANDLER_H
