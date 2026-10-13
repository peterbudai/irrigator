#pragma once

#include <cmath>
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

// Shared by valve_switch.yaml's sensor lambda: reads a single relay register
// and logs the result, returning NAN on error.
inline float read_relay_register(esphome::i2c::I2CDevice *device, uint8_t a_register) {
  uint8_t state = 0;
  auto error = device->read_register(a_register, &state, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW("valve_sensor", "Relay controller read error: %d", error);
    return NAN;
  }
  ESP_LOGD("valve_sensor", "Relay controller register 0x%02X value: %d", a_register, state);
  return state != 0 ? 1.0 : 0.0;
}

// Shared by valve_switch.yaml's turn_on_action/turn_off_action: writes a
// single relay register and logs the result.
inline void write_relay_register(esphome::i2c::I2CDevice *device, uint8_t a_register, uint8_t value) {
  uint8_t state = value;
  auto error = device->write_register(a_register, &state, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW("valve_switch", "Relay controller write error: %d", error);
    return;
  }
  ESP_LOGD("valve_switch", "Relay controller register 0x%02X set to: %d", a_register, state);
}
