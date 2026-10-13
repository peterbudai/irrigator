#pragma once

#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

// Shared by status_light.yaml and main_control.yaml: reads a single
// user_control register and logs the result, returning false on I2C error.
inline bool read_user_control_register(esphome::i2c::I2CDevice *device, uint8_t a_register, uint8_t *value) {
  auto error = device->read_register(a_register, value, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW("user_control", "User controller read error: %d", error);
    return false;
  }
  ESP_LOGD("user_control", "User controller register 0x%02X value: %d", a_register, *value);
  return true;
}

// Shared by status_light.yaml and main_control.yaml: writes a single
// user_control register and logs the result.
inline void write_user_control_register(esphome::i2c::I2CDevice *device, uint8_t a_register, uint8_t value) {
  uint8_t state = value;
  auto error = device->write_register(a_register, &state, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW("user_control", "User controller write error: %d", error);
    return;
  }
  ESP_LOGD("user_control", "User controller register 0x%02X set to: %d", a_register, state);
}
