#pragma once

#include "esphome/core/log.h"
#include "esphome/core/optional.h"
#include "esphome/components/i2c/i2c.h"

// Reads a single register from an I2C device, logging the result under `tag`.
// Returns an empty optional (and logs a warning) on I2C error.
inline esphome::optional<uint8_t> read_i2c_register(esphome::i2c::I2CDevice *device, uint8_t a_register,
                                                     const char *tag) {
  uint8_t value = 0;
  auto error = device->read_register(a_register, &value, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW(tag, "I2C register %02X:%02X read error: %d", device->get_i2c_address(), a_register, error);
    return {};
  }
  ESP_LOGD(tag, "I2C register %02X:%02X value: %d", device->get_i2c_address(), a_register, value);
  return value;
}

// Writes a single register to an I2C device, logging the result under `tag`.
inline void write_i2c_register(esphome::i2c::I2CDevice *device, uint8_t a_register, uint8_t value, const char *tag) {
  uint8_t state = value;
  auto error = device->write_register(a_register, &state, 1);
  if (error != esphome::i2c::ERROR_OK) {
    ESP_LOGW(tag, "I2C register %02X:%02X write error: %d", device->get_i2c_address(), a_register, error);
    return;
  }
  ESP_LOGD(tag, "I2C register %02X:%02X set to: %d", device->get_i2c_address(), a_register, state);
}
