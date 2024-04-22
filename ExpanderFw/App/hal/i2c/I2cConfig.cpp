/*
 * I2cConfig.cpp
 *
 *  Created on: Dec 18, 2023
 *      Author: Alexander
 */

#include "hal/i2c/I2cConfig.hpp"
#include "enum/magic_enum.hpp"
#include "i2c.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_CONFIG 1
#if ((DEBUG_ENABLE_I2C_CONFIG == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cCfg]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal {
namespace i2c {

I2cConfig::I2cConfig(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle) : i2c_id_{ i2c_id }, i2c_handle_{ i2c_handle } {}

void I2cConfig::config(uint32_t clock_freq, uint32_t slave_addr, SlaveAddrWidth addr_width, bool pullups_enabled) {
  DEBUG_INFO("Config I2c(%d) clock_freq: %d", magic_enum::enum_integer(i2c_id_), clock_freq);
  DEBUG_INFO("Config I2c(%d) slave_addr: %d", magic_enum::enum_integer(i2c_id_), slave_addr);
  DEBUG_INFO("Config I2c(%d) addr_width: %s", magic_enum::enum_integer(i2c_id_),
             magic_enum::enum_name(addr_width).cbegin());
  DEBUG_INFO("Config I2c(%d) pullups_enabled: %d", magic_enum::enum_integer(i2c_id_), pullups_enabled);

  uint32_t timing = 0;
  uint32_t address_mode = (addr_width == SlaveAddrWidth::SevenBit) ? I2C_ADDRESSINGMODE_7BIT : I2C_ADDRESSINGMODE_10BIT;

  if (clock_freq == 100000) {  // 100 kHz
    timing = 0x30909DEC;
  } else if (clock_freq == 400000) {  // 400 kHz
    timing = 0x00F07BFF;
  } else if (clock_freq == 1000000) {  // 1000 kHz
    timing = 0x00701F6B;
  } else {
    timing = 0x00F07BFF;  // 400 kHz
    DEBUG_ERROR("Invalid clock_freq (%d) configuration. Fallback to 400 kHz!", clock_freq);
  }

  // DeInitialize I2c
  HAL_I2C_DeInit(i2c_handle_);

  // ReInitialize I2c
  if (i2c_id_ == I2cId::I2c0) {
    MX_I2C1_ReInit(timing, address_mode, pullups_enabled);
  } else {
    MX_I2C3_ReInit(timing, address_mode, pullups_enabled);
  }
};

} /* namespace i2c */
} /* namespace hal */
