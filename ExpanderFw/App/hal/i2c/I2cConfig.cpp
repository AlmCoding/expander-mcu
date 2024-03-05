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
  if (clock_freq == 100000) {  // 100 kHz
    timing = 0x30909DEC;
  } else if (clock_freq == 400000) {  // 400 kHz
    timing = 0x00F07BFF;
  } else if (clock_freq == 100000) {  // 1000 kHz
    timing = 0x00701F6B;
  } else {
    timing = 0x00F07BFF;  // 400 kHz
    DEBUG_ERROR("Invalid clock_freq (%d) configuration. Fallback to 400 kHz!", clock_freq);
  }

  // DeInitialize I2c
  HAL_I2C_MspDeInit(i2c_handle_);

  if (i2c_id_ == I2cId::I2c0) {
    i2c_handle_->Instance = I2C1;
  } else {
    i2c_handle_->Instance = I2C2;
  }
  i2c_handle_->Init.Timing = timing;
  i2c_handle_->Init.OwnAddress1 = slave_addr << 1;
  i2c_handle_->Init.AddressingMode =
      (addr_width == SlaveAddrWidth::SevenBit) ? I2C_ADDRESSINGMODE_7BIT : I2C_ADDRESSINGMODE_10BIT;
  i2c_handle_->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  i2c_handle_->Init.OwnAddress2 = 0;
  i2c_handle_->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  i2c_handle_->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  i2c_handle_->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(i2c_handle_) != HAL_OK) {
    Error_Handler();
  }

  /** Configure pullups
   */
  config_pullups(pullups_enabled);

  /** Configure Analog filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(i2c_handle_, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(i2c_handle_, 0) != HAL_OK) {
    Error_Handler();
  }
};

void I2cConfig::config_pullups(bool pullups_enabled) {
  GPIO_InitTypeDef GPIO_InitStruct = {};

  if (i2c_id_ == I2cId::I2c0) {
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = (pullups_enabled == true) ? GPIO_PULLUP : GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  } else {
    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = (pullups_enabled == true) ? GPIO_PULLUP : GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  }
}

} /* namespace i2c */
} /* namespace hal */
