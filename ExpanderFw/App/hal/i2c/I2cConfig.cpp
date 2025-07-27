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

namespace hal::i2c {

I2cConfig::I2cConfig(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle) : i2c_id_{ i2c_id }, i2c_handle_{ i2c_handle } {}

uint32_t I2cConfig::poll() {
  return (service_status_ == true) ? 1 : 0;
}

Status_t I2cConfig::scheduleRequest(Request* request, uint32_t seq_num) {
  request_ = *request;
  sequence_number_ = seq_num;
  service_status_ = true;

  DEBUG_INFO("Config I2c(%d) clock_freq: %d", magic_enum::enum_integer(i2c_id_), request_.clock_freq);
  DEBUG_INFO("Config I2c(%d) slave_addr: 0x%X", magic_enum::enum_integer(i2c_id_), request_.slave_addr);
  DEBUG_INFO("Config I2c(%d) addr_width: %s", magic_enum::enum_integer(i2c_id_),
             magic_enum::enum_name(request_.slave_addr_width).cbegin());

  uint32_t address_mode = 0;
  if (request_.slave_addr_width == SlaveAddrWidth::SevenBit) {
    address_mode = I2C_ADDRESSINGMODE_7BIT;
  } else if (request_.slave_addr_width == SlaveAddrWidth::TenBit) {
    address_mode = I2C_ADDRESSINGMODE_10BIT;
  } else {
    DEBUG_ERROR("Invalid slave_addr_width (%d) configuration!", static_cast<uint32_t>(request_.slave_addr_width));
    request_.status_code = RequestStatus::InvalidSlaveAddrWidth;
    return Status_t::Error;
  }

  uint32_t timing = 0;
  if (request_.clock_freq == 10000) {  // 10 kHz
    timing = 0x6010C7FF;
  } else if (request_.clock_freq == 40000) {  // 40 kHz
    timing = 0xC000102B;
  } else if (request_.clock_freq == 100000) {  // 100 kHz
    timing = 0x00707CBB;
  } else if (request_.clock_freq == 400000) {  // 400 kHz
    timing = 0x00300F38;
  } else if (request_.clock_freq == 1000000) {  // 1000 kHz
    timing = 0x00100413;
  } else {
    DEBUG_ERROR("Invalid clock_freq (%d) configuration!", request_.clock_freq);
    request_.status_code = RequestStatus::InvalidClockFreq;
    return Status_t::Error;
  }

  // Request is valid, set request status
  request_.status_code = RequestStatus::Ongoing;

  // DeInitialize I2c
  HAL_I2C_DeInit(i2c_handle_);

  // ReInitialize I2c
  if (i2c_id_ == I2cId::I2c0) {
    MX_I2C1_ReInit(address_mode, request_.slave_addr, timing, false);
  } else {
    MX_I2C3_ReInit(address_mode, request_.slave_addr, timing, false);
  }

  request_.status_code = RequestStatus::Complete;
  return Status_t::Ok;
};

Status_t I2cConfig::serviceStatus(StatusInfo* info) {
  Status_t status = Status_t::Ok;

  if (service_status_ == false) {
    DEBUG_ERROR("No request to service!");
    return Status_t::Error;
  }

  info->sequence_number = sequence_number_;
  info->request = request_;

  service_status_ = false;
  return status;
}

uint32_t I2cConfig::getSlaveAddress() const {
  return request_.slave_addr;
}

} /* namespace hal::i2c */
