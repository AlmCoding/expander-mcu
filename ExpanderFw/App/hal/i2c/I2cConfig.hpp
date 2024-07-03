/*
 * I2cConfig.hpp
 *
 *  Created on: Dec 18, 2023
 *      Author: Alexander
 */

#ifndef HAL_I2C_I2CCONFIG_HPP_
#define HAL_I2C_I2CCONFIG_HPP_

#include "common.hpp"
#include "main.h"
#include "tx_api.h"

namespace hal {
namespace i2c {

enum class I2cId {
  I2c0 = 0,
  I2c1,
};

class I2cConfig {
 public:
  enum class RequestStatus {
    Ok = 0,
    InvalidClockFreq,
    InvalidSlaveAddr,
    InvalidSlaveAddrWidth,
    InvalidMemAddrWidth,
    InterfaceError,
  };

  enum class SlaveAddrWidth {
    SevenBit = 0,
    TenBit,
  };

  typedef struct {
    uint32_t request_id;
    RequestStatus status_code;
    uint32_t clock_freq;
    uint32_t slave_addr;
    SlaveAddrWidth slave_addr_width;
    bool pullups_enabled;
  } Request;

  typedef struct {
    uint32_t sequence_number;
    uint32_t request_id;
    RequestStatus status_code;
  } StatusInfo;

  I2cConfig(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cConfig() = default;

  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info);

 private:
  I2cId i2c_id_;
  I2C_HandleTypeDef* i2c_handle_;

  Request request_ = {};
  bool service_status_ = false;

  uint32_t seqence_number_ = 0;
};

} /* namespace i2c */
} /* namespace hal */

#endif /* HAL_I2C_I2CCONFIG_HPP_ */
