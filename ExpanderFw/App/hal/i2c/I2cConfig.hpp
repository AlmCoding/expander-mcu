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

enum class SlaveAddrWidth {
  SevenBit = 0,
  TenBit,
};

class I2cConfig {
 public:
  I2cConfig(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cConfig() = default;

  void config(uint32_t clock_freq, uint32_t slave_addr, SlaveAddrWidth addr_width, bool pullups_enabled);

 private:
  I2cId i2c_id_;
  I2C_HandleTypeDef* i2c_handle_;
};

} /* namespace i2c */
} /* namespace hal */

#endif /* HAL_I2C_I2CCONFIG_HPP_ */
