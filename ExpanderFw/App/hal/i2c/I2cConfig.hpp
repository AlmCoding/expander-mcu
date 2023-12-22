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

class I2cConfig {
 public:
  I2cConfig(I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cConfig() = default;

 private:
  I2C_HandleTypeDef* i2c_handle_;

};

} /* namespace i2c */
} /* namespace hal */

#endif /* HAL_I2C_I2CCONFIG_HPP_ */
