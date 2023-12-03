/*
 * I2cIrq.hpp
 *
 *  Created on: Sep 3, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CIRQ_HPP_
#define HAL_I2C_I2CIRQ_HPP_

#include "common.hpp"
#include "hal/i2c/I2cMaster.hpp"
#include "hal/i2c/I2cSlave.hpp"

namespace hal::i2c {

class I2cIrq {
 private:
  constexpr static size_t I2cCount = 2;

 public:
  // Deleted copy constructor and assignment operator to enforce singleton
  I2cIrq(const I2cIrq&) = delete;
  I2cIrq& operator=(const I2cIrq&) = delete;

  static I2cIrq& getInstance() {
    static I2cIrq instance;
    return instance;
  }

  Status_t registerI2cMaster(I2cMaster* i2c_master);
  void masterWriteCpltCb(I2C_HandleTypeDef* hi2c);
  void masterReadCpltCb(I2C_HandleTypeDef* hi2c);

  Status_t registerI2cSlave(I2cSlave* i2c_slave);

 private:
  I2cIrq();

  I2cMaster* i2c_master_[I2cCount] = { nullptr };
  I2cSlave* i2c_slave_[I2cCount] = { nullptr };
  size_t registered_master_ = 0;
  size_t registered_slave_ = 0;
};

}  // namespace hal::i2c

#endif /* HAL_I2C_I2CIRQ_HPP_ */
