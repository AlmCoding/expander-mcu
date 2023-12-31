/*
 * I2cIrq.hpp
 *
 *  Created on: Sep 3, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CIRQ_HPP_
#define HAL_I2C_I2CIRQ_HPP_

#include "common.hpp"
#include "enum/magic_enum.hpp"
#include "hal/i2c/I2cConfig.hpp"
#include "hal/i2c/I2cMaster.hpp"
#include "hal/i2c/I2cSlave.hpp"

namespace hal::i2c {

class I2cIrq {
 private:
  constexpr static size_t I2cCount = magic_enum::enum_count<I2cId>();

 public:
  // Deleted copy constructor and assignment operator to enforce singleton
  I2cIrq(const I2cIrq&) = delete;
  I2cIrq& operator=(const I2cIrq&) = delete;

  static I2cIrq& getInstance() {
    static I2cIrq instance;
    return instance;
  }

  void registerI2cMaster(I2cMaster* i2c_master);
  I2cMaster* getMaster(I2C_HandleTypeDef* hi2c);
  void masterWriteCpltCb(I2C_HandleTypeDef* hi2c);
  void masterReadCpltCb(I2C_HandleTypeDef* hi2c);

  void registerI2cSlave(I2cSlave* i2c_slave);
  I2cSlave* getSlave(I2C_HandleTypeDef* hi2c);
  void enableSlaveListen(I2C_HandleTypeDef* hi2c);
  void disableSlaveListen(I2C_HandleTypeDef* hi2c);
  void slaveMatchMasterWriteCb(I2C_HandleTypeDef* hi2c);
  void slaveMatchMasterReadCb(I2C_HandleTypeDef* hi2c);
  void slaveListenCpltCb(I2C_HandleTypeDef* hi2c);

  // void slaveMasterWriteCpltCb(I2C_HandleTypeDef* hi2c);
  // void slaveMasterReadCpltCb(I2C_HandleTypeDef* hi2c);

 private:
  I2cIrq() = default;

  I2cMaster* i2c_master_[I2cCount] = {};
  I2cSlave* i2c_slave_[I2cCount] = {};
  bool slave_match_master_write = true;
};

}  // namespace hal::i2c

#endif /* HAL_I2C_I2CIRQ_HPP_ */
