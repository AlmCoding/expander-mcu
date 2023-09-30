/*
 * I2cIrq.cpp
 *
 *  Created on: Sep 3, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cIrq.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_IRQ
#ifdef DEBUG_ENABLE_I2C_IRQ
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cIrq]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::i2c {

I2cIrq::I2cIrq() {}

Status_t I2cIrq::registerI2cMaster(I2cMaster* i2c_master) {
  Status_t status;

  if (i2c_master == nullptr) {
    DEBUG_ERROR("Invalid I2cMaster register attempt")
    return Status_t::Error;
  }

  // Check if already registered
  for (size_t i = 0; i < registered_master_; i++) {
    if (i2c_master_[i] == i2c_master) {
      return Status_t::Ok;
    }
  }

  if (registered_master_ < I2cCount) {
    DEBUG_INFO("Register I2cMaster(%d) [ok]", registered_master_)
    i2c_master_[registered_master_] = i2c_master;
    registered_master_++;
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Register I2cMaster(%d) [failed]", registered_master_)
    status = Status_t::Error;
  }

  return status;
}

void I2cIrq::masterWriteCpltCb(I2C_HandleTypeDef* hi2c) {
  for (size_t i = 0; i < registered_master_; i++) {
    if (i2c_master_[i]->i2c_handle_ == hi2c) {
      i2c_master_[i]->writeCompleteCb();
      break;
    }
  }
}

void I2cIrq::masterReadCpltCb(I2C_HandleTypeDef* hi2c) {
  for (size_t i = 0; i < registered_master_; i++) {
    if (i2c_master_[i]->i2c_handle_ == hi2c) {
      i2c_master_[i]->readCompleteCb();
      break;
    }
  }
}

extern "C" {
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  I2cIrq::getInstance().masterWriteCpltCb(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  I2cIrq::getInstance().masterReadCpltCb(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  I2cIrq::getInstance().masterWriteCpltCb(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  I2cIrq::getInstance().masterReadCpltCb(hi2c);
}

}  // extern "C"

}  // namespace hal::i2c
