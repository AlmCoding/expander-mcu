/*
 * I2cIrq.cpp
 *
 *  Created on: Sep 3, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cIrq.hpp"
#include "enum/magic_enum.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_IRQ 1
#if ((DEBUG_ENABLE_I2C_IRQ == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cIrq]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cIrq]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cIrq]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::i2c {

I2cId I2cIrq::getId(I2C_HandleTypeDef* hi2c) {
  for (size_t i = 0; i < I2cCount; i++) {
    if (i2c_master_[i]->i2c_handle_ == hi2c) {
      return i2c_master_[i]->i2c_id_;
    }
  }

  DEBUG_ERROR("Invalid I2C handle!");
  ETL_ASSERT(false, ETL_ERROR(0));
  return I2cId::I2c0;
}

void I2cIrq::registerI2cMaster(I2cMaster* i2c_master) {
  ETL_ASSERT(i2c_master != nullptr, ETL_ERROR(0));
  size_t idx = magic_enum::enum_integer(i2c_master->i2c_id_);

  // Check if already registered
  if (i2c_master_[idx] == i2c_master) {
    DEBUG_INFO("Register I2cMaster(%d) [SKIPPED]", idx);
    return;
  }

  DEBUG_INFO("Register I2cMaster(%d) [OK]", idx);
  ETL_ASSERT(i2c_master_[idx] == nullptr, ETL_ERROR(0));
  i2c_master_[idx] = i2c_master;
}

I2cMaster* I2cIrq::getMaster(I2C_HandleTypeDef* hi2c) {
  for (size_t i = 0; i < I2cCount; i++) {
    if (i2c_master_[i]->i2c_handle_ == hi2c) {
      return i2c_master_[i];
    }
  }
  return nullptr;
}

void I2cIrq::masterWriteCpltCb(I2C_HandleTypeDef* hi2c) {
  I2cMaster* master = getMaster(hi2c);
  if (master != nullptr) {
    master->writeCompleteCb();
  }
}

void I2cIrq::masterReadCpltCb(I2C_HandleTypeDef* hi2c) {
  I2cMaster* master = getMaster(hi2c);
  if (master != nullptr) {
    master->readCompleteCb();
  }
}

void I2cIrq::masterErrorCb(I2C_HandleTypeDef* hi2c) {
  I2cMaster* master = getMaster(hi2c);
  if (master != nullptr) {
    master->errorCb();
  }
}

void I2cIrq::registerI2cSlave(I2cSlave* i2c_slave) {
  ETL_ASSERT(i2c_slave != nullptr, ETL_ERROR(0));
  size_t idx = magic_enum::enum_integer(i2c_slave->i2c_id_);

  // Check if already registered
  if (i2c_slave_[idx] == i2c_slave) {
    DEBUG_INFO("Register I2cSlave(%d) [SKIPPED]", idx);
    return;
  }

  DEBUG_INFO("Register I2cSlave(%d) [OK]", idx);
  ETL_ASSERT(i2c_slave_[idx] == nullptr, ETL_ERROR(0));
  i2c_slave_[idx] = i2c_slave;
}

I2cSlave* I2cIrq::getSlave(I2C_HandleTypeDef* hi2c) {
  for (size_t i = 0; i < I2cCount; i++) {
    if (i2c_slave_[i]->i2c_handle_ == hi2c) {
      return i2c_slave_[i];
    }
  }
  return nullptr;
}

void I2cIrq::enableSlaveListen(I2C_HandleTypeDef* hi2c) {
  HAL_StatusTypeDef hal_status = HAL_I2C_EnableListen_IT(hi2c);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Enable I2cSlave(%d) listen [FAILED]", magic_enum::enum_integer(getId(hi2c)));
  }
  // ETL_ASSERT(hal_status == HAL_OK, ETL_ERROR(0));
}

void I2cIrq::disableSlaveListen(I2C_HandleTypeDef* hi2c) {
  HAL_StatusTypeDef hal_status = HAL_I2C_DisableListen_IT(hi2c);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Disable I2cSlave(%d) listen [FAILED]", magic_enum::enum_integer(getId(hi2c)));
  }
  // ETL_ASSERT(hal_status == HAL_OK, ETL_ERROR(0));
}

void I2cIrq::slaveMatchMasterWriteCb(I2C_HandleTypeDef* hi2c) {
  I2cSlave* slave = getSlave(hi2c);
  if (slave != nullptr) {
    slave_match_master_write = true;
    slave->slaveMatchMasterWriteCb();
  }
}

void I2cIrq::slaveMatchMasterReadCb(I2C_HandleTypeDef* hi2c) {
  I2cSlave* slave = getSlave(hi2c);
  if (slave != nullptr) {
    slave_match_master_write = false;
    slave->slaveMatchMasterReadCb();
  }
}

void I2cIrq::slaveListenCpltCb(I2C_HandleTypeDef* hi2c) {
  I2cSlave* slave = getSlave(hi2c);
  if (slave != nullptr) {
    if (slave_match_master_write == true) {
      slave->writeCompleteCb();
    } else {
      slave->readCompleteCb();
    }
    enableSlaveListen(hi2c);
  }
}

/*
void I2cIrq::slaveMasterWriteCpltCb(I2C_HandleTypeDef* hi2c) {
  I2cSlave* slave = getSlave(hi2c);
  if (slave != nullptr) {
    slave->writeCompleteCb();
  }
}

void I2cIrq::slaveMasterReadCpltCb(I2C_HandleTypeDef* hi2c) {
  I2cSlave* slave = getSlave(hi2c);
  if (slave != nullptr) {
    slave->readCompleteCb();
  }
}
*/

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

void HAL_I2C_AddrCallback(I2C_HandleTypeDef* hi2c, uint8_t TransferDirection, uint16_t /*AddrMatchCode*/) {
  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    // Master TX slave RX
    I2cIrq::getInstance().slaveMatchMasterWriteCb(hi2c);
  } else {
    // Master RX slave TX
    I2cIrq::getInstance().slaveMatchMasterReadCb(hi2c);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef* hi2c) {
  I2cIrq::getInstance().slaveListenCpltCb(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef* /*hi2c*/) {}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef* /*hi2c*/) {}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* /*hi2c*/) {}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  uint32_t error = HAL_I2C_GetError(hi2c);

  DEBUG_ERROR("Error callback I2c(%d) (error: %d)", magic_enum::enum_integer(I2cIrq::getInstance().getId(hi2c)), error);

  if (error == HAL_I2C_ERROR_AF) {
    I2cIrq::getInstance().masterErrorCb(hi2c);
  }
}

}  // extern "C"
}  // namespace hal::i2c
