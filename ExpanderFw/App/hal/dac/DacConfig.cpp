/*
 * DacConfig.cpp
 *
 *  Created on: Jul 24, 2025
 *      Author: Alexander L.
 */

#include "hal/dac/DacConfig.hpp"
#include "enum/magic_enum.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "hal/dac/DACxx6x.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_DAC_CONFIG 1
#if ((DEBUG_ENABLE_DAC_CONFIG == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][DacCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][DacCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][DacCfg]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::dac {

DacConfig::DacConfig(SPI_HandleTypeDef* spi_handle) : spi_handle_{ spi_handle } {}

Status_t DacConfig::config() {
  Status_t status = init();

  if (status == Status_t::Ok) {
    DEBUG_INFO("Config DacConfig [OK]");
  } else {
    DEBUG_ERROR("Config DacConfig [FAILED]");
    status = Status_t::Error;
  }

  ETL_ASSERT(status == Status_t::Ok, ETL_ERROR(0));
  return status;
}

Status_t DacConfig::init() {
  Status_t status = Status_t::Ok;

  service_status_ = false;

  // Reset the DAC
  if (softwareReset() != Status_t::Ok) {
    DEBUG_ERROR("Software reset [FAILED]");
    return status;
  }

  // Enable internal voltage reference
  if (enableInternalVoltageRef() != Status_t::Ok) {
    DEBUG_ERROR("Enable internal voltage reference [FAILED]");
    return status;
  }

  // Set default gains
  if (setDefaultGains() != Status_t::Ok) {
    DEBUG_ERROR("Set default gains [FAILED]");
    return status;
  }

  // Set default values
  if (setDefaultValues() != Status_t::Ok) {
    DEBUG_ERROR("Set default values [FAILED]");
    return status;
  }

  return status;
}

uint32_t DacConfig::poll() {
  return (service_status_ == true) ? 1 : 0;
}

Status_t DacConfig::scheduleRequest(Request* request, uint32_t seq_num) {
  request_ = *request;
  seqence_number_ = seq_num;
  service_status_ = true;

  DEBUG_INFO("DacConfig mode: %s", magic_enum::enum_name(request_.mode).cbegin());
  DEBUG_INFO("DacConfig sampling_rate: %d", request_.sampling_rate);
  DEBUG_INFO("DacConfig periodic_samples: %d", request_.periodic_samples);

  if ((request_.mode != Mode::Static) && (request_.mode != Mode::Periodic) && (request_.mode != Mode::Streaming)) {
    DEBUG_ERROR("Invalid mode (%d) configuration!", static_cast<uint32_t>(request_.mode));
    request_.status_code = RequestStatus::InvalidMode;
    return Status_t::Error;
  }

  request_.status_code = RequestStatus::Ongoing;

  // TODO ...

  request_.status_code = RequestStatus::Complete;
  return Status_t::Ok;
}

Status_t DacConfig::serviceStatus(StatusInfo* info) {
  Status_t status = Status_t::Ok;

  if (service_status_ == false) {
    DEBUG_ERROR("No request to service!");
    return Status_t::Error;
  }

  info->sequence_number = seqence_number_;
  info->request = request_;

  service_status_ = false;
  return status;
}

Status_t DacConfig::softwareReset() {
  HAL_StatusTypeDef hal_status;
  uint8_t data[] = { DAC_CMD_SOFTWARE_RESET, 0, DAC_DATA_POWER_ON_RESET };

  hal_status = HAL_SPI_Transmit(spi_handle_, data, sizeof(data), 0);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Software reset failed (HAL error: %d)", hal_status);
    return Status_t::Error;
  }
  return Status_t::Ok;
}

Status_t DacConfig::enableInternalVoltageRef() {
  // Note: Resets gains to 2 for both channels => setDefaultGains() must be called afterwards
  HAL_StatusTypeDef hal_status;
  uint8_t data[] = { DAC_CMD_ENABLE_OR_DISABLE_VREF, 0, DAC_DATA_VREF_ENABLE };

  hal_status = HAL_SPI_Transmit(spi_handle_, data, sizeof(data), 0);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Enable internal voltage reference failed (HAL error: %d)", hal_status);
    return Status_t::Error;
  }
  return Status_t::Ok;
}

Status_t DacConfig::setDefaultGains() {
  // Note: Resets gains to 1 for both channels
  HAL_StatusTypeDef hal_status;
  uint8_t data[] = { DAC_CMD_WRITE_REG_N, DAC_ADDR_GAIN, DAC_DATA_GAIN_A1_B1 };

  hal_status = HAL_SPI_Transmit(spi_handle_, data, sizeof(data), 0);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Set default gains failed (HAL error: %d)", hal_status);
    return Status_t::Error;
  }
  return Status_t::Ok;
}

Status_t DacConfig::setDefaultValues() {
  // Note: Resets values to 0xffff / 2 for both channels
  HAL_StatusTypeDef hal_status;
  uint8_t data[] = { DAC_CMD_WRITE_REG_N_UPDATE_REG_N, DAC_ADDR_DAC_AB,  //
                     static_cast<uint8_t>(DefalutValue >> 8), static_cast<uint8_t>(DefalutValue & 0xFF) };

  hal_status = HAL_SPI_Transmit(spi_handle_, data, sizeof(data), 0);
  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Set default values failed (HAL error: %d)", hal_status);
    return Status_t::Error;
  }
  return Status_t::Ok;
}

}  // namespace hal::dac
