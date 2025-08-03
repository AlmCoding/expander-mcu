/*
 * DacConfig.hpp
 *
 *  Created on: Jul 24, 2025
 *      Author: Alexander L.
 */

#ifndef HAL_DAC_DACCONFIG_HPP_
#define HAL_DAC_DACCONFIG_HPP_

#include "common.hpp"
#include "main.h"
#include "tx_api.h"

namespace hal::dac {

class DacConfig {
 private:
  constexpr static uint16_t DefalutValue = 0xffff / 2;

 public:
  enum class RequestStatus {
    NotInit = 0,
    // Pending, => not needed, request is immediately ongoing
    Ongoing,
    Complete,
    InvalidMode,
    InvalidSamplingRate,
    InvalidPeriodicSamples,
    InterfaceError,
  };

  enum class Mode {
    Static = 0,
    Periodic,
    Streaming,
  };

  typedef struct {
    uint32_t request_id;
    RequestStatus status_code;
    Mode mode;
    uint32_t sampling_rate;     // in Hz
    uint32_t periodic_samples;  // number of samples for periodic mode
  } Request;

  typedef struct {
    uint32_t sequence_number;
    Request request;
  } StatusInfo;

  DacConfig(SPI_HandleTypeDef* spi_handle);
  virtual ~DacConfig() = default;

  Status_t config();
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info);

 private:
  Status_t softwareReset();
  Status_t enableInternalVoltageRef();
  Status_t setDefaultGains();
  Status_t setDefaultValues();

  SPI_HandleTypeDef* spi_handle_ = nullptr;

  Request request_;
  bool service_status_ = false;

  uint32_t seqence_number_ = 0;
};

}  // namespace hal::dac

#endif /* HAL_DAC_DACCONFIG_HPP_ */
