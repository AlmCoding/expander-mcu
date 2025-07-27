/*
 * DacConfig.cpp
 *
 *  Created on: Jul 24, 2025
 *      Author: Alexander L.
 */

#include "hal/dac/DacConfig.hpp"
#include "enum/magic_enum.hpp"
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

}  // namespace hal::dac
