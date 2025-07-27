/*
 * DacService.cpp
 *
 *  Created on: Jul 22, 2025
 *      Author: Alexander L.
 */

#include "app/dac_srv/DacService.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/dac.pb.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_DAC_SERVICE 1
#if ((DEBUG_ENABLE_DAC_SERVICE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][DacSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][DacSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][DacSrv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::dac_srv {

void DacService::init(app::ctrl::RequestSrvCallback request_service_cb) {
  request_service_cb_ = request_service_cb;

  dac_controller_.config();
}

void DacService::poll() {
  uint32_t request_cnt = 0;

  if (dac_controller_.poll() > 0) {
    srv_info_.service_data = true;
    request_cnt++;
  }

  if (dac_config_.poll() > 0) {
    srv_info_.service_config = true;
    request_cnt++;
  }

  if (request_cnt > 0) {
    request_service_cb_(request_cnt);
  }
}

int32_t DacService::postRequest(const uint8_t* data, size_t size) {
  int32_t status = -1;

  /* Allocate space for the decoded message. */
  dac_proto_DacMsg dac_msg = dac_proto_DacMsg_init_zero;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(data, size);

  /* Now we are ready to decode the message. */
  if (pb_decode(&stream, dac_proto_DacMsg_fields, &dac_msg) == false) {
    DEBUG_ERROR("ProtoBuf decode [FAILED]");
    return -1;
  }

  if (dac_msg.which_msg == dac_proto_DacMsg_data_request_tag) {
    status = postDataRequest(&dac_msg);

  } else if (dac_msg.which_msg == dac_proto_DacMsg_config_request_tag) {
    status = postConfigRequest(&dac_msg);

  } else {
    DEBUG_ERROR("Invalid request message!");
  }

  return status;
}

int32_t DacService::postDataRequest(dac_proto_DacMsg* msg) {
  int32_t status = -1;
  hal::dac::DacController::Request request = {};

  request.request_id = msg->msg.data_request.request_id;
  request.status_code = hal::dac::DacController::RequestStatus::NotInit;
  request.run = msg->msg.data_request.run;
  request.sample_count_ch1 = msg->msg.data_request.data_ch1.size / sizeof(hal::dac::DacController::Sample_t);
  request.sample_count_ch2 = msg->msg.data_request.data_ch2.size / sizeof(hal::dac::DacController::Sample_t);

  DEBUG_INFO("Post data request (req: %d)", msg->msg.data_request.request_id);

  if (dac_controller_.scheduleRequest(
          &request,                                                                                    //
          reinterpret_cast<hal::dac::DacController::Sample_t*>(msg->msg.data_request.data_ch1.bytes),  //
          reinterpret_cast<hal::dac::DacController::Sample_t*>(msg->msg.data_request.data_ch2.bytes),  //
          msg->sequence_number) == Status_t::Ok) {
    status = 0;
  }

  return status;
}

int32_t DacService::postConfigRequest(dac_proto_DacMsg* msg) {
  int32_t status = -1;
  hal::dac::DacConfig::Request request = {};

  request.request_id = msg->msg.config_request.request_id;
  request.status_code = hal::dac::DacConfig::RequestStatus::NotInit;
  request.mode = static_cast<hal::dac::DacConfig::Mode>(msg->msg.config_request.mode);
  request.sampling_rate = msg->msg.config_request.sampling_rate;
  request.periodic_samples = msg->msg.config_request.periodic_samples;

  DEBUG_INFO("Post config request (req: %d)", msg->msg.config_request.request_id);

  if (dac_config_.scheduleRequest(&request, msg->sequence_number) == Status_t::Ok) {
    status = 0;
  }

  return status;
}

int32_t DacService::serviceRequest(uint8_t* data, size_t max_size) {
  Status_t sts = Status_t::Error;

  /* Allocate space for the decoded message. */
  dac_proto_DacMsg dac_msg = dac_proto_DacMsg_init_zero;

  if (srv_info_.service_data == true) {
    dac_msg.which_msg = dac_proto_DacMsg_data_status_tag;
    sts = serviceDataRequest(&dac_msg);
    srv_info_.service_data = false;

  } else if (srv_info_.service_config == true) {
    dac_msg.which_msg = dac_proto_DacMsg_config_status_tag;
    sts = serviceConfigRequest(&dac_msg);
    srv_info_.service_config = false;

  } else {
    DEBUG_ERROR("Nothing to service (return 0)");
    return 0;
  }

  if (sts != Status_t::Ok) {
    return -1;
  }

  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_size);
  ETL_ASSERT(max_size >= sizeof(dac_proto_DacMsg), ETL_ERROR(0));

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, dac_proto_DacMsg_fields, &dac_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  return stream.bytes_written;
}

Status_t DacService::serviceDataRequest(dac_proto_DacMsg* msg) {
  hal::dac::DacController::StatusInfo info = {};

  if (dac_controller_.serviceStatus(&info) != Status_t::Ok) {
    DEBUG_ERROR("Srv data status [FAILED]");
    return Status_t::Error;
  }

  msg->sequence_number = info.sequence_number;
  msg->which_msg = dac_proto_DacMsg_data_status_tag;

  msg->msg.data_status.request_id = info.request.request_id;
  msg->msg.data_status.status_code = DacService::convertDataStatus(info.request.status_code);
  msg->msg.data_status.queue_space = info.queue_space;
  msg->msg.data_status.buffer_space_ch1 = info.buffer_space_ch1;
  msg->msg.data_status.buffer_space_ch2 = info.buffer_space_ch2;

  DEBUG_INFO("Srv data status (req: %d) [OK]", msg->msg.data_request.request_id);
  return Status_t::Ok;
}

Status_t DacService::serviceConfigRequest(dac_proto_DacMsg* msg) {
  hal::dac::DacConfig::StatusInfo info = {};

  if (dac_config_.serviceStatus(&info) != Status_t::Ok) {
    DEBUG_ERROR("Srv config status [FAILED]");
    return Status_t::Error;
  }

  msg->sequence_number = info.sequence_number;
  msg->which_msg = dac_proto_DacMsg_config_status_tag;

  msg->msg.config_status.request_id = info.request.request_id;
  msg->msg.config_status.status_code = DacService::convertConfigStatus(info.request.status_code);

  DEBUG_INFO("Srv config status (req: %d) [OK]", info.request.request_id);

  return Status_t::Ok;
}

dac_proto_DacDataStatusCode DacService::convertDataStatus(hal::dac::DacController::RequestStatus status) {
  switch (status) {
    case hal::dac::DacController::RequestStatus::NotInit:
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_NOT_INIT;
    case hal::dac::DacController::RequestStatus::NoSpace:
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_BUFFER_OVERFLOW;
    case hal::dac::DacController::RequestStatus::Complete:
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_SUCCESS;
    case hal::dac::DacController::RequestStatus::BadRequest:
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_BAD_REQUEST;
    case hal::dac::DacController::RequestStatus::InterfaceError:
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_INTERFACE_ERROR;
    case hal::dac::DacController::RequestStatus::Ongoing:
    default: {
      ETL_ASSERT(false, ETL_ERROR(0));
      return dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_INTERFACE_ERROR;
    }
  }
}

dac_proto_DacConfigStatusCode DacService::convertConfigStatus(hal::dac::DacConfig::RequestStatus status) {
  switch (status) {
    case hal::dac::DacConfig::RequestStatus::NotInit:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_NOT_INIT;
    case hal::dac::DacConfig::RequestStatus::Complete:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_SUCCESS;
    case hal::dac::DacConfig::RequestStatus::InvalidMode:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_INVALID_MODE;
    case hal::dac::DacConfig::RequestStatus::InvalidSamplingRate:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_INVALID_SAMPLING_RATE;
    case hal::dac::DacConfig::RequestStatus::InvalidPeriodicSamples:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_INVALID_PERIODIC_SAMPLES;
    case hal::dac::DacConfig::RequestStatus::InterfaceError:
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_INTERFACE_ERROR;
    case hal::dac::DacConfig::RequestStatus::Ongoing:
    default: {
      ETL_ASSERT(false, ETL_ERROR(0));
      return dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_INTERFACE_ERROR;
    }
  }
}

}  // namespace app::dac_srv
