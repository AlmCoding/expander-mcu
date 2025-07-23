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
}

void DacService::poll() {
  uint32_t request_cnt = 0;

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

  DEBUG_INFO("Post data request (req: %d)", msg->msg.data_request.request_id);
  srv_info_.service_data = true;
  request_service_cb_(1);

  return status;
}

int32_t DacService::postConfigRequest(dac_proto_DacMsg* msg) {
  int32_t status = -1;

  DEBUG_INFO("Post config request (req: %d)", msg->msg.config_request.request_id);
  srv_info_.service_config = true;
  request_service_cb_(1);

  return status;
}

int32_t DacService::serviceRequest(uint8_t* data, size_t max_size) {
  Status_t sts = Status_t::Error;

  /* Allocate space for the decoded message. */
  dac_proto_DacMsg dac_msg = dac_proto_DacMsg_init_zero;

  if (srv_info_.service_data == true) {
    dac_msg.which_msg = dac_proto_DacMsg_data_status_tag;
    sts = serviceDataRequest(nullptr, &dac_msg);
    srv_info_.service_data = false;

  } else if (srv_info_.service_config == true) {
    dac_msg.which_msg = dac_proto_DacMsg_config_status_tag;
    sts = serviceConfigRequest(nullptr, &dac_msg);
    srv_info_.service_config = false;

  } else {
    DEBUG_ERROR("Nothing to service (return 0)");
    return 0;
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

Status_t DacService::serviceDataRequest(int* dac_master, dac_proto_DacMsg* msg) {
  // Here you would implement the logic to handle the data request.

  msg->sequence_number = 0;  // Set sequence number as needed
  msg->which_msg = dac_proto_DacMsg_data_status_tag;

  msg->msg.data_status.request_id = 2;
  msg->msg.data_status.status_code = dac_proto_DacDataStatusCode::dac_proto_DacDataStatusCode_DATA_SUCCESS;
  msg->msg.data_status.buffer_space_ch1 = 0;  // Set buffer space as needed
  msg->msg.data_status.buffer_space_ch2 = 0;  // Set buffer space as needed

  DEBUG_INFO("Srv data status (req: %d)", msg->msg.data_request.request_id);

  return Status_t::Ok;
}

Status_t DacService::serviceConfigRequest(int* dac_config, dac_proto_DacMsg* msg) {
  // Here you would implement the logic to handle the config request.

  msg->sequence_number = 0;  // Set sequence number as needed
  msg->which_msg = dac_proto_DacMsg_config_status_tag;

  msg->msg.config_status.request_id = 1;
  msg->msg.config_status.status_code = dac_proto_DacConfigStatusCode::dac_proto_DacConfigStatusCode_CFG_SUCCESS;

  DEBUG_INFO("Srv config status (req: %d)", msg->msg.config_request.request_id);

  return Status_t::Ok;
}

/*
i2c_proto_I2cStatusCode DacService::convertMasterStatus(hal::i2c::I2cMaster::RequestStatus status) {
  switch (status) {
    case hal::i2c::I2cMaster::RequestStatus::NotInit:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NOT_INIT;
    case hal::i2c::I2cMaster::RequestStatus::NoSpace:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NO_SPACE;
    case hal::i2c::I2cMaster::RequestStatus::Complete:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SUCCESS;
    case hal::i2c::I2cMaster::RequestStatus::SlaveBusy:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SLAVE_BUSY;
    case hal::i2c::I2cMaster::RequestStatus::SlaveNack:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SLAVE_NACK;
    case hal::i2c::I2cMaster::RequestStatus::BadRequest:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_BAD_REQUEST;
    case hal::i2c::I2cMaster::RequestStatus::InterfaceError:
    case hal::i2c::I2cMaster::RequestStatus::Pending:
    case hal::i2c::I2cMaster::RequestStatus::Ongoing:
    default:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_INTERFACE_ERROR;
  }
}

i2c_proto_I2cStatusCode DacService::convertSlaveStatus(hal::i2c::I2cSlave::RequestStatus status) {
  switch (status) {
    case hal::i2c::I2cSlave::RequestStatus::NotInit:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NOT_INIT;
    case hal::i2c::I2cSlave::RequestStatus::NoSpace:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NO_SPACE;
    case hal::i2c::I2cSlave::RequestStatus::Complete:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SUCCESS;
    case hal::i2c::I2cSlave::RequestStatus::SlaveBusy:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SLAVE_BUSY;
    case hal::i2c::I2cSlave::RequestStatus::BadRequest:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_BAD_REQUEST;
    case hal::i2c::I2cSlave::RequestStatus::InterfaceError:
    case hal::i2c::I2cSlave::RequestStatus::Pending:
    default:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_INTERFACE_ERROR;
  }
}
*/

}  // namespace app::dac_srv
