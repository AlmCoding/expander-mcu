/*
 * I2cService.cpp
 *
 *  Created on: Aug 18, 2023
 *      Author: Alexander L.
 */

#include "app/i2c_srv/I2cService.hpp"
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/i2c.pb.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART_SERVICE
#ifdef DEBUG_ENABLE_UART_SERVICE
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cSrv]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cSrv]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cSrv]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::i2c_srv {

I2cService::I2cService() {}

I2cService::~I2cService() {}

void I2cService::init(app::ctrl::RequestSrvCallback request_service_cb) {
  i2cMaster0_.config(/*DefaultClockRate*/);

  request_service_cb_ = request_service_cb;
}

void I2cService::poll() {
  uint32_t request_cnt = i2cMaster0_.poll();
  // request_cnt += i2cSlave0_.poll();

  if (request_cnt > 0) {
    request_service_cb_(request_cnt);
  }
}

int32_t I2cService::postRequest(const uint8_t* data, size_t len) {
  int32_t status = -1;

  /* Allocate space for the decoded message. */
  i2c_proto_I2cMsg i2c_msg = i2c_proto_I2cMsg_init_zero;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(data, len);

  /* Now we are ready to decode the message. */
  if (pb_decode(&stream, i2c_proto_I2cMsg_fields, &i2c_msg) == false) {
    DEBUG_ERROR("ProtoBuf decode [failed]");
    return -1;
  }

  if (i2c_msg.which_msg == i2c_proto_I2cMsg_master_request_tag) {
    status = postMasterRequest(&i2c_msg);

  } else if (i2c_msg.which_msg == i2c_proto_I2cMsg_cfg_tag) {
    status = 0;

  } else {
    DEBUG_ERROR("Invalid request message!");
  }

  return status;
}

int32_t I2cService::postMasterRequest(i2c_proto_I2cMsg* msg) {
  int32_t status = -1;
  hal::i2c::I2cMaster::Request request;

  request.status_code = hal::i2c::I2cMaster::RequestStatus::NotInit;
  request.request_id = static_cast<uint16_t>(msg->msg.master_request.request_id);
  request.slave_addr = static_cast<uint16_t>(msg->msg.master_request.slave_addr);
  request.write_size = static_cast<uint16_t>(msg->msg.master_request.write_data.size);
  request.read_size = static_cast<uint16_t>(msg->msg.master_request.read_size);
  request.sequence_id = static_cast<uint16_t>(msg->msg.master_request.sequence_id);
  request.sequence_idx = static_cast<uint16_t>(msg->msg.master_request.sequence_idx);

  if (i2cMaster0_.scheduleRequest(&request, static_cast<uint8_t*>(msg->msg.master_request.write_data.bytes),
                                  msg->sequence_number) == Status_t::Ok) {
    status = 0;
  }

  return status;
}

int32_t I2cService::serviceRequest(uint8_t* data, size_t max_len) {
  /* Allocate space for the decoded message. */
  i2c_proto_I2cMsg i2c_msg = i2c_proto_I2cMsg_init_zero;
  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_len);

  if (serviceMasterStatusRequest(&i2c_msg, max_len) == Status_t::Error) {
    return -1;
  }

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, i2c_proto_I2cMsg_fields, &i2c_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [failed]");
    return -1;
  }

  return stream.bytes_written;
}

Status_t I2cService::serviceMasterStatusRequest(i2c_proto_I2cMsg* msg, size_t max_len) {
  Status_t status;
  hal::i2c::I2cMaster::StatusInfo info;

  if (i2cMaster0_.serviceStatus(&info, msg->msg.master_status.read_data.bytes, max_len) == Status_t::Ok) {
    msg->sequence_number = info.sequence_number;
    msg->which_msg = i2c_proto_I2cMsg_master_status_tag;

    msg->msg.master_status.status_code = static_cast<i2c_proto_I2cMasterStatusCode>(info.status_code);
    msg->msg.master_status.request_id = info.request_id;
    msg->msg.master_status.read_data.size = info.read_size;
    msg->msg.master_status.queue_space = info.queue_space;
    msg->msg.master_status.buffer_space1 = info.buffer_space1;
    msg->msg.master_status.buffer_space2 = info.buffer_space2;

    DEBUG_INFO("Srv status (req: %d) [ok]", info.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Srv status (req: %d) [failed]", info.request_id);
    status = Status_t::Error;
  }

  return status;
}

}  // namespace app::i2c_srv
