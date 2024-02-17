/*
 * GpioService.cpp
 *
 *  Created on: 1 Jul 2023
 *      Author: Alexander L.
 */

#include "app/ctrl_srv/CtrlService.hpp"
#include "main.h"  // NVIC_SystemReset();
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/ctrl.pb.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_CTRL_SERVICE 1
#if ((DEBUG_ENABLE_CTRL_SERVICE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][CtrlSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][CtrlSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][CtrlSrv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::ctrl_srv {

void CtrlService::init(app::ctrl::RequestSrvCallback request_service_cb) {
  request_service_cb_ = request_service_cb;
}

int32_t CtrlService::postRequest(const uint8_t* data, size_t size) {
  int32_t status = -1;

  /* Allocate space for the decoded message. */
  ctrl_proto_CtrlMsg ctrl_msg = ctrl_proto_CtrlMsg_init_zero;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(data, size);

  /* Now we are ready to decode the message. */
  if (pb_decode(&stream, ctrl_proto_CtrlMsg_fields, &ctrl_msg) == false) {
    DEBUG_ERROR("ProtoBuf decode [FAILED]");
    return -1;
  }

  if (ctrl_msg.system_reset == true) {
    NVIC_SystemReset();
  }

  return status;
}

int32_t CtrlService::serviceRequest(uint8_t* data, size_t max_size) {
  /* Allocate space for the decoded message. */
  ctrl_proto_CtrlMsg ctrl_msg = ctrl_proto_CtrlMsg_init_zero;
  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_size);

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, ctrl_proto_CtrlMsg_fields, &ctrl_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  return stream.bytes_written;
}

}  // namespace app::ctrl_srv
