/*
 * GpioService.cpp
 *
 *  Created on: 1 Jul 2023
 *      Author: Alexander L.
 */

#include "app/ctrl_srv/CtrlService.hpp"
#include "app/ctrl_srv/DeviceInfo.hpp"
#include "main.h"  // NVIC_SystemReset();
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/ctrl.pb.h"
#include "util/boot.hpp"
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

void CtrlService::poll() {
  if (service_device_info_ == true) {
    request_service_cb_(1);
  }
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

  seqence_number_ = ctrl_msg.sequence_number;

  if (ctrl_msg.which_msg == ctrl_proto_CtrlMsg_ctrl_request_tag) {
    status = postCtrlRequest(&ctrl_msg);
  } else {
    DEBUG_ERROR("Invalid request message!");
  }

  return status;
}

int32_t CtrlService::postCtrlRequest(ctrl_proto_CtrlMsg* msg) {
  int32_t status = -1;
  request_id_ = msg->msg.ctrl_request.request_id;

  if (msg->msg.ctrl_request.get_device_info == true) {
    DEBUG_INFO("Post device info request");
    service_device_info_ = true;
    status = 0;

  } else if (msg->msg.ctrl_request.reset_system == true) {
    DEBUG_INFO("Reset system request");
    NVIC_SystemReset();

  } else if (msg->msg.ctrl_request.start_bootloader == true) {
    DEBUG_INFO("Start bootloader request");
    util::requestBootloader();
    NVIC_SystemReset();

  } else {
    DEBUG_ERROR("Invalid control request!");
  }

  return status;
}

int32_t CtrlService::serviceRequest(uint8_t* data, size_t max_size) {
  /* Allocate space for the decoded message. */
  ctrl_proto_CtrlMsg ctrl_msg = ctrl_proto_CtrlMsg_init_zero;

  app::ctrl_srv::DeviceInfo::Info info = {};
  ctrl_srv::DeviceInfo::getDeviceInfo(&info);

  ctrl_msg.sequence_number = seqence_number_;
  ctrl_msg.which_msg = ctrl_proto_CtrlMsg_device_info_tag;

  ctrl_msg.msg.device_info.request_id = request_id_;
  ctrl_msg.msg.device_info.device_type = info.device_type;
  ctrl_msg.msg.device_info.hardware_version = info.hardware_version;

  ctrl_msg.msg.device_info.firmware_version_major = info.firmware_version_major;
  ctrl_msg.msg.device_info.firmware_version_minor = info.firmware_version_minor;
  ctrl_msg.msg.device_info.firmware_version_patch = info.firmware_version_patch;
  strncpy(ctrl_msg.msg.device_info.git_hash, info.git_hash, sizeof(ctrl_msg.msg.device_info.git_hash));

  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_size);

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, ctrl_proto_CtrlMsg_fields, &ctrl_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  DEBUG_INFO("Service device info request [OK]");
  service_device_info_ = false;
  return stream.bytes_written;
}

}  // namespace app::ctrl_srv
