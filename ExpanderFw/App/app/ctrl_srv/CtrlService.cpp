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
    service_device_info_ = true;
    status = 0;

  } else if (msg->msg.ctrl_request.reset_system == true) {
    NVIC_SystemReset();

  } else if (msg->msg.ctrl_request.start_bootloader == true) {
    startBootloader();

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

  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_size);

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, ctrl_proto_CtrlMsg_fields, &ctrl_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  service_device_info_ = false;
  return stream.bytes_written;
}

void CtrlService::startBootloader() {
  // Try this if not working:
  // https://community.st.com/t5/stm32-mcus/jump-to-bootloader-from-application-on-stm32h7-devices/ta-p/49510

  void (*SysMemBootJump)(void);

  /**
   * Step: Set system memory address.
   *       For other families, check AN2606 document table 110 with descriptions of memory addresses
   */
  volatile uint32_t addr = 0x0BF90000;

  /**
   * Step: Disable RCC, set it to default (after reset) settings
   *       Internal clock, no PLL, etc.
   */
  HAL_RCC_DeInit();

  /**
   * Step: Disable systick timer and reset it to default values
   */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /**
   * Step: Disable all interrupts
   */
  __disable_irq();

  /**
   * Step: Remap system memory to address 0x0000 0000 in address space
   *       For each family registers may be different.
   *       Check reference manual for each family.
   *
   *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
   *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
   *       For others, check family reference manual
   */
  //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

  /**
   * Step: Set jump memory location for system memory
   *       Use address with 4 bytes offset which specifies jump location where program starts
   */
  SysMemBootJump = (void (*)(void))(*((uint32_t*)(addr + 4)));

  /**
   * Step: Set main stack pointer.
   *       This step must be done last otherwise local variables in this function
   *       don't have proper value since stack pointer is located on different position
   *
   *       Set direct address location which specifies stack pointer in SRAM location
   */
  __set_MSP(*(uint32_t*)addr);
  /**
   * Step: Actually call our function to jump to set location
   *       This will start system memory execution
   */
  SysMemBootJump();
}

}  // namespace app::ctrl_srv
