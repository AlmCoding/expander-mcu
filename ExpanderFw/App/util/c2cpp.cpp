/*
 * c2cpp.cpp
 *
 *  Created on: 18 May 2023
 *      Author: Alexander L.
 */

#include "util/c2cpp.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_C2CPP
#ifdef DEBUG_ENABLE_C2CPP
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][c2cpp]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][c2cpp]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][c2cpp]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace util {

void notifyUsbDeviceActivate(void* cdc_acm) {
  bool success = false;

  os::msg::BaseMsg msg = {
    .id = os::msg::MsgId::UsbDeviceActivate,
    .type = driver::tf::TfMsgType::NumValues,
    .cnt = 0,
    .ptr = cdc_acm,
  };

  success = os::msg::send_msg(os::msg::MsgQueueId::UsbReadThreadQueue, &msg, 0);
  success = success && os::msg::send_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &msg, 0);

  if (success == false) {
    DEBUG_INFO("Send UsbDeviceActivate msg [FAILED]");
  }
}

void notifyUsbDeviceDeactivate() {}

}  // namespace util
