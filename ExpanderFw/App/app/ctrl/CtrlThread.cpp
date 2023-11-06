/*
 * CtrlThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/ctrl/CtrlThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "main.h"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_CTRL_THREAD
#ifdef DEBUG_ENABLE_CTRL_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][CtrlThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][CtrlThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][CtrlThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::ctrl {

uint32_t CtrlThread::msg_count_ = 0;

void CtrlThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};

  DEBUG_INFO("Setup [OK]");

  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::CtrlThreadQueue, &msg, os::CtrlThread_CycleTicks) == true) {
      processMsg(&msg);
    }

    // Heart beat led
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  }
}

void CtrlThread::processMsg(os::msg::BaseMsg* msg) {
  util::Stopwatch stopwatch{};
  DEBUG_INFO("Notification received: %d", ++msg_count_);

  switch (msg->id) {
    case os::msg::MsgId::TriggerThread:
    case os::msg::MsgId::UsbDeviceActivate:
    case os::msg::MsgId::UsbDeviceDeactivate:
    case os::msg::MsgId::ServiceUpstreamRequest:
    default: {
      break;
    }
  }
}

}  // namespace app::ctrl
