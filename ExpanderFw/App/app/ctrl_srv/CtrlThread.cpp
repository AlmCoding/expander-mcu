/*
 * CtrlThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/ctrl_srv/CtrlThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "main.h"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_CTRL_THREAD 1
#if ((DEBUG_ENABLE_CTRL_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][CtrlThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][CtrlThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][CtrlThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::ctrl_srv {

app::ctrl_srv::CtrlService* CtrlThread::ctrl_service_ = nullptr;
os::msg::RequestCnt CtrlThread::ongoing_service_cnt_ = 0;
uint32_t CtrlThread::msg_count_ = 0;

void CtrlThread::execute(uint32_t /*thread_input*/) {
  app::ctrl_srv::CtrlService ctrl_service{};
  os::msg::BaseMsg msg = {};

  // Initialize service with notification callback
  ctrl_service_ = &ctrl_service;
  ctrl_service_->init(requestService_cb);

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(CtrlThread::ThreadTfMsgType, postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(CtrlThread::ThreadTfMsgType, serviceRequest_cb);

  DEBUG_INFO("Setup [OK]");

  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::CtrlThreadQueue, &msg, os::CtrlThread_CycleTicks) == true) {
      // processMsg(&msg);
    }

    // Heart beat led
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  }
}

void CtrlThread::requestService_cb(os::msg::RequestCnt cnt) {
  if (ongoing_service_cnt_ > 0) {
    return;
  }
  ongoing_service_cnt_ = cnt;

  os::msg::BaseMsg req_msg = {
    .id = os::msg::MsgId::ServiceUpstreamRequest,
    .type = CtrlThread::ThreadTfMsgType,
    .cnt = cnt,
    .ptr = nullptr,
  };

  if (os::msg::send_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &req_msg) == true) {
    DEBUG_INFO("Notify usbWriteTask (not: %d, cnt: %d  msg: %d) [OK]", ++msg_count_, cnt, CtrlThread::ThreadTfMsgType);
  } else {
    DEBUG_ERROR("Notify usbWriteTask (not: %d, cnt: %d  msg: %d) [FAILED]", ++msg_count_, cnt,
                CtrlThread::ThreadTfMsgType);
  }
}

int32_t CtrlThread::postRequest_cb(const uint8_t* data, size_t size) {
  return ctrl_service_->postRequest(data, size);
}

int32_t CtrlThread::serviceRequest_cb(uint8_t* data, size_t max_size) {
  ETL_ASSERT(ongoing_service_cnt_ > 0, ETL_ERROR(0));
  ongoing_service_cnt_--;

  int32_t size = ctrl_service_->serviceRequest(data, max_size);

  if (size > 0) {
    DEBUG_INFO("Service request (not: %d, size: %d) [OK]", msg_count_, size);
  } else {
    DEBUG_ERROR("Service request (not: %d, size: %d) [FAILED]", msg_count_, size);
  }

  return size;
}

}  // namespace app::ctrl_srv
