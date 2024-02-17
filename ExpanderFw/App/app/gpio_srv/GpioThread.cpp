/*
 * GpioThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/gpio_srv/GpioThread.hpp"

#include "driver/tf/FrameDriver.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_GPIO_THREAD 1
#if ((DEBUG_ENABLE_GPIO_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][GpioThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][GpioThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][GpioThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::gpio_srv {

app::gpio_srv::GpioService* GpioThread::gpio_service_ = nullptr;
os::msg::RequestCnt GpioThread::ongoing_service_cnt_ = 0;
uint32_t GpioThread::msg_count_ = 0;

void GpioThread::execute(uint32_t /*thread_input*/) {
  app::gpio_srv::GpioService gpio_service{};
  os::msg::BaseMsg msg = {};

  // Initialize service with notification callback
  gpio_service_ = &gpio_service;
  gpio_service_->init(requestService_cb);

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(GpioThread::ThreadTfMsgType, postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(GpioThread::ThreadTfMsgType, serviceRequest_cb);

  DEBUG_INFO("Setup [OK]");

  /* Infinite loop */
  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::GpioThreadQueue, &msg, os::GpioThread_CycleTicks) == true) {
      // process msg
    }
  }
}

void GpioThread::requestService_cb(os::msg::RequestCnt cnt) {
  if (ongoing_service_cnt_ > 0) {
    return;
  }
  ongoing_service_cnt_ = cnt;

  os::msg::BaseMsg req_msg = {
    .id = os::msg::MsgId::ServiceUpstreamRequest,
    .type = GpioThread::ThreadTfMsgType,
    .cnt = cnt,
    .ptr = nullptr,
  };

  if (os::msg::send_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &req_msg) == true) {
    DEBUG_INFO("Notify usbWriteTask: %d [OK]", ++msg_count_);
  } else {
    DEBUG_ERROR("Notify usbWriteTask: %d [FAILED]", ++msg_count_);
  }
}

int32_t GpioThread::postRequest_cb(const uint8_t* data, size_t size) {
  return gpio_service_->postRequest(data, size);
}

int32_t GpioThread::serviceRequest_cb(uint8_t* data, size_t max_size) {
  ETL_ASSERT(ongoing_service_cnt_ > 0, ETL_ERROR(0));
  ongoing_service_cnt_--;

  int32_t size = gpio_service_->serviceRequest(data, max_size);

  if (size > 0) {
    DEBUG_INFO("Service request (not: %d, size: %d) [OK]", msg_count_, size);
  } else {
    DEBUG_ERROR("Service request (not: %d, size: %d) [FAILED]", msg_count_, size);
  }

  return size;
}

}  // namespace app::gpio_srv
