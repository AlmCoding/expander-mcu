/*
 * DacThread.cpp
 *
 *  Created on: Jul 21, 2025
 *      Author: Alexander L.
 */

#include "app/dac_srv/DacThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_DAC_THREAD 1
#if ((DEBUG_ENABLE_DAC_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][DacThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][DacThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][DacThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::dac_srv {

app::dac_srv::DacService* DacThread::dac_service_ = nullptr;
os::msg::RequestCnt DacThread::ongoing_service_cnt_ = 0;
uint32_t DacThread::msg_count_ = 0;

void DacThread::execute(uint32_t /*thread_input*/) {
  app::dac_srv::DacService dac_service{};
  os::msg::BaseMsg msg = {};

  // Initialize service with notification callback
  dac_service_ = &dac_service;
  dac_service_->init(requestService_cb);

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(DacThread::ThreadTfMsgType, postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(DacThread::ThreadTfMsgType, serviceRequest_cb);

  DEBUG_INFO("Setup [OK]");

  /* Infinite loop */
  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::DacThreadQueue, &msg, os::DacThread_CycleTicks) == true) {
      // process msg
    }
    dac_service_->poll();
  }
}

void DacThread::requestService_cb(os::msg::RequestCnt cnt) {
  if (ongoing_service_cnt_ > 0) {
    return;
  }
  ongoing_service_cnt_ = cnt;

  os::msg::BaseMsg req_msg = {
    .id = os::msg::MsgId::ServiceUpstreamRequest,
    .type = DacThread::ThreadTfMsgType,
    .cnt = cnt,
    .ptr = nullptr,
  };

  if (os::msg::send_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &req_msg) == true) {
    DEBUG_INFO("Notify usbWriteTask (not: %d, cnt: %d  msg: %d) [OK]", ++msg_count_, cnt, DacThread::ThreadTfMsgType);
  } else {
    DEBUG_ERROR("Notify usbWriteTask (not: %d, cnt: %d  msg: %d) [FAILED]", ++msg_count_, cnt,
                DacThread::ThreadTfMsgType);
  }
}

int32_t DacThread::postRequest_cb(const uint8_t* data, size_t size) {
  return dac_service_->postRequest(data, size);
}

int32_t DacThread::serviceRequest_cb(uint8_t* data, size_t max_size) {
  ETL_ASSERT(ongoing_service_cnt_ > 0, ETL_ERROR(0));
  ongoing_service_cnt_--;

  int32_t size = dac_service_->serviceRequest(data, max_size);

  if (size > 0) {
    DEBUG_INFO("Service request (not: %d, size: %d) [OK]", msg_count_, size);
  } else {
    DEBUG_ERROR("Service request (not: %d, size: %d) [FAILED]", msg_count_, size);
  }

  return size;
}

}  // namespace app::dac_srv
