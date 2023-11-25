/*
 * UartThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/uart_srv/UartThread.hpp"
#include "app/uart_srv/UartService.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART_THREAD
#ifdef DEBUG_ENABLE_UART_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][UartThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][UartThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][UartThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::uart_srv {

app::uart_srv::UartService UartThread::uart_service_{};
bool UartThread::ongoing_service_ = false;
uint32_t UartThread::msg_count_ = 0;

void UartThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};

  // Initialize service with notification callback
  uart_service_.init(uartTask_requestService_cb);

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(UartThread::TaskTfMsgType, uartTask_postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(UartThread::TaskTfMsgType, uartTask_serviceRequest_cb);

  DEBUG_INFO("Setup [OK]");

  /* Infinite loop */
  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::UartThreadQueue, &msg, os::UartThread_CycleTicks) == true) {
      // process msg
    }
    uart_service_.poll();
  }
}

void UartThread::uartTask_requestService_cb(os::msg::RequestCnt cnt) {
  if (ongoing_service_ == true) {
    return;
  }
  ongoing_service_ = true;

  os::msg::BaseMsg req_msg = {
    .id = os::msg::MsgId::ServiceUpstreamRequest,
    .type = UartThread::TaskTfMsgType,
    .cnt = cnt,
    .ptr = nullptr,
  };

  if (os::msg::send_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &req_msg) == true) {
    DEBUG_INFO("Notify usbWriteTask: %d [OK]", ++msg_count_);
  } else {
    DEBUG_ERROR("Notify usbWriteTask: %d [FAILED]", ++msg_count_);
  }
}

int32_t UartThread::uartTask_postRequest_cb(const uint8_t* data, size_t len) {
  return uart_service_.postRequest(data, len);
}

int32_t UartThread::uartTask_serviceRequest_cb(uint8_t* data, size_t max_len) {
  ongoing_service_ = false;
  int32_t len = uart_service_.serviceRequest(data, max_len);

  if (len > 0) {
    DEBUG_INFO("Service request (not: %d, len %d) [OK]", msg_count_, len);
  } else {
    DEBUG_ERROR("Service request (not: %d, len %d) [FAILED]", msg_count_, len);
  }

  return len;
}

}  // namespace app::uart_srv
