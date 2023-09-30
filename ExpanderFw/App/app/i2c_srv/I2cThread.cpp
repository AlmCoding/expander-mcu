/*
 * I2cThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/i2c_srv/I2cThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_THREAD
#ifdef DEBUG_ENABLE_I2C_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::i2c_srv {

app::i2c_srv::I2cService I2cThread::i2c_service_{};
bool I2cThread::ongoing_service_ = false;
uint32_t I2cThread::msg_count_ = 0;

void I2cThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};

  // Initialize service with notification callback
  i2c_service_.init(requestService_cb);

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(I2cThread::ThreadTfMsgType, postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(I2cThread::ThreadTfMsgType, serviceRequest_cb);

  /* Infinite loop */
  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg, os::I2cThread_CycleTicks) == true) {
      // process msg
    }
    i2c_service_.poll();
  }
}

void I2cThread::requestService_cb(os::msg::RequestCnt cnt) {
  if (ongoing_service_ == true) {
    return;
  }
  ongoing_service_ = true;

  os::msg::BaseMsg req_msg = {
    .id = os::msg::MsgId::ServiceUpstreamRequest,
    .type = I2cThread::ThreadTfMsgType,
    .cnt = cnt,
  };

  if (os::msg::send_msg(os::msg::MsgQueueId::CtrlThreadQueue, &req_msg) == true) {
    DEBUG_INFO("Notify ctrlTask: %d [ok]", ++msg_count_);
  } else {
    DEBUG_ERROR("Notify ctrlTask: %d [failed]", ++msg_count_);
  }
}

int32_t I2cThread::postRequest_cb(const uint8_t* data, size_t len) {
  return i2c_service_.postRequest(data, len);
}

int32_t I2cThread::serviceRequest_cb(uint8_t* data, size_t max_len) {
  ongoing_service_ = false;
  int32_t len = i2c_service_.serviceRequest(data, max_len);

  DEBUG_INFO("Service request: %d [ok]", msg_count_);
  return len;
}

}  // namespace app::i2c_srv