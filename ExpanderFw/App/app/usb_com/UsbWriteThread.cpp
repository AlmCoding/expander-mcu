/*
 * UsbWriteThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/usb_com/UsbWriteThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_USB_WRITE_THREAD 1
#if ((DEBUG_ENABLE_USB_WRITE_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][UsbWriteThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][UsbWriteThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][UsbWriteThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::usb_com {

uint32_t UsbWriteThread::msg_count_ = 0;
UX_SLAVE_DEVICE* UsbWriteThread::device_ = nullptr;
UX_SLAVE_CLASS_CDC_ACM* UsbWriteThread::cdc_acm_ = nullptr;
uint8_t UsbWriteThread::usb_write_buffer_[UsbWriteThread::UsbWriteBufferSize];
uint8_t* UsbWriteThread::echo_data_ = nullptr;
size_t UsbWriteThread::echo_size_ = 0;

void UsbWriteThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};

  // Register callback for incoming msg
  driver::tf::FrameDriver::getInstance().registerRxCallback(UsbWriteThread::ThreadTfMsgType, postRequest_cb);

  // Register callback for outgoing msg
  driver::tf::FrameDriver::getInstance().registerTxCallback(UsbWriteThread::ThreadTfMsgType, serviceRequest_cb);

  DEBUG_INFO("Setup [OK]");

  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &msg, os::UsbWriteThread_CycleTicks) == true) {
      processMsg(&msg);
    }
  }
}

void UsbWriteThread::processMsg(os::msg::BaseMsg* msg) {
  DEBUG_INFO("Notification received: %d", ++msg_count_);

  switch (msg->id) {
    case os::msg::MsgId::ServiceUpstreamRequest: {
      serviceUpstream(msg);
      break;
    }
    case os::msg::MsgId::UsbDeviceActivate: {
      if (msg->ptr != nullptr) {
        cdc_acm_ = static_cast<UX_SLAVE_CLASS_CDC_ACM*>(msg->ptr);
        DEBUG_INFO("Usb device activate [OK]");
      } else {
        DEBUG_INFO("Usb device activate [FAILED]");
      }
      break;
    }
    case os::msg::MsgId::UsbDeviceDeactivate: {
      cdc_acm_ = nullptr;
      DEBUG_INFO("Usb device deactivate [OK]");
    }
    case os::msg::MsgId::TriggerThread:
    default: {
      break;
    }
  }
}

void UsbWriteThread::serviceUpstream(os::msg::BaseMsg* msg) {
  util::Stopwatch stopwatch{};
  auto& tf_driver = driver::tf::FrameDriver::getInstance();

  if (cdc_acm_ == nullptr) {
    return;
  }

  while (msg->cnt > 0) {
    DEBUG_INFO("Service upstream (not: %d, idx: %d, msg: %d)", msg_count_, msg->cnt, msg->type);
    stopwatch.start();
    tf_driver.callTxCallback(msg->type, UsbWriteThread::usb_write_buffer_, UsbWriteThread::UsbWriteBufferSize);
    stopwatch.stop();
    DEBUG_INFO("Service upstream (not: %d, idx: %d, msg: %d, time: %d us) [OK]",  //
               msg_count_, msg->cnt, msg->type, stopwatch.time());
    msg->cnt--;
  }
}

void UsbWriteThread::sendData(const uint8_t* data, uint32_t size) {
  ETL_ASSERT(size <= UsbMaxWriteSize, ETL_ERROR(0));
  uint32_t actual_size = 0;

  uint32_t status = ux_device_class_cdc_acm_write(cdc_acm_, const_cast<uint8_t*>(data), size, &actual_size);

  if ((status != UX_SUCCESS) || (actual_size != size)) {
    DEBUG_ERROR("Usb send data (size: %d) [FAILED]", size);
  }
}

int32_t UsbWriteThread::postRequest_cb(const uint8_t* data, size_t size) {
  echo_data_ = const_cast<uint8_t*>(data);
  echo_size_ = size;
  auto& tf_driver = driver::tf::FrameDriver::getInstance();
  tf_driver.callTxCallback(UsbWriteThread::ThreadTfMsgType, UsbWriteThread::usb_write_buffer_,
                           UsbWriteThread::UsbWriteBufferSize);
  return 0;
}

int32_t UsbWriteThread::serviceRequest_cb(uint8_t* data, size_t max_size) {
  ETL_ASSERT(echo_size_ <= max_size, ETL_ERROR(0));
  memcpy(data, echo_data_, echo_size_);
  return echo_size_;
}

}  // namespace app::usb_com
