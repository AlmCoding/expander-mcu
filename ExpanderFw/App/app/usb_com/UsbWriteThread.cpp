/*
 * UsbWriteThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/usb_com/UsbWriteThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_USB_WRITE_THREAD
#ifdef DEBUG_ENABLE_USB_WRITE_THREAD
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
uint8_t UsbWriteThread::usb_write_buffer_[UsbWriteBufferSize];

void UsbWriteThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};

  DEBUG_INFO("Setup [OK]");

  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::UsbWriteThreadQueue, &msg, os::UsbWriteThread_CycleTicks) == true) {
      processMsg(&msg);
    }
  }
}

void UsbWriteThread::processMsg(os::msg::BaseMsg* msg) {
  switch (msg->id) {
    case os::msg::MsgId::ServiceUpstreamRequest: {
      DEBUG_INFO("Notification received: %d", ++msg_count_);
      if (cdc_acm_ != nullptr) {
        serviceUpstream(msg);
      }
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

  while (msg->cnt > 0) {
    DEBUG_INFO("Service upstream (not: %d, idx: %d, msg: %d)", msg_count_, msg->cnt, msg->type);
    stopwatch.start();
    tf_driver.callTxCallback(msg->type);
    stopwatch.stop();
    DEBUG_INFO("Service upstream (not: %d, idx: %d, msg: %d, time: %d us) [OK]",  //
               msg_count_, msg->cnt, msg->type, stopwatch.time());
    msg->cnt--;
  }
}

void UsbWriteThread::sendData(const uint8_t* data, uint32_t size) {
  uint32_t actual_size = 0;
  uint32_t status = ux_device_class_cdc_acm_write(cdc_acm_, const_cast<uint8_t*>(data), size, &actual_size);

  if ((status != UX_SUCCESS) || (actual_size != size)) {
    DEBUG_ERROR("Usb send data (size: %d) [FAILED]", size);
  }
}

}  // namespace app::usb_com
