/*
 * UsbReadThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/usb_com/UsbReadThread.hpp"
#include "app/usb_com/UsbWriteThread.hpp"
#include "driver/tf/FrameDriver.hpp"
#include "os/msg/msg_broker.hpp"
#include "os/thread.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_USB_READ_THREAD 1
#if ((DEBUG_ENABLE_USB_READ_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][UsbReadThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][UsbReadThread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][UsbReadThread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

// #define DIRECT_ECHO_LOOP

namespace app::usb_com {

uint32_t UsbReadThread::msg_count_ = 0;
UX_SLAVE_DEVICE* UsbReadThread::device_ = nullptr;
UX_SLAVE_CLASS_CDC_ACM* UsbReadThread::cdc_acm_ = nullptr;
uint8_t UsbReadThread::usb_read_buffer_[UsbReadThread::UsbReadBufferSize];

void UsbReadThread::execute(uint32_t /*thread_input*/) {
  os::msg::BaseMsg msg = {};
  util::Stopwatch stopwatch{};
  uint32_t actual_length = 0;

  // Get the pointer to the device.
  device_ = &_ux_system_slave->ux_system_slave_device;

  DEBUG_INFO("Setup [OK]");

  while (1) {
    if (os::msg::receive_msg(os::msg::MsgQueueId::UsbReadThreadQueue, &msg, 0) == true) {
      processMsg(&msg);
    }

    // Check if device is configured
    if ((device_->ux_slave_device_state == UX_DEVICE_CONFIGURED) && (cdc_acm_ != nullptr)) {
      // Read the received data in blocking mode
      ux_device_class_cdc_acm_read(cdc_acm_, usb_read_buffer_, sizeof(usb_read_buffer_), &actual_length);

      if (actual_length > 0) {
        stopwatch.start();
#ifdef DIRECT_ECHO_LOOP
        UsbWriteThread::sendData(usb_read_buffer_, actual_length);
#else
        auto& tf_driver = driver::tf::FrameDriver::getInstance();
        tf_driver.receiveData(usb_read_buffer_, actual_length);
#endif
        stopwatch.stop();
        DEBUG_INFO("Service downstream (size: %d, time: %d us) [OK]", actual_length, stopwatch.time());
      }

    } else {
      // Sleep for 10ms
      tx_thread_sleep(Ticks10ms);
    }
  }
}

void UsbReadThread::processMsg(os::msg::BaseMsg* msg) {
  DEBUG_INFO("Notification received: %d", ++msg_count_);

  switch (msg->id) {
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
    case os::msg::MsgId::ServiceUpstreamRequest:
    default: {
      break;
    }
  }
}

}  // namespace app::usb_com
