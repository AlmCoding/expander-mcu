/*
 * UsbWriteThread.hpp
 *
 *  Created on: Now 3, 2023
 *      Author: Alexander L.
 */

#ifndef APP_USB_COM_USBWRITETHREAD_HPP_
#define APP_USB_COM_USBWRITETHREAD_HPP_

#include "common.hpp"
#include "os/msg/msg_def.hpp"
#include "ux_device_cdc_acm.h"

namespace app::usb_com {

class UsbWriteThread {
 public:
  constexpr static size_t UsbMaxWriteSize = 64;
  constexpr static size_t UsbWriteBufferSize = 256 + 64;
  constexpr static driver::tf::TfMsgType ThreadTfMsgType = driver::tf::TfMsgType::EchoMsg;

  static void execute(uint32_t thread_input);
  static void sendData(const uint8_t* data, size_t size);

 private:
  static void processMsg(os::msg::BaseMsg* msg);
  static void serviceUpstream(os::msg::BaseMsg* msg);

  static int32_t postRequest_cb(const uint8_t* data, size_t size);
  static int32_t serviceRequest_cb(uint8_t* data, size_t max_size);

  static uint32_t msg_count_;
  static UX_SLAVE_DEVICE* device_;
  static UX_SLAVE_CLASS_CDC_ACM* cdc_acm_;
  static uint8_t usb_write_buffer_[UsbWriteBufferSize];

  static uint8_t* echo_data_;
  static size_t echo_size_;

  UsbWriteThread() = delete;
  virtual ~UsbWriteThread() = delete;
};

}  // namespace app::usb_com

#endif /* APP_USB_COM_USBWRITETHREAD_HPP_ */
