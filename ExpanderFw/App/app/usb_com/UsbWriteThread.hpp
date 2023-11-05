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

constexpr static size_t UsbWriteBufferSize = 2048;

class UsbWriteThread {
 public:
  static void execute(uint32_t thread_input);

 private:
  static void processMsg(os::msg::BaseMsg* msg);

  static uint32_t msg_count_;
  static UX_SLAVE_DEVICE* device_;
  static UX_SLAVE_CLASS_CDC_ACM* cdc_acm_;
  static uint8_t usb_write_buffer_[UsbWriteBufferSize];

  UsbWriteThread() = delete;
  virtual ~UsbWriteThread() = delete;
};

}  // namespace app::usb_com

#endif /* APP_USB_COM_USBWRITETHREAD_HPP_ */
