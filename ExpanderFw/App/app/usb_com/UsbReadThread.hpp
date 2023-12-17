/*
 * UsbReadThread.hpp
 *
 *  Created on: Now 3, 2023
 *      Author: Alexander L.
 */

#ifndef APP_USB_COM_USBREADTHREAD_HPP_
#define APP_USB_COM_USBREADTHREAD_HPP_

#include "common.hpp"
#include "os/msg/msg_def.hpp"
#include "ux_device_cdc_acm.h"

namespace app::usb_com {

class UsbReadThread {
 public:
  constexpr static size_t UsbMaxReadSize = 64;
  constexpr static size_t UsbReadBufferSize = UsbMaxReadSize;
  static void execute(uint32_t thread_input);

 private:
  static void processMsg(os::msg::BaseMsg* msg);

  static uint32_t msg_count_;
  static UX_SLAVE_DEVICE* device_;
  static UX_SLAVE_CLASS_CDC_ACM* cdc_acm_;
  static uint8_t usb_read_buffer_[UsbReadBufferSize];

  UsbReadThread() = delete;
  virtual ~UsbReadThread() = delete;
};

}  // namespace app::usb_com

#endif /* APP_USB_COM_USBREADTHREAD_HPP_ */
