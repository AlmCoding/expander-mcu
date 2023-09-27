/*
 * usbComTypes.hpp
 *
 *  Created on: 27 May 2023
 *      Author: Alexander L.
 */

#ifndef APP_USB_USBCOMTYPES_HPP_
#define APP_USB_USBCOMTYPES_HPP_

#include "common.hpp"

extern "C" {
#include "tf/TF_Config.h"
}

namespace app::usb {

enum class UsbMsgType : uint8_t {
  CtrlMsg = 0,
  UartMsg,
  I2cMsg,
  SpiMsg,
  CanMsg,
  GpioMsg,
  PwmMsg,
  AdcMsg,
  // Keep this at the end
  NumValues,
};

static_assert(static_cast<uint8_t>(UsbMsgType::NumValues) <= TF_MAX_TYPE_LST,
              "Increase TF_MAX_TYPE_LST in TF_Config.h file.");

}  // namespace app::usb

#endif /* APP_USB_USBCOMTYPES_HPP_ */
