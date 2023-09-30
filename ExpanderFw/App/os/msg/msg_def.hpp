/*
 * msg_def.hpp
 *
 *  Created on: May 7, 2023
 *      Author: Alexander L.
 */

#ifndef OS_MSG_MSG_DEF_HPP_
#define OS_MSG_MSG_DEF_HPP_

#include "app/usb/usbComTypes.hpp"

namespace os::msg {

enum class MsgQueue {
  CtrlTaskQueue = 0,
  UartTaskQueue,
  GpioTaskQueue,
  I2cTaskQueue,
};

enum class MsgId {
  TriggerTask = 0,
  ServiceUpstreamRequest,
};

typedef uint32_t RequestCnt;

typedef struct {
  MsgId id;
  app::usb::UsbMsgType type;
  RequestCnt cnt;
} BaseMsg;

}  // namespace os::msg

#endif /* OS_MSG_MSG_DEF_HPP_ */
