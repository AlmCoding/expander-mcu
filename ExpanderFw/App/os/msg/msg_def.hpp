/*
 * msg_def.hpp
 *
 *  Created on: May 7, 2023
 *      Author: Alexander L.
 */

#ifndef OS_MSG_MSG_DEF_HPP_
#define OS_MSG_MSG_DEF_HPP_

#include "driver/tf/tfMsgTypes.hpp"

namespace os::msg {

enum class MsgQueueId {
  UsbReadThreadQueue = 0,
  UsbWriteThreadQueue,
  CtrlThreadQueue,
  UartThreadQueue,
  GpioThreadQueue,
  I2cThreadQueue,
};

enum class MsgId {
  TriggerThread = 0,
  UsbDeviceActivate,
  UsbDeviceDeactivate,
  ServiceUpstreamRequest,
};

typedef uint32_t RequestCnt;

typedef struct {
  MsgId id;
  driver::tf::TfMsgType type;
  RequestCnt cnt;
  void* ptr;
} BaseMsg;
static_assert((sizeof(BaseMsg) % sizeof(uint32_t)) == 0, "ThreadX queue messages must be a multiple of 4 bytes!");
static_assert(((sizeof(BaseMsg) == 4) || (sizeof(BaseMsg) == 8) || (sizeof(BaseMsg) == 16) || (sizeof(BaseMsg) == 32) ||
               (sizeof(BaseMsg) == 64)),
              "ThreadX queue messages must be of size 4, 8, 16, 32 or 64 bytes!");

}  // namespace os::msg

#endif /* OS_MSG_MSG_DEF_HPP_ */
