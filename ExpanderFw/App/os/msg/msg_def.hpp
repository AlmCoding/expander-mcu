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

enum class MsgQueueId : uint8_t {
  CtrlThreadQueue = 0,
  UartThreadQueue,
  GpioThreadQueue,
  I2cThreadQueue,
};

enum class MsgId : uint8_t {
  TriggerThread = 0,
  ServiceUpstreamRequest,
};

typedef uint32_t RequestCnt;

typedef struct {
  MsgId id;
  driver::tf::TfMsgType type;
  RequestCnt cnt;
} BaseMsg;

}  // namespace os::msg

#endif /* OS_MSG_MSG_DEF_HPP_ */
