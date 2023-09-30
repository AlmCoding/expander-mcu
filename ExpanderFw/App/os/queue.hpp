/*
 * queue_cfg.hpp
 *
 *  Created on: May 5, 2023
 *      Author: Alexander L.
 */

#ifndef OS_QUEUE_HPP_
#define OS_QUEUE_HPP_

#include "common.hpp"
#include "os/msg/msg_def.hpp"
#include "tx_api.h"

namespace os {

constexpr char CtrlQueue_Name[] = "CtrlQueue";
constexpr char UartQueue_Name[] = "UartQueue";
constexpr char GpioQueue_Name[] = "GpioQueue";
constexpr char I2cQueue_Name[] = "I2cQueue";

constexpr size_t CtrlQueue_MsgCnt = 8;
constexpr size_t UartQueue_MsgCnt = 8;
constexpr size_t GpioQueue_MsgCnt = 8;
constexpr size_t I2cQueue_MsgCnt = 8;

constexpr size_t QueueMessageSize = sizeof(msg::BaseMsg);

UINT createQueues(VOID* memory_ptr);
// osMessageQueueId_t getQueue(msg::MsgQueue queue);

}  // namespace os

#endif /* OS_QUEUE_HPP_ */
