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

constexpr char UsbReadQueue_Name[] = "UsbReadQueue";
constexpr char UsbWriteQueue_Name[] = "UsbWriteQueue";
constexpr char CtrlQueue_Name[] = "CtrlQueue";
constexpr char I2cQueue_Name[] = "I2cQueue";
constexpr char DacQueue_Name[] = "DacQueue";
// constexpr char GpioQueue_Name[] = "GpioQueue";

constexpr size_t UsbReadQueue_MaxMsgCnt = 8;
constexpr size_t UsbWriteQueue_MaxMsgCnt = 8;
constexpr size_t CtrlQueue_MaxMsgCnt = 8;
constexpr size_t I2cQueue_MaxMsgCnt = 8;
constexpr size_t DacQueue_MaxMsgCnt = 8;
// constexpr size_t GpioQueue_MaxMsgCnt = 8;

constexpr size_t QueueMessageSize = sizeof(msg::BaseMsg);

UINT createQueues(VOID* memory_ptr);
TX_QUEUE* getQueue(msg::MsgQueueId queue);

}  // namespace os

#endif /* OS_QUEUE_HPP_ */
