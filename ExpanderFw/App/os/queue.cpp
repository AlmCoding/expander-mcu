/*
 * queue.cpp
 *
 *  Created on: May 5, 2023
 *      Author: Alexander L.
 */

#include "os/queue.hpp"
#include "os/msg/msg_def.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_QUEUE
#ifdef DEBUG_ENABLE_QUEUE
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][queue]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][queue]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][queue]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace os {

static TX_QUEUE ctrl_queue_;
static TX_QUEUE uart_queue_;
static TX_QUEUE gpio_queue_;
static TX_QUEUE i2c_queue_;

UINT createQueues(VOID* memory_ptr) {
  TX_BYTE_POOL* byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR* pointer;

  //*************************************************************************************************
  // Allocate the stack for CtrlQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, CtrlQueue_MsgCnt * QueueMessageSize * sizeof(ULONG), TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [failed]", CtrlQueue_Name);
    return TX_POOL_ERROR;
  }
  /* Create CtrlQueue */
  if (tx_queue_create(&ctrl_queue_,                       //
                      const_cast<char*>(CtrlQueue_Name),  //
                      QueueMessageSize, pointer,          //
                      CtrlQueue_MsgCnt * QueueMessageSize * sizeof(ULONG)) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [failed]", CtrlQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for UartQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UartQueue_MsgCnt * QueueMessageSize * sizeof(ULONG), TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [failed]", UartQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create UartQueue
  if (tx_queue_create(&uart_queue_,                       //
                      const_cast<char*>(UartQueue_Name),  //
                      QueueMessageSize, pointer,          //
                      UartQueue_MsgCnt * QueueMessageSize * sizeof(ULONG)) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [failed]", UartQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for GpioQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, GpioQueue_MsgCnt * QueueMessageSize * sizeof(ULONG), TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [failed]", GpioQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create GpioQueue
  if (tx_queue_create(&gpio_queue_,                       //
                      const_cast<char*>(GpioQueue_Name),  //
                      QueueMessageSize, pointer,          //
                      GpioQueue_MsgCnt * QueueMessageSize * sizeof(ULONG)) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [failed]", GpioQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for I2cQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, I2cQueue_MsgCnt * QueueMessageSize * sizeof(ULONG), TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [failed]", I2cQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create I2cQueue
  if (tx_queue_create(&i2c_queue_,                       //
                      const_cast<char*>(I2cQueue_Name),  //
                      QueueMessageSize, pointer,         //
                      I2cQueue_MsgCnt * QueueMessageSize * sizeof(ULONG)) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [failed]", I2cQueue_Name);
    return TX_QUEUE_ERROR;
  }

  DEBUG_INFO("Create queues (pool: %d) [ok]", byte_pool->tx_byte_pool_available);
  return TX_SUCCESS;
}

TX_QUEUE* getQueue(msg::MsgQueueId queue) {
  TX_QUEUE* qhdl = nullptr;

  switch (queue) {
    case msg::MsgQueueId::CtrlTaskQueue: {
      qhdl = &ctrl_queue_;
      break;
    }
    case msg::MsgQueueId::UartTaskQueue: {
      qhdl = &uart_queue_;
      break;
    }
    case msg::MsgQueueId::GpioTaskQueue: {
      qhdl = &gpio_queue_;
      break;
    }
    case msg::MsgQueueId::I2cTaskQueue: {
      qhdl = &i2c_queue_;
      break;
    }
    default: {
      break;
    }
  }

  return qhdl;
}

}  // namespace os
