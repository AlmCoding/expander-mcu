/*
 * queue.cpp
 *
 *  Created on: May 5, 2023
 *      Author: Alexander L.
 */

#include "os/queue.hpp"
#include "os/msg/msg_def.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_QUEUE 1
#if ((DEBUG_ENABLE_QUEUE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][queue]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][queue]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][queue]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace os {

static TX_QUEUE usb_read_queue_;
static TX_QUEUE usb_write_queue_;
static TX_QUEUE ctrl_queue_;
// static TX_QUEUE uart_queue_;
// static TX_QUEUE gpio_queue_;
static TX_QUEUE i2c_queue_;

UINT createQueues(VOID* memory_ptr) {
  TX_BYTE_POOL* byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR* pointer;

  //*************************************************************************************************
  // Allocate the stack for UsbReadQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UsbReadQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UsbReadQueue_Name);
    return TX_POOL_ERROR;
  }
  /* Create UsbReadQueue */
  if (tx_queue_create(&usb_read_queue_,                              //
                      const_cast<char*>(UsbReadQueue_Name),          //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      UsbReadQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UsbReadQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for UsbWriteQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UsbWriteQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) !=
      TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UsbWriteQueue_Name);
    return TX_POOL_ERROR;
  }
  /* Create UsbReadQueue */
  if (tx_queue_create(&usb_write_queue_,                             //
                      const_cast<char*>(UsbWriteQueue_Name),         //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      UsbWriteQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UsbWriteQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for CtrlQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, CtrlQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", CtrlQueue_Name);
    return TX_POOL_ERROR;
  }
  /* Create CtrlQueue */
  if (tx_queue_create(&ctrl_queue_,                                  //
                      const_cast<char*>(CtrlQueue_Name),             //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      CtrlQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", CtrlQueue_Name);
    return TX_QUEUE_ERROR;
  }

  //*************************************************************************************************
  // Allocate the stack for UartQueue
  /*
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UartQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UartQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create UartQueue
  if (tx_queue_create(&uart_queue_,                                  //
                      const_cast<char*>(UartQueue_Name),             //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      UartQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UartQueue_Name);
    return TX_QUEUE_ERROR;
  }
  */

  //*************************************************************************************************
  // Allocate the stack for GpioQueue
  /*
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, GpioQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", GpioQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create GpioQueue
  if (tx_queue_create(&gpio_queue_,                                  //
                      const_cast<char*>(GpioQueue_Name),             //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      GpioQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", GpioQueue_Name);
    return TX_QUEUE_ERROR;
  }
  */

  //*************************************************************************************************
  // Allocate the stack for I2cQueue
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, I2cQueue_MaxMsgCnt * QueueMessageSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", I2cQueue_Name);
    return TX_POOL_ERROR;
  }
  // Create I2cQueue
  if (tx_queue_create(&i2c_queue_,                                   //
                      const_cast<char*>(I2cQueue_Name),              //
                      QueueMessageSize / sizeof(uint32_t), pointer,  //
                      I2cQueue_MaxMsgCnt * QueueMessageSize) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", I2cQueue_Name);
    return TX_QUEUE_ERROR;
  }

  DEBUG_INFO("Create queues (pool: %d) [OK]", byte_pool->tx_byte_pool_available);
  return TX_SUCCESS;
}

TX_QUEUE* getQueue(msg::MsgQueueId queue) {
  TX_QUEUE* qhdl = nullptr;

  switch (queue) {
    case msg::MsgQueueId::UsbReadThreadQueue: {
      qhdl = &usb_read_queue_;
      break;
    }
    case msg::MsgQueueId::UsbWriteThreadQueue: {
      qhdl = &usb_write_queue_;
      break;
    }
    case msg::MsgQueueId::CtrlThreadQueue: {
      qhdl = &ctrl_queue_;
      break;
    }
    case msg::MsgQueueId::UartThreadQueue: {
      // qhdl = &uart_queue_;
      break;
    }
    case msg::MsgQueueId::GpioThreadQueue: {
      // qhdl = &gpio_queue_;
      break;
    }
    case msg::MsgQueueId::I2cThreadQueue: {
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
