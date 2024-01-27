/*
 * thread.cpp
 *
 *  Created on: May 4, 2023
 *      Author: Alexander L.
 */

#include "os/thread.hpp"
#include "app/ctrl/CtrlThread.hpp"
#include "app/gpio_srv/GpioThread.hpp"
#include "app/i2c_srv/I2cThread.hpp"
#include "app/uart_srv/UartThread.hpp"
#include "app/usb_com/UsbReadThread.hpp"
#include "app/usb_com/UsbWriteThread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_THREAD 1
#if ((DEBUG_ENABLE_THREAD == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][thread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][thread]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][thread]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace os {

static TX_THREAD usb_read_thread_;
static TX_THREAD usb_write_thread_;
static TX_THREAD ctrl_thread_;
static TX_THREAD uart_thread_;
static TX_THREAD gpio_thread_;
static TX_THREAD i2c_thread_;

UINT createThreads(VOID* memory_ptr) {
  TX_BYTE_POOL* byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR* pointer;

  //*************************************************************************************************
  // Allocate usbRead stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UsbReadThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UsbReadThread_Name);
    return TX_POOL_ERROR;
  }
  // Create usbRead thread
  if (tx_thread_create(&usb_read_thread_,                                 //
                       const_cast<char*>(UsbReadThread_Name),             //
                       app::usb_com::UsbReadThread::execute, 0, pointer,  //
                       UsbReadThread_StackSize,                           //
                       UsbReadThread_Priority,                            //
                       UsbReadThread_PreemptionThreshold,                 //
                       UsbReadThread_TimeSlice,                           //
                       UsbReadThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UsbReadThread_Name);
    return TX_THREAD_ERROR;
  }

  //*************************************************************************************************
  // Allocate usbWrite stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UsbWriteThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UsbWriteThread_Name);
    return TX_POOL_ERROR;
  }
  // Create usbWrite thread
  if (tx_thread_create(&usb_write_thread_,                                 //
                       const_cast<char*>(UsbWriteThread_Name),             //
                       app::usb_com::UsbWriteThread::execute, 0, pointer,  //
                       UsbWriteThread_StackSize,                           //
                       UsbWriteThread_Priority,                            //
                       UsbWriteThread_PreemptionThreshold,                 //
                       UsbWriteThread_TimeSlice,                           //
                       UsbWriteThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UsbWriteThread_Name);
    return TX_THREAD_ERROR;
  }

  //*************************************************************************************************
  // Allocate ctrl stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, CtrlThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", CtrlThread_Name);
    return TX_POOL_ERROR;
  }
  // Create ctrl thread
  if (tx_thread_create(&ctrl_thread_,                               //
                       const_cast<char*>(CtrlThread_Name),          //
                       app::ctrl::CtrlThread::execute, 0, pointer,  //
                       CtrlThread_StackSize,                        //
                       CtrlThread_Priority,                         //
                       CtrlThread_PreemptionThreshold,              //
                       CtrlThread_TimeSlice,                        //
                       CtrlThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", CtrlThread_Name);
    return TX_THREAD_ERROR;
  }

  //*************************************************************************************************
  // Allocate uart stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, UartThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", UartThread_Name);
    return TX_POOL_ERROR;
  }
  // Create uart thread
  if (tx_thread_create(&uart_thread_,                                   //
                       const_cast<char*>(UartThread_Name),              //
                       app::uart_srv::UartThread::execute, 0, pointer,  //
                       UartThread_StackSize,                            //
                       UartThread_Priority,                             //
                       UartThread_PreemptionThreshold,                  //
                       UartThread_TimeSlice,                            //
                       UartThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", UartThread_Name);
    return TX_THREAD_ERROR;
  }

  //*************************************************************************************************
  // Allocate gpio stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, GpioThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", GpioThread_Name);
    return TX_POOL_ERROR;
  }
  // Create gpio thread
  if (tx_thread_create(&gpio_thread_,                                   //
                       const_cast<char*>(GpioThread_Name),              //
                       app::gpio_srv::GpioThread::execute, 0, pointer,  //
                       GpioThread_StackSize,                            //
                       GpioThread_Priority,                             //
                       GpioThread_PreemptionThreshold,                  //
                       GpioThread_TimeSlice,                            //
                       GpioThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", GpioThread_Name);
    return TX_THREAD_ERROR;
  }

  //*************************************************************************************************
  // Allocate i2c stack
  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, I2cThread_StackSize, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Allocate %s stack [FAILED]", I2cThread_Name);
    return TX_POOL_ERROR;
  }
  // Create i2c thread
  if (tx_thread_create(&i2c_thread_,                                  //
                       const_cast<char*>(I2cThread_Name),             //
                       app::i2c_srv::I2cThread::execute, 0, pointer,  //
                       I2cThread_StackSize,                           //
                       I2cThread_Priority,                            //
                       I2cThread_PreemptionThreshold,                 //
                       I2cThread_TimeSlice,                           //
                       I2cThread_AutoStart) != TX_SUCCESS) {
    DEBUG_ERROR("Create %s [FAILED]", I2cThread_Name);
    return TX_THREAD_ERROR;
  }

  DEBUG_INFO("Create threads (pool: %d) [OK]", byte_pool->tx_byte_pool_available);
  return TX_SUCCESS;
}

}  // namespace os
