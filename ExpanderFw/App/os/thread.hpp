/*
 * thread.hpp
 *
 *  Created on: May 4, 2023
 *      Author: Alexander L.
 */

#ifndef OS_THREAD_HPP_
#define OS_THREAD_HPP_

#include "common.hpp"
#include "tx_api.h"

namespace os {

constexpr char UsbReadThread_Name[] = "UsbReadThread";
constexpr char UsbWriteThread_Name[] = "UsbWriteThread";
constexpr char CtrlThread_Name[] = "CtrlThread";
constexpr char I2cThread_Name[] = "I2cThread";
constexpr char DacThread_Name[] = "DacThread";
// constexpr char GpioThread_Name[] = "GpioThread";

constexpr TickNum UsbWriteThread_CycleTicks = TX_WAIT_FOREVER;  // event driven
constexpr TickNum CtrlThread_CycleTicks = Ticks500ms;           // heart beat led
constexpr TickNum I2cThread_CycleTicks = Ticks1ms;
constexpr TickNum DacThread_CycleTicks = Ticks10ms;
// constexpr TickNum GpioThread_CycleTicks = TX_WAIT_FOREVER;  // event driven

constexpr uint32_t UsbReadThread_StackSize = 4096;
constexpr uint32_t UsbWriteThread_StackSize = 4096;
constexpr uint32_t CtrlThread_StackSize = 1024;
constexpr uint32_t I2cThread_StackSize = 8192;
constexpr uint32_t DacThread_StackSize = 8192;
// constexpr uint32_t GpioThread_StackSize = 1024;

constexpr uint32_t UsbReadThread_Priority = 10;
constexpr uint32_t UsbWriteThread_Priority = 10;
constexpr uint32_t CtrlThread_Priority = 10;
constexpr uint32_t I2cThread_Priority = 10;
constexpr uint32_t DacThread_Priority = 10;
// constexpr uint32_t GpioThread_Priority = 10;

constexpr uint32_t UsbReadThread_PreemptionThreshold = 10;
constexpr uint32_t UsbWriteThread_PreemptionThreshold = 10;
constexpr uint32_t CtrlThread_PreemptionThreshold = 10;
constexpr uint32_t I2cThread_PreemptionThreshold = 10;
constexpr uint32_t DacThread_PreemptionThreshold = 10;
// constexpr uint32_t GpioThread_PreemptionThreshold = 10;

constexpr uint32_t UsbReadThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t UsbWriteThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t CtrlThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t I2cThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t DacThread_TimeSlice = TX_NO_TIME_SLICE;
// constexpr uint32_t GpioThread_TimeSlice = TX_NO_TIME_SLICE;

constexpr uint32_t UsbReadThread_AutoStart = TX_AUTO_START;
constexpr uint32_t UsbWriteThread_AutoStart = TX_AUTO_START;
constexpr uint32_t CtrlThread_AutoStart = TX_AUTO_START;
constexpr uint32_t I2cThread_AutoStart = TX_AUTO_START;
constexpr uint32_t DacThread_AutoStart = TX_AUTO_START;
// constexpr uint32_t GpioThread_AutoStart = TX_AUTO_START;

UINT createThreads(VOID* memory_ptr);

}  // namespace os

#endif /* OS_THREAD_HPP_ */
