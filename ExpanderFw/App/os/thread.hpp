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

constexpr char CtrlThread_Name[] = "CtrlThread";
constexpr char UartThread_Name[] = "UartThread";
constexpr char GpioThread_Name[] = "GpioThread";
constexpr char I2cThread_Name[] = "I2cThread";

constexpr TickNum CtrlThread_CycleTime = 1000;  // event driven
constexpr TickNum UartThread_CycleTime = 1;
constexpr TickNum GpioThread_CycleTime = 1000;  // event driven
constexpr TickNum I2cThread_CycleTime = 1;

constexpr uint32_t CtrlThread_StackSize = 4096;
constexpr uint32_t UartThread_StackSize = 4096;
constexpr uint32_t GpioThread_StackSize = 4096;
constexpr uint32_t I2cThread_StackSize = 4096;

constexpr uint32_t CtrlThread_Priority = 10;
constexpr uint32_t UartThread_Priority = 10;
constexpr uint32_t GpioThread_Priority = 10;
constexpr uint32_t I2cThread_Priority = 10;

constexpr uint32_t CtrlThread_PreemptionThreshold = 10;
constexpr uint32_t UartThread_PreemptionThreshold = 10;
constexpr uint32_t GpioThread_PreemptionThreshold = 10;
constexpr uint32_t I2cThread_PreemptionThreshold = 10;

constexpr uint32_t CtrlThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t UartThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t GpioThread_TimeSlice = TX_NO_TIME_SLICE;
constexpr uint32_t I2cThread_TimeSlice = TX_NO_TIME_SLICE;

constexpr uint32_t CtrlThread_AutoStart = TX_AUTO_START;
constexpr uint32_t UartThread_AutoStart = TX_AUTO_START;
constexpr uint32_t GpioThread_AutoStart = TX_AUTO_START;
constexpr uint32_t I2cThread_AutoStart = TX_AUTO_START;

UINT createThreads(VOID* memory_ptr);

}  // namespace os

#endif /* OS_THREAD_HPP_ */
