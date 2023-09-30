/*
 * UartThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/uart_srv/UartThread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART_THREAD
#ifdef DEBUG_ENABLE_UART_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][UartThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][UartThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][UartThread]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::uart_srv {

void UartThread::Execute(ULONG /*thread_input*/) {
  while (1) {
    tx_thread_sleep(10);
  }
}

}  // namespace app::uart_srv
