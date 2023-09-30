/*
 * GpioThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/gpio_srv/GpioThread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_GPIO_THREAD
#ifdef DEBUG_ENABLE_GPIO_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][GpioThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][GpioThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][GpioThread]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::gpio_srv {

void GpioThread::Execute(ULONG /*thread_input*/) {
  while (1) {
    tx_thread_sleep(10);
  }
}

}  // namespace app::gpio_srv
