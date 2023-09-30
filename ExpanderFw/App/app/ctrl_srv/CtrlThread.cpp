/*
 * CtrlThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/ctrl_srv/CtrlThread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_CTRL_THREAD
#ifdef DEBUG_ENABLE_CTRL_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][CtrlThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][CtrlThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][CtrlThread]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::ctrl_srv {

void CtrlThread::Execute(ULONG /*thread_input*/) {
  while (1) {
    tx_thread_sleep(10);
  }
}

}  // namespace app::ctrl_srv
