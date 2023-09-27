/*
 * builder.cpp
 *
 *  Created on: May 4, 2023
 *      Author: Alexander L.
 */

#include "os/builder.hpp"
#include "tx_api.h"

#define DEBUG_ENABLE_BUILDER
#ifdef DEBUG_ENABLE_BUILDER
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][builder]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][builder]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][builder]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace os {

void enterOs() {
  tx_kernel_enter();
}

UINT buildOs(VOID* memory_ptr) {
  /* Init scheduler */

  /*
  createMutexes();
  createTimers();
  createQueues();
  createTasks();
  */

  return TX_SUCCESS;
}

}  // namespace os
