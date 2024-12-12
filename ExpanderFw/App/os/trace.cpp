/*
 * trace.cpp
 *
 *  Created on: Jan 21, 2024
 *      Author: Alexander L.
 */

#include "os/trace.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_TRACE 1
#if ((DEBUG_ENABLE_TRACE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][trace]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][trace]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][trace]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace os {

//static uint8_t trace_buffer[Trace_BufferSize] __attribute__((section(".trace")));

UINT enableTracing() {
  /*
  if (tx_trace_enable(trace_buffer, sizeof(trace_buffer), Trace_MaxThreadCount) != TX_SUCCESS) {
    DEBUG_ERROR("Enable os tracing [FAILED]");
    return TX_ACTIVATE_ERROR;
  }
  */

  DEBUG_INFO("Enable os tracing [OK]");
  return TX_SUCCESS;
}

}  // namespace os
