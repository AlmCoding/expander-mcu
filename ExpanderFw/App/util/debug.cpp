/*
 * debug.cpp
 *
 *  Created on: 18 May 2023
 *      Author: Alexander L.
 */

#include "util/debug.hpp"
#include <cstdarg>
#include "rtt/RTT/SEGGER_RTT.h"

namespace util::dbg {

void initDebug() {
#if (ENABLE_RTT_DEBUG_OUTPUT == 1)
  SEGGER_RTT_Init();
#endif
}

void print(uint8_t term, const char* format, ...) {
#if (ENABLE_RTT_DEBUG_OUTPUT == 1)
  va_list args;
  va_start(args, format);
  SEGGER_RTT_vprintf(term, format, &args);
  va_end(args);
#else
  (void)term;
  (void)format;
#endif
}

}  // namespace util::dbg
