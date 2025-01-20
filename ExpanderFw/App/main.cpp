/*
 * main.cpp
 *
 *  Created on: May 3, 2023
 *      Author: Alexander L.
 */

#include "main.h"
#include "etl/error_handler.h"
#include "os/builder.hpp"
#include "util/boot.hpp"
#include "util/debug.hpp"
#include "util/time.hpp"

#define DEBUG_ENABLE_MAIN 1
#if ((DEBUG_ENABLE_MAIN == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][main]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][main]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][main]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  // Init debug output
  util::dbg::initDebug();
  DEBUG_INFO("Init Debug [OK]");

  // Init HW (CubeMX generated)
  initPeripherals();
  DEBUG_INFO("Init Peripherals [OK]");

  // Init timebase
  util::initTimebase();
  DEBUG_INFO("Init Timebase [OK]");

  // Check if bootloader is requested
  if (util::isBootloaderRequested() == true) {
    DEBUG_INFO("Starting bootloader [...]");
    util::startBootloader();
  }

  // Enter operating system
  os::enterOs();

  // We never get here as control is taken by the os
  ETL_ASSERT(false, ETL_ERROR(0));
  while (1) {
  }
}
