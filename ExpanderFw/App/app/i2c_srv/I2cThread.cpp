/*
 * I2cThread.cpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#include "app/i2c_srv/I2cThread.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_THREAD
#ifdef DEBUG_ENABLE_I2C_THREAD
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cThread]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cThread]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::i2c_srv {

void I2cThread::Execute(ULONG /*thread_input*/) {
  while (1) {
    tx_thread_sleep(10);
  }
}

}  // namespace app::i2c_srv
