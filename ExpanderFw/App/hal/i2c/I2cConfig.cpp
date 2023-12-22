/*
 * I2cConfig.cpp
 *
 *  Created on: Dec 18, 2023
 *      Author: Alexander
 */

#include "hal/i2c/I2cConfig.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_CONFIG
#ifdef DEBUG_ENABLE_I2C_CONFIG
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cCfg]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cCfg]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal {
namespace i2c {

I2cConfig::I2cConfig(I2C_HandleTypeDef* i2c_handle) : i2c_handle_{ i2c_handle } {}

} /* namespace i2c */
} /* namespace hal */
