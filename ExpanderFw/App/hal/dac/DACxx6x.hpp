/*
 * DACxx6x.hpp
 *
 *  Created on: Jul 24, 2025
 *      Author: Alexander L.
 */

#ifndef HAL_DAC_DACXX6X_HPP_
#define HAL_DAC_DACXX6X_HPP_

#include "common.hpp"

namespace hal::dac {

// constexpr uint8_t DAC_CMD_MASK = 0b00111000;

constexpr uint8_t DAC_CMD_WRITE_REG_N = 0b00000000;
constexpr uint8_t DAC_CMD_UPDATE_REG_N = 0b00001000;
constexpr uint8_t DAC_CMD_WRITE_REG_N_UPDATE_ALL = 0b00010000;
constexpr uint8_t DAC_CMD_WRITE_REG_N_UPDATE_REG_N = 0b00011000;
constexpr uint8_t DAC_CMD_POWER_UP_OR_DOWN = 0b00100000;
constexpr uint8_t DAC_CMD_SOFTWARE_RESET = 0b00101000;
constexpr uint8_t DAC_CMD_ENABLE_OR_DISABLE_VREF = 0b00111000;

// constexpr uint8_t DAC_ADDR_MASK = 0b00000111;

constexpr uint8_t DAC_ADDR_DAC_A = 0b00000001;
constexpr uint8_t DAC_ADDR_DAC_B = 0b00000010;
constexpr uint8_t DAC_ADDR_DAC_AB = 0b00000011;
constexpr uint8_t DAC_ADDR_GAIN = 0b00000010;

// DAC data values for configuration commands
constexpr uint8_t DAC_DATA_POWER_ON_RESET = 0b00000001;

constexpr uint8_t DAC_DATA_POWER_UP_DAC_A = 0b00000001;
constexpr uint8_t DAC_DATA_POWER_UP_DAC_B = 0b00000010;
constexpr uint8_t DAC_DATA_POWER_UP_DAC_AB = 0b00000011;

constexpr uint8_t DAC_DATA_GAIN_A1_B1 = 0b00000011;
constexpr uint8_t DAC_DATA_GAIN_A2_B2 = 0b00000000;

constexpr uint8_t DAC_DATA_VREF_DISABLE = 0b00000000;
constexpr uint8_t DAC_DATA_VREF_ENABLE = 0b00000001;

}  // namespace hal::dac

#endif /* HAL_DAC_DACXX6X_HPP_ */
