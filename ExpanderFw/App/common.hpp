/*
 * common.hpp
 *
 *  Created on: May 7, 2023
 *      Author: Alexander L.
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <cstdint>
#include <cstring>

#define BITS_SET(reg, bits) ((bool)((reg & bits) == bits))
#define BITS_NOT_SET(reg, bits) (!BITS_SET(reg, bits))

typedef uint32_t TickNum;
typedef int32_t MilliSeconds;
typedef int32_t MicroSeconds;

constexpr MicroSeconds Time1ms = 1000;
constexpr MicroSeconds Time3ms = 3 * Time1ms;
constexpr MicroSeconds Time4ms = 5 * Time1ms;
constexpr MicroSeconds Time10ms = 10 * Time1ms;
constexpr MicroSeconds Time100ms = 100 * Time1ms;

constexpr TickNum Ticks1ms = 1;
constexpr TickNum Ticks3ms = 3 * Ticks1ms;
constexpr TickNum Ticks5ms = 5 * Ticks1ms;
constexpr TickNum Ticks10ms = 10 * Ticks1ms;
constexpr TickNum Ticks100ms = 100 * Ticks1ms;
constexpr TickNum Ticks500ms = 500 * Ticks1ms;

constexpr uint32_t I2cRequestMaxWriteSize = 128;
constexpr uint32_t I2cRequestMaxReadSize = 128;

enum class Status_t {
  Error = -1,
  Ok = 0,
  Busy = 1,
};

#endif /* COMMON_HPP_ */
