/*
 * Gpio.cpp
 *
 *  Created on: May 6, 2023
 *      Author: Alexander L.
 */

#include "hal/gpio/Gpio.hpp"
#include "hal/gpio/GpioIrq.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_GPIO 1
#if ((DEBUG_ENABLE_GPIO == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][Gpio]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][Gpio]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][Gpio]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

#define IS_OUTPUT_MODE(x) ((x == Gpio::Mode::OutputPushPull) || (x == Gpio::Mode::OutputOpenDrain))

namespace hal::gpio {

Gpio::Gpio(GPIO_TypeDef* port, uint16_t pin, IRQn_Type irq, Id id)
    : port_{ port }, pin_{ pin }, irq_{ irq }, id_{ id } {}

Status_t Gpio::config(Mode mode) {
  Status_t status = Status_t::Ok;

  if (mode_ == mode) {
    return status;
  }

  switch (mode) {
    case Mode::InputPullDown: {
      configInputPullDown();
      break;
    }
    case Mode::InputPullUp: {
      configInputPullUp();
      break;
    }
    case Mode::InputNoPull: {
      configInputNoPull();
      break;
    }
    case Mode::OutputPushPull: {
      configOutputPushPull();
      break;
    }
    case Mode::OutputOpenDrain: {
      configOutputOpenDrain();
      break;
    }
    case Mode::NotInitialized:
    default: {
      status = Status_t::Error;
      break;
    }
  }

  DEBUG_INFO("Cfg Gpio(%d), mode: %d => %d", id_, mode_, mode);

  if (IS_OUTPUT_MODE(mode) == true) {
    GpioIrq::getInstance().deregisterGpio(this);
  } else {
    GpioIrq::getInstance().registerGpio(this);
  }

  mode_ = mode;
  return status;
}

bool Gpio::readPin() {
  if ((in_interrupt_read_ == false) || (IS_OUTPUT_MODE(mode_) == true)) {
    auto state = HAL_GPIO_ReadPin(port_, pin_);
    DEBUG_INFO("Read Gpio(%d): %d", id_, state);
    return static_cast<bool>(state);

  } else {
    in_interrupt_read_ = false;
    return in_interrupt_state_;
  }
}

void Gpio::writePin(bool state) {
  if (IS_OUTPUT_MODE(mode_) == true) {
    DEBUG_INFO("Write Gpio(%d): %d", id_, state);
    HAL_GPIO_WritePin(port_, pin_, static_cast<GPIO_PinState>(state));
  }
}

void Gpio::configInputPullDown() {
  GPIO_InitTypeDef init_struct = {};

  init_struct.Pin = pin_;
  init_struct.Mode = GPIO_MODE_IT_RISING_FALLING;
  init_struct.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(port_, &init_struct);
}

void Gpio::configInputPullUp() {
  GPIO_InitTypeDef init_struct = {};

  init_struct.Pin = pin_;
  init_struct.Mode = GPIO_MODE_IT_RISING_FALLING;
  init_struct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init(port_, &init_struct);
}

void Gpio::configInputNoPull() {
  GPIO_InitTypeDef init_struct = {};

  init_struct.Pin = pin_;
  init_struct.Mode = GPIO_MODE_IT_RISING_FALLING;
  init_struct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(port_, &init_struct);
}

void Gpio::configOutputPushPull() {
  GPIO_InitTypeDef init_struct = {};

  init_struct.Pin = pin_;
  init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  init_struct.Pull = GPIO_NOPULL;
  init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;

  HAL_GPIO_Init(port_, &init_struct);
}

void Gpio::configOutputOpenDrain() {
  GPIO_InitTypeDef init_struct = {};

  init_struct.Pin = pin_;
  init_struct.Mode = GPIO_MODE_OUTPUT_OD;
  init_struct.Pull = GPIO_NOPULL;
  init_struct.Speed = GPIO_SPEED_FREQ_MEDIUM;

  HAL_GPIO_Init(port_, &init_struct);
}

void Gpio::extiCb() {
  auto state = HAL_GPIO_ReadPin(port_, pin_);
  DEBUG_INFO("Read Gpio(%d): %d [exti]", id_, state);
  in_interrupt_state_ = static_cast<bool>(state);
  in_interrupt_read_ = true;
}

}  // namespace hal::gpio
