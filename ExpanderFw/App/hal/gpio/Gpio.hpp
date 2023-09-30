/*
 * Gpio.hpp
 *
 *  Created on: May 6, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_GPIO_GPIO_HPP_
#define HAL_GPIO_GPIO_HPP_

#include "common.hpp"
#include "main.h"

namespace hal::gpio {

class Gpio {
 public:
  enum class Id {
    Gpio0 = 0,
    Gpio1,
    Gpio2,
    Gpio3,
    Gpio4,
    Gpio5,
    Gpio6,
    Gpio7,
    GpioCount,
  };

  enum class Mode {
    InputPullDown = 0,
    InputPullUp,
    InputNoPull,
    OutputPushPull,
    OutputOpenDrain,
    NotInitialized,
  };

  Gpio(GPIO_TypeDef* port, uint16_t pin, IRQn_Type irq, Id id);
  virtual ~Gpio() = default;

  Status_t config(Mode mode);
  bool readPin();
  void writePin(bool state);

 private:
  void configInputPullDown();
  void configInputPullUp();
  void configInputNoPull();
  void configOutputPushPull();
  void configOutputOpenDrain();
  void extiCb();

  GPIO_TypeDef* port_;
  uint16_t pin_;
  IRQn_Type irq_;
  Id id_;
  Mode mode_ = Mode::NotInitialized;

  bool in_interrupt_read_ = false;
  bool in_interrupt_state_ = false;

  friend class GpioIrq;
};

}  // namespace hal::gpio

#endif /* HAL_GPIO_GPIO_HPP_ */
