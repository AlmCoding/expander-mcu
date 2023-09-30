/*
 * GpioIrq.hpp
 *
 *  Created on: Jul 9, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_GPIO_GPIOIRQ_HPP_
#define HAL_GPIO_GPIOIRQ_HPP_

#include "app/gpio_srv/GpioService.hpp"
#include "common.hpp"
#include "hal/gpio/Gpio.hpp"

namespace hal::gpio {

class GpioIrq {
 public:
  // Deleted copy constructor and assignment operator to enforce singleton
  GpioIrq(const GpioIrq&) = delete;
  GpioIrq& operator=(const GpioIrq&) = delete;

  static GpioIrq& getInstance() {
    static GpioIrq instance;
    return instance;
  }

  void registerRequestSrvCallback(app::ctrl::RequestSrvCallback request_service_cb);
  Status_t registerGpio(Gpio* gpio);
  Status_t deregisterGpio(Gpio* gpio);
  void extiCb(uint16_t pin);

 private:
  GpioIrq();

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
  Gpio* gpio_[static_cast<size_t>(Gpio::Id::GpioCount)] = { nullptr };
  size_t registered_ = 0;
};

}  // namespace hal::gpio

#endif /* HAL_GPIO_GPIOIRQ_HPP_ */
