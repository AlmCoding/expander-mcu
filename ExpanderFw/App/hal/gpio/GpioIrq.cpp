/*
 * GpioIrq.cpp
 *
 *  Created on: Jul 9, 2023
 *      Author: Alexander L.
 */

#include "hal/gpio/GpioIrq.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_GPIO_IRQ
#ifdef DEBUG_ENABLE_GPIO_IRQ
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][GpioIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][GpioIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][GpioIrq]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::gpio {

GpioIrq::GpioIrq() {}

void GpioIrq::registerRequestSrvCallback(app::ctrl::RequestSrvCallback request_service_cb) {
  request_service_cb_ = request_service_cb;
}

Status_t GpioIrq::registerGpio(Gpio* gpio) {
  Status_t status;

  if (gpio == nullptr) {
    DEBUG_ERROR("Invalid Gpio register attempt")
    return Status_t::Error;
  }

  // Check if already registered
  for (size_t i = 0; i < registered_; i++) {
    if (gpio_[i] == gpio) {
      return Status_t::Ok;
    }
  }

  if (registered_ < static_cast<size_t>(Gpio::Id::GpioCount)) {
    DEBUG_INFO("Register Gpio(%d) [ok]", gpio->id_)

    gpio_[registered_] = gpio;
    HAL_NVIC_EnableIRQ(gpio->irq_);

    registered_++;
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Register Gpio(%d) [failed]", gpio->id_)
    status = Status_t::Error;
  }

  return status;
}

Status_t GpioIrq::deregisterGpio(Gpio* gpio) {
  Status_t status = Status_t::Ok;
  bool shift = false;
  bool shared_irq = false;

  // Find gpio in array
  for (size_t i = 0; i < registered_; i++) {
    if (gpio_[i] == gpio) {
      // Disable irq
      HAL_NVIC_DisableIRQ(gpio->irq_);
      shift = true;

    } else if (gpio_[i]->irq_ == gpio->irq_) {
      // Irq is shared between gpios
      shared_irq = true;
    }

    if (shift == true) {
      gpio_[i] = gpio_[i + 1];
    }
  }

  if (shift == true) {
    registered_--;
    gpio_[registered_] = nullptr;

    DEBUG_INFO("Deregister gpio(%d) [ok]", gpio->id_)
  }

  if (shared_irq == true) {
    HAL_NVIC_EnableIRQ(gpio->irq_);
  }

  return status;
}

void GpioIrq::extiCb(uint16_t pin) {
  for (size_t i = 0; i < registered_; i++) {
    if (gpio_[i]->pin_ == pin) {
      gpio_[i]->extiCb();
      break;
    }
  }

  // Request to be serviced by ctrlTask
  if (request_service_cb_ != nullptr) {
    request_service_cb_(1);
  }
}

extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  GpioIrq::getInstance().extiCb(GPIO_Pin);
}
}  // extern "C"

}  // namespace hal::gpio
