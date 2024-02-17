/*
 * GpioService.hpp
 *
 *  Created on: 1 Jul 2023
 *      Author: Alexander L.
 */

#ifndef APP_GPIO_SRV_GPIOSERVICE_HPP_
#define APP_GPIO_SRV_GPIOSERVICE_HPP_

#include "app/ctrl_srv/ctrlTypes.hpp"
#include "gpio.h"
#include "hal/gpio/Gpio.hpp"
#include "proto_c/uart.pb.h"

namespace app::gpio_srv {

class GpioService {
 public:
  GpioService() = default;
  virtual ~GpioService() = default;

  void init(app::ctrl::RequestSrvCallback request_service_cb);

  int32_t postRequest(const uint8_t* data, size_t size);
  int32_t serviceRequest(uint8_t* data, size_t max_size);

 private:
  hal::gpio::Gpio gpio0_{ GPIOC, GPIO_0_Pin, EXTI8_IRQn, hal::gpio::Gpio::Id::Gpio0 };
  hal::gpio::Gpio gpio1_{ GPIOC, GPIO_1_Pin, EXTI9_IRQn, hal::gpio::Gpio::Id::Gpio1 };
  hal::gpio::Gpio gpio2_{ GPIOC, GPIO_2_Pin, EXTI10_IRQn, hal::gpio::Gpio::Id::Gpio2 };
  hal::gpio::Gpio gpio3_{ GPIOC, GPIO_3_Pin, EXTI11_IRQn, hal::gpio::Gpio::Id::Gpio3 };
  hal::gpio::Gpio gpio4_{ GPIOC, GPIO_4_Pin, EXTI12_IRQn, hal::gpio::Gpio::Id::Gpio4 };
  hal::gpio::Gpio gpio5_{ GPIOD, GPIO_5_Pin, EXTI2_IRQn, hal::gpio::Gpio::Id::Gpio5 };
  hal::gpio::Gpio gpio6_{ GPIOF, GPIO_6_Pin, EXTI3_IRQn, hal::gpio::Gpio::Id::Gpio6 };
  hal::gpio::Gpio gpio7_{ GPIOF, GPIO_7_Pin, EXTI5_IRQn, hal::gpio::Gpio::Id::Gpio7 };

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
  uint32_t seqence_number_ = 0;
};

}  // namespace app::gpio_srv

#endif /* APP_GPIO_SRV_GPIOSERVICE_HPP_ */
