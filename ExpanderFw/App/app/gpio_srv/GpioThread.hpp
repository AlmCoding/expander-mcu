/*
 * GpioThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_GPIOTHREAD_HPP_
#define APP_CTRL_SRV_GPIOTHREAD_HPP_

#include "tx_api.h"

namespace app::gpio_srv {

class GpioThread {
 public:
  GpioThread() = delete;
  virtual ~GpioThread() = delete;

  static void Execute(ULONG thread_input);

 private:
};

}  // namespace app::gpio_srv

#endif /* APP_CTRL_SRV_GPIOTHREAD_HPP_ */
