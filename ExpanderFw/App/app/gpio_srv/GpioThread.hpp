/*
 * GpioThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_GPIOTHREAD_HPP_
#define APP_CTRL_SRV_GPIOTHREAD_HPP_

#include "app/gpio_srv/GpioService.hpp"
#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::gpio_srv {

class GpioThread {
 public:
  constexpr static driver::tf::TfMsgType ThreadTfMsgType = driver::tf::TfMsgType::GpioMsg;
  static void execute(uint32_t thread_input);

 private:
  static void requestService_cb(os::msg::RequestCnt cnt);
  static int32_t postRequest_cb(const uint8_t* data, size_t size);
  static int32_t serviceRequest_cb(uint8_t* data, size_t max_size);

  static app::gpio_srv::GpioService gpio_service_;
  static bool ongoing_service_;
  static uint32_t msg_count_;

  GpioThread() = delete;
  virtual ~GpioThread() = delete;
};

}  // namespace app::gpio_srv

#endif /* APP_CTRL_SRV_GPIOTHREAD_HPP_ */
