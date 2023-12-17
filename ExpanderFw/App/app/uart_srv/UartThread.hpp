/*
 * UartThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_UARTTHREAD_HPP_
#define APP_CTRL_SRV_UARTTHREAD_HPP_

#include "app/uart_srv/UartService.hpp"
#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::uart_srv {

class UartThread {
 public:
  constexpr static driver::tf::TfMsgType TaskTfMsgType = driver::tf::TfMsgType::UartMsg;
  static void execute(uint32_t thread_input);

 private:
  static void uartTask_requestService_cb(os::msg::RequestCnt cnt);
  static int32_t uartTask_postRequest_cb(const uint8_t* data, size_t size);
  static int32_t uartTask_serviceRequest_cb(uint8_t* data, size_t max_size);

  static app::uart_srv::UartService uart_service_;
  static bool ongoing_service_;
  static uint32_t msg_count_;

  UartThread() = delete;
  virtual ~UartThread() = delete;
};

}  // namespace app::uart_srv

#endif /* APP_CTRL_SRV_UARTTHREAD_HPP_ */
