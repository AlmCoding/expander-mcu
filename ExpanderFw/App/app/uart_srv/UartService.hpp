/*
 * UartService.hpp
 *
 *  Created on: May 6, 2023
 *      Author: Alexander L.
 */

#ifndef APP_UART_SRV_UARTSERVICE_HPP_
#define APP_UART_SRV_UARTSERVICE_HPP_

#include "app/ctrl/ctrlTypes.hpp"
#include "hal/uart/Uart.hpp"
#include "proto_c/uart.pb.h"
#include "usart.h"

namespace app::uart_srv {

constexpr uint32_t DefaultBaudRate = 115200;

class UartService {
 public:
  UartService();
  virtual ~UartService();

  void init(app::ctrl::RequestSrvCallback request_service_cb);
  void poll();

  int32_t postRequest(const uint8_t* data, size_t len);
  int32_t serviceRequest(uint8_t* data, size_t max_len);

 private:
  Status_t serviceStatusRequest(uart_proto_UartMsg* msg, size_t max_len);

  hal::uart::Uart uart0_{ &huart2 };

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
};

}  // namespace app::uart_srv

#endif /* APP_UART_SRV_UARTSERVICE_HPP_ */
