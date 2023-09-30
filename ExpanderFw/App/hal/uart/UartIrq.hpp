/*
 * UartIrq.hpp
 *
 *  Created on: 17 Jun 2023
 *      Author: Alexander L.
 */

#ifndef HAL_UART_UARTIRQ_HPP_
#define HAL_UART_UARTIRQ_HPP_

#include "common.hpp"
#include "hal/uart/Uart.hpp"

namespace hal::uart {

class UartIrq {
 private:
  constexpr static size_t UartCount = 2;

 public:
  // Deleted copy constructor and assignment operator to enforce singleton
  UartIrq(const UartIrq&) = delete;
  UartIrq& operator=(const UartIrq&) = delete;

  static UartIrq& getInstance() {
    static UartIrq instance;
    return instance;
  }

  Status_t registerUart(Uart* uart);
  void txCompleteCb(UART_HandleTypeDef* huart);
  void rxCompleteCb(UART_HandleTypeDef* huart);

 private:
  UartIrq();

  Uart* uart_[UartCount] = { nullptr };
  size_t registered_ = 0;
};

}  // namespace hal::uart

#endif /* HAL_UART_UARTIRQ_HPP_ */
