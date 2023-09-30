/*
 * UartIrq.cpp
 *
 *  Created on: 17 Jun 2023
 *      Author: Alexander L.
 */

#include "hal/uart/UartIrq.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART_IRQ
#ifdef DEBUG_ENABLE_UART_IRQ
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][UartIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][UartIrq]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][UartIrq]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::uart {

UartIrq::UartIrq() {}

Status_t UartIrq::registerUart(Uart* uart) {
  Status_t status;

  if (uart == nullptr) {
    DEBUG_ERROR("Invalid Uart register attempt")
    return Status_t::Error;
  }

  // Check if already registered
  for (size_t i = 0; i < registered_; i++) {
    if (uart_[i] == uart) {
      return Status_t::Ok;
    }
  }

  if (registered_ < UartCount) {
    DEBUG_INFO("Register Uart(%d) [ok]", registered_)
    uart_[registered_] = uart;
    registered_++;
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Register Uart(%d) [failed]", registered_)
    status = Status_t::Error;
  }

  return status;
}

void UartIrq::txCompleteCb(UART_HandleTypeDef* huart) {
  for (size_t i = 0; i < registered_; i++) {
    if (uart_[i]->uart_handle_ == huart) {
      uart_[i]->txCompleteCb();
      break;
    }
  }
}

void UartIrq::rxCompleteCb(UART_HandleTypeDef* huart) {
  for (size_t i = 0; i < sizeof(uart_); i++) {
    if (uart_[i]->uart_handle_ == huart) {
      uart_[i]->rxCompleteCb();
      break;
    }
  }
}

extern "C" {
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  UartIrq::getInstance().txCompleteCb(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  UartIrq::getInstance().rxCompleteCb(huart);
}
}  // extern "C"

}  // namespace hal::uart
