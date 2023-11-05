/*
 * Uart.cpp
 *
 *  Created on: May 6, 2023
 *      Author: Alexander L.
 */

#include "hal/uart/Uart.hpp"
#include "hal/uart/UartIrq.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART
#ifdef DEBUG_ENABLE_UART
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][Uart]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][Uart]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][Uart]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

#define DMA_RX_WRITE_POS 0  //(sizeof(rx_buffer_) - uart_handle_->hdmarx->Instance->CNDTR)
#define DMA_TX_READ_POS 0   //(uart_handle_->hdmatx->Instance->CMAR - reinterpret_cast<uint32_t>(tx_buffer_))

namespace hal::uart {

Uart::Uart(UART_HandleTypeDef* uart_handle) : uart_handle_{ uart_handle } {}

Status_t Uart::config(uint32_t baudrate) {
  Status_t status;
  stopDma();

  // Set new configuration
  uart_handle_->Init.BaudRate = baudrate;

  if (HAL_UART_Init(uart_handle_) == HAL_OK) {
    DEBUG_INFO("Config [OK]");
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Config [FAILED]");
    status = Status_t::Error;
  }

  if (init() == Status_t::Ok) {
    DEBUG_INFO("Init [OK]");

  } else {
    DEBUG_ERROR("Init [FAILED]");
    status = Status_t::Error;
  }

  UartIrq::getInstance().registerUart(this);
  return status;
}

Status_t Uart::init() {
  this_tx_start_ = 0;
  next_tx_start_ = 0;
  next_tx_end_ = 0;
  tx_complete_ = true;
  tx_overflow_ = false;

  rx_read_pos_ = 0;
  rx_overflow_ = false;

  send_status_msg_ = false;
  seqence_number_ = 0;

  return startRx();
}

uint32_t Uart::poll() {
  uint32_t service_requests = 0;

#if (START_UART_REQUEST_IMMEDIATELY == false)
  startTx();
#endif

  if (isRxBufferEmpty() == false) {
    send_status_msg_ = true;
  }

  if (send_status_msg_ == true) {
    service_requests = 1;
  }

  return service_requests;
}

Status_t Uart::stopDma() {
  Status_t status;

  if (HAL_UART_DMAStop(uart_handle_) == HAL_OK) {
    DEBUG_INFO("Stop dma [OK]");
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Stop dma [FAILED]");
    status = Status_t::Error;
  }

  return status;
}

Status_t Uart::startRx() {
  Status_t status;

  if (HAL_UART_Receive_DMA(uart_handle_, rx_buffer_, sizeof(rx_buffer_)) == HAL_OK) {
    DEBUG_INFO("Start rx [OK]");
    status = Status_t::Ok;

  } else {
    DEBUG_INFO("Start rx [FAILED]");
    status = Status_t::Error;
  }

  return status;
}

bool Uart::isRxBufferEmpty() {
  if (rx_read_pos_ == DMA_RX_WRITE_POS) {
    return true;
  } else {
    return false;
  }
}

size_t Uart::getFreeTxSpace(uint32_t seq_num) {
  /* From next_tx_end_ to DMA_TX_READ_POS - 1
   * The -1 is needed because [next_tx_start_, next_tx_end_[
   * would mean that next_tx_end_ == next_tx_start
   * when scheduled tx len == sizeof(tx_buffer).
   */

  size_t free_tx_space = sizeof(tx_buffer_) - 1;
  size_t next_tx_end = next_tx_end_;
  size_t dma_tx_read_pos;

  if (BITS_SET(uart_handle_->gState, HAL_UART_STATE_BUSY_TX) == true) {
    dma_tx_read_pos = DMA_TX_READ_POS;
  } else {
    dma_tx_read_pos = next_tx_start_;
  }

  if (dma_tx_read_pos != next_tx_end) {
    free_tx_space += dma_tx_read_pos - next_tx_end;

    if (dma_tx_read_pos > next_tx_end) {
      free_tx_space -= sizeof(tx_buffer_);
    }
  }

  DEBUG_INFO("[%d, dma: %d, [%d, %d[", this_tx_start_, dma_tx_read_pos, next_tx_start_, next_tx_end);
  DEBUG_INFO("Free tx space (len: %d, seq: %d)", free_tx_space, seq_num);
  return free_tx_space;
}

Status_t Uart::scheduleTx(const uint8_t* data, size_t len, uint32_t seq_num) {
  Status_t status;

  if (getFreeTxSpace(seq_num) >= len) {
    DEBUG_INFO("Sched. tx (len: %d, seq: %d)", len, seq_num);

    size_t new_tx_end;
    size_t tx_len = len;
    size_t len_to_buf_end = sizeof(tx_buffer_) - next_tx_end_;

    if (len > len_to_buf_end) {
      tx_len = len_to_buf_end;
    }

    std::memcpy(tx_buffer_ + next_tx_end_, data, tx_len);
    len -= tx_len;

    if (tx_len < len_to_buf_end) {
      new_tx_end = next_tx_end_ + tx_len;
    } else {
      new_tx_end = 0;
    }

    if (len > 0) {
      std::memcpy(tx_buffer_, data + tx_len, len);
      new_tx_end = len;
    }

    next_tx_end_ = new_tx_end;
    tx_complete_ = false;
    tx_overflow_ = false;
    status = Status_t::Ok;

#if (START_UART_REQUEST_IMMEDIATELY == true)
    startTx();
#endif

  } else {
    DEBUG_ERROR("Tx overflow (len: %d, seq: %d)", len, seq_num);
    tx_overflow_ = true;
    send_status_msg_ = true;
    status = Status_t::Error;
  }

  seqence_number_ = seq_num;

  // Trigger uart task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::UartThreadQueue, &msg);

  return status;
}

Status_t Uart::startTx() {
  Status_t status;
  HAL_StatusTypeDef tx_status;

  // Return if tx busy
  if (BITS_SET(uart_handle_->gState, HAL_UART_STATE_BUSY_TX) == true) {
    return Status_t::Busy;
  }

  // Check for new data
  if (next_tx_end_ != next_tx_start_) {
    size_t tx_len;
    size_t new_tx_start;

    // Protect against change due to new scheduleTx (interrupt context)
    size_t next_tx_end = next_tx_end_;

    if (next_tx_start_ < next_tx_end) {
      tx_len = next_tx_end - next_tx_start_;
      new_tx_start = next_tx_end;

    } else {
      tx_len = sizeof(tx_buffer_) - next_tx_start_;
      new_tx_start = 0;
    }

    // Start transmit
    tx_status = HAL_UART_Transmit_DMA(uart_handle_, tx_buffer_ + next_tx_start_, static_cast<uint16_t>(tx_len));
    if (tx_status == HAL_OK) {
      this_tx_start_ = next_tx_start_;
      next_tx_start_ = new_tx_start;

      DEBUG_INFO("Start tx (len: %d) [OK]", tx_len);
      DEBUG_INFO("Tx=[%d, dma: %d, %d[", this_tx_start_, DMA_TX_READ_POS, next_tx_start_);
      status = Status_t::Ok;

    } else {
      DEBUG_ERROR("Start tx (len: %d) [FAILED]", tx_len);
      status = Status_t::Error;
    }

  } else {
    // Nothing to be done
    status = Status_t::Ok;
  }

  return status;
}

void Uart::txCompleteCb() {
  DEBUG_INFO("Tx cplt (seq: %d)", seqence_number_);
  DEBUG_INFO("Cplt=[%d, dma: %d, [%d, %d[", this_tx_start_, DMA_TX_READ_POS, next_tx_start_, next_tx_end_);

  // Check for new data
  if (next_tx_end_ != next_tx_start_) {
    startTx();

  } else {
    this_tx_start_ = 0;
    next_tx_start_ = 0;
    next_tx_end_ = 0;
    tx_complete_ = true;
  }

  send_status_msg_ = true;

  // Trigger uart task for fast service notification
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::UartThreadQueue, &msg);
}

void Uart::rxCompleteCb() {
  DEBUG_INFO("Rx cplt (seq: %d)", seqence_number_);

  // Trigger uart task for fast service notification
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::UartThreadQueue, &msg);
}

size_t Uart::serviceRx(uint8_t* data, size_t max_len) {
  int32_t rx_cnt;
  bool rx_circulation = false;
  size_t dma_rx_write_pos = DMA_RX_WRITE_POS;

  rx_cnt = dma_rx_write_pos - rx_read_pos_;
  if (rx_cnt < 0) {
    rx_cnt = sizeof(rx_buffer_) - rx_read_pos_;
    rx_circulation = true;
  }

  if (rx_cnt > static_cast<int32_t>(max_len)) {
    rx_cnt = max_len;
  }

  std::memcpy(data, rx_buffer_ + rx_read_pos_, rx_cnt);

  if (rx_circulation == true) {
    if ((dma_rx_write_pos + rx_cnt) > max_len) {
      dma_rx_write_pos = max_len - rx_cnt;
    }

    if (dma_rx_write_pos > 0) {
      std::memcpy(data + rx_cnt, rx_buffer_, dma_rx_write_pos);
      rx_cnt += dma_rx_write_pos;
    }
  }

  rx_read_pos_ = dma_rx_write_pos;
  return rx_cnt;
}

Status_t Uart::serviceStatus(StatusInfo* status, uint8_t* data, size_t max_len) {
  send_status_msg_ = false;

  status->sequence_number = seqence_number_;
  status->rx_overflow = rx_overflow_;
  status->tx_overflow = tx_overflow_;
  status->tx_complete = tx_complete_;
  status->rx_space = 0;
  status->tx_space = static_cast<uint16_t>(getFreeTxSpace(status->sequence_number));
  status->rx_size = static_cast<uint16_t>(serviceRx(data, max_len));

  return Status_t::Ok;
}

} /* namespace hal::uart */
