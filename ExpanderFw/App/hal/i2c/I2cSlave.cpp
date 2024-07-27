/*
 * I2cSlave.cpp
 *
 *  Created on: Nov 25, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cSlave.hpp"
#include "enum/magic_enum.hpp"
#include "etl/algorithm.h"      // etl::max
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "hal/i2c/I2cIrq.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_SLAVE 1
#if ((DEBUG_ENABLE_I2C_SLAVE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cSlv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cSlv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cSlv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

// #define DMA_RX_MEM_WRITE_POS (sizeof(temp_buffer_) - __HAL_DMA_GET_COUNTER(i2c_handle_->hdmarx))
// #define DMA_TX_MEM_READ_POS (sizeof(data_buffer_) - getDataAddress() - __HAL_DMA_GET_COUNTER(i2c_handle_->hdmatx))

namespace hal::i2c {

I2cSlave::I2cSlave(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle) : i2c_id_{ i2c_id }, i2c_handle_{ i2c_handle } {
  uint32_t sts = TX_SUCCESS;
  sts = tx_queue_create(&request_queue_,                        //
                        const_cast<char*>("I2cSlaveRequestQ"),  //
                        sizeof(Request) / sizeof(uint32_t),     //
                        request_queue_buffer_,                  //
                        sizeof(request_queue_buffer_));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
}

Status_t I2cSlave::config(MemAddrWidth mem_addr_width) {
  mem_addr_width_ = mem_addr_width;

  Status_t status = init();

  if (status == Status_t::Ok) {
    I2cIrq::getInstance().registerI2cSlave(this);
    DEBUG_INFO("Init I2cSlave(%d) [OK]", magic_enum::enum_integer(i2c_id_));

  } else {
    DEBUG_ERROR("Init I2cSlave(%d) [FAILED]", magic_enum::enum_integer(i2c_id_));
    status = Status_t::Error;
  }

  ETL_ASSERT(status == Status_t::Ok, ETL_ERROR(0));
  return status;
}

Status_t I2cSlave::init() {
  Status_t status = Status_t::Ok;

  tx_queue_flush(&request_queue_);

  memset(data_buffer_, 0, sizeof(data_buffer_));
  memset(temp_buffer_, 0, sizeof(temp_buffer_));

  access_id_ = 0;
  seqence_number_ = 0;

  I2cIrq::getInstance().enableSlaveListen(i2c_handle_);
  return status;
}

uint32_t I2cSlave::poll() {
  uint32_t service_requests = 0;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots < RequestQueue_MaxItemCnt) {
    service_requests = 1;
  }

  return service_requests;
}

Status_t I2cSlave::scheduleRequest(Request* request, uint8_t* mem_data, uint32_t seq_num) {
  uint32_t sts = TX_SUCCESS;

  uint32_t free_slots = 0;
  sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Check request queue space
  if (free_slots == 0) {
    DEBUG_ERROR("Queue overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  if (((request->write_addr + request->write_size) > sizeof(data_buffer_)) ||
      ((request->read_addr + request->read_size) > sizeof(data_buffer_))) {
    DEBUG_ERROR("Bad request (req: %d)", request->request_id);
    request->status_code = RequestStatus::BadRequest;
    return exitScheduleRequest(request, seq_num);
  }

  // Copy write data to buffer
  if (request->write_size > 0) {
    std::memcpy(data_buffer_ + request->write_addr, mem_data, request->write_size);
  }

  // Set request status to complete
  request->status_code = RequestStatus::Complete;

  // Add request to request_queue
  sts = tx_queue_send(&request_queue_, request, TX_NO_WAIT);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  return exitScheduleRequest(request, seq_num);
}

Status_t I2cSlave::exitScheduleRequest(Request* request, uint32_t seq_num) {
  Status_t status = Status_t::Ok;

  if (request->status_code == RequestStatus::Complete) {
    DEBUG_INFO("Sched. request (req: %d) [OK]", request->request_id);
    status = Status_t::Ok;

  } else if (request->status_code == RequestStatus::NoSpace) {
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);
    DEBUG_ERROR("Rejected request (req: %d) is NOT reported!", request->request_id);
    status = Status_t::Error;

  } else if (request->status_code == RequestStatus::BadRequest) {
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);
    uint32_t sts = tx_queue_send(&request_queue_, request, TX_NO_WAIT);
    ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
    status = Status_t::Error;

  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  // Update sequence number
  seqence_number_ = seq_num;

  // Trigger i2c task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);

  return status;
}

int32_t I2cSlave::getDataAddress() {
  uint16_t data_address = 0;

  if (mem_addr_width_ == MemAddrWidth::OneByte) {
    data_address = *(uint8_t*)temp_buffer_;
  } else {
    data_address = *(uint16_t*)temp_buffer_;
    data_address = __builtin_bswap16(data_address);  // Swap bytes
  }

  ETL_ASSERT(data_address < sizeof(temp_buffer_), ETL_ERROR(0));
  return static_cast<int32_t>(data_address);
}

void I2cSlave::slaveMatchMasterWriteCb() {
  // Slave read, master write
  // HAL_StatusTypeDef hal_status =
  //    HAL_I2C_Slave_Seq_Receive_DMA(i2c_handle_, temp_buffer_, sizeof(temp_buffer_), I2C_FIRST_AND_LAST_FRAME);

  HAL_StatusTypeDef hal_status =
      HAL_I2C_Slave_Seq_Receive_IT(i2c_handle_, temp_buffer_, sizeof(temp_buffer_), I2C_FIRST_FRAME);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start receive [OK]");
  } else {
    DEBUG_ERROR("Start receive [FAILED]");
  }
}

void I2cSlave::slaveMatchMasterReadCb() {
  // Slave write, master read
  size_t data_address = getDataAddress();
  size_t max_size = sizeof(data_buffer_) - data_address;

  // HAL_StatusTypeDef hal_status = HAL_I2C_Slave_Seq_Transmit_DMA(i2c_handle_, data_buffer_ + data_address,
  //                                                              static_cast<uint16_t>(max_size), I2C_LAST_FRAME);

  HAL_StatusTypeDef hal_status = HAL_I2C_Slave_Seq_Transmit_IT(i2c_handle_, data_buffer_ + data_address,
                                                               static_cast<uint16_t>(max_size), I2C_LAST_FRAME);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start transmit (addr: 0x%04X) [OK]", data_address);
  } else {
    DEBUG_ERROR("Start transmit (addr: 0x%04X) [FAILED]", data_address);
  }
}

void I2cSlave::writeCompleteCb() {
  // size_t rx_cnt = DMA_RX_MEM_WRITE_POS - sizeof(uint16_t);

  int32_t rx_total = sizeof(temp_buffer_) - i2c_handle_->XferSize;
  int32_t rx_cnt = rx_total - static_cast<int32_t>(mem_addr_width_);

  if (rx_total >= static_cast<int32_t>(mem_addr_width_)) {
    mem_address_ = getDataAddress();
  } else {
    mem_address_ = -1;
    ETL_ASSERT(false, ETL_ERROR(0));  // TODO: Remove assert and handle such case properly
  }

  ETL_ASSERT(static_cast<size_t>(mem_address_ + rx_cnt) <= sizeof(data_buffer_), ETL_ERROR(0));
  DEBUG_INFO("writeCompleteCb (addr: 0x%04X, size: %d) [OK]", mem_address_, rx_cnt);

  if (rx_cnt > 0) {
    memcpy(data_buffer_ + mem_address_, temp_buffer_ + static_cast<size_t>(mem_addr_width_), rx_cnt);

    // return;  // Suppress notification (for testing only)

    if (notifyAccessRequest(rx_cnt, mem_address_, 0, 0) == Status_t::Ok) {
      DEBUG_INFO("Notify write-access (access id: %d, size: %d) [OK]", access_id_, rx_cnt);
    } else {
      DEBUG_ERROR("Notify write-access (access id: %d, size: %d) [FAILED]", access_id_, rx_cnt);
    }
  }
}

void I2cSlave::readCompleteCb() {
  // int32_t tx_cnt = DMA_TX_MEM_READ_POS - 9;  // TODO: Why -9?

  if (mem_address_ < 0) {
    DEBUG_ERROR("Duplicate read-access (access id: %d) [DUPLICATE]", access_id_);
    return;
  }

  // ETL_ASSERT(mem_address_ >= 0, ETL_ERROR(0));
  size_t max_size = sizeof(data_buffer_) - mem_address_;
  int32_t tx_cnt = max_size - i2c_handle_->XferSize - 1;  // TODO: Why -1?

  ETL_ASSERT(tx_cnt >= 0, ETL_ERROR(0));
  DEBUG_INFO("readCompleteCb (addr: 0x%04X, size: %d) [OK]", mem_address_, tx_cnt);

  // return;  // Suppress notification (for testing only)

  if (notifyAccessRequest(0, 0, tx_cnt, mem_address_) == Status_t::Ok) {
    DEBUG_INFO("Notify read-access (access id: %d, size: %d) [OK]", access_id_, tx_cnt);
  } else {
    DEBUG_ERROR("Notify read-access (access id: %d, size: %d) [FAILED]", access_id_, tx_cnt);
  }

  // Reset memory address
  mem_address_ = -1;
}

Status_t I2cSlave::notifyAccessRequest(size_t write_size, size_t write_addr, size_t read_size, size_t read_addr) {
  Status_t status = Status_t::Ok;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Increment access id
  access_id_++;

  // Check request queue space
  if (free_slots == 0) {
    DEBUG_ERROR("Queue overflow (access: %d)", access_id_);
    return Status_t::Error;
  }

  Request request = {
    .request_id = 0,
    .access_id = access_id_,
    .status_code = RequestStatus::Complete,
    .write_size = static_cast<uint16_t>(write_size),
    .read_size = static_cast<uint16_t>(read_size),
    .write_addr = write_addr,
    .read_addr = read_addr,
  };

  // Add request to request_queue
  sts = tx_queue_send(&request_queue_, &request, TX_NO_WAIT);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Trigger i2c task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);

  return status;
}

Status_t I2cSlave::serviceStatus(StatusInfo* info) {
  Status_t status = Status_t::Ok;

  Request* request = &info->request;
  if (tx_queue_receive(&request_queue_, request, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("No requests/notifications to service [FAILED]");
    return Status_t::Error;
  }

  info->sequence_number = seqence_number_;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(free_slots);

  return status;
}

Status_t I2cSlave::copyData(size_t addr, uint8_t* data, size_t size) {
  Status_t status = Status_t::Ok;

  if ((addr + size) > sizeof(data_buffer_)) {
    DEBUG_ERROR("Bad request (addr: 0x%04X, size: %d)", addr, size);
    return Status_t::Error;
  }

  std::memcpy(data, data_buffer_ + addr, size);
  return status;
}

} /* namespace hal::i2c */
