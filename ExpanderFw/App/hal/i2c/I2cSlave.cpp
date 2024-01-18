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

#define DEBUG_ENABLE_I2C_SLAVE
#ifdef DEBUG_ENABLE_I2C_SLAVE
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cSlv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cSlv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cSlv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

#define DMA_RX_MEM_WRITE_POS (sizeof(temp_buffer_) - __HAL_DMA_GET_COUNTER(i2c_handle_->hdmarx))
#define DMA_TX_MEM_READ_POS (sizeof(data_buffer_) - getDataAddress() - __HAL_DMA_GET_COUNTER(i2c_handle_->hdmatx))

namespace hal {
namespace i2c {

I2cSlave::I2cSlave(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle) : i2c_id_{ i2c_id }, i2c_handle_{ i2c_handle } {
  uint32_t sts = TX_SUCCESS;
  sts = tx_queue_create(&request_queue_,                           //
                        const_cast<char*>("I2cSlaveRequestQ"),     //
                        sizeof(QueueItem), request_queue_buffer_,  //
                        RequestQueue_MaxItemCnt * sizeof(QueueItem) * sizeof(ULONG));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
}

Status_t I2cSlave::config() {
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

  memset(request_buffer_, 0, sizeof(request_buffer_));
  request_buffer_idx_ = 0;

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

  RequestSlot* request_slot = setupRequestSlot(request);
  ETL_ASSERT(request_slot != nullptr, ETL_ERROR(0));

  // Add request to request_queue
  QueueItem queue_item = { .slot = request_slot };
  sts = tx_queue_send(&request_queue_, &queue_item, 0);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  return exitScheduleRequest(request, seq_num);
}

I2cSlave::RequestSlot* I2cSlave::setupRequestSlot(Request* request) {
  RequestSlot* slot = nullptr;

  // Search for free slot
  size_t counter = RequestBufferSize;
  while ((request_buffer_[request_buffer_idx_].used == true) && (counter > 0)) {
    request_buffer_idx_++;
    if (request_buffer_idx_ >= RequestBufferSize) {
      request_buffer_idx_ = 0;
    }
    counter--;
  }

  // Copy request into slot
  if (request_buffer_[request_buffer_idx_].used == false) {
    slot = &request_buffer_[request_buffer_idx_];
    slot->request = *request;
    slot->used = true;
  }

  return slot;
}

Status_t I2cSlave::exitScheduleRequest(Request* request, uint32_t seq_num) {
  Status_t status = Status_t::Ok;

  if (request->status_code == RequestStatus::Complete) {
    DEBUG_INFO("Sched. request (req: %d) [OK]", request->request_id);
    status = Status_t::Ok;

  } else if (request->status_code == RequestStatus::NoSpace) {
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);
    DEBUG_ERROR("Rejected request (req: %d) is NOT reported!", request->request_id);

  } else if (request->status_code == RequestStatus::BadRequest) {
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);
    RequestSlot* request_slot = setupRequestSlot(request);
    ETL_ASSERT(request_slot != nullptr, ETL_ERROR(0));

    QueueItem queue_item = { .slot = request_slot };
    uint32_t sts = tx_queue_send(&request_queue_, &queue_item, 0);
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

size_t I2cSlave::getDataAddress() {
  uint16_t data_address = *(uint16_t*)temp_buffer_;
  data_address = __builtin_bswap16(data_address);  // Swap bytes
  return static_cast<size_t>(data_address);
}

void I2cSlave::slaveMatchMasterWriteCb() {
  // Slave read, master write
  HAL_StatusTypeDef hal_status =
      HAL_I2C_Slave_Seq_Receive_DMA(i2c_handle_, temp_buffer_, sizeof(temp_buffer_), I2C_FIRST_AND_LAST_FRAME);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start receive [OK]");
  } else {
    DEBUG_ERROR("Start receive [FAILED]");
  }
}

void I2cSlave::slaveMatchMasterReadCb() {
  // Slave write, master read
  size_t data_address = getDataAddress();
  uint16_t max_size = static_cast<uint16_t>(sizeof(data_buffer_) - data_address);

  HAL_StatusTypeDef hal_status =
      HAL_I2C_Slave_Seq_Transmit_DMA(i2c_handle_, data_buffer_ + data_address, max_size, I2C_FIRST_AND_LAST_FRAME);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start transmit (addr: 0x%04X) [OK]", data_address);
  } else {
    DEBUG_ERROR("Start transmit (addr: 0x%04X) [FAILED]", data_address);
  }
}

void I2cSlave::writeCompleteCb() {
  size_t rx_cnt = DMA_RX_MEM_WRITE_POS - sizeof(uint16_t);
  size_t data_address = getDataAddress();

  ETL_ASSERT((data_address + rx_cnt) <= sizeof(data_buffer_), ETL_ERROR(0));
  DEBUG_INFO("writeCompleteCb (addr: 0x%04X, cnt: %d) [OK]", data_address, rx_cnt);

  if (rx_cnt > 0) {
    memcpy(data_buffer_ + data_address, temp_buffer_ + sizeof(uint16_t), rx_cnt);

    // return; // Suppress notification

    RequestSlot* access_slot = notifyAccessRequest(rx_cnt, data_address, 0, 0);
    if (access_slot != nullptr) {
      DEBUG_INFO("Notify write-access (access: %d) [OK]", access_slot->request.access_id);
    } else {
      DEBUG_ERROR("Notify write-access (access: %d) [FAILED]", access_slot->request.access_id);
    }
  }
}

void I2cSlave::readCompleteCb() {
  size_t tx_cnt = DMA_TX_MEM_READ_POS - 9;  // TODO: Why -9?
  size_t data_address = getDataAddress();
  DEBUG_INFO("readCompleteCb (addr: 0x%04X, cnt: %d) [OK]", data_address, tx_cnt);

  // return; // Suppress notification

  RequestSlot* access_slot = notifyAccessRequest(0, 0, tx_cnt, data_address);
  if (access_slot != nullptr) {
    DEBUG_INFO("Notify read-access (access: %d) [OK]", access_slot->request.access_id);
  } else {
    DEBUG_ERROR("Notify read-access (access: %d) [FAILED]", access_slot->request.access_id);
  }
}

I2cSlave::RequestSlot* I2cSlave::notifyAccessRequest(size_t write_size, size_t write_addr, size_t read_size,
                                                     size_t read_addr) {
  // Increment access id
  access_id_++;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Check request queue space
  if (free_slots == 0) {
    DEBUG_ERROR("Queue overflow (access: %d)", access_id_);
    return nullptr;
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
  RequestSlot* access_slot = setupRequestSlot(&request);
  ETL_ASSERT(access_slot != nullptr, ETL_ERROR(0));

  // Add request to request_queue
  QueueItem queue_item = { .slot = access_slot };
  sts = tx_queue_send(&request_queue_, &queue_item, 0);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Trigger i2c task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);

  return access_slot;
}

Status_t I2cSlave::serviceStatus(StatusInfo* info, uint8_t* mem_data, size_t max_size) {
  Status_t status = Status_t::Ok;

  QueueItem queue_item = {};
  if (tx_queue_receive(&request_queue_, &queue_item, 0) != TX_SUCCESS) {
    DEBUG_ERROR("No requests/notifications to service [FAILED]");
    return Status_t::Error;
  }

  Request* request = &queue_item.slot->request;
  info->sequence_number = seqence_number_;
  info->request = *request;
  info->size = 0;

  if (request->status_code == RequestStatus::Complete) {
    if ((request->request_id > 0) && (request->read_size > 0)) {
      // Feedback on read request from PC
      ETL_ASSERT(request->read_size <= max_size, ETL_ERROR(0));
      std::memcpy(mem_data, data_buffer_ + request->read_addr, request->read_size);
      info->size = request->read_size;

    } else if ((request->access_id > 0) && (request->write_size > 0)) {
      // Report slave write access notification to PC
      ETL_ASSERT(request->write_size <= max_size, ETL_ERROR(0));
      std::memcpy(mem_data, data_buffer_ + request->write_addr, request->write_size);
      info->size = request->write_size;
    }

  } else {
    // TODO
  }

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(free_slots);

  // Release slot
  queue_item.slot->used = false;
  return Status_t::Ok;
}

} /* namespace i2c */
} /* namespace hal */
