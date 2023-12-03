/*
 * I2cSlave.cpp
 *
 *  Created on: Nov 25, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cSlave.hpp"
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

namespace hal {
namespace i2c {

I2cSlave::I2cSlave(I2C_HandleTypeDef* i2c_handle) : i2c_handle_{ i2c_handle } {
  uint32_t sts;
  sts = tx_queue_create(&request_queue_,                           //
                        const_cast<char*>("I2cSlaveRequestQ"),     //
                        sizeof(QueueItem), request_queue_buffer_,  //
                        RequestQueue_MaxItemCnt * sizeof(QueueItem) * sizeof(ULONG));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
}

Status_t I2cSlave::config() {
  Status_t status;

  if (init() == Status_t::Ok) {
    DEBUG_INFO("Init [OK]");

  } else {
    DEBUG_ERROR("Init [FAILED]");
    status = Status_t::Error;
  }

  I2cIrq::getInstance().registerI2cSlave(this);
  return status;
}

Status_t I2cSlave::init() {
  tx_queue_flush(&request_queue_);

  memset(request_buffer_, 0, sizeof(request_buffer_));
  request_buffer_idx_ = 0;

  memset(data_buffer_, 0, sizeof(data_buffer_));

  seqence_number_ = 0;

  return Status_t::Ok;
}

uint32_t I2cSlave::poll() {
  uint32_t service_requests = 0;
  uint32_t sts, free_slots;

  sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots < RequestQueue_MaxItemCnt) {
    service_requests = 1;
  }

  return service_requests;
}

Status_t I2cSlave::scheduleRequest(Request* request, uint8_t* mem_data, uint32_t seq_num) {
  uint32_t sts;
  uint32_t free_slots;

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
  Status_t status;
  uint32_t sts;

  if (request->status_code == RequestStatus::NoSpace) {
    DEBUG_ERROR("Rejected request (req: %d) is NOT reported!", request->request_id);
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);

  } else if (request->status_code == RequestStatus::BadRequest) {
    DEBUG_ERROR("Sched. request (req: %d) [FAILED]", request->request_id);
    RequestSlot* request_slot = setupRequestSlot(request);
    ETL_ASSERT(request_slot != nullptr, ETL_ERROR(0));

    QueueItem queue_item = { .slot = request_slot };
    sts = tx_queue_send(&request_queue_, &queue_item, 0);
    ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
    status = Status_t::Error;

  } else {
    DEBUG_INFO("Sched. request (req: %d) [OK]", request->request_id);
    status = Status_t::Ok;
  }

  // Update sequence number
  seqence_number_ = seq_num;

  // Trigger i2c task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);

  return status;
}

Status_t I2cSlave::serviceStatus(StatusInfo* info, uint8_t* mem_data, size_t max_size) {
  Status_t status = Status_t::Ok;

  QueueItem queue_item;
  if (tx_queue_receive(&request_queue_, &queue_item, 0) != TX_SUCCESS) {
    DEBUG_ERROR("No requests to service [FAILED]");
    status = Status_t::Error;
  }

  Request* request = &queue_item.slot->request;
  info->sequence_number = seqence_number_;
  info->request = *request;
  info->size = 0;

  if (request->status_code == RequestStatus::Complete) {
    if ((request->request_id > 0) && (request->read_size > 0)) {
      // Feedback on read request
      ETL_ASSERT(request->read_size <= max_size, ETL_ERROR(0));
      std::memcpy(mem_data, data_buffer_ + request->read_addr, request->read_size);
      info->size = request->read_size;

    } else if ((request->access_id > 0) && (request->write_size > 0)) {
      // Report slave access
      ETL_ASSERT(request->read_size <= max_size, ETL_ERROR(0));
      std::memcpy(mem_data, data_buffer_ + request->write_addr, request->write_size);
      info->size = request->write_size;
    }
  }

  uint32_t sts;
  uint32_t free_slots;
  sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(free_slots);

  // Release slot
  queue_item.slot->used = false;
  return status;
}

} /* namespace i2c */
} /* namespace hal */
