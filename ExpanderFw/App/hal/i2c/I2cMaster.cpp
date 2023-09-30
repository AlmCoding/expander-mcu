/*
 * I2cMaster.cpp
 *
 *  Created on: Aug 15, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cMaster.hpp"
#include "etl/algorithm.h"      // etl::max
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "hal/i2c/I2cIrq.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_MASTER
#ifdef DEBUG_ENABLE_I2C_MASTER
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cMstr]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cMstr]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cMstr]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::i2c {

I2cMaster::I2cMaster(I2C_HandleTypeDef* i2c_handle) : i2c_handle_{ i2c_handle } {
  uint32_t sts;
  sts = tx_queue_create(&pending_queue_,                         //
                        const_cast<char*>("I2cMasterPendQ"),     //
                        sizeof(Request), pending_queue_buffer_,  //
                        RequestQueue_MsgCnt * sizeof(I2cMaster::Request) * sizeof(ULONG));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  sts = tx_queue_create(&complete_queue_,                         //
                        const_cast<char*>("I2cMasterCpltQ"),      //
                        sizeof(Request), complete_queue_buffer_,  //
                        RequestQueue_MsgCnt * sizeof(I2cMaster::Request) * sizeof(ULONG));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  memset(&request_, 0, sizeof(Request));
  request_.status_code = RequestStatus::Complete;
}

Status_t I2cMaster::config() {
  Status_t status;

  if (init() == Status_t::Ok) {
    DEBUG_INFO("Init [ok]")

  } else {
    DEBUG_ERROR("Init [failed]")
    status = Status_t::Error;
  }

  I2cIrq::getInstance().registerI2cMaster(this);
  return status;
}

Status_t I2cMaster::init() {
  tx_queue_flush(&pending_queue_);
  tx_queue_flush(&complete_queue_);

  memset(&request_, 0, sizeof(Request));
  request_.status_code = RequestStatus::Complete;

  request_complete_ = true;
  sequence_.ongoing = false;

  return Status_t::Ok;
}

uint32_t I2cMaster::poll() {
  uint32_t service_requests = 0;
  uint32_t sts, free_slots;

#if (START_I2_REQUEST_IMMEDIATELY == false)
  startRequest();
#endif

  sts = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots < RequestQueue_MsgCnt) {
    service_requests = 1;
  }

  return service_requests;
}

Status_t I2cMaster::scheduleRequest(Request* request, uint8_t* write_data, uint32_t seq_num) {
  uint32_t sts, free_slots;

  sts = tx_queue_info_get(&pending_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Check pending queue space
  if (free_slots == 0) {
    DEBUG_ERROR("Queue overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  if (allocateBufferSpace(request) == Status_t::Error) {
    DEBUG_ERROR("Buffer overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  // Copy write data to buffer
  if (request->write_size > 0) {
    std::memcpy(data_buffer_ + request->write_start, write_data, request->write_size);
  }

  // Set request status to pending
  request->status_code = RequestStatus::Pending;

  // Add request to pending_queue
  sts = tx_queue_send(&pending_queue_, request, 0);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

#if (START_I2_REQUEST_IMMEDIATELY == true)
  startRequest();
#endif

  return exitScheduleRequest(request, seq_num);
}

Status_t I2cMaster::exitScheduleRequest(Request* request, uint32_t seq_num) {
  Status_t status;

  if (request->status_code == hal::i2c::I2cMaster::RequestStatus::NoSpace) {
    DEBUG_INFO("Sched. request (req: %d) [failed]", request->request_id);
    UINT sts = tx_queue_send(&complete_queue_, request, 0);
    ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
    status = Status_t::Error;

  } else {
    DEBUG_INFO("Sched. request (req: %d) [ok]", request->request_id);
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

Status_t I2cMaster::serviceStatus(StatusInfo* info, uint8_t* read_data, size_t max_size) {
  Status_t status = Status_t::Ok;
  Request request;

  if (tx_queue_receive(&complete_queue_, &request, 0) != TX_SUCCESS) {
    status = Status_t::Error;
  }

  info->sequence_number = seqence_number_;
  info->status_code = request.status_code;
  info->request_id = request.request_id;

  if (request.status_code == RequestStatus::Complete) {
    info->read_size = request.read_size;

    ETL_ASSERT(request.read_size <= max_size, ETL_ERROR(0));
    if (request.read_size > 0) {
      std::memcpy(read_data, data_buffer_ + request.read_start, request.read_size);
    }

    // Free buffer space
    freeBufferSpace(&request);

  } else {
    info->read_size = 0;
  }

  uint32_t sts_pend, sts_cplt;
  uint32_t free_slots_pend, free_slots_cplt;
  sts_pend = tx_queue_info_get(&pending_queue_, nullptr, nullptr, &free_slots_pend, nullptr, nullptr, nullptr);
  sts_cplt = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots_cplt, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts_pend == TX_SUCCESS, ETL_ERROR(0));
  ETL_ASSERT(sts_cplt == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(etl::min(free_slots_pend, free_slots_cplt));

  Space free = getFreeSpace();
  info->buffer_space1 = static_cast<uint16_t>(free.space1);
  info->buffer_space2 = static_cast<uint16_t>(free.space2);

  return status;
}

I2cMaster::Space I2cMaster::getFreeSpace() {
  Space free;

  if (data_start_ < data_end_) {
    free.space1 = sizeof(data_buffer_) - data_end_ - 1;

    if (data_start_ > 0) {
      free.space2 = data_start_ - 1;
    } else {
      free.space2 = 0;
    }

  } else if (data_start_ > data_end_) {
    free.space1 = data_start_ - data_end_ - 1;
    free.space2 = 0;

  } else {
    free.space1 = sizeof(data_buffer_) - 1;
    free.space2 = 0;
  }

  ETL_ASSERT((free.space1 + free.space2) <= (sizeof(data_buffer_) - 1), ETL_ERROR(0));
  return free;
}

Status_t I2cMaster::allocateBufferSpace(Request* request) {
  Status_t status = Status_t::Ok;
  Space free = getFreeSpace();

  DEBUG_INFO("Before allocate (req: %d) [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             request->request_id, data_start_, data_end_, free.space1, free.space2);

  if (free.space1 >= (request->write_size + request->read_size)) {
    // Place everything into space1
    request->write_start = data_end_;
    data_end_ += request->write_size;
    request->read_start = data_end_;
    data_end_ += request->read_size;

  } else if ((request->write_size >= request->read_size) &&  //
             (free.space1 >= request->write_size) &&         //
             (free.space2 >= request->read_size)) {
    // Put write data into space1 and read data into space2
    request->write_start = data_end_;
    request->read_start = 0;
    data_end_ = request->read_size;

  } else if ((request->write_size < request->read_size) &&  //
             (free.space1 >= request->read_size) &&         //
             (free.space2 >= request->write_size)) {
    // Put read data into space1 and write data into space2
    request->read_start = data_end_;
    request->write_start = 0;
    data_end_ = request->write_size;

  } else if (free.space2 >= (request->write_size + request->read_size)) {
    // Place everything into space2
    request->write_start = 0;
    data_end_ = request->write_size;
    request->read_start = data_end_;
    data_end_ += request->read_size;

  } else {
    // No enough free space
    DEBUG_ERROR("Allocate space (req. id: %d) [failed]", request->request_id);
    status = Status_t::Error;
  }

  free = getFreeSpace();
  DEBUG_INFO("After allocate (req: %d, ws: %d (%d), rs: %d (%d)) [ds: %d, de: %d, sp1: %d, sp2: %d]",
             request->request_id,                                                                 //
             request->write_start, request->write_size, request->read_start, request->read_size,  //
             data_start_, data_end_, free.space1, free.space2);

  return status;
}

void I2cMaster::freeBufferSpace(Request* request) {
  Space free = getFreeSpace();
  DEBUG_INFO("Before free (req: %d, ws: %d (%d), rs: %d (%d)) [ds: %d, de: %d, sp1: %d, sp2: %d]",
             request->request_id,                                                                 //
             request->write_start, request->write_size, request->read_start, request->read_size,  //
             data_start_, data_end_, free.space1, free.space2);

  if (data_start_ == request->write_start) {
    data_start_ += request->write_size;

    if (data_start_ == request->read_start) {
      data_start_ += request->read_size;
    } else if (request->read_start == 0) {
      data_start_ = request->read_size;
    } else {
      ETL_ASSERT(false, ETL_ERROR(0));
    }

  } else if (data_start_ == request->read_start) {
    data_start_ += request->read_size;

    if (request->write_start == 0) {
      data_start_ = request->write_size;
    } else {
      ETL_ASSERT(false, ETL_ERROR(0));
    }

  } else if (request->write_start == 0) {
    // Skip gap at buffer end
    data_start_ = request->write_size + request->read_size;

  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  free = getFreeSpace();
  DEBUG_INFO("After free (req: %d) [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             request->request_id, data_start_, data_end_, free.space1, free.space2);
}

Status_t I2cMaster::startRequest() {
  Status_t status = Status_t::Ok;
  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(i2c_handle_);
  uint32_t sts, free_slots;

  sts = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if ((request_complete_ == false) ||  // Request ongoing
      (free_slots == 0) ||             // Full complete queue
      (state != HAL_I2C_STATE_READY)) {
    // Ongoing request or full complete queue or busy i2c
    return Status_t::Busy;
  }

  if (tx_queue_receive(&pending_queue_, &request_, 0) == TX_SUCCESS) {
    request_.status_code = RequestStatus::Ongoing;

    // Check for sequence start
    if ((request_.sequence_idx > 0) && (sequence_.ongoing == false)) {
      sequence_.max_idx = request_.sequence_idx;
      sequence_.id = request_.sequence_id;
      sequence_.ongoing = true;
    }

    // Request taken from queue
    if ((request_.write_size > 0) && (request_.read_size == 0)) {
      // Write only
      status = startWrite();
    } else if ((request_.read_size > 0) && (request_.write_size == 0)) {
      // Read only
      status = startRead();
    } else if ((request_.read_size > 0) && (request_.write_size <= 2) && (sequence_.ongoing == false)) {
      // Read register
      status = startReadReg();
    } else if ((request_.read_size > 0) && (request_.write_size > 0)) {
      // Write + read
      status = startWrite();
    } else {
      DEBUG_ERROR("Start request (req: %d) [failed]", request_.request_id);
      status = Status_t::Error;
    }
  }

  return status;
}

Status_t I2cMaster::startWrite() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint16_t slave_addr = request_.slave_addr << 1;
  uint32_t xfer_options;

  if (sequence_.ongoing == true) {
    if (request_.sequence_idx == sequence_.max_idx) {
      // First frame in sequence
      xfer_options = I2C_FIRST_FRAME;
    } else if ((request_.sequence_idx == 0) && (request_.read_size == 0)) {
      // Last frame in sequence and request
      xfer_options = I2C_LAST_FRAME;
    } else {
      // Other frame
      xfer_options = I2C_NEXT_FRAME;
    }

  } else {
    if (request_.read_size == 0) {
      xfer_options = I2C_FIRST_AND_LAST_FRAME;
    } else {
      xfer_options = I2C_FIRST_FRAME;
    }
  }

  hal_status = HAL_I2C_Master_Seq_Transmit_DMA(i2c_handle_,                          // Handle
                                               slave_addr,                           // Addr
                                               data_buffer_ + request_.write_start,  // Data
                                               request_.write_size,                  // Length
                                               xfer_options);
  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start write (req: %d) [ok]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start write (req: %d) [failed]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

Status_t I2cMaster::startReadReg() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint16_t slave_addr = request_.slave_addr << 1;
  uint16_t reg_addr = *(uint16_t*)(data_buffer_ + request_.write_start);

  hal_status = HAL_I2C_Mem_Read_DMA(i2c_handle_,                         // Handle
                                    slave_addr,                          // Slave addr
                                    reg_addr,                            // Register addr
                                    request_.write_size,                 // Addr size
                                    data_buffer_ + request_.read_start,  // Ptr
                                    request_.read_size);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start read reg (req: %d) [ok]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start read reg (req: %d) [failed]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

Status_t I2cMaster::startRead() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint16_t slave_addr = request_.slave_addr << 1;
  uint32_t xfer_options;

  if (sequence_.ongoing == true) {
    if ((request_.sequence_idx == sequence_.max_idx) && (request_.write_size == 0)) {
      // First frame in sequence and request
      xfer_options = I2C_FIRST_FRAME;
    } else if (request_.sequence_idx == 0) {
      // Last frame in sequence and request
      xfer_options = I2C_LAST_FRAME;
    } else {
      // Other frame
      xfer_options = I2C_NEXT_FRAME;
    }

  } else {
    if (request_.write_size == 0) {
      xfer_options = I2C_FIRST_AND_LAST_FRAME;
    } else {
      xfer_options = I2C_LAST_FRAME;
    }
  }

  hal_status = HAL_I2C_Master_Seq_Receive_DMA(i2c_handle_,                         // Handle
                                              slave_addr,                          // Addr
                                              data_buffer_ + request_.read_start,  // Data
                                              request_.read_size,                  // Length
                                              xfer_options);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start read (req: %d) [ok]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start read (req: %d) [failed]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

void I2cMaster::writeCompleteCb() {
  if (request_.read_size > 0) {
    startRead();

  } else {
    DEBUG_INFO("Write cplt (req: %d) [ok]", request_.request_id);
    complete();
  }
}

void I2cMaster::readCompleteCb() {
  DEBUG_INFO("Read cplt (req: %d) [ok]", request_.request_id);
  complete();
}

void I2cMaster::complete() {
  request_.status_code = RequestStatus::Complete;

  // Add request to complete_queue
  if (tx_queue_send(&complete_queue_, &request_, 0) != TX_SUCCESS) {
    DEBUG_ERROR("Cplt queue put (req: %d) [failed]", request_.request_id);
  }

  if (request_.sequence_idx == 0) {
    sequence_.ongoing = false;
  }

  request_complete_ = true;

  // Trigger i2c task for fast service notification
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);
}

}  // namespace hal::i2c
