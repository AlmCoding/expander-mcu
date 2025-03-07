/*
 * I2cMaster.cpp
 *
 *  Created on: Aug 15, 2023
 *      Author: Alexander L.
 */

#include "hal/i2c/I2cMaster.hpp"
#include "enum/magic_enum.hpp"
#include "etl/algorithm.h"      // etl::max
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "hal/i2c/I2cIrq.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_I2C_MASTER 1
#if ((DEBUG_ENABLE_I2C_MASTER == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cMstr]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cMstr]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cMstr]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::i2c {

I2cMaster::I2cMaster(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle) : i2c_id_{ i2c_id }, i2c_handle_{ i2c_handle } {
  uint32_t sts = TX_SUCCESS;
  sts = tx_queue_create(&pending_queue_,                      //
                        const_cast<char*>("I2cMasterPendQ"),  //
                        sizeof(Request) / sizeof(uint32_t),   //
                        pending_queue_buffer_,                //
                        sizeof(pending_queue_buffer_));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  sts = tx_queue_create(&complete_queue_,                     //
                        const_cast<char*>("I2cMasterCpltQ"),  //
                        sizeof(Request) / sizeof(uint32_t),   //
                        complete_queue_buffer_,               //
                        sizeof(complete_queue_buffer_));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
}

Status_t I2cMaster::config() {
  Status_t status = init();

  if (status == Status_t::Ok) {
    I2cIrq::getInstance().registerI2cMaster(this);
    DEBUG_INFO("Init I2cMaster(%d) [OK]", magic_enum::enum_integer(i2c_id_));

  } else {
    DEBUG_ERROR("Init I2cMaster(%d) [FAILED]", magic_enum::enum_integer(i2c_id_));
    status = Status_t::Error;
  }

  ETL_ASSERT(status == Status_t::Ok, ETL_ERROR(0));
  return status;
}

Status_t I2cMaster::init() {
  Status_t status = Status_t::Ok;

  tx_queue_flush(&pending_queue_);
  tx_queue_flush(&complete_queue_);

  data_buffer_end_ = 0;
  data_start_ = 0;
  data_end_ = 0;

  request_ = {};
  transfer_state_ = TransferState::Write;
  request_ongoing_ = false;
  sequence_state_ = {};

  seqence_number_ = 0;
  return status;
}

uint32_t I2cMaster::poll() {
  uint32_t service_requests = 0;

#if (START_I2_REQUEST_IMMEDIATELY == false)
  startRequest();
#endif

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots < RequestQueue_MaxItemCnt) {
    service_requests = 1;
  }

  return service_requests;
}

I2cMaster::Space I2cMaster::getFreeSpace() {
  Space space = {};

  if (data_start_ <= data_end_) {
    space.end_to_back = sizeof(data_buffer_) - data_end_;
    space.front_to_start = data_start_;

  } else {
    space.end_to_back = data_start_ - data_end_;
    space.front_to_start = 0;
  }

  if (space.end_to_back > 0) {
    space.end_to_back--;
  }

  if (space.front_to_start > 0) {
    space.front_to_start--;
  }

  DEBUG_INFO("Space [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             data_start_, data_end_, space.end_to_back, space.front_to_start);
  ETL_ASSERT((space.end_to_back + space.front_to_start) <= (sizeof(data_buffer_) - 1), ETL_ERROR(0));
  return space;
}

Status_t I2cMaster::scheduleRequest(Request* request, uint8_t* write_data, uint32_t seq_num) {
  uint32_t sts = TX_SUCCESS;

  // TODO: Check for bad request (e.g. read size too big)

  uint32_t free_slots = 0;
  sts = tx_queue_info_get(&pending_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Check pending queue space
  if (free_slots == 0) {
    DEBUG_WARN("Queue overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  if (allocateBufferSpace(request) == Status_t::Error) {
    DEBUG_WARN("Buffer overflow (req: %d)", request->request_id);
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
  sts = tx_queue_send(&pending_queue_, request, TX_NO_WAIT);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  return exitScheduleRequest(request, seq_num);
}

Status_t I2cMaster::allocateBufferSpace(Request* request) {
  Status_t status = Status_t::Ok;
  Space space = getFreeSpace();

  DEBUG_INFO("Before allocate (req: %d) [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             request->request_id, data_start_, data_end_, space.end_to_back, space.front_to_start);

  size_t front_offset = 0;
  size_t back_offset = data_end_;

  if (request->write_size >= request->read_size) {
    status = allocateBufferSection(&front_offset, &back_offset, &space, &request->write_start, request->write_size);
    if (status == Status_t::Ok) {
      status = allocateBufferSection(&front_offset, &back_offset, &space, &request->read_start, request->read_size);
    }

  } else {
    status = allocateBufferSection(&front_offset, &back_offset, &space, &request->read_start, request->read_size);
    if (status == Status_t::Ok) {
      status = allocateBufferSection(&front_offset, &back_offset, &space, &request->write_start, request->write_size);
    }
  }

  if (status == Status_t::Ok) {
    if (front_offset == 0) {
      data_end_ = back_offset;
    } else {
      data_end_ = front_offset;
    }

  } else {
    // Not enough free space
    DEBUG_WARN("Allocate space (req. id: %d) [FAILED]", request->request_id);
    request->write_start = SIZE_MAX;
    request->read_start = SIZE_MAX;
  }

  space = getFreeSpace();
  DEBUG_INFO("After allocate (req: %d, ws: %d (%d), rs: %d (%d)) [ds: %d, de: %d, sp1: %d, sp2: %d]",
             request->request_id,                                                                 //
             request->write_start, request->write_size, request->read_start, request->read_size,  //
             data_start_, data_end_, space.end_to_back, space.front_to_start);

  return status;
}

Status_t I2cMaster::allocateBufferSection(size_t* front_offset, size_t* back_offset, Space* space,
                                          size_t* section_start, size_t section_size) {
  Status_t status = Status_t::Ok;

  // Fill space1 (end_to_back) first if possible
  if (space->end_to_back >= section_size) {
    // Place section into space1
    *section_start = *back_offset;
    *back_offset += section_size;
    space->end_to_back -= section_size;

  } else if (space->front_to_start >= section_size) {
    // Place section into space2
    *section_start = *front_offset;
    *front_offset += section_size;
    space->front_to_start -= section_size;

  } else {
    // Not enough free space
    status = Status_t::Error;
  }

  return status;
}

Status_t I2cMaster::exitScheduleRequest(Request* request, uint32_t seq_num) {
  Status_t status = Status_t::Ok;

  if (request->status_code == RequestStatus::Pending) {
    DEBUG_INFO("Sched. request (req: %d) [OK]", request->request_id);
    status = Status_t::Ok;

#if (START_I2_REQUEST_IMMEDIATELY == true)
    startRequest();
#endif

  } else if (request->status_code == RequestStatus::NoSpace) {
    DEBUG_WARN("Sched. request (req: %d) [FAILED]", request->request_id);
    status = Status_t::Error;

    uint32_t sts = tx_queue_send(&complete_queue_, request, TX_NO_WAIT);
    if (sts != TX_SUCCESS) {
      DEBUG_ERROR("Report rejected request (req: %d) [FAILED]", request->request_id);
    }

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

bool I2cMaster::readyForNewStart() {
  HAL_I2C_StateTypeDef state = HAL_I2C_GetState(i2c_handle_);
  if ((request_ongoing_ == true) || ((state != HAL_I2C_STATE_LISTEN) && (state != HAL_I2C_STATE_READY))) {
    // Ongoing request or i2c busy
    return false;
  }

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots == 0) {
    // Full complete queue
    return false;
  }

  // Ready for new start
  return true;
}

Status_t I2cMaster::startRequest() {
  Status_t status = Status_t::Ok;

  if (readyForNewStart() == false) {
    return Status_t::Busy;
  }

  if (tx_queue_receive(&pending_queue_, &request_, TX_NO_WAIT) == TX_SUCCESS) {
    request_ongoing_ = true;
    request_.status_code = RequestStatus::Ongoing;
    request_.slave_addr <<= 1;  // HAL functions require slave addr to be shifted

    // Check for sequence start
    if ((request_.sequence_idx > 0) && (sequence_state_.ongoing == false)) {
      sequence_state_.max_idx = request_.sequence_idx;
      sequence_state_.seq_id = request_.sequence_id;
      sequence_state_.ongoing = true;
    }

    // Disable slave listening
    I2cIrq::getInstance().disableSlaveListen(i2c_handle_);

    // Request taken from queue
    if ((request_.write_size > 0) && (request_.read_size == 0)) {
      // Write only
      transfer_state_ = TransferState::Write;
      status = startWrite();

    } else if ((request_.read_size > 0) && (request_.write_size == 0)) {
      // Read only
      transfer_state_ = TransferState::Read;
      status = startRead();
      /*
        } else if ((request_.read_size > 0) && (request_.write_size <= 2) && (sequence_state_.ongoing == false)) {
          // Read register
          transfer_state_ = TransferState::Write;
          status = startReadReg();
      */
    } else if ((request_.read_size > 0) && (request_.write_size > 0)) {
      // Write and read
      transfer_state_ = TransferState::Write;
      status = startWrite();

    } else {
      DEBUG_ERROR("Start request (req: %d) [FAILED]", request_.request_id);
      status = Status_t::Error;
    }
  }

  return status;
}

Status_t I2cMaster::startWrite() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint32_t xfer_options;

  if (sequence_state_.ongoing == true) {
    if (request_.sequence_idx == sequence_state_.max_idx) {
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

  hal_status = HAL_I2C_Master_Seq_Transmit_DMA(i2c_handle_,                                 // Handle
                                               static_cast<uint16_t>(request_.slave_addr),  // Addr
                                               data_buffer_ + request_.write_start,         // Data
                                               request_.write_size,                         // Length
                                               xfer_options);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start write (req: %d) [OK]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start write (req: %d) [FAILED]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

Status_t I2cMaster::startReadReg() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint16_t reg_addr = *(uint16_t*)(data_buffer_ + request_.write_start);
  reg_addr = __builtin_bswap16(reg_addr);  // Swap bytes

  hal_status = HAL_I2C_Mem_Read_DMA(i2c_handle_,                                 // Handle
                                    static_cast<uint16_t>(request_.slave_addr),  // Slave addr
                                    reg_addr,                                    // Register addr
                                    request_.write_size,                         // Addr size
                                    data_buffer_ + request_.read_start,          // Ptr
                                    request_.read_size);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start read reg (req: %d) [OK]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start read reg (req: %d) [FAILED]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

Status_t I2cMaster::startRead() {
  Status_t status;
  HAL_StatusTypeDef hal_status;
  uint32_t xfer_options;

  if (sequence_state_.ongoing == true) {
    if ((request_.sequence_idx == sequence_state_.max_idx) && (request_.write_size == 0)) {
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

  hal_status = HAL_I2C_Master_Seq_Receive_DMA(i2c_handle_,                                 // Handle
                                              static_cast<uint16_t>(request_.slave_addr),  // Addr
                                              data_buffer_ + request_.read_start,          // Data
                                              request_.read_size,                          // Length
                                              xfer_options);

  if (hal_status == HAL_OK) {
    DEBUG_INFO("Start read (req: %d) [OK]", request_.request_id);
    status = Status_t::Ok;

  } else {
    DEBUG_ERROR("Start read (req: %d) [FAILED]", request_.request_id);
    status = Status_t::Error;
  }

  return status;
}

void I2cMaster::writeCompleteCb() {
  if (request_.read_size > 0) {
    startRead();
  } else {
    DEBUG_INFO("Write cplt (req: %d) [OK]", request_.request_id);
    complete(RequestStatus::Complete);
  }
}

void I2cMaster::readCompleteCb() {
  DEBUG_INFO("Read cplt (req: %d) [OK]", request_.request_id);
  complete(RequestStatus::Complete);
}

void I2cMaster::errorCb() {
  if (request_ongoing_ == false) {
    DEBUG_INFO("Ignore error in idle state!");
    return;
  }
  DEBUG_INFO("Error detected (req: %d) [OK]", request_.request_id);

  // https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf

  if (transfer_state_ == TransferState::Write) {
    uint32_t remaining_cbr = (i2c_handle_->hdmatx->Instance->CBR1 & DMA_CBR1_BNDT);
    uint32_t remaining_fifo = (i2c_handle_->hdmatx->Instance->CSR & DMA_CSR_FIFOL_Msk) >> DMA_CSR_FIFOL_Pos;

    if (remaining_cbr == request_.write_size) {
      request_.nack_byte_idx = 0;  // Not needed
      complete(RequestStatus::SlaveBusy);
      return;
    }

    uint32_t tx_cnt = request_.write_size - remaining_cbr - remaining_fifo - 1;
    request_.nack_byte_idx = tx_cnt - 1;
    DEBUG_INFO("Byte NACK (idx: %d) by slave!", request_.nack_byte_idx);

  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  complete(RequestStatus::SlaveNack);
}

void I2cMaster::complete(RequestStatus status_code) {
  request_.status_code = status_code;

  // Add request to complete_queue
  if (tx_queue_send(&complete_queue_, &request_, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("Cplt queue put (req: %d) [FAILED]", request_.request_id);
  } else {
    DEBUG_INFO("Cplt queue put (req: %d) [OK]", request_.request_id);
  }

  if (request_.sequence_idx == 0) {
    sequence_state_.ongoing = false;
    I2cIrq::getInstance().enableSlaveListen(i2c_handle_);
  }

  request_ = {};
  request_ongoing_ = false;

  // Trigger i2c task for fast service notification
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::I2cThreadQueue, &msg);
}

Status_t I2cMaster::serviceStatus(StatusInfo* info, uint8_t* read_data, size_t max_size) {
  Status_t status = Status_t::Ok;

  Request request = {};
  if (tx_queue_receive(&complete_queue_, &request, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("No cplt. request to service!");
    return Status_t::Error;
  }

  info->sequence_number = seqence_number_;
  info->request_id = request.request_id;
  info->status_code = request.status_code;
  info->nack_byte_idx = request.nack_byte_idx;
  info->read_size = 0;

  if (request.read_size > max_size) {
    DEBUG_ERROR("Read size bigger than max size!");
    status = Status_t::Error;

  } else if (request.status_code == RequestStatus::Complete) {
    info->read_size = request.read_size;
    std::memcpy(read_data, data_buffer_ + request.read_start, request.read_size);

  } else if (request.status_code == RequestStatus::SlaveNack) {
    // No read data available (because write got terminated)

  } else if (request.status_code == RequestStatus::SlaveBusy) {
    // No read data available (because slave is busy)

  } else if (request.status_code == RequestStatus::NoSpace) {
    // No read data available (because request was not processed)

  } else {
    DEBUG_ERROR("Unhandled request status!");
    status = Status_t::Error;
  }

  if (request.status_code != RequestStatus::NoSpace &&  //
      request.status_code != RequestStatus::BadRequest) {
    // Free buffer space
    freeBufferSpace(&request);
  }

  uint32_t sts_pend, sts_cplt;
  uint32_t free_slots_pend, free_slots_cplt;
  sts_pend = tx_queue_info_get(&pending_queue_, nullptr, nullptr, &free_slots_pend, nullptr, nullptr, nullptr);
  sts_cplt = tx_queue_info_get(&complete_queue_, nullptr, nullptr, &free_slots_cplt, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts_pend == TX_SUCCESS, ETL_ERROR(0));
  ETL_ASSERT(sts_cplt == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(etl::min(free_slots_pend, free_slots_cplt));

  Space space = getFreeSpace();
  info->buffer_space1 = static_cast<uint16_t>(space.end_to_back);
  info->buffer_space2 = static_cast<uint16_t>(space.front_to_start);

  return status;
}

void I2cMaster::freeBufferSpace(Request* request) {
  Space free_space = getFreeSpace();
  DEBUG_INFO("Before free (req: %d, ws: %d (%d), rs: %d (%d)) [ds: %d, de: %d, sp1: %d, sp2: %d]",
             request->request_id,                                                                 //
             request->write_start, request->write_size, request->read_start, request->read_size,  //
             data_start_, data_end_, free_space.end_to_back, free_space.front_to_start);

  if (data_start_ == request->write_start) {
    // Write section placed before read section
    data_start_ += request->write_size;

    if (data_start_ == request->read_start) {
      // Read section placed directly after write section
      data_start_ += request->read_size;
    } else if (request->read_start == 0) {
      // Read section placed at the beginning of the buffer
      data_start_ = request->read_size;
    } else {
      ETL_ASSERT(false, ETL_ERROR(0));
    }

  } else if (data_start_ == request->read_start) {
    // Read section placed before write section
    data_start_ += request->read_size;

    if (data_start_ == request->write_start) {
      // Write section placed directly after read section
      data_start_ += request->write_size;
    } else if (request->write_start == 0) {
      // Write section placed at the beginning of the buffer
      data_start_ = request->write_size;
    } else {
      ETL_ASSERT(false, ETL_ERROR(0));
    }

  } else if ((request->write_start == 0) || (request->read_start == 0)) {
    // Skip gap at buffer end
    data_start_ = request->write_size + request->read_size;

  } else {
    ETL_ASSERT(false, ETL_ERROR(0));
  }

  // Reset when buffer empty
  if (data_start_ == data_end_) {
    data_start_ = 0;
    data_end_ = 0;
  }

  free_space = getFreeSpace();
  DEBUG_INFO("After free (req: %d) [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             request->request_id, data_start_, data_end_, free_space.end_to_back, free_space.front_to_start);
}

}  // namespace hal::i2c
