/*
 * DacController.cpp
 *
 *  Created on: Jul 22, 2025
 *      Author: Alexander L.
 */

#include "hal/dac/DacController.hpp"
#include "enum/magic_enum.hpp"
#include "etl/algorithm.h"      // etl::max
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "hal/dac/DACxx6x.hpp"
#include "os/msg/msg_broker.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_DAC_CONTROLLER 1
#if ((DEBUG_ENABLE_DAC_CONTROLLER == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][DacCtrl]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][DacCtrl]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][DacCtrl]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace hal::dac {

DacController::DacController(SPI_HandleTypeDef* spi_handle) : spi_handle_{ spi_handle } {
  uint32_t sts = TX_SUCCESS;
  sts = tx_queue_create(&request_queue_,                       //
                        const_cast<char*>("DacCtrlRequestQ"),  //
                        sizeof(Request) / sizeof(uint32_t),    //
                        request_queue_buffer_,                 //
                        sizeof(request_queue_buffer_));
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
}

Status_t DacController::config(bool config_ch0, DacConfig::Mode mode_ch0, bool config_ch1, DacConfig::Mode mode_ch1) {
  Status_t status = init(config_ch0, config_ch1);

  if (config_ch0 == false && config_ch1 == false) {
    DEBUG_ERROR("No channel to configure!");
    return Status_t::Error;
  }

  if (config_ch0 == true) {
    mode_ch0_ = mode_ch0;
    DEBUG_INFO("DacController (ch0) mode: %s", magic_enum::enum_name(mode_ch0).cbegin());
  }

  if (config_ch1 == true) {
    mode_ch1_ = mode_ch1;
    DEBUG_INFO("DacController (ch1) mode: %s", magic_enum::enum_name(mode_ch1).cbegin());
  }

  if (status == Status_t::Ok) {
    DEBUG_INFO("Config DacController [OK]");
  } else {
    DEBUG_ERROR("Config DacController [FAILED]");
    status = Status_t::Error;
  }

  ETL_ASSERT(status == Status_t::Ok, ETL_ERROR(0));
  return status;
}

Status_t DacController::init(bool init_ch0, bool init_ch1) {
  Status_t status = Status_t::Ok;

  if (init_ch0 == true) {
    std::memset(data_buffer_ch0_, 0, sizeof(data_buffer_ch0_));
    buffer_state_ch0_ = {};
    run_ch0_ = false;
  }

  if (init_ch1 == true) {
    std::memset(data_buffer_ch1_, 0, sizeof(data_buffer_ch1_));
    buffer_state_ch1_ = {};
    run_ch1_ = false;
  }

  if (init_ch0 == true && init_ch1 == true) {
    tx_queue_flush(&request_queue_);
  }

  sequence_number_ = 0;
  return status;
}

uint32_t DacController::poll() {
  uint32_t service_requests = 0;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  if (free_slots < RequestQueue_MaxItemCnt) {
    service_requests = 1;
  }

  return service_requests;
}

Status_t DacController::scheduleRequest(Request* request, Sample_t* data_ch0, Sample_t* data_ch1, uint32_t seq_num) {
  uint32_t sts = TX_SUCCESS;

  // Check for invalid sample counts
  if ((request->sample_count_ch0 == 0 && request->sample_count_ch1 == 0) ||
      (request->sample_count_ch0 > DataBufferSize) || (request->sample_count_ch1 > DataBufferSize)) {
    DEBUG_ERROR("Invalid request (req: %d)", request->request_id);
    request->status_code = RequestStatus::BadRequest;
    return exitScheduleRequest(request, seq_num);
  }

  uint32_t free_slots = 0;
  sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  // Check request queue space
  if (free_slots == 0) {
    DEBUG_ERROR("Queue overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  request->status_code = RequestStatus::Ongoing;

  if (allocateBufferSpace(request, data_ch0, data_ch1) != Status_t::Ok) {
    DEBUG_ERROR("Buffer overflow (req: %d)", request->request_id);
    request->status_code = RequestStatus::NoSpace;
    return exitScheduleRequest(request, seq_num);
  }

  DacUpdate dac_update;
  if (request->run_ch0 == true) {
    run_ch0_ = true;
    dac_update = (request->run_ch1 == true) ? DacUpdate::No : DacUpdate::Yes;
    updateSample(DacId::Dac0, dac_update);
  }

  if (request->run_ch1 == true) {
    run_ch1_ = true;
    dac_update = (request->run_ch0 == true) ? DacUpdate::All : DacUpdate::Yes;
    updateSample(DacId::Dac1, dac_update);
  }

  // Set request status to complete
  request->status_code = RequestStatus::Complete;

  // Add request to request_queue
  sts = tx_queue_send(&request_queue_, request, TX_NO_WAIT);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));

  return exitScheduleRequest(request, seq_num);
}

Status_t DacController::exitScheduleRequest(Request* request, uint32_t seq_num) {
  Status_t status = Status_t::Ok;

  if (request->status_code == RequestStatus::Complete) {
    DEBUG_INFO("Sched. request (req: %d) [OK]", request->request_id);

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
  sequence_number_ = seq_num;

  // Trigger DAC task
  os::msg::BaseMsg msg = {};
  msg.id = os::msg::MsgId::TriggerThread;
  os::msg::send_msg(os::msg::MsgQueueId::DacThreadQueue, &msg);

  return status;
}

DacController::Space DacController::getFreeSpace(BufferState* buffer_state) {
  Space space = {};

  if (buffer_state->data_start <= buffer_state->data_end) {
    space.end_to_back = DataBufferSize - buffer_state->data_end;
    space.front_to_start = buffer_state->data_start;

  } else {
    space.end_to_back = buffer_state->data_start - buffer_state->data_end;
    space.front_to_start = 0;
  }

  if (space.front_to_start > 0) {
    space.front_to_start -= 1;  // Reserve space for empty/full buffer
  } else if (space.end_to_back > 0) {
    space.end_to_back -= 1;  // Reserve space for empty/full buffer
  }

  DEBUG_INFO("Space [ds: %d, de: %d, sp1: %d, sp2: %d]",  //
             buffer_state->data_start, buffer_state->data_end, space.end_to_back, space.front_to_start);
  // Ensure that the total free space does not exceed the buffer size -1 (to distinguish between empty and full buffer)
  ETL_ASSERT((space.end_to_back + space.front_to_start) <= (DataBufferSize - 1), ETL_ERROR(0));
  return space;
}

Status_t DacController::allocateBufferSpace(Request* request, Sample_t* data_ch0, Sample_t* data_ch1) {
  Status_t status = Status_t::Ok;

  if (request->sample_count_ch0 > 0) {
    status = allocateBufferSection(&buffer_state_ch0_, data_ch0, data_buffer_ch0_, request->sample_count_ch0);
  }

  if (request->sample_count_ch1 > 0) {
    status = allocateBufferSection(&buffer_state_ch1_, data_ch1, data_buffer_ch1_, request->sample_count_ch1);
  }

  if (status != Status_t::Ok) {
    DEBUG_WARN("Allocate space (req: %d) [FAILED]", request->request_id);
  }

  return status;
}

Status_t DacController::allocateBufferSection(BufferState* buffer_state, Sample_t* src_data, Sample_t* dest_data,
                                              size_t size) {
  Status_t status = Status_t::Ok;
  Space free_space = getFreeSpace(buffer_state);

  if ((free_space.end_to_back + free_space.front_to_start) < size) {
    DEBUG_ERROR("Not enough free space (size: %d)", size);
    return Status_t::Error;
  }

  size_t size1 = etl::min(free_space.end_to_back, size);
  size_t size2 = size - size1;

  // Place data into space1 (end_to_back)
  std::memcpy(dest_data + buffer_state->data_end, src_data, size1 * sizeof(Sample_t));
  buffer_state->data_end += size1;

  if (size2 > 0) {
    // Place remaining data into space2 (front_to_start)
    std::memcpy(dest_data, src_data + size1, size2 * sizeof(Sample_t));
    buffer_state->data_end = size2;
  }

  return status;
}

Status_t DacController::serviceStatus(StatusInfo* info) {
  Status_t status = Status_t::Ok;

  Request* request = &info->request;
  if (tx_queue_receive(&request_queue_, request, TX_NO_WAIT) != TX_SUCCESS) {
    DEBUG_ERROR("No request to service!");
    return Status_t::Error;
  }

  info->sequence_number = sequence_number_;

  uint32_t free_slots = 0;
  uint32_t sts = tx_queue_info_get(&request_queue_, nullptr, nullptr, &free_slots, nullptr, nullptr, nullptr);
  ETL_ASSERT(sts == TX_SUCCESS, ETL_ERROR(0));
  info->queue_space = static_cast<uint16_t>(free_slots);

  Space free_space_ch0 = getFreeSpace(&buffer_state_ch0_);
  Space free_space_ch1 = getFreeSpace(&buffer_state_ch1_);
  info->buffer_space_ch0 = static_cast<uint16_t>(free_space_ch0.end_to_back + free_space_ch0.front_to_start);
  info->buffer_space_ch1 = static_cast<uint16_t>(free_space_ch1.end_to_back + free_space_ch1.front_to_start);
  return status;
}

Status_t DacController::updateSample(DacId dac_id, DacUpdate dac_update) {
  Status_t status = Status_t::Ok;

  DacConfig::Mode mode = (dac_id == DacId::Dac0) ? mode_ch0_ : mode_ch1_;
  Sample_t* data_buffer = (dac_id == DacId::Dac0) ? data_buffer_ch0_ : data_buffer_ch1_;
  BufferState* buffer_state = (dac_id == DacId::Dac0) ? &buffer_state_ch0_ : &buffer_state_ch1_;
  // bool run = (dac_id == DacId::Dac0) ? run_ch0_ : run_ch1_;

  switch (mode) {
    case DacConfig::Mode::Static: {
      DEBUG_INFO("Update sample (dac: %s, mode: Static, update: %s)",  //
                 magic_enum::enum_name(dac_id).cbegin(), magic_enum::enum_name(dac_update).cbegin());
      Sample_t sample = data_buffer[buffer_state->data_start];
      status = writeValue(dac_id, sample, dac_update);
      buffer_state->data_start += 1;
      if (buffer_state->data_start >= DataBufferSize) {
        buffer_state->data_start = 0;  // Wrap around
      }
      break;
    }
    case DacConfig::Mode::Periodic: {
      break;
    }
    case DacConfig::Mode::Streaming: {
      break;
    }
    default: {
      ETL_ASSERT(false, ETL_ERROR(0));
    }
  }

  return status;
}

Status_t DacController::writeValue(DacId dac_id, uint16_t value, DacUpdate update) {
  HAL_StatusTypeDef hal_status;
  uint8_t data[3];

  switch (update) {
    case DacUpdate::No:
      data[0] = DAC_CMD_WRITE_REG_N;
      break;
    case DacUpdate::Yes:
      data[0] = DAC_CMD_WRITE_REG_N_UPDATE_REG_N;
      break;
    case DacUpdate::All:
      data[0] = DAC_CMD_WRITE_REG_N_UPDATE_ALL;
      break;
    default:
      ETL_ASSERT(false, ETL_ERROR(0));
  }

  if (dac_id == DacId::Dac0) {
    data[0] |= DAC_ADDR_DAC_A;
  } else {
    data[0] |= DAC_ADDR_DAC_B;
  }

  data[1] = static_cast<uint8_t>(value >> 8);  // MSB
  data[2] = static_cast<uint8_t>(value & 0xFF);

  HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_RESET);
  hal_status = HAL_SPI_Transmit(spi_handle_, data, sizeof(data), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_SET);

  if (hal_status != HAL_OK) {
    DEBUG_ERROR("Write value failed (HAL error: %d)", hal_status);
    return Status_t::Error;
  }
  return Status_t::Ok;
}

}  // namespace hal::dac
