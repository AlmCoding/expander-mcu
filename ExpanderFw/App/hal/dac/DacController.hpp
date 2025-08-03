/*
 * DacController.hpp
 *
 *  Created on: Jul 22, 2025
 *      Author: Alexander L.
 */

#ifndef HAL_DAC_DACCONTROLLER_HPP_
#define HAL_DAC_DACCONTROLLER_HPP_

#include "common.hpp"
#include "hal/dac/DacConfig.hpp"
#include "main.h"
#include "tx_api.h"

namespace hal::dac {

class DacController {
 private:
  constexpr static size_t RequestQueue_MaxItemCnt = 4;
  constexpr static size_t DataBufferSize = 1024 + 1;  // +1 to distinguish between empty and full buffer

 public:
  typedef uint16_t Sample_t;

  enum class DacId {
    Dac0 = 0,
    Dac1,
  };

  enum class DacUpdate {
    No = 0,
    Yes,
    All,
  };

  enum class RequestStatus {
    NotInit = 0,
    NoSpace,
    // Pending, => not needed, request is immediately ongoing
    Ongoing,
    Complete,
    BadRequest,
    InterfaceError,
  };

  typedef struct {
    uint32_t request_id;
    RequestStatus status_code;
    bool run;
    uint32_t sample_count_ch1;
    uint32_t sample_count_ch2;
    uint32_t place_holder_word6 = 0;
    uint32_t place_holder_word7 = 0;
    uint32_t place_holder_word8 = 0;
  } Request;
  static_assert((sizeof(Request) % sizeof(uint32_t)) == 0, "ThreadX queue messages must be a multiple of 4 bytes!");
  static_assert(((sizeof(Request) == 4) || (sizeof(Request) == 8) || (sizeof(Request) == 16) ||
                 (sizeof(Request) == 32) || (sizeof(Request) == 64)),
                "ThreadX queue messages must be of size 4, 8, 16, 32 or 64 bytes!");

  typedef struct {
    uint32_t sequence_number;
    Request request;
    uint16_t queue_space;
    uint16_t buffer_space_ch1;
    uint16_t buffer_space_ch2;
  } StatusInfo;

  DacController(SPI_HandleTypeDef* spi_handle);
  virtual ~DacController() = default;

  Status_t config(DacConfig::Mode mode = DacConfig::Mode::Static);
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, Sample_t* data_ch1, Sample_t* data_ch2, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info);

 private:
  typedef struct {
    size_t data_start;  // Start position of data in data buffer
    size_t data_end;    // End position of data in data buffer
  } BufferState;

  typedef struct {
    size_t end_to_back;     // [data_end_ to buffer end[
    size_t front_to_start;  // [0 to data_start_[
  } Space;

  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);
  Space getFreeSpace(BufferState* buffer_info);
  Status_t allocateBufferSpace(Request* request, Sample_t* data_ch1, Sample_t* data_ch2);
  Status_t allocateBufferSection(BufferState* buffer_info, Sample_t* src_data, Sample_t* dest_data, size_t size);

  Status_t updateSample(bool ch1, bool ch2);
  Status_t writeValue(DacId dac_id, uint16_t value, DacUpdate update);

  SPI_HandleTypeDef* spi_handle_ = nullptr;

  TX_QUEUE request_queue_;
  uint32_t request_queue_buffer_[RequestQueue_MaxItemCnt * (sizeof(Request) / sizeof(uint32_t))];

  DacConfig::Mode mode_ = DacConfig::Mode::Static;

  Sample_t data_buffer_ch1[DataBufferSize];
  Sample_t data_buffer_ch2[DataBufferSize];
  BufferState buffer_state_ch1_ = {};
  BufferState buffer_state_ch2_ = {};

  uint32_t sequence_number_ = 0;
};

}  // namespace hal::dac

#endif  // HAL_DAC_DACCONTROLLER_HPP_
