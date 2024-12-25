/*
 * I2cMaster.hpp
 *
 *  Created on: Aug 15, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CMASTER_HPP_
#define HAL_I2C_I2CMASTER_HPP_

#include "common.hpp"
#include "hal/i2c/I2cConfig.hpp"
#include "main.h"
#include "tx_api.h"

#define START_I2_REQUEST_IMMEDIATELY false

namespace hal::i2c {

class I2cMaster {
 private:
  constexpr static size_t RequestQueue_MaxItemCnt = 4;
  constexpr static size_t DataBufferSize = 64 + 1;

 public:
  enum class RequestStatus {
    NotInit = 0,
    NoSpace,
    Pending,
    Ongoing,
    Complete,
    SlaveBusy,
    SlaveNack,
    BadRequest,
    InterfaceError,
  };

  typedef struct {
    uint32_t request_id;
    RequestStatus status_code;
    uint32_t slave_addr;
    uint16_t write_size;
    uint16_t read_size;
    size_t write_start;         // Start position of write data in data buffer
    size_t read_start;          // Start position of read section in data buffer
    uint16_t sequence_id;       // Sequence of read/writes with restart
    uint16_t sequence_idx;      // Index of current request in sequence
    uint32_t nack_byte_number;  // Byte number where the NACK occurred
  } Request;
  static_assert((sizeof(Request) % sizeof(uint32_t)) == 0, "ThreadX queue messages must be a multiple of 4 bytes!");
  static_assert(((sizeof(Request) == 4) || (sizeof(Request) == 8) || (sizeof(Request) == 16) ||
                 (sizeof(Request) == 32) || (sizeof(Request) == 64)),
                "ThreadX queue messages must be of size 4, 8, 16, 32 or 64 bytes!");

  typedef struct {
    uint32_t sequence_number;
    uint32_t request_id;
    RequestStatus status_code;
    uint16_t read_size;
    uint16_t queue_space;
    uint16_t buffer_space1;
    uint16_t buffer_space2;
    uint32_t nack_byte_number;
  } StatusInfo;

  I2cMaster(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cMaster() = default;

  Status_t config();
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint8_t* write_data, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info, uint8_t* read_data, size_t max_size);

 private:
  enum class TransferState {
    Write = 0,
    Read,
  };

  typedef struct {
    size_t end_to_back;     // [data_end_ to buffer end[
    size_t front_to_start;  // [0 to data_start_[
  } Space;

  typedef struct {
    uint16_t seq_id;
    uint16_t max_idx;
    bool ongoing;
  } SequenceState;

  Space getFreeSpace();
  Status_t allocateBufferSpace(Request* request);
  Status_t allocateBufferSection(size_t* front_offset, size_t* back_offset, Space* space, size_t* section_start,
                                 size_t section_size);
  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);
  bool readyForNewStart();
  Status_t startRequest();
  Status_t startWrite();
  Status_t startReadReg();
  Status_t startRead();
  void writeCompleteCb();
  void readCompleteCb();
  void errorCb();
  void complete(RequestStatus status_code);
  void freeBufferSpace(Request* request);

  I2cId i2c_id_;
  I2C_HandleTypeDef* i2c_handle_;
  TX_QUEUE pending_queue_;
  TX_QUEUE complete_queue_;

  uint32_t pending_queue_buffer_[RequestQueue_MaxItemCnt * (sizeof(Request) / sizeof(uint32_t))];
  uint32_t complete_queue_buffer_[RequestQueue_MaxItemCnt * (sizeof(Request) / sizeof(uint32_t))];

  uint8_t data_buffer_[DataBufferSize];
  size_t data_buffer_end_ = DataBufferSize - 1;
  size_t data_start_ = 0;
  size_t data_end_ = 0;

  Request request_ = {};
  bool request_ongoing_ = false;
  TransferState transfer_state_ = TransferState::Write;
  SequenceState sequence_state_ = {};

  uint32_t seqence_number_ = 0;

  friend class I2cIrq;
};

}  // namespace hal::i2c

#endif /* HAL_I2C_I2CMASTER_HPP_ */
