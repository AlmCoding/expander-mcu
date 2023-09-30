/*
 * I2cMaster.hpp
 *
 *  Created on: Aug 15, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CMASTER_HPP_
#define HAL_I2C_I2CMASTER_HPP_

#include "common.hpp"
#include "main.h"
#include "tx_api.h"

#define START_I2_REQUEST_IMMEDIATELY false

namespace hal::i2c {

class I2cMaster {
 private:
  constexpr static size_t RequestQueue_MsgCnt = 8;
  constexpr static size_t DataBufferSize = 64 + 1;

  typedef struct {
    size_t space1;  // Starts at data_end_
    size_t space2;  // Starts at 0
  } Space;

  typedef struct {
    bool ongoing;
    uint16_t id;
    uint16_t max_idx;
  } SequenceState;

 public:
  enum class RequestStatus {
    NotInit = 0,
    NoSpace,
    Pending,
    Ongoing,
    Complete,
    SlaveBusy,
    InterfaceError,
  };

  typedef struct {
    RequestStatus status_code;
    uint16_t request_id;
    uint16_t slave_addr;
    uint16_t write_size;
    uint16_t read_size;
    size_t write_start;
    size_t read_start;
    uint16_t sequence_id;
    uint16_t sequence_idx;
  } Request;

  typedef struct {
    uint32_t sequence_number;
    RequestStatus status_code;
    uint16_t request_id;
    uint16_t read_size;
    uint16_t queue_space;
    uint16_t buffer_space1;
    uint16_t buffer_space2;
  } StatusInfo;

  I2cMaster(I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cMaster() = default;

  Status_t config();
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint8_t* write_data, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info, uint8_t* read_data, size_t max_size);

 private:
  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);
  Space getFreeSpace();
  Status_t allocateBufferSpace(Request* request);
  void freeBufferSpace(Request* request);
  Status_t startRequest();
  Status_t startWrite();
  Status_t startReadReg();
  Status_t startRead();
  void writeCompleteCb();
  void readCompleteCb();
  void complete();

  I2C_HandleTypeDef* i2c_handle_;

  TX_QUEUE pending_queue_;
  TX_QUEUE complete_queue_;
  uint8_t pending_queue_buffer_[RequestQueue_MsgCnt * sizeof(Request) * sizeof(ULONG)];
  uint8_t complete_queue_buffer_[RequestQueue_MsgCnt * sizeof(Request) * sizeof(ULONG)];

  uint8_t data_buffer_[DataBufferSize];
  size_t data_start_ = 0;
  size_t data_end_ = 0;

  bool request_complete_ = true;
  Request request_;
  SequenceState sequence_ = {};

  uint32_t seqence_number_ = 0;

  friend class I2cIrq;
};

}  // namespace hal::i2c

#endif /* HAL_I2C_I2CMASTER_HPP_ */
