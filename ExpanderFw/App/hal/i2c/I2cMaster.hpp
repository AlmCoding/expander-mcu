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
  constexpr static size_t RequestQueue_MaxItemCnt = 4;
  constexpr static size_t RequestBufferSize = RequestQueue_MaxItemCnt * 2;
  constexpr static size_t DataBufferSize = 64 + 1;

  typedef struct {
    size_t end_to_back;     // [data_end_ to end[
    size_t front_to_start;  // [0 to data_start_[
  } Space;

  typedef struct {
    uint16_t seq_id;
    uint16_t max_idx;
    bool ongoing;
  } SequenceState;

  typedef struct {
    Request request;
    bool used;
  } RequestSlot;

  typedef struct {
    RequestSlot* slot;
  } QueueItem;

  Space getFreeSpace();
  Status_t allocateBufferSpace(Request* request);
  RequestSlot* setupRequestSlot(Request* request);
  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);
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

  uint8_t pending_queue_buffer_[RequestQueue_MaxItemCnt * sizeof(QueueItem) * sizeof(ULONG)];
  uint8_t complete_queue_buffer_[RequestQueue_MaxItemCnt * sizeof(QueueItem) * sizeof(ULONG)];
  RequestSlot request_buffer_[RequestBufferSize];
  size_t request_buffer_idx_ = 0;

  uint8_t data_buffer_[DataBufferSize];
  size_t data_start_ = 0;
  size_t data_end_ = 0;

  RequestSlot* request_slot_ = nullptr;
  Request* request_ = nullptr;
  SequenceState sequence_state_ = {};

  uint32_t seqence_number_ = 0;

  friend class I2cIrq;
};

}  // namespace hal::i2c

#endif /* HAL_I2C_I2CMASTER_HPP_ */
