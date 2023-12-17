/*
 * I2cSlave.hpp
 *
 *  Created on: Nov 25, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CSLAVE_HPP_
#define HAL_I2C_I2CSLAVE_HPP_

#include "common.hpp"
#include "main.h"
#include "tx_api.h"

namespace hal {
namespace i2c {

class I2cSlave {
 private:
  constexpr static size_t RequestQueue_MaxItemCnt = 4;
  constexpr static size_t RequestBufferSize = RequestQueue_MaxItemCnt;
  constexpr static size_t DataBufferSize = 512;

 public:
  enum class RequestStatus {
    NotInit = 0,
    NoSpace,
    Pending,
    Complete,
    SlaveBusy,
    BadRequest,
    InterfaceError,
  };

  typedef struct {
    uint32_t request_id;
    uint32_t access_id;
    RequestStatus status_code;
    uint16_t write_size;
    uint16_t read_size;
    size_t write_addr;
    size_t read_addr;
  } Request;

  typedef struct {
    uint32_t sequence_number;
    Request request;
    uint16_t size;
    uint16_t queue_space;
  } StatusInfo;

  I2cSlave(I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cSlave() = default;

  Status_t config();
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint8_t* mem_data, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info, uint8_t* mem_data, size_t max_size);

 private:
  typedef struct {
    Request request;
    bool used;
  } RequestSlot;

  typedef struct {
    RequestSlot* slot;
  } QueueItem;

  RequestSlot* setupRequestSlot(Request* request);
  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);

  I2C_HandleTypeDef* i2c_handle_;
  TX_QUEUE request_queue_;

  uint8_t request_queue_buffer_[RequestQueue_MaxItemCnt * sizeof(QueueItem) * sizeof(ULONG)];
  RequestSlot request_buffer_[RequestBufferSize];
  size_t request_buffer_idx_ = 0;

  uint8_t data_buffer_[DataBufferSize];

  uint32_t seqence_number_ = 0;

  friend class I2cIrq;
};

} /* namespace i2c */
} /* namespace hal */

#endif /* HAL_I2C_I2CSLAVE_HPP_ */
