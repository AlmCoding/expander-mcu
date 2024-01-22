/*
 * I2cSlave.hpp
 *
 *  Created on: Nov 25, 2023
 *      Author: Alexander L.
 */

#ifndef HAL_I2C_I2CSLAVE_HPP_
#define HAL_I2C_I2CSLAVE_HPP_

#include "common.hpp"
#include "hal/i2c/I2cConfig.hpp"
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
    uint16_t size;
    uint16_t queue_space;
  } StatusInfo;

  I2cSlave(I2cId i2c_id, I2C_HandleTypeDef* i2c_handle);
  virtual ~I2cSlave() = default;

  Status_t config();
  Status_t init();
  uint32_t poll();

  Status_t scheduleRequest(Request* request, uint8_t* mem_data, uint32_t seq_num);
  Status_t serviceStatus(StatusInfo* info, uint8_t* mem_data, size_t max_size);

 private:
  Status_t exitScheduleRequest(Request* request, uint32_t seq_num);
  size_t getDataAddress();
  void slaveMatchMasterWriteCb();
  void slaveMatchMasterReadCb();
  void writeCompleteCb();
  void readCompleteCb();
  Status_t notifyAccessRequest(size_t write_size, size_t write_addr, size_t read_size, size_t read_addr);

  I2cId i2c_id_;
  I2C_HandleTypeDef* i2c_handle_;
  TX_QUEUE request_queue_;
  uint32_t request_queue_buffer_[RequestQueue_MaxItemCnt * (sizeof(Request) / sizeof(uint32_t))];

  uint8_t data_buffer_[DataBufferSize];
  uint8_t temp_buffer_[DataBufferSize];

  uint32_t access_id_ = 0;
  uint32_t seqence_number_ = 0;

  friend class I2cIrq;
};

} /* namespace i2c */
} /* namespace hal */

#endif /* HAL_I2C_I2CSLAVE_HPP_ */
