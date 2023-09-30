/*
 * I2cService.hpp
 *
 *  Created on: Aug 18, 2023
 *      Author: Alexander L.
 */

#ifndef APP_I2C_SRV_I2CSERVICE_HPP_
#define APP_I2C_SRV_I2CSERVICE_HPP_

#include "app/ctrl/ctrlTypes.hpp"
#include "hal/i2c/I2cMaster.hpp"
#include "i2c.h"
#include "proto_c/i2c.pb.h"

namespace app::i2c_srv {

constexpr uint32_t DefaultClockRate = 100000;

class I2cService {
 public:
  I2cService();
  virtual ~I2cService();

  void init(app::ctrl::RequestSrvCallback request_service_cb);
  void poll();

  int32_t postRequest(const uint8_t* data, size_t len);
  int32_t serviceRequest(uint8_t* data, size_t max_len);

 private:
  int32_t postMasterRequest(i2c_proto_I2cMsg* msg);
  Status_t serviceMasterStatusRequest(i2c_proto_I2cMsg* msg, size_t max_len);

  hal::i2c::I2cMaster i2cMaster0_{ &hi2c2 };
  // hal::uart::I2cSlave i2cSlave0_{ &hi2c1 };

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
};

}  // namespace app::i2c_srv

#endif /* APP_I2C_SRV_I2CSERVICE_HPP_ */
