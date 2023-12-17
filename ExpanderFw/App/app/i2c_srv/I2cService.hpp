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
#include "hal/i2c/I2cSlave.hpp"
#include "i2c.h"
#include "proto_c/i2c.pb.h"

namespace app::i2c_srv {

constexpr uint32_t DefaultClockRate = 400e3;

class I2cService {
 public:
  typedef struct {
    bool service_master0;
    bool service_master1;
    bool service_slave0;
    bool service_slave1;
  } ServiceInfo;

  I2cService();
  virtual ~I2cService();

  void init(app::ctrl::RequestSrvCallback request_service_cb);
  void poll();

  int32_t postRequest(const uint8_t* data, size_t size);
  int32_t serviceRequest(uint8_t* data, size_t max_size);

 private:
  int32_t postMasterRequest(i2c_proto_I2cMsg* msg);
  int32_t postSlaveRequest(i2c_proto_I2cMsg* msg);
  Status_t serviceMasterRequest(hal::i2c::I2cMaster* i2c_master, i2c_proto_I2cMsg* msg, size_t max_size);
  Status_t serviceSlaveRequest(hal::i2c::I2cSlave* i2c_slave, i2c_proto_I2cMsg* msg, size_t max_size);

  hal::i2c::I2cMaster i2c_master0_{ &hi2c1 };
  hal::i2c::I2cMaster i2c_master1_{ &hi2c2 };
  hal::i2c::I2cSlave i2c_slave0_{ &hi2c1 };
  hal::i2c::I2cSlave i2c_slave1_{ &hi2c2 };

  ServiceInfo srv_info_ = {};

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
};

}  // namespace app::i2c_srv

#endif /* APP_I2C_SRV_I2CSERVICE_HPP_ */
