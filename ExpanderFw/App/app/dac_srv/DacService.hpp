/*
 * DacService.hpp
 *
 *  Created on: Jul 22, 2025
 *      Author: Alexander L.
 */

#ifndef APP_I2C_SRV_DACSERVICE_HPP_
#define APP_I2C_SRV_DACSERVICE_HPP_

#include "app/ctrl_srv/ctrlTypes.hpp"
#include "proto_c/dac.pb.h"

namespace app::dac_srv {

class DacService {
 public:
  typedef struct {
    bool service_data;
    bool service_config;
  } ServiceInfo;

  DacService() = default;
  virtual ~DacService() = default;

  void init(app::ctrl::RequestSrvCallback request_service_cb);
  void poll();

  int32_t postRequest(const uint8_t* data, size_t size);
  int32_t serviceRequest(uint8_t* data, size_t max_size);

 private:
  int32_t postDataRequest(dac_proto_DacMsg* msg);
  int32_t postConfigRequest(dac_proto_DacMsg* msg);

  Status_t serviceDataRequest(int* dac_master, dac_proto_DacMsg* msg);
  Status_t serviceConfigRequest(int* dac_config, dac_proto_DacMsg* msg);

  // static dac_proto_DacDataStatusCode convertDataStatus(hal::i2c::I2cMaster::RequestStatus status);
  // static dac_proto_DacConfigStatusCode convertConfigStatus(hal::i2c::I2cSlave::RequestStatus status);

  ServiceInfo srv_info_ = {};

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
};

}  // namespace app::dac_srv

#endif /* APP_I2C_SRV_DACSERVICE_HPP_ */
