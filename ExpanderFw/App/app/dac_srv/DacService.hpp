/*
 * DacService.hpp
 *
 *  Created on: Jul 22, 2025
 *      Author: Alexander L.
 */

#ifndef APP_I2C_SRV_DACSERVICE_HPP_
#define APP_I2C_SRV_DACSERVICE_HPP_

#include "app/ctrl_srv/ctrlTypes.hpp"
#include "hal/dac/DacConfig.hpp"
#include "hal/dac/DacController.hpp"
#include "proto_c/dac.pb.h"
#include "spi.h"

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

  Status_t serviceDataRequest(dac_proto_DacMsg* msg);
  Status_t serviceConfigRequest(dac_proto_DacMsg* msg);

  static dac_proto_DacDataStatusCode convertDataStatus(hal::dac::DacController::RequestStatus status);
  static dac_proto_DacConfigStatusCode convertConfigStatus(hal::dac::DacConfig::RequestStatus status);

  hal::dac::DacController dac_controller_{ &hspi3 };
  hal::dac::DacConfig dac_config_{ &hspi3 };

  ServiceInfo srv_info_ = {};

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
};

}  // namespace app::dac_srv

#endif /* APP_I2C_SRV_DACSERVICE_HPP_ */
