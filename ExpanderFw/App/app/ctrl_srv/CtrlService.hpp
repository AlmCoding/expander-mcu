/*
 * CtrlService.hpp
 *
 *  Created on: 17 Feb 2024
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_CTRLSERVICE_HPP_
#define APP_CTRL_SRV_CTRLSERVICE_HPP_

#include "app/ctrl_srv/ctrlTypes.hpp"
#include "proto_c/ctrl.pb.h"

namespace app::ctrl_srv {

class CtrlService {
 public:
  CtrlService() = default;
  virtual ~CtrlService() = default;

  void init(app::ctrl::RequestSrvCallback request_service_cb);
  void poll();

  int32_t postRequest(const uint8_t* data, size_t size);
  int32_t serviceRequest(uint8_t* data, size_t max_size);

 private:
  int32_t postCtrlRequest(ctrl_proto_CtrlMsg* msg);

  app::ctrl::RequestSrvCallback request_service_cb_ = nullptr;
  uint32_t seqence_number_ = 0;

  uint32_t request_id_ = 0;
  bool service_device_info_ = false;
};

}  // namespace app::ctrl_srv

#endif /* APP_CTRL_SRV_CTRLSERVICE_HPP_ */
