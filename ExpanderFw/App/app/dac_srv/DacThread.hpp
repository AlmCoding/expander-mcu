/*
 * DacThread.hpp
 *
 *  Created on: Jul 21, 2025
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_DACTHREAD_HPP_
#define APP_CTRL_SRV_DACTHREAD_HPP_

#include "app/dac_srv/DacService.hpp"
#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::dac_srv {

class DacThread {
 public:
  constexpr static driver::tf::TfMsgType ThreadTfMsgType = driver::tf::TfMsgType::DacMsg;
  static void execute(uint32_t thread_input);

 private:
  static void requestService_cb(os::msg::RequestCnt cnt);
  static int32_t postRequest_cb(const uint8_t* data, size_t size);
  static int32_t serviceRequest_cb(uint8_t* data, size_t max_size);

  static app::dac_srv::DacService* dac_service_;
  static os::msg::RequestCnt ongoing_service_cnt_;
  static uint32_t msg_count_;

  DacThread() = delete;
  virtual ~DacThread() = delete;
};

}  // namespace app::dac_srv

#endif /* APP_CTRL_SRV_DACTHREAD_HPP_ */