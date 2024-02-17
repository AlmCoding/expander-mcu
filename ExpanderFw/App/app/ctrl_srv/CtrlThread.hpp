/*
 * CtrlThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_CTRLTHREAD_HPP_
#define APP_CTRL_SRV_CTRLTHREAD_HPP_

#include "app/ctrl_srv/CtrlService.hpp"
#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::ctrl_srv {

class CtrlThread {
 public:
  constexpr static driver::tf::TfMsgType ThreadTfMsgType = driver::tf::TfMsgType::CtrlMsg;
  static void execute(uint32_t thread_input);

 private:
  static void requestService_cb(os::msg::RequestCnt cnt);
  static int32_t postRequest_cb(const uint8_t* data, size_t size);
  static int32_t serviceRequest_cb(uint8_t* data, size_t max_size);

  static app::ctrl_srv::CtrlService* ctrl_service_;
  static os::msg::RequestCnt ongoing_service_cnt_;
  static uint32_t msg_count_;

  CtrlThread() = delete;
  virtual ~CtrlThread() = delete;
};

}  // namespace app::ctrl_srv

#endif /* APP_CTRL_SRV_CTRLTHREAD_HPP_ */
