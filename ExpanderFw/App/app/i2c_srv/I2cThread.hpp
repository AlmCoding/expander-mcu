/*
 * I2cThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_I2CTHREAD_HPP_
#define APP_CTRL_SRV_I2CTHREAD_HPP_

#include "app/i2c_srv/I2cService.hpp"
#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::i2c_srv {

class I2cThread {
 public:
  constexpr static driver::tf::TfMsgType ThreadTfMsgType = driver::tf::TfMsgType::I2cMsg;
  static void execute(uint32_t thread_input);

 private:
  static void requestService_cb(os::msg::RequestCnt cnt);
  static int32_t postRequest_cb(const uint8_t* data, size_t size);
  static int32_t serviceRequest_cb(uint8_t* data, size_t max_size);

  static app::i2c_srv::I2cService i2c_service_;
  static bool ongoing_service_;
  static uint32_t msg_count_;

  I2cThread() = delete;
  virtual ~I2cThread() = delete;
};

}  // namespace app::i2c_srv

#endif /* APP_CTRL_SRV_I2CTHREAD_HPP_ */
