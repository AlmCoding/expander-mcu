/*
 * CtrlThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_CTRLTHREAD_HPP_
#define APP_CTRL_CTRLTHREAD_HPP_

#include "common.hpp"
#include "os/msg/msg_def.hpp"

namespace app::ctrl {

class CtrlThread {
 public:
  static void execute(uint32_t thread_input);

 private:
  static void processMsg(os::msg::BaseMsg* msg);
  static uint32_t msg_count_;

  CtrlThread() = delete;
  virtual ~CtrlThread() = delete;
};

}  // namespace app::ctrl

#endif /* APP_CTRL_CTRLTHREAD_HPP_ */
