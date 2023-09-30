/*
 * CtrlThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_CTRLTHREAD_HPP_
#define APP_CTRL_SRV_CTRLTHREAD_HPP_

#include "tx_api.h"

namespace app::ctrl_srv {

class CtrlThread {
 public:
  CtrlThread() = delete;
  virtual ~CtrlThread() = delete;

  static void Execute(ULONG thread_input);

 private:
};

}  // namespace app::ctrl_srv

#endif /* APP_CTRL_SRV_CTRLTHREAD_HPP_ */
