/*
 * I2cThread.hpp
 *
 *  Created on: Sep 28, 2023
 *      Author: Alexander L.
 */

#ifndef APP_CTRL_SRV_I2CTHREAD_HPP_
#define APP_CTRL_SRV_I2CTHREAD_HPP_

#include "tx_api.h"

namespace app::i2c_srv {

class I2cThread {
 public:
  I2cThread() = delete;
  virtual ~I2cThread() = delete;

  static void Execute(ULONG thread_input);

 private:
};

}  // namespace app::i2c_srv

#endif /* APP_CTRL_SRV_I2CTHREAD_HPP_ */
