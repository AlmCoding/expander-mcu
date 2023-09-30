/*
 * msg_broker.hpp
 *
 *  Created on: May 8, 2023
 *      Author: Alexander L.
 */

#ifndef OS_MSG_MSG_BROKER_HPP_
#define OS_MSG_MSG_BROKER_HPP_

#include "common.hpp"
#include "os/msg/msg_def.hpp"
#include "tx_api.h"

namespace os::msg {

bool send_msg(MsgQueueId queue, BaseMsg* msg, TickNum timeout = TX_WAIT_FOREVER);
bool receive_msg(MsgQueueId queue, BaseMsg* msg, TickNum timeout);

}  // namespace os::msg

#endif /* OS_MSG_MSG_BROKER_HPP_ */
