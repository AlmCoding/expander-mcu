/*
 * msg_broker.cpp
 *
 *  Created on: May 8, 2023
 *      Author: Alexander L.
 */

#include "os/msg/msg_broker.hpp"
#include "os/queue.hpp"

namespace os::msg {

bool send_msg(MsgQueueId queue, BaseMsg* msg, TickNum timeout) {
  bool success = false;

  TX_QUEUE* qhdl = os::getQueue(queue);
  if (tx_queue_send(qhdl, msg, timeout) == TX_SUCCESS) {
    success = true;
  }

  return success;
}

bool receive_msg(MsgQueueId queue, BaseMsg* msg, TickNum timeout) {
  bool success = false;

  TX_QUEUE* qhdl = os::getQueue(queue);
  if (tx_queue_receive(qhdl, msg, timeout) == TX_SUCCESS) {
    success = true;
  }

  return success;
}

}  // namespace os::msg
