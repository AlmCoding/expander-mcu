/*
 * msg_broker.cpp
 *
 *  Created on: May 8, 2023
 *      Author: Alexander L.
 */

#include "os/msg/msg_broker.hpp"
#include "os/queue.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_MSG_BROKER 1
#if ((DEBUG_ENABLE_MSG_BROKER == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][msg_broker]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][msg_broker]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][msg_broker]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

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
