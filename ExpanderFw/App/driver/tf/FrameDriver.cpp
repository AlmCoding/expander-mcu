/*
 * FrameDriver.cpp
 *
 *  Created on: 24 May 2023
 *      Author: Alexander L.
 */

#include "driver/tf/FrameDriver.hpp"
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_FRAME_DRIVER
#ifdef DEBUG_ENABLE_FRAME_DRIVER
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][FrmDrv]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][FrmDrv]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][FrmDrv]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace driver::tf {

TF_Result typeCallback(TinyFrame* tf, TF_Msg* msg);
TF_Result genericCallback(TinyFrame* tf, TF_Msg* msg);

FrameDriver::FrameDriver() {
  TF_InitStatic(&tf_, TF_SLAVE);
  TF_AddGenericListener(&tf_, genericCallback);
}

Status_t FrameDriver::registerTxCallback(app::usb::UsbMsgType type, TxCallback callback) {
  tx_callbacks_[static_cast<uint8_t>(type)] = callback;
  return Status_t::Ok;
}

void FrameDriver::callTxCallback(app::usb::UsbMsgType type) {
  TF_Msg msg = { 0 };

  msg.type = static_cast<TF_TYPE>(type);
  msg.len = static_cast<TF_LEN>(tx_callbacks_[static_cast<uint8_t>(type)](tx_buffer_, sizeof(tx_buffer_)));
  msg.data = tx_buffer_;

  if (msg.len > 0) {
    DEBUG_INFO("O=> msg: type: %d, len: %d", msg.type, msg.len);
    TF_Send(&tf_, &msg);

  } else {
    DEBUG_ERROR("Msg (type: %d) is empty!", msg.type)
  }
}

Status_t FrameDriver::registerRxCallback(app::usb::UsbMsgType type, RxCallback callback) {
  Status_t status = Status_t::Error;

  if (TF_AddTypeListener(&tf_, static_cast<uint8_t>(type), typeCallback) == true) {
    rx_callbacks_[static_cast<uint8_t>(type)] = callback;
    status = Status_t::Ok;
  }

  return status;
}

void FrameDriver::callRxCallback(app::usb::UsbMsgType type, const uint8_t* data, size_t len) {
  rx_callbacks_[static_cast<uint8_t>(type)](data, len);
}

void FrameDriver::receiveData(const uint8_t* data, size_t len) {
  TF_Accept(&tf_, data, len);
}

void FrameDriver_receiveData(const uint8_t* data, size_t len) {
  util::Stopwatch stopwatch{};
  stopwatch.start();

  driver::tf::FrameDriver& frameDriver = driver::tf::FrameDriver::getInstance();
  frameDriver.receiveData(data, len);

  stopwatch.stop();
  DEBUG_INFO("USB rx: %d us", stopwatch.time());
}

TF_Result typeCallback(TinyFrame* /*tf*/, TF_Msg* msg) {
  DEBUG_INFO("=>I msg: type: %d, len: %d", msg->type, msg->len);

  driver::tf::FrameDriver& frameDriver = driver::tf::FrameDriver::getInstance();
  frameDriver.callRxCallback(static_cast<app::usb::UsbMsgType>(msg->type), msg->data, msg->len);

  return TF_STAY;
}

TF_Result genericCallback(TinyFrame* /*tf*/, TF_Msg* msg) {
  DEBUG_WARN("Unknown msg type: %d", msg->type);
  return TF_STAY;
}

}  // namespace driver::tf
