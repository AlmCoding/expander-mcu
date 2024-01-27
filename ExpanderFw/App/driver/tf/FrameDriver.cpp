/*
 * FrameDriver.cpp
 *
 *  Created on: 24 May 2023
 *      Author: Alexander L.
 */

#include "driver/tf/FrameDriver.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "util/Stopwatch.hpp"
#include "util/debug.hpp"

#define DEBUG_ENABLE_FRAME_DRIVER 1
#if ((DEBUG_ENABLE_FRAME_DRIVER == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][FrmDrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][FrmDrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][FrmDrv]: " f "\n", ##__VA_ARGS__)
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

Status_t FrameDriver::registerTxCallback(TfMsgType type, TxCallback callback) {
  auto callback_slot = &tx_callbacks_[static_cast<uint8_t>(type)];
  ETL_ASSERT(*callback_slot == nullptr, ETL_ERROR(0));
  *callback_slot = callback;
  return Status_t::Ok;
}

void FrameDriver::callTxCallback(TfMsgType type, uint8_t* buffer, size_t max_size) {
  TF_Msg msg = {};

  msg.type = static_cast<TF_TYPE>(type);
  msg.len = static_cast<TF_LEN>(tx_callbacks_[static_cast<uint8_t>(type)](buffer, max_size));
  msg.data = buffer;

  if (msg.len > 0) {
    DEBUG_INFO("O=> msg (type: %d, size: %d)", msg.type, msg.len);
    TF_Send(&tf_, &msg);

  } else {
    DEBUG_ERROR("Msg (type: %d) is empty!", msg.type);
  }
}

Status_t FrameDriver::registerRxCallback(TfMsgType type, RxCallback callback) {
  Status_t status = Status_t::Error;

  if (TF_AddTypeListener(&tf_, static_cast<uint8_t>(type), typeCallback) == true) {
    rx_callbacks_[static_cast<uint8_t>(type)] = callback;
    status = Status_t::Ok;
  }

  return status;
}

void FrameDriver::callRxCallback(TfMsgType type, const uint8_t* data, size_t size) {
  auto callback = rx_callbacks_[static_cast<uint8_t>(type)];
  ETL_ASSERT(callback != nullptr, ETL_ERROR(0));
  callback(data, size);
}

void FrameDriver::receiveData(const uint8_t* data, size_t size) {
  TF_Accept(&tf_, data, size);
}

void FrameDriver_receiveData(const uint8_t* data, size_t size) {
  util::Stopwatch stopwatch{};
  stopwatch.start();

  auto& tf_driver = driver::tf::FrameDriver::getInstance();
  tf_driver.receiveData(data, size);

  stopwatch.stop();
  DEBUG_INFO("USB rx: %d us", stopwatch.time());
}

TF_Result typeCallback(TinyFrame* /*tf*/, TF_Msg* msg) {
  DEBUG_INFO("=>I msg (type: %d, size: %d)", msg->type, msg->len);

  auto& tf_driver = driver::tf::FrameDriver::getInstance();
  tf_driver.callRxCallback(static_cast<TfMsgType>(msg->type), msg->data, msg->len);

  return TF_STAY;
}

TF_Result genericCallback(TinyFrame* /*tf*/, TF_Msg* msg) {
  DEBUG_WARN("Unknown msg type: %d", msg->type);
  return TF_STAY;
}

}  // namespace driver::tf
