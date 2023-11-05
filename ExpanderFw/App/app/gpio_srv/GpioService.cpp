/*
 * GpioService.cpp
 *
 *  Created on: 1 Jul 2023
 *      Author: Alexander L.
 */

#include "app/gpio_srv/GpioService.hpp"
#include "hal/gpio/GpioIrq.hpp"
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/gpio.pb.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_GPIO_SERVICE
#ifdef DEBUG_ENABLE_GPIO_SERVICE
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][GpioSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][GpioSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][GpioSrv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::gpio_srv {

GpioService::GpioService() {}

GpioService::~GpioService() {}

void GpioService::init(app::ctrl::RequestSrvCallback request_service_cb) {
  gpio0_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio1_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio2_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio3_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio4_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio5_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio6_.config(hal::gpio::Gpio::Mode::InputPullDown);
  gpio7_.config(hal::gpio::Gpio::Mode::InputPullDown);

  request_service_cb_ = request_service_cb;
  hal::gpio::GpioIrq::getInstance().registerRequestSrvCallback(request_service_cb);
}

int32_t GpioService::postRequest(const uint8_t* data, size_t len) {
  int32_t status;

  /* Allocate space for the decoded message. */
  gpio_proto_GpioMsg gpio_msg = gpio_proto_GpioMsg_init_zero;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(data, len);

  /* Now we are ready to decode the message. */
  if (pb_decode(&stream, gpio_proto_GpioMsg_fields, &gpio_msg) == false) {
    DEBUG_ERROR("ProtoBuf decode [FAILED]");
    return -1;
  }

  if (gpio_msg.which_msg == gpio_proto_GpioMsg_data_tag) {
    gpio0_.writePin(gpio_msg.msg.data.gpio0);
    gpio1_.writePin(gpio_msg.msg.data.gpio1);
    gpio2_.writePin(gpio_msg.msg.data.gpio2);
    gpio3_.writePin(gpio_msg.msg.data.gpio3);
    gpio4_.writePin(gpio_msg.msg.data.gpio4);
    gpio5_.writePin(gpio_msg.msg.data.gpio5);
    gpio6_.writePin(gpio_msg.msg.data.gpio6);
    gpio7_.writePin(gpio_msg.msg.data.gpio7);
    request_service_cb_(1);
    status = 0;

  } else if (gpio_msg.which_msg == gpio_proto_GpioMsg_cfg_tag) {
    gpio0_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio0));
    gpio1_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio1));
    gpio2_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio2));
    gpio3_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio3));
    gpio4_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio4));
    gpio5_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio5));
    gpio6_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio6));
    gpio7_.config(static_cast<hal::gpio::Gpio::Mode>(gpio_msg.msg.cfg.gpio7));
    request_service_cb_(1);
    status = 0;

  } else {
    DEBUG_ERROR("Unknown ProtoBuf msg!");
    status = -1;
  }

  seqence_number_ = gpio_msg.sequence_number;
  return status;
}

int32_t GpioService::serviceRequest(uint8_t* data, size_t max_len) {
  /* Allocate space for the decoded message. */
  gpio_proto_GpioMsg gpio_msg = gpio_proto_GpioMsg_init_zero;
  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_len);

  gpio_msg.sequence_number = seqence_number_;
  gpio_msg.which_msg = gpio_proto_GpioMsg_data_tag;

  gpio_msg.msg.data.gpio0 = gpio0_.readPin();
  gpio_msg.msg.data.gpio1 = gpio1_.readPin();
  gpio_msg.msg.data.gpio2 = gpio2_.readPin();
  gpio_msg.msg.data.gpio3 = gpio3_.readPin();
  gpio_msg.msg.data.gpio4 = gpio4_.readPin();
  gpio_msg.msg.data.gpio5 = gpio5_.readPin();
  gpio_msg.msg.data.gpio6 = gpio6_.readPin();
  gpio_msg.msg.data.gpio7 = gpio7_.readPin();

  DEBUG_INFO("Srv data (seq: %d)", gpio_msg.sequence_number);

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, gpio_proto_GpioMsg_fields, &gpio_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  return stream.bytes_written;
}

}  // namespace app::gpio_srv
