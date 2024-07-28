/*
 * I2cService.cpp
 *
 *  Created on: Aug 18, 2023
 *      Author: Alexander L.
 */

#include "app/i2c_srv/I2cService.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "os/msg/msg_broker.hpp"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_c/i2c.pb.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_UART_SERVICE 1
#if ((DEBUG_ENABLE_UART_SERVICE == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][I2cSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][I2cSrv]: " f "\n", ##__VA_ARGS__)
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][I2cSrv]: " f "\n", ##__VA_ARGS__)
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

namespace app::i2c_srv {

void I2cService::init(app::ctrl::RequestSrvCallback request_service_cb) {
  i2c_slave0_.config(hal::i2c::I2cSlave::MemAddrWidth::TwoByte);
  i2c_slave1_.config(hal::i2c::I2cSlave::MemAddrWidth::TwoByte);

  i2c_master0_.config();
  i2c_master1_.config();

  request_service_cb_ = request_service_cb;
}

void I2cService::poll() {
  uint32_t request_cnt = 0;

  if (i2c_master1_.poll() > 0) {
    srv_info_.service_master1 = true;
    request_cnt++;
  }

  if (i2c_master0_.poll() > 0) {
    srv_info_.service_master0 = true;
    request_cnt++;
  }

  if (i2c_slave1_.poll() > 0) {
    srv_info_.service_slave1 = true;
    request_cnt++;
  }

  if (i2c_slave0_.poll() > 0) {
    srv_info_.service_slave0 = true;
    request_cnt++;
  }

  if (i2c_config1_.poll() > 0) {
    srv_info_.service_config1 = true;
    request_cnt++;
  }

  if (i2c_config0_.poll() > 0) {
    srv_info_.service_config0 = true;
    request_cnt++;
  }

  if (request_cnt > 0) {
    request_service_cb_(request_cnt);
  }
}

int32_t I2cService::postRequest(const uint8_t* data, size_t size) {
  int32_t status = -1;

  /* Allocate space for the decoded message. */
  i2c_proto_I2cMsg i2c_msg = i2c_proto_I2cMsg_init_zero;
  /* Create a stream that reads from the buffer. */
  pb_istream_t stream = pb_istream_from_buffer(data, size);

  /* Now we are ready to decode the message. */
  if (pb_decode(&stream, i2c_proto_I2cMsg_fields, &i2c_msg) == false) {
    DEBUG_ERROR("ProtoBuf decode [FAILED]");
    return -1;
  }

  if (i2c_msg.which_msg == i2c_proto_I2cMsg_master_request_tag) {
    status = postMasterRequest(&i2c_msg);

  } else if (i2c_msg.which_msg == i2c_proto_I2cMsg_slave_request_tag) {
    status = postSlaveRequest(&i2c_msg);

  } else if (i2c_msg.which_msg == i2c_proto_I2cMsg_config_request_tag) {
    status = postConfigRequest(&i2c_msg);

  } else {
    DEBUG_ERROR("Invalid request message!");
  }

  return status;
}

int32_t I2cService::postMasterRequest(i2c_proto_I2cMsg* msg) {
  int32_t status = -1;
  hal::i2c::I2cMaster* i2c_master = nullptr;
  hal::i2c::I2cMaster::Request request = {};

  request.request_id = msg->msg.master_request.request_id;
  request.status_code = hal::i2c::I2cMaster::RequestStatus::NotInit;
  request.slave_addr = static_cast<uint16_t>(msg->msg.master_request.slave_addr);
  request.write_size = static_cast<uint16_t>(msg->msg.master_request.write_data.size);
  request.read_size = static_cast<uint16_t>(msg->msg.master_request.read_size);
  request.sequence_id = static_cast<uint16_t>(msg->msg.master_request.sequence_id);
  request.sequence_idx = static_cast<uint16_t>(msg->msg.master_request.sequence_idx);

  if (msg->i2c_id == i2c_proto_I2cId::i2c_proto_I2cId_I2C0) {
    DEBUG_INFO("Post master(0) request (req: %d)", request.request_id);
    i2c_master = &i2c_master0_;
  } else {
    DEBUG_INFO("Post master(1) request (req: %d)", request.request_id);
    i2c_master = &i2c_master1_;
  }

  if (i2c_master->scheduleRequest(&request, static_cast<uint8_t*>(msg->msg.master_request.write_data.bytes),
                                  msg->sequence_number) == Status_t::Ok) {
    status = 0;
  }

  return status;
}

int32_t I2cService::postSlaveRequest(i2c_proto_I2cMsg* msg) {
  int32_t status = -1;
  hal::i2c::I2cSlave* i2c_slave = nullptr;
  hal::i2c::I2cSlave::Request request = {};

  request.request_id = msg->msg.slave_request.request_id;
  request.access_id = 0;
  request.status_code = hal::i2c::I2cSlave::RequestStatus::NotInit;
  request.write_size = static_cast<uint16_t>(msg->msg.slave_request.write_data.size);
  request.read_size = static_cast<uint16_t>(msg->msg.slave_request.read_size);
  request.write_addr = msg->msg.slave_request.write_addr;
  request.read_addr = msg->msg.slave_request.read_addr;

  if (msg->i2c_id == i2c_proto_I2cId::i2c_proto_I2cId_I2C0) {
    DEBUG_INFO("Post slave(0) request (req: %d)", request.request_id);
    i2c_slave = &i2c_slave0_;
  } else {
    DEBUG_INFO("Post slave(1) request (req: %d)", request.request_id);
    i2c_slave = &i2c_slave1_;
  }

  if (i2c_slave->scheduleRequest(&request, static_cast<uint8_t*>(msg->msg.slave_request.write_data.bytes),
                                 msg->sequence_number) == Status_t::Ok) {
    status = 0;
  }

  return status;
}

int32_t I2cService::postConfigRequest(i2c_proto_I2cMsg* msg) {
  int32_t status = -1;
  hal::i2c::I2cConfig* i2c_config = nullptr;
  hal::i2c::I2cSlave* i2c_slave = nullptr;
  hal::i2c::I2cMaster* i2c_master = nullptr;
  hal::i2c::I2cConfig::Request request = {};

  request.request_id = msg->msg.config_request.request_id;
  request.clock_freq = msg->msg.config_request.clock_freq;
  request.slave_addr = msg->msg.config_request.slave_addr;
  request.pullups_enabled = msg->msg.config_request.pullups_enabled;

  if (msg->msg.config_request.slave_addr_width == i2c_proto_AddressWidth::i2c_proto_AddressWidth_Bits7) {
    request.slave_addr_width = hal::i2c::I2cConfig::SlaveAddrWidth::SevenBit;
  } else if (msg->msg.config_request.slave_addr_width == i2c_proto_AddressWidth::i2c_proto_AddressWidth_Bits10) {
    request.slave_addr_width = hal::i2c::I2cConfig::SlaveAddrWidth::TenBit;
  } else {
    DEBUG_ERROR("Invalid SlaveAddrWidth configuration!");
    return -1;
  }

  hal::i2c::I2cSlave::MemAddrWidth mem_addr_width = hal::i2c::I2cSlave::MemAddrWidth::OneByte;
  if (msg->msg.config_request.mem_addr_width == i2c_proto_AddressWidth::i2c_proto_AddressWidth_Bits8) {
    mem_addr_width = hal::i2c::I2cSlave::MemAddrWidth::OneByte;
  } else if (msg->msg.config_request.mem_addr_width == i2c_proto_AddressWidth::i2c_proto_AddressWidth_Bits16) {
    mem_addr_width = hal::i2c::I2cSlave::MemAddrWidth::TwoByte;
  } else {
    DEBUG_ERROR("Invalid MemAddrWidth configuration!");
    return -1;
  }

  if (msg->i2c_id == i2c_proto_I2cId::i2c_proto_I2cId_I2C0) {
    DEBUG_INFO("Post config(0) request (req: %d)", request.request_id);
    i2c_config = &i2c_config0_;
    i2c_slave = &i2c_slave0_;
    i2c_master = &i2c_master0_;
  } else {
    DEBUG_INFO("Post config(1) request (req: %d)", request.request_id);
    i2c_config = &i2c_config1_;
    i2c_slave = &i2c_slave1_;
    i2c_master = &i2c_master1_;
  }

  if (i2c_config->scheduleRequest(&request, msg->sequence_number) == Status_t::Ok) {
    DEBUG_INFO("Config request [OK]");
    i2c_slave->config(mem_addr_width);
    i2c_master->config();
    status = 0;
  }

  return status;
}

int32_t I2cService::serviceRequest(uint8_t* data, size_t max_size) {
  Status_t sts = Status_t::Error;

  /* Allocate space for the decoded message. */
  i2c_proto_I2cMsg i2c_msg = i2c_proto_I2cMsg_init_zero;

  if (srv_info_.service_slave0 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C0;
    sts = serviceSlaveRequest(&i2c_slave0_, &i2c_msg);
    srv_info_.service_slave0 = false;

  } else if (srv_info_.service_slave1 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C1;
    sts = serviceSlaveRequest(&i2c_slave1_, &i2c_msg);
    srv_info_.service_slave1 = false;

  } else if (srv_info_.service_master0 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C0;
    sts = serviceMasterRequest(&i2c_master0_, &i2c_msg);
    srv_info_.service_master0 = false;

  } else if (srv_info_.service_master1 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C1;
    sts = serviceMasterRequest(&i2c_master1_, &i2c_msg);
    srv_info_.service_master1 = false;

  } else if (srv_info_.service_config0 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C0;
    sts = serviceConfigRequest(&i2c_config0_, &i2c_msg);
    srv_info_.service_config0 = false;

  } else if (srv_info_.service_config1 == true) {
    i2c_msg.i2c_id = i2c_proto_I2cId::i2c_proto_I2cId_I2C1;
    sts = serviceConfigRequest(&i2c_config1_, &i2c_msg);
    srv_info_.service_config1 = false;

  } else {
    DEBUG_ERROR("Nothing to service (return 0)");
    return 0;
  }

  if (sts == Status_t::Error) {
    return -1;
  }

  /* Create a stream that will write to our buffer. */
  pb_ostream_t stream = pb_ostream_from_buffer(data, max_size);
  ETL_ASSERT(max_size >= sizeof(i2c_proto_I2cMsg), ETL_ERROR(0));

  /* Now we are ready to encode the message! */
  if (pb_encode(&stream, i2c_proto_I2cMsg_fields, &i2c_msg) == false) {
    DEBUG_ERROR("ProtoBuf encode [FAILED]");
    return -1;
  }

  return stream.bytes_written;
}

Status_t I2cService::serviceMasterRequest(hal::i2c::I2cMaster* i2c_master, i2c_proto_I2cMsg* msg) {
  size_t master_id;
  hal::i2c::I2cMaster::StatusInfo info;

  if (i2c_master == &i2c_master0_) {
    master_id = 0;
  } else {
    master_id = 1;
  }

  if (i2c_master->serviceStatus(&info, msg->msg.master_status.read_data.bytes,
                                sizeof(msg->msg.master_status.read_data.bytes)) != Status_t::Ok) {
    DEBUG_ERROR("Srv master(%d) status (request id: UNKNOWN) [FAILED]", master_id);
    return Status_t::Error;
  }

  msg->sequence_number = info.sequence_number;
  msg->which_msg = i2c_proto_I2cMsg_master_status_tag;

  msg->msg.master_status.request_id = info.request_id;
  msg->msg.master_status.status_code = convertMasterStatus(info.status_code);
  msg->msg.master_status.read_data.size = info.read_size;
  msg->msg.master_status.queue_space = info.queue_space;
  msg->msg.master_status.buffer_space1 = info.buffer_space1;
  msg->msg.master_status.buffer_space2 = info.buffer_space2;

  DEBUG_INFO("Srv master(%d) status (request id: %d) [OK]", master_id, info.request_id);
  return Status_t::Ok;
}

Status_t I2cService::serviceSlaveRequest(hal::i2c::I2cSlave* i2c_slave, i2c_proto_I2cMsg* msg) {
  size_t slave_id;
  hal::i2c::I2cSlave::StatusInfo info;

  if (i2c_slave == &i2c_slave0_) {
    slave_id = 0;
  } else {
    slave_id = 1;
  }

  if (i2c_slave->serviceStatus(&info) != Status_t::Ok) {
    DEBUG_ERROR("Srv slave(%d) status (request/access id: UNKNOWN) [FAILED]", slave_id);
    return Status_t::Error;
  }

  msg->sequence_number = info.sequence_number;

  if (info.request.request_id != 0 && info.request.access_id == 0) {
    msg->which_msg = i2c_proto_I2cMsg_slave_status_tag;

    msg->msg.slave_status.request_id = info.request.request_id;
    msg->msg.slave_status.status_code = convertSlaveStatus(info.request.status_code);
    msg->msg.slave_status.queue_space = info.queue_space;

    size_t read_size = info.request.read_size;
    if (read_size > sizeof(msg->msg.slave_status.read_data.bytes)) {
      DEBUG_ERROR("Srv slave(%d) status (request id: %d) [FAILED]", slave_id, info.request.request_id);
      return Status_t::Error;
    }

    msg->msg.slave_status.read_data.size = static_cast<uint16_t>(read_size);
    i2c_slave->copyData(info.request.read_addr, msg->msg.slave_status.read_data.bytes, read_size);

    DEBUG_INFO("Srv slave(%d) status (request id: %d) [OK]", slave_id, info.request.request_id);
    return Status_t::Ok;

  } else if (info.request.access_id != 0 && info.request.request_id == 0) {
    msg->which_msg = i2c_proto_I2cMsg_slave_notification_tag;

    msg->msg.slave_notification.access_id = info.request.access_id;
    msg->msg.slave_notification.status_code = convertSlaveStatus(info.request.status_code);
    msg->msg.slave_notification.queue_space = info.queue_space;

    size_t read_size = info.request.read_size;
    size_t write_size = info.request.write_size;
    if (read_size > sizeof(msg->msg.slave_notification.read_data.bytes) ||
        write_size > sizeof(msg->msg.slave_notification.write_data.bytes)) {
      DEBUG_ERROR("Srv slave(%d) notification (access id: %d) [FAILED]", slave_id, info.request.request_id);
      return Status_t::Error;
    }
    msg->msg.slave_notification.addr = (info.request.write_addr > 0) ? info.request.write_addr : info.request.read_addr;

    msg->msg.slave_notification.read_data.size = static_cast<uint16_t>(read_size);
    i2c_slave->copyData(info.request.read_addr, msg->msg.slave_notification.read_data.bytes, read_size);

    msg->msg.slave_notification.write_data.size = static_cast<uint16_t>(write_size);
    i2c_slave->copyData(info.request.write_addr, msg->msg.slave_notification.write_data.bytes, write_size);

    DEBUG_INFO("Srv slave(%d) notification (access id: %d) [OK]", slave_id, info.request.access_id);
    return Status_t::Ok;

  } else {
    DEBUG_ERROR("Srv slave(%d) status (invalid request/access id combination) [FAILED]", slave_id);
    return Status_t::Error;
  }
}

Status_t I2cService::serviceConfigRequest(hal::i2c::I2cConfig* i2c_config, i2c_proto_I2cMsg* msg) {
  size_t config_id;
  hal::i2c::I2cConfig::StatusInfo info;

  if (i2c_config == &i2c_config0_) {
    config_id = 0;
  } else {
    config_id = 1;
  }

  if (i2c_config->serviceStatus(&info) != Status_t::Ok) {
    DEBUG_ERROR("Srv config(%d) status [FAILED]", config_id);
    return Status_t::Error;
  }

  msg->sequence_number = info.sequence_number;
  msg->which_msg = i2c_proto_I2cMsg_config_status_tag;

  msg->msg.config_status.request_id = info.request_id;
  msg->msg.config_status.status_code = static_cast<i2c_proto_I2cConfigStatusCode>(info.status_code);

  DEBUG_INFO("Srv config(%d) status [OK]", config_id);
  return Status_t::Ok;
}

i2c_proto_I2cStatusCode I2cService::convertMasterStatus(hal::i2c::I2cMaster::RequestStatus status) {
  switch (status) {
    case hal::i2c::I2cMaster::RequestStatus::NotInit:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NOT_INIT;
    case hal::i2c::I2cMaster::RequestStatus::NoSpace:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NO_SPACE;
    case hal::i2c::I2cMaster::RequestStatus::Complete:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SUCCESS;
    case hal::i2c::I2cMaster::RequestStatus::SlaveBusy:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SLAVE_NO_ACK;
    case hal::i2c::I2cMaster::RequestStatus::BadRequest:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_BAD_REQUEST;
    case hal::i2c::I2cMaster::RequestStatus::InterfaceError:
    case hal::i2c::I2cMaster::RequestStatus::Pending:
    case hal::i2c::I2cMaster::RequestStatus::Ongoing:
    default:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_INTERFACE_ERROR;
  }
}

i2c_proto_I2cStatusCode I2cService::convertSlaveStatus(hal::i2c::I2cSlave::RequestStatus status) {
  switch (status) {
    case hal::i2c::I2cSlave::RequestStatus::NotInit:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NOT_INIT;
    case hal::i2c::I2cSlave::RequestStatus::NoSpace:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_NO_SPACE;
    case hal::i2c::I2cSlave::RequestStatus::Complete:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SUCCESS;
    case hal::i2c::I2cSlave::RequestStatus::SlaveBusy:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_SLAVE_NO_ACK;
    case hal::i2c::I2cSlave::RequestStatus::BadRequest:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_BAD_REQUEST;
    case hal::i2c::I2cSlave::RequestStatus::InterfaceError:
    case hal::i2c::I2cSlave::RequestStatus::Pending:
    default:
      return i2c_proto_I2cStatusCode::i2c_proto_I2cStatusCode_STS_INTERFACE_ERROR;
  }
}

}  // namespace app::i2c_srv
