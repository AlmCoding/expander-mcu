/*
 * DeviceInfo.hpp
 *
 *  Created on: Aug 6, 2024
 *      Author: Alexander
 */

#ifndef APP_CTRL_SRV_DEVICEINFO_HPP_
#define APP_CTRL_SRV_DEVICEINFO_HPP_

#include "common.hpp"

namespace app::ctrl_srv {

class DeviceInfo {
 private:
  static constexpr uint8_t DeviceType = 1;
  static constexpr uint8_t FirmwareVersionMajor = 1;
  static constexpr uint8_t FirmwareVersionMinor = 0;
  static constexpr uint8_t FirmwareVersionPatch = 0;

 public:
  struct Info {
    uint8_t device_type;
    uint8_t hardware_version;

    uint8_t firmware_version_major;
    uint8_t firmware_version_minor;
    uint8_t firmware_version_patch;
    char git_hash[42];
  };

  static void getDeviceInfo(Info* info);

 private:
  DeviceInfo() = delete;
  virtual ~DeviceInfo() = delete;

  static uint8_t readHardwareVersion();
};

} /* namespace app::ctrl_srv */

#endif /* APP_CTRL_SRV_DEVICEINFO_HPP_ */
