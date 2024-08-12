/*
 * DeviceInfo.cpp
 *
 *  Created on: Aug 6, 2024
 *      Author: Alexander
 */

#include "app/ctrl_srv/DeviceInfo.hpp"
#include "main.h"

namespace app::ctrl_srv {

void DeviceInfo::getDeviceInfo(Info* info) {
  info->device_type = DeviceType;
  info->hardware_version = readHardwareVersion();

  info->firmware_version_major = FirmwareVersionMajor;
  info->firmware_version_minor = FirmwareVersionMinor;
  info->firmware_version_patch = FirmwareVersionPatch;
}

uint8_t DeviceInfo::readHardwareVersion() {
  uint8_t hw_version = 0;

  // Read hardware version from GPIOs
  hw_version |= static_cast<uint8_t>(HAL_GPIO_ReadPin(HW_VERSION_0_GPIO_Port, HW_VERSION_0_Pin) << 0);
  hw_version |= static_cast<uint8_t>(HAL_GPIO_ReadPin(HW_VERSION_1_GPIO_Port, HW_VERSION_1_Pin) << 1);
  hw_version |= static_cast<uint8_t>(HAL_GPIO_ReadPin(HW_VERSION_2_GPIO_Port, HW_VERSION_2_Pin) << 2);

  return hw_version;
}

} /* namespace app::ctrl_srv */
