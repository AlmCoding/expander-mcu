# USB to I2C Firmware README

## Overview
This repository contains the firmware for a USB to I2C Expander device, enabling communication between USB hosts and I2C slave devices. The firmware supports two I2C interfaces, allowing for flexible integration in various applications. Users can control the device via a Python library or a dedicated UI application.

## Features
- **USB Host Communication**: Implements USB protocol for interaction with host devices.
- **I2C Master Mode**: Read from and write to I2C slave devices seamlessly.
- **I2C Slave Notifications**: Receive notifications for events or data changes from I2C slaves.
- **Local Slave Configuration**: Configure local I2C slaves easily.
- **Basic Configuration Options**: Set I2C bus speed and slave addresses effortlessly.
- **Error Handling**: Basic error handling for communication failures.

## Getting Started
To begin using the firmware, download it via the STM32CubeProgrammer or the UI application.
