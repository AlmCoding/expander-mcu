## Version 1.0.1 - I2C Slave Bug Fixes

### Overview
This version includes bug fixes and improvements to enhance stability and reliability.

### Changes
- **Bug Fixes**: Resolve I2C slave write size issue.

### Known Issues
- None

### Future Work
- Implement additional I2C features (e.g., multi-master support).

---

## Version 1.0.0 - Stable Release

### Overview
This is the first stable release of the STM32 firmware for the USB to I2C Expander device. 
This version includes bug fixes and improvements to enhance stability and reliability.

### Changes
- **Bug Fixes**: Addressed various issues to improve USB and I2C communication stability.
- **Enhanced Error Handling**: Improved robustness in detecting and handling communication errors.
- **Performance Improvements**: Optimized certain operations for better response times.

### Known Issues
- None

### Future Work
- Implement additional I2C features (e.g., multi-master support).
- Improve performance and response times.

---

## Version 0.0.1 - Initial Release

### Overview
This is the first official release of the STM32 firmware for the USB to I2C Expander device.  
This firmware enables communication between USB hosts and I2C slave devices,  
allowing for seamless integration in various applications.

### Features
- **USB Host Communication**: Implements USB protocol for host device interaction.
- **I2C Master Mode**: Supports reading from and writing to I2C slave devices.
- **I2C Slave Notifications**: Notifies the host of any events or data changes from I2C slaves.
- **Local Slave Configuration**: Enables users to configure local I2C slaves.
- **Basic Configuration**: Easy-to-use configuration options for I2C bus speed and slave addresses.
- **Error Handling**: Basic error handling for communication failures.

### Changes
- **Initial Codebase**: Established foundational codebase for USB and I2C communication.
- **Documentation**: Included basic documentation for setup and usage.

### Known Issues
- No advanced error logging implemented.

### Future Work
- Implement additional I2C features (e.g., multi-master support).
- Enhance error handling and logging.
- Improve performance and response times.
