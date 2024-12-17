Controller Porting

1. IOC file setup
2. Convert to C++ project
3. Compile project

4. Copy App and Libraries folders
5. Add to source folders (for debug and release config)
6. Add include paths (for both configs)
7. Add definitions
8. Change user code sections
	- Core/Src/main.c
	- Core/Src/main.h
	- AZURE_RTOS/App/app_azure_rtos.c
	- Core/Inc/tim.h
	- Core/Inc/i2c.h
	- Core/Inc/i2c.c
	- USBX/App/app_usbx_device.c
	- USBX/App/app_device_cdc_amc.c
9. Set compiler versions, warning settings, build steps and post build outputs
10. Update linker file (for TraceX)

ENABLE_RTT_DEBUG_OUTPUT  
SEGGER_RTT_MODE_DEFAULT = SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL  
SEGGER_RTT_MODE_DEFAULT = SEGGER_RTT_MODE_NO_BLOCK_SKIP
