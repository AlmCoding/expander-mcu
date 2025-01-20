/*
 * boot.cpp
 *
 *  Created on: Jan 20, 2025
 *      Author: Alexander L.
 */

#include "util/boot.hpp"
#include "etl/error_handler.h"  // etl::ETL_ASSERT()
#include "main.h"

namespace util {

constexpr uint32_t BootMagicNum = 0x5A5A5A5A;
constexpr uint32_t BootMagicAddr = BKPSRAM_BASE_NS;
constexpr uint32_t BootloaderAddr = 0x0BF97000;  // for STM32H523

bool isBootloaderRequested() {
  uint32_t value = *(uint32_t*)BootMagicAddr;
  if (value == BootMagicNum) {
    return true;
  }
  return false;
}

void requestBootloader() {
  HAL_PWR_EnableBkUpAccess();
  *(uint32_t*)BootMagicAddr = BootMagicNum;

  // Check if the bootloader is requested
  ETL_ASSERT(isBootloaderRequested() == true, ETL_ERROR(0));
}

void clearBootloaderRequest() {
  HAL_PWR_EnableBkUpAccess();
  *(uint32_t*)BootMagicAddr = 0;

  // Check if the bootloader is not requested
  ETL_ASSERT(isBootloaderRequested() == false, ETL_ERROR(0));
}

void startBootloader() {
  void (*SysMemBootJump)(void);

  clearBootloaderRequest();

  /* Set the address of the entry point to bootloader */
  volatile uint32_t addr = BootloaderAddr;

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  uint8_t cnt = sizeof(NVIC->ICER) / sizeof(*NVIC->ICER);
  for (uint8_t i = 0; i < cnt; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* Re-enable all interrupts */
  __enable_irq();

  /* Set up the jump to bootloader address + 4 */
  SysMemBootJump = (void (*)(void))(*((uint32_t*)((addr + 4))));

  /* Set the main stack pointer to the bootloader stack */
  __set_MSP(*(uint32_t*)addr);

  /* Call the function to jump to bootloader location */
  SysMemBootJump();

  /* Jump is done successfully */
  while (1) {
    /* Code should never reach this loop */
  }
}

}  // namespace util
