/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ramcfg.c
  * @brief   This file provides code for the configuration
  *          of the RAMCFG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ramcfg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RAMCFG_HandleTypeDef hramcfg_BKPRAM;

/* RAMCFG init function */
void MX_RAMCFG_Init(void)
{

  /* USER CODE BEGIN RAMCFG_Init 0 */

  /* USER CODE END RAMCFG_Init 0 */

  /* USER CODE BEGIN RAMCFG_Init 1 */

  /* USER CODE END RAMCFG_Init 1 */

  /** Initialize RAMCFG BKPRAM
  */
  hramcfg_BKPRAM.Instance = RAMCFG_BKPRAM;
  if (HAL_RAMCFG_Init(&hramcfg_BKPRAM) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMCFG_Init 2 */

  /* USER CODE END RAMCFG_Init 2 */

}

void HAL_RAMCFG_MspInit(RAMCFG_HandleTypeDef* ramcfgHandle)
{

  /* USER CODE BEGIN RAMCFG_MspInit 0 */

  /* USER CODE END RAMCFG_MspInit 0 */
    /* RAMCFG clock enable */
    __HAL_RCC_RAMCFG_CLK_ENABLE();
  /* USER CODE BEGIN RAMCFG_MspInit 1 */

  /* USER CODE END RAMCFG_MspInit 1 */
}

void HAL_RAMCFG_MspDeInit(RAMCFG_HandleTypeDef* ramcfgHandle)
{

  /* USER CODE BEGIN RAMCFG_MspDeInit 0 */

  /* USER CODE END RAMCFG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RAMCFG_CLK_DISABLE();
  /* USER CODE BEGIN RAMCFG_MspDeInit 1 */

  /* USER CODE END RAMCFG_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
