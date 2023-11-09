/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void initPeripherals();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define GPIO_6_Pin GPIO_PIN_3
#define GPIO_6_GPIO_Port GPIOF
#define GPIO_6_EXTI_IRQn EXTI3_IRQn
#define GPIO_7_Pin GPIO_PIN_5
#define GPIO_7_GPIO_Port GPIOF
#define GPIO_7_EXTI_IRQn EXTI5_IRQn
#define VBUS_SENSE_Pin GPIO_PIN_2
#define VBUS_SENSE_GPIO_Port GPIOC
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define GPIO_0_Pin GPIO_PIN_8
#define GPIO_0_GPIO_Port GPIOC
#define GPIO_0_EXTI_IRQn EXTI8_IRQn
#define GPIO_1_Pin GPIO_PIN_9
#define GPIO_1_GPIO_Port GPIOC
#define GPIO_1_EXTI_IRQn EXTI9_IRQn
#define T_VCP_TX_Pin GPIO_PIN_9
#define T_VCP_TX_GPIO_Port GPIOA
#define T_VCP_RX_Pin GPIO_PIN_10
#define T_VCP_RX_GPIO_Port GPIOA
#define GPIO_2_Pin GPIO_PIN_10
#define GPIO_2_GPIO_Port GPIOC
#define GPIO_2_EXTI_IRQn EXTI10_IRQn
#define GPIO_3_Pin GPIO_PIN_11
#define GPIO_3_GPIO_Port GPIOC
#define GPIO_3_EXTI_IRQn EXTI11_IRQn
#define GPIO_4_Pin GPIO_PIN_12
#define GPIO_4_GPIO_Port GPIOC
#define GPIO_4_EXTI_IRQn EXTI12_IRQn
#define GPIO_5_Pin GPIO_PIN_2
#define GPIO_5_GPIO_Port GPIOD
#define GPIO_5_EXTI_IRQn EXTI2_IRQn
#define UCPD_DBn_Pin GPIO_PIN_5
#define UCPD_DBn_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
