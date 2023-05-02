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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define _Z_Pin GPIO_PIN_0
#define _Z_GPIO_Port GPIOA
#define _Z_EXTI_IRQn EXTI0_IRQn
#define _X_Pin GPIO_PIN_1
#define _X_GPIO_Port GPIOA
#define _Y_Pin GPIO_PIN_2
#define _Y_GPIO_Port GPIOA
#define _XX_Pin GPIO_PIN_3
#define _XX_GPIO_Port GPIOA
#define _YY_Pin GPIO_PIN_4
#define _YY_GPIO_Port GPIOA
#define IRQ_nrf24l01_Pin GPIO_PIN_0
#define IRQ_nrf24l01_GPIO_Port GPIOB
#define CSN_nrf24l01_Pin GPIO_PIN_1
#define CSN_nrf24l01_GPIO_Port GPIOB
#define CE_nrf24l01_Pin GPIO_PIN_10
#define CE_nrf24l01_GPIO_Port GPIOB
#define key0_Pin GPIO_PIN_12
#define key0_GPIO_Port GPIOB
#define key0_EXTI_IRQn EXTI15_10_IRQn
#define key1_Pin GPIO_PIN_13
#define key1_GPIO_Port GPIOB
#define key1_EXTI_IRQn EXTI15_10_IRQn
#define key2_Pin GPIO_PIN_14
#define key2_GPIO_Port GPIOB
#define key2_EXTI_IRQn EXTI15_10_IRQn
#define key3_Pin GPIO_PIN_15
#define key3_GPIO_Port GPIOB
#define key3_EXTI_IRQn EXTI15_10_IRQn
#define key4_Pin GPIO_PIN_8
#define key4_GPIO_Port GPIOA
#define key4_EXTI_IRQn EXTI9_5_IRQn
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
