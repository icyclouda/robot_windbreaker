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
#include "stm32f4xx_hal.h"

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
#define R_F_NEG_Pin GPIO_PIN_2
#define R_F_NEG_GPIO_Port GPIOE
#define R_B_POS_Pin GPIO_PIN_3
#define R_B_POS_GPIO_Port GPIOE
#define R_B_NEG_Pin GPIO_PIN_4
#define R_B_NEG_GPIO_Port GPIOE
#define steering_engine_4_Pin GPIO_PIN_5
#define steering_engine_4_GPIO_Port GPIOE
#define steering_engine_5_Pin GPIO_PIN_6
#define steering_engine_5_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define encoder_LF_B_Pin GPIO_PIN_0
#define encoder_LF_B_GPIO_Port GPIOA
#define encoder_LF_A_Pin GPIO_PIN_1
#define encoder_LF_A_GPIO_Port GPIOA
#define steering_engine_1_Pin GPIO_PIN_7
#define steering_engine_1_GPIO_Port GPIOA
#define steering_engine_2_Pin GPIO_PIN_0
#define steering_engine_2_GPIO_Port GPIOB
#define steering_engine_3_Pin GPIO_PIN_1
#define steering_engine_3_GPIO_Port GPIOB
#define motor_LF_Pin GPIO_PIN_9
#define motor_LF_GPIO_Port GPIOE
#define motor_RF_Pin GPIO_PIN_11
#define motor_RF_GPIO_Port GPIOE
#define motor_LB_Pin GPIO_PIN_13
#define motor_LB_GPIO_Port GPIOE
#define motor_RB_Pin GPIO_PIN_14
#define motor_RB_GPIO_Port GPIOE
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define steering_engine_6_Pin GPIO_PIN_14
#define steering_engine_6_GPIO_Port GPIOB
#define steering_engine_7_Pin GPIO_PIN_15
#define steering_engine_7_GPIO_Port GPIOB
#define encoder_RB_A_Pin GPIO_PIN_12
#define encoder_RB_A_GPIO_Port GPIOD
#define encoder_RB_B_Pin GPIO_PIN_13
#define encoder_RB_B_GPIO_Port GPIOD
#define encoder_LB_B_Pin GPIO_PIN_6
#define encoder_LB_B_GPIO_Port GPIOC
#define encoder_LB_A_Pin GPIO_PIN_7
#define encoder_LB_A_GPIO_Port GPIOC
#define encoder_RF_A_Pin GPIO_PIN_15
#define encoder_RF_A_GPIO_Port GPIOA
#define L_F_NEG_Pin GPIO_PIN_5
#define L_F_NEG_GPIO_Port GPIOD
#define L_F_POS_Pin GPIO_PIN_6
#define L_F_POS_GPIO_Port GPIOD
#define L_B_NEG_Pin GPIO_PIN_7
#define L_B_NEG_GPIO_Port GPIOD
#define encoder_RF_B_Pin GPIO_PIN_3
#define encoder_RF_B_GPIO_Port GPIOB
#define CE_nrf24l01_Pin GPIO_PIN_6
#define CE_nrf24l01_GPIO_Port GPIOB
#define CSN_nrf24l01_Pin GPIO_PIN_7
#define CSN_nrf24l01_GPIO_Port GPIOB
#define IRQ_nrf24l01_Pin GPIO_PIN_8
#define IRQ_nrf24l01_GPIO_Port GPIOB
#define steering_engine_8_Pin GPIO_PIN_9
#define steering_engine_8_GPIO_Port GPIOB
#define L_B_POS_Pin GPIO_PIN_0
#define L_B_POS_GPIO_Port GPIOE
#define R_F_POS_Pin GPIO_PIN_1
#define R_F_POS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
