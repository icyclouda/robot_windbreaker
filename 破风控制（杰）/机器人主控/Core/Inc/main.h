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
typedef struct
{
  float kp;       
  float ki;     
  float kd;                         
} PID_parameter;
	  
typedef struct
{
  int target;       
  int data_now;     
  int data_before; 
  int integral;
  int last_target;
} PID_variables;

typedef enum
{
	L_F = 0,
	L_B ,
	R_F ,
	R_B ,
	M_X ,
	M_Y ,
	M_K ,
	M_A , 
	M_S1,
	M_S2,
	M_S3,
	arspace
}pwm_output_TypeDef;

typedef enum
{
	SYMBOL   = 0,
	MODE     = 1,
	OPTION   = 2,
	DATA_L_X = 3,
	DATA_L_Y = 4,
	DATA_R_X = 5,
	DATA_R_Y = 6,
	JAW      = 7,
	LIFT     = 8,
	signal_array_Space = 32
}signal_TypeDef;


typedef enum
{
	
	
	
	
	
	option_space
}mani_option;

typedef enum{
	MANI =0,
	AJUS =1,
	MOVI =2,
	MODE_STATE_space
}MODE_state;

typedef enum{
	no_fault = 0,
	m1_fault = 1,
	m2_fault = 2,
	xp_fault = 3,
	xm_fault = 4,
	yp_fault = 5,
	ym_fault = 6,
	ap_fault = 7,
	am_fault = 8,
	fault_Space
}mani_fault_state;

typedef enum{
	mani_sita1     =  0,
	mani_sita2,  
	mani_sita3,
	mani_L_sita1,     
	mani_L_sita2,     
	mani_L_sita3,
	mani_L_X,
	mani_L_Y,
	mani_L_A,
	mani_m1,   
	mani_m2,  
	mani_n,
	mani_v,
	mani_dx_target, 
	mani_dy_target, 
	mani_da_target,	
	mani_arrspace
}manipulator_arrspace;
	
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
#define signal_per_round   330 //一圈多少个信号
#define speed_param 5  //1500/signal_per_round 
#define l1 235
#define l2 215
#define P  0.5
#define Q  0.5

#define error_m1            error_status[m1_fault]=1
#define if_m1_error        	error_status[m1_fault]==1

#define error_m2            error_status[m2_fault]=1
#define if_m2_error        	error_status[m2_fault]==1

#define error_x_plus        error_status[xp_fault]=1//x+后出错
#define error_x_minus       error_status[xm_fault]=1
#define error_y_plus        error_status[yp_fault]=1
#define error_y_minus       error_status[ym_fault]=1
#define error_a_plus        error_status[ap_fault]=1
#define error_a_minus       error_status[am_fault]=1

#define if_error_x_plus     error_status[xp_fault]==1
#define if_error_x_minus    error_status[xm_fault]==0
#define if_error_y_plus     error_status[yp_fault]==1
#define if_error_y_minus    error_status[ym_fault]==0
#define if_error_a_plus     error_status[ap_fault]==1
#define if_error_a_minus    error_status[am_fault]==0

#define no_fault_happended  for(int i=0;i<fault_Space;i++){error_status[i]=0;}

#define x_last_plus       	mani_status[mani_dx_target]=1
#define x_last_minus		mani_status[mani_dx_target]=0
#define y_last_plus       	mani_status[mani_dy_target]=1
#define y_last_minus		mani_status[mani_dy_target]=0
#define a_last_plus       	mani_status[mani_da_target]=1
#define a_last_minus		mani_status[mani_da_target]=0

#define if_x_plus           mani_status[mani_dx_target]==1
#define if_y_plus           mani_status[mani_dy_target]==1
#define if_a_plus           mani_status[mani_da_target]==1
#define if_x_minus          mani_status[mani_dx_target]==0
#define if_y_minus          mani_status[mani_dy_target]==0
#define if_a_minus          mani_status[mani_da_target]==0

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
