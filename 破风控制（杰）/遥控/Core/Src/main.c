/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "stdio.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum
{
	SYMBOL    = 0,
	MODE      = 1,
	OPTION    = 2,
	DATA_L_X = 3,
	DATA_L_Y = 4,
	DATA_R_X = 5,
	DATA_R_Y = 6,
	JAW      = 7,
	LIFT     = 8,
	signal_array_Space = 32
}signal_TypeDef;

extern unsigned char gImage_windbreaker[390];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char flag_nrf24l01 = 0;
unsigned char tmp_buf[32] = {0};
unsigned char temp;
unsigned char differences=0;
unsigned int i = 0;

uint32_t signal[20];
uint16_t data_z=0;
unsigned char data_x1_average=0;
unsigned char data_y1_average=0;
unsigned char data_x2_average=0;
unsigned char data_y2_average=0;
int mode=2,mode0,mode1,mode2,mode3,mode4;
int	option=2,option0,option1,option2,option3,option4,option5,option6,option7,option8,option9,option10;
int gate=0,jaw=0,lift=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	NRF24L01_Init();
	OLED_init();
	OLED_display_on();
	OLED_refresh_gram();
	OLED_operate_gram(PEN_CLEAR);
	OLED_show_string(1, 13, "WIND");
	OLED_show_string(3, 13, "BREAKERS");
	OLED_refresh_gram();
	OLED_DrawBMP(0, 0,63, 7,gImage_windbreaker);//破风的logo
	HAL_Delay(2000);

  
//  temp = 4;
  while(NRF24L01_Check()){
	OLED_show_string(2, 1, "NRF_TEST_FAIL");
	OLED_refresh_gram();
	 HAL_Delay(500);
	}//检测nrf24l01   
  
	OLED_operate_gram(PEN_CLEAR);
	OLED_show_string(3, 1, "NRF_TEST_succeeded");
	HAL_Delay(1000);
	OLED_refresh_gram();
	TX_Mode();	
	MX_ADC1_Init();
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	OLED_operate_gram(PEN_CLEAR);
	HAL_ADC_Stop_DMA(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
	
	if(gate){OLED_show_string(0, 1, "R");}
	else    {OLED_show_string(0, 1, "F");}
	
	if(tmp_buf[JAW]){OLED_show_string(0, 14, "I");}
	else    {OLED_show_string(0, 18, "V");}
	
	
	OLED_show_string(2,1, ">");
	OLED_show_string(2,8, "---->");
	OLED_printf(0, 9,"m=%d",mode);
	OLED_printf(1, 9,"o=%d",option);
	OLED_printf(3, 9,"x=%d",tmp_buf[DATA_L_X]);
	OLED_printf(4, 9,"y=%d",tmp_buf[DATA_L_Y]);
	  
	if(mode>2){mode=2;}
	if(mode<0){mode=0;}

	mode0=mode;
	OLED_show_string(mode0,3, "MOVI");

	mode1=mode+1;
	OLED_show_string(mode1,3, "ADJU");
	mode2=mode+2;
	OLED_show_string(mode2,3, "MANI");
//---------------------------菜单的显示


	if(mode==2){   	            //---------------------moving
		if(option>2){option=2;}
		if(option<0){option=0;}
		option0=option;
		OLED_show_string(option0,14, "Fas");//o=2
		option1=option+1;
		OLED_show_string(option1,14, "MID");//o=1
		option2=option+2;
		OLED_show_string(option2,14, "SLO");//o=0
	}
	else if(mode==1){
		if(option>2){option=2;}
		if(option<0){option=0;}
		option0=option;
		OLED_show_string(option0,14, "RGH");//o=2
		option1=option+1;
		OLED_show_string(option1,14, "MID");//o=1
		option2=option+2;
		OLED_show_string(option2,14, "TIY");//o=0
	}
	else if(mode==0){
		
	  if(option<=2){option+=10;}
	  if(option>=11){option-=10;}
	  
	  option1=option;
	  if(option1>4){option1-=10;}
	  OLED_show_string(option1,14, "prot");//2
	  
	  option2=option+1;
	  if(option2>4){option2-=10;}
	  OLED_show_string(option2,14, "up");//1
	  
	  option3=option+2;
	  if(option3>4){option3-=10;}
	  OLED_show_string(option3,14, "plac");//10
		
	  option4=option+3;
	  if(option4>4){option4-=10;}
	  OLED_show_string(option4,14, "down");//9
	  
	  option5=option+4;
	  if(option5>4){option5-=10;}
	  OLED_show_string(option5,14, "plac");//8
	  
	  option6=option+5;
	  if(option6>4){option6-=10;}
	  OLED_show_string(option6,14, "free");/7
	
		option7=option+6;
	  if(option7>4){option7-=10;}
	  OLED_show_string(option7,14, "mid");//6
		
	  option8=option+7;
	  if(option8>4){option8-=10;}
	  OLED_show_string(option8,14, "plac");//5
	  
	  option9=option+8;
	  if(option9>4){option9-=10;}
	  OLED_show_string(option9,14, "tou");//4
	  
	  option10=option+9;
	  if(option10>4){option10-=10;}
	  OLED_show_string(option10,14, "plac");//3
	
	
	
	
	}
	
	
	
	  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == (&htim2)){
		
		OLED_refresh_gram();
		OLED_operate_gram(PEN_CLEAR);
		i++;
		
		
		
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	if(GPIO_Pin == key1_Pin){
		if(HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==GPIO_PIN_RESET){//使用薄弱的if来防抖
			mode--;//遥控菜单的选择
			option=2;
			HAL_ADC_Stop_DMA(&hadc1);
			gate=0;}
		}
	if(GPIO_Pin == key3_Pin){
		if(HAL_GPIO_ReadPin(key3_GPIO_Port,key3_Pin)==GPIO_PIN_RESET){
			mode++;
		    option=2;
			HAL_ADC_Stop_DMA(&hadc1);
		    gate=0;}
		}
	if(GPIO_Pin == key0_Pin){
		if(HAL_GPIO_ReadPin(key0_GPIO_Port,key0_Pin)==GPIO_PIN_RESET){
			option--;
			HAL_ADC_Stop_DMA(&hadc1);
			gate=0;}
		}
	if(GPIO_Pin == key4_Pin){
		if(HAL_GPIO_ReadPin(key4_GPIO_Port,key4_Pin)==GPIO_PIN_RESET){
			option++;
			HAL_ADC_Stop_DMA(&hadc1);
			gate=0;}
		}
	if(GPIO_Pin == key2_Pin){
		if(HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==GPIO_PIN_RESET){
			if(gate){
				gate=0;
				HAL_ADC_Stop_DMA(&hadc1);
			}		
			else{
				gate=1;
				HAL_ADC_Start_DMA(&hadc1 ,signal ,20);
			}	
		}	
	}
	if(GPIO_Pin == _Z_Pin){
		if(HAL_GPIO_ReadPin(_Z_GPIO_Port,_Z_Pin)==GPIO_PIN_RESET){
			if(jaw==0){tmp_buf[JAW]=0;jaw=1;}
			else     {tmp_buf[JAW]=1;jaw=0;}
		
		
		}
	}
	if(GPIO_Pin == _LIFT_Pin){
		if(HAL_GPIO_ReadPin(_LIFT_GPIO_Port,_LIFT_Pin) == GPIO_PIN_RESET ){
			if(lift==0){tmp_buf[LIFT]=0;lift=1;}
			else       {tmp_buf[LIFT]=1;lift=0;}
		}
	}
}
	



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	HAL_ADC_Stop_DMA(&hadc1);
	
	for(unsigned char i=0;i<20;i+=4){
	data_y1_average+=(signal[i  ]&0xfff)/80;
	data_x1_average+=(signal[i+1]&0xfff)/80;
	data_x2_average+=(signal[i+2]&0xfff)/80;
	data_y2_average+=(signal[i+3]&0xfff)/80;
		
	}
	
	
	data_x1_average = data_x1_average/5;
	data_y1_average = data_y1_average/5;
	data_x2_average = data_x2_average/5;
	data_y2_average = data_y2_average/5;//均值滤波
	
	
	
//	printf("\n \r x1:%d -- y1:%d  ",data_x1_average	,data_y1_average);
	
	if(differences<5){differences++;}
	else {differences=0;}
	tmp_buf[SYMBOL]   = differences    ;//异化
	tmp_buf[MODE]     = mode           ;
	tmp_buf[OPTION]   = option         ;
	tmp_buf[JAW]    =    jaw         ;
	tmp_buf[DATA_L_X]= data_x1_average;
	tmp_buf[DATA_L_Y]= data_y1_average;
	tmp_buf[DATA_R_X]= data_x2_average;
	tmp_buf[DATA_R_Y]= data_y2_average;
	NRF24L01_TxPacket(tmp_buf);
	
	
	
	
	data_x1_average=0;
	data_y1_average=0;
	data_x2_average=0;
	data_y2_average=0;
	
	HAL_ADC_Start_DMA(&hadc1 ,signal ,20);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
