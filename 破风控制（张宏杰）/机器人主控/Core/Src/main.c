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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "core_cm4.h"
#include "NRF24L01.h"
#include "stdio.h"
#include "Data_uart.h"
#include "OLED.h"
#include "math.h"
//USE_HAL_DRIVER,STM32F407xx
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---BEGIN-----PRIVATE--TYPEDEF---------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
#define signal_per_round   330 
#define speed_param 5  //1500/signal_per_round 
#define l1 215
#define l2 250


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
	L_B = 1,
	R_F = 2,
	R_B = 3,
	M_X = 4,
	M_Y = 5,
	M_K = 6,
	M_A = 7,
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
	signal_array_Space = 32
}signal_TypeDef;

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
	fault_Space
}mani_fault_state;

typedef enum{
	mani_sita1  =  0,
	mani_sita2  =  1,
	mani_sita3  =  2,
	mani_m1     =  3,
	mani_m2     =  4,
	mani_n      =  5,
	mani_v      =  6,
	mani_arrspace
}manipulator_arrspace;
	
	
	

extern unsigned char gImage_windbreaker[390];
//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---END-----PRIVATE--TYPEDEF---------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---BEGIN-----PRIVATE--VARIABLES---------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
unsigned char controller_signal[signal_array_Space] = {0};
int           pwm_output[arspace]= {0};
short         motor_speed[arspace] = {0};
int           target[arspace]={0};
double        mani_status[mani_arrspace]={0};

PID_parameter drive_pidp_LF;
PID_parameter drive_pidp_LB;
PID_parameter drive_pidp_RF;
PID_parameter drive_pidp_RB;

PID_variables drive_pidv_L_F;
PID_variables drive_pidv_L_B;
PID_variables drive_pidv_R_F;
PID_variables drive_pidv_R_B;


unsigned char temp;
unsigned char data_L_X=0,data_L_Y=0,data_R_X=0,data_R_Y=0;
unsigned int i=0;
         int motor_rotate_speed=0 , target_cal=0;



int temp_X,temp_Y,temp_K,temp_A;
int           m_reset_flag   =	 0;
int           m_error_flag   =   0;

//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---END-----PRIVATE------VARIABLES--------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---BEGIN-----PRIVATE--FUNCTION---PROTOTYPES---------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
void PWM_output(int *pwm);
void get_speed(short *rotate_speed);

void set_pidp(PID_parameter *hpidp,int KP, int KI ,int KD);
void input_pidv(PID_variables *hpidv,int Target,int Data_now);
int pid_cal(PID_parameter *hpidp,PID_variables *hpidv);
void pid_protect(PID_parameter *hpidp,PID_variables *hpidv);
int abs(int a);
int nominus(int a);
int pwm_limit(int pwm);
void mani_cul(double x,double y,double sita);

//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---BEGIN-----PRIVATE--FUNCTION---PROTOTYPES----------------------
//--------------------------------------------------------------------------------------------------------------------------
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE---BEGIN--------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
    OLED_init();
	OLED_display_on();
	OLED_refresh_gram();
	OLED_operate_gram(PEN_CLEAR);
	OLED_show_string(3, 13, "WIND");
	OLED_show_string(4, 13, "BREAKERS");
	OLED_refresh_gram();
	OLED_DrawBMP(0, 0,63, 7,gImage_windbreaker);
	HAL_Delay(1000);
	
   
   NRF24L01_Init();
	while(NRF24L01_Check()){
		OLED_show_string(2, 0, "NRF24L01_test_fail");
	}
	OLED_operate_gram(PEN_CLEAR);
	OLED_refresh_gram();
	OLED_show_string(2, 0, "NRF24L01_test_succeeded");
    
	RX_Mode();
	
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);	    
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);	   
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);
	


	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1); 
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2); 	
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1); 
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2); 
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); 
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); 	
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1); 
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2); 

	
	HAL_TIM_Base_Start_IT(&htim6); //counter_set
	
    HAL_TIM_Base_Start_IT(&htim7); //oled_refresh_tim
  //-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE------END-----------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  //-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE------BEGIN---WHILE----------------------------------
//--------------------------------------------------------------------------------------------------------------------------
	  	if(NRF24L01_RxPacket(controller_signal)==0){
		data_L_X=controller_signal[DATA_L_X];
	    data_L_Y=controller_signal[DATA_L_Y];
		data_R_X=controller_signal[DATA_R_X];
	    data_R_Y=controller_signal[DATA_R_Y];
		}
		else{
			data_L_X=25;
			data_L_Y=23;
			data_R_X=24;
			data_R_Y=25;
			
		}
		if(controller_signal[MODE]==MOVI){
			OLED_printf(0,0,"dLX=%d-dLY=%d",data_L_X,data_L_Y);
			OLED_printf(1,0,"dRX=%d-dRY=%d",data_R_X,data_R_Y);
			OLED_printf(2,0,"LFS=%d-RFS=%d",motor_speed[L_F],motor_speed[R_F]);
			OLED_printf(3,0,"LBS=%d-RBS=%d",motor_speed[L_B],motor_speed[R_B]);
			
  }
		else if(controller_signal[MODE]==MANI){
			if(m_error_flag==m1_fault)           {OLED_printf(2,0,"error_m1=%lf", mani_status[mani_m1]);}
			else if(m_error_flag==m2_fault)      {OLED_printf(2,0,"error_m2=%lf", mani_status[mani_m2]);}
			else{	
				OLED_printf(2,0,"s1=%lf-s2=%lf",mani_status[mani_sita1],mani_status[mani_sita2]);
				OLED_printf(3,0,"s3=%lf",mani_status[mani_sita3 ]);
			}
			OLED_printf(4,0,"X=%lf-Y=%lf-sita=%lf",(double)target[M_X],(double)target[M_Y],(double)target[M_A]);
		}
		HAL_Delay(10);
  
  }
  //-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE-------END-----WHILE------------------------------
//--------------------------------------------------------------------------------------------------------------------------
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE------BEGIN-----------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
	
    if (htim == (&htim6))
    {
		if(controller_signal[MODE]==MOVI){//------------------------------------------moving
			m_reset_flag=0;
			if(controller_signal[OPTION]==2){
				motor_rotate_speed=330;
				set_pidp(&drive_pidp_LF,50,0.2 ,5);
				set_pidp(&drive_pidp_LB,50,0.2 ,5);
				set_pidp(&drive_pidp_RF,50,0.2 ,5);
				set_pidp(&drive_pidp_RB,50,0.2 ,5);
			}
			else if (controller_signal[OPTION]==1){
				motor_rotate_speed=160;
				set_pidp(&drive_pidp_LF,20,0.1 ,1);
				set_pidp(&drive_pidp_LB,20,0.1 ,1);
				set_pidp(&drive_pidp_RF,20,0.1 ,1);
				set_pidp(&drive_pidp_RB,20,0.1 ,1);
			}
			else if(controller_signal[OPTION]==0){
				motor_rotate_speed=80;
				set_pidp(&drive_pidp_LF,10,0.001 ,0.1);
				set_pidp(&drive_pidp_LB,10,0.001 ,0.1);
				set_pidp(&drive_pidp_RF,10,0.001 ,0.1);
				set_pidp(&drive_pidp_RB,10,0.001 ,0.1);
			}
			else{
				motor_rotate_speed=0;
			}
			target_cal=motor_rotate_speed/25;
			
			target[L_F]=target_cal*((int)data_L_X-25)+target_cal*((int)data_R_Y-25)/2;
			target[L_B]=target_cal*((int)data_L_Y-23)+target_cal*((int)data_R_Y-25)/2;
			target[R_F]=target_cal*((int)data_L_Y-23)-target_cal*((int)data_R_Y-25)/2;
			target[R_B]=target_cal*((int)data_L_X-25)-target_cal*((int)data_R_Y-25)/2;

			get_speed(motor_speed);
			
			input_pidv(&drive_pidv_L_F,target[L_F],(int)motor_speed[L_F]);
			pid_protect(&drive_pidp_LF,&drive_pidv_L_F);
			pwm_output[L_F]= pwm_limit(pid_cal(&drive_pidp_LF,&drive_pidv_L_F)); //LF
			drive_pidv_L_F.data_before=drive_pidv_L_F.data_now;
			drive_pidv_L_F.last_target=drive_pidv_L_F.target;
			
			input_pidv(&drive_pidv_R_F,target[R_F],(int)motor_speed[R_F]);
			pid_protect(&drive_pidp_RF,&drive_pidv_R_F);
			pwm_output[R_F]=pwm_limit( pid_cal(&drive_pidp_RF,&drive_pidv_R_F)); //RF
			drive_pidv_R_F.data_before=drive_pidv_R_F.data_now;
			drive_pidv_R_F.last_target=drive_pidv_R_F.target;
			
			input_pidv(&drive_pidv_L_B,target[L_B],(int)motor_speed[L_B]);
			pid_protect(&drive_pidp_LB,&drive_pidv_L_B);
			pwm_output[L_B]=pwm_limit(pid_cal(&drive_pidp_LB,&drive_pidv_L_B)); //LB
			drive_pidv_L_B.data_before=drive_pidv_L_B.data_now;
			drive_pidv_L_B.last_target=drive_pidv_L_B.target;
			
			input_pidv(&drive_pidv_R_B,target[R_B],(int)motor_speed[R_B]);
			pid_protect(&drive_pidp_RB,&drive_pidv_R_B);		
			pwm_output[R_B]=pwm_limit(pid_cal(&drive_pidp_RB,&drive_pidv_R_B)); //RB
			drive_pidv_R_B.data_before=drive_pidv_R_B.data_now;
			drive_pidv_R_B.last_target=drive_pidv_R_B.target;
			
			PWM_output(pwm_output);
			UART_SendData4(motor_speed[L_F],motor_speed[R_F],motor_speed[L_B],motor_speed[R_B]);
			
		}
		else if(controller_signal[MODE]==MANI){//----------------------------------------------manipulator
			
			temp_X=4*((int)data_R_X-25);
			temp_Y=4*((int)data_R_Y-23);
			temp_A=4*((int)data_L_Y-25);
			temp_K=4*((int)data_L_X-24);
			if(m_reset_flag ==0){
				target[M_X]=0;
				target[M_Y]=35;
				target[M_A]=0;
				target[M_K]=0;
				m_reset_flag=1;
			}
			//--------------------------------------------------x
			if(temp_X>90)                   {target[M_X]+=3;}
			else if(temp_X<=90&&temp_X>50)  {target[M_X]+=2;}
			else if(temp_X<=50&&temp_X>10)  {target[M_X]+=1;}
			else if(temp_X<-10&&temp_X>=-50){target[M_X]-=1;}
			else if(temp_X<-50&&temp_X>=90) {target[M_X]-=2;}
			else if(temp_X<-90)             {target[M_X]-=3;}
				if     (target[M_X]<=0)		    {target[M_X]=0  ;}
				else if(target[M_X]>=l1+l2)    	{target[M_X]=l1+l2;}           
			//---------------------------------------------------y
			if(temp_Y>90)                   {target[M_Y]+=3;}
			else if(temp_Y<=90&&temp_Y>50)  {target[M_Y]+=2;}
			else if(temp_Y<=50&&temp_Y>10)  {target[M_Y]+=1;}
			else if(temp_Y<-10&&temp_Y>=-50){target[M_Y]-=1;}
			else if(temp_Y<-50&&temp_Y>=90) {target[M_Y]-=2;}
			else if(temp_Y<-90)				{target[M_Y]-=3;}
				if     (target[M_Y]<=0)		    {target[M_Y]=0  ;}
				else if(target[M_Y]>=l1+l2)    	{target[M_Y]=l1+l2;}
			//---------------------------------------------------k
			if(temp_K>90)                   {target[M_K]=100;}
			else if(temp_K<=90&&temp_K>50)  {target[M_K]=70;}
			else if(temp_K<=50&&temp_K>10)  {target[M_K]=40;}
			else if(temp_K<-10&&temp_K>=-50){target[M_K]=-40;}
			else if(temp_K<-50&&temp_K>=90) {target[M_K]=-70;}
			else if(temp_K<-90)             {target[M_K]=-100;}
			else  							{target[M_K]=0;}
			//----------------------------------------------------a
			if(temp_A>90)                   {target[M_A]+=3;}
			else if(temp_A<=90&&temp_A>50)  {target[M_A]+=2;}
			else if(temp_A<=50&&temp_A>10)  {target[M_A]+=1;}
			else if(temp_A<-10&&temp_A>=-50){target[M_A]-=1;}
			else if(temp_A<-50&&temp_A>=90) {target[M_A]-=2;}
			else if(temp_A<-90)             {target[M_A]-=3;}//0-180
				if     (target[M_A]<=0)		    {target[M_A]=0;}
				else if(target[M_A]>=90)    	{target[M_A]=90;} 
			//----------------------------------------------------------------
//			target[M_X]=0;
//				
//			target[M_Y]=l1+l2;
//			
//			target[M_A]=0;
			mani_cul((double)target[M_X],(double)target[M_Y],(double)target[M_A]*3.141593/180);
			
			get_speed(motor_speed);
			input_pidv(&drive_pidv_L_F,target[M_K],(int)motor_speed[L_F]);
			pid_protect(&drive_pidp_LF,&drive_pidv_L_F);
			pwm_output[L_F]= pwm_limit(pid_cal(&drive_pidp_LF,&drive_pidv_L_F)); //LF
			drive_pidv_L_F.data_before=drive_pidv_L_F.data_now;
			drive_pidv_L_F.last_target=drive_pidv_L_F.target;
			
			input_pidv(&drive_pidv_R_F,-target[M_K],(int)motor_speed[R_F]);
			pid_protect(&drive_pidp_RF,&drive_pidv_R_F);
			pwm_output[R_F]=pwm_limit( pid_cal(&drive_pidp_RF,&drive_pidv_R_F)); //RF
			drive_pidv_R_F.data_before=drive_pidv_R_F.data_now;
			drive_pidv_R_F.last_target=drive_pidv_R_F.target;
			
			input_pidv(&drive_pidv_L_B,-target[M_K],(int)motor_speed[L_B]);
			pid_protect(&drive_pidp_LB,&drive_pidv_L_B);
			pwm_output[L_B]=pwm_limit(pid_cal(&drive_pidp_LB,&drive_pidv_L_B)); //LB
			drive_pidv_L_B.data_before=drive_pidv_L_B.data_now;
			drive_pidv_L_B.last_target=drive_pidv_L_B.target;
			
			input_pidv(&drive_pidv_R_B,target[M_K],(int)motor_speed[R_B]);
			pid_protect(&drive_pidp_RB,&drive_pidv_R_B);		
			pwm_output[R_B]=pwm_limit(pid_cal(&drive_pidp_RB,&drive_pidv_R_B)); //RB
			drive_pidv_R_B.data_before=drive_pidv_R_B.data_now;
			drive_pidv_R_B.last_target=drive_pidv_R_B.target;
			
			PWM_output(pwm_output);
			
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,(uint16_t)(mani_status[mani_sita1]*2000/3.141593+500));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,(uint16_t)(mani_status[mani_sita2]*2000/3.141593+500));
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,2500);

		
		}
    }
	if(htim == (&htim7)){
		
		OLED_refresh_gram();
		OLED_operate_gram(PEN_CLEAR);
		i++;

	}
}


//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
void PWM_output(int *pwm ){
	
	if(pwm[L_F]>0){
		HAL_GPIO_WritePin(L_F_POS_GPIO_Port,L_F_POS_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_F_NEG_GPIO_Port,L_F_NEG_Pin,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(L_F_POS_GPIO_Port,L_F_POS_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_F_NEG_GPIO_Port,L_F_NEG_Pin,GPIO_PIN_RESET);
	}
	if(pwm[L_B]>0){
		HAL_GPIO_WritePin(L_B_POS_GPIO_Port,L_B_POS_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(L_B_NEG_GPIO_Port,L_B_NEG_Pin,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(L_B_POS_GPIO_Port,L_B_POS_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(L_B_NEG_GPIO_Port,L_B_NEG_Pin,GPIO_PIN_RESET);
	}
	
	if(pwm[R_F]>0){
		HAL_GPIO_WritePin(R_F_POS_GPIO_Port,R_F_POS_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_F_NEG_GPIO_Port,R_F_NEG_Pin,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(R_F_POS_GPIO_Port,R_F_POS_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_F_NEG_GPIO_Port,R_F_NEG_Pin,GPIO_PIN_SET);
	}
	if(pwm[R_B]>0){
		HAL_GPIO_WritePin(R_B_POS_GPIO_Port,R_B_POS_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(R_B_NEG_GPIO_Port,R_B_NEG_Pin,GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(R_B_POS_GPIO_Port,R_B_POS_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(R_B_NEG_GPIO_Port,R_B_NEG_Pin,GPIO_PIN_SET);
	}
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint16_t)(abs(pwm[L_F])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint16_t)(abs(pwm[R_F])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint16_t)(abs(pwm[L_B])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,(uint16_t)(abs(pwm[R_B])));

}
int pwm_limit(int pwm){
	
	if(pwm>999)return 999;
	else if(pwm<-999)return -999;
	else return pwm;
}
void get_speed(short *rotate_speed){
	
	rotate_speed[L_F]= (short)(__HAL_TIM_GET_COUNTER(&htim5)*speed_param);  
	__HAL_TIM_SET_COUNTER(&htim5,0);
	
	rotate_speed[L_B]= (short)(__HAL_TIM_GET_COUNTER(&htim8)*speed_param);
	__HAL_TIM_SET_COUNTER(&htim8,0);
	
	rotate_speed[R_F]= (short)(__HAL_TIM_GET_COUNTER(&htim2)*speed_param);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	
	rotate_speed[R_B]= (short)(__HAL_TIM_GET_COUNTER(&htim4)*speed_param);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
}

void input_pidv(PID_variables *hpidv,int Target,int Data_now){
	
	if(Target>motor_rotate_speed){
		hpidv->target=motor_rotate_speed;
	
	}
	else if(Target<-motor_rotate_speed){
		hpidv->target=-motor_rotate_speed;
	}
	else {hpidv->target=Target;}
	hpidv->data_now=Data_now;
	hpidv->integral+=(Target-Data_now);
	if(Target<20&&Target>-20){hpidv->target=0;}
	if(hpidv->integral > 1000|| hpidv->integral < -1000||(hpidv->target<50&&hpidv->target>-50)){
		hpidv->integral=0;}
	
}

	
int pid_cal(PID_parameter *hpidp,PID_variables *hpidv){//--------------------------------pid_calculate
	int error,differential,pwm_control;
	error=hpidv->target - hpidv->data_now;
	differential = hpidv->data_now - hpidv->data_before;
	pwm_control = hpidp->kp*error- hpidp->kd*differential+ hpidp->ki*hpidv->integral;
	return pwm_control;
    }

void mani_cul(double x,double y,double sita){

    mani_status[mani_m1]=-(l2*l2-l1*l1-x*x-y*y)/(2*l1*sqrt(x*x+y*y));
    if(mani_status[mani_m1]>1||mani_status[mani_m1]<-1){
        m_error_flag   =   m1_fault;
    }
    else{
		if(y==0){mani_status[mani_v]=1.570796;}
		else{mani_status[mani_v]=atan(x/y);}
        mani_status[mani_sita1]=3.1415926-asin(mani_status[mani_m1])-mani_status[mani_v];  
        if(mani_status[mani_sita1]<=0&&mani_status[mani_sita1]!=-3.1415926){
            mani_status[mani_sita1]+=3.1415926;
                }
        if(mani_status[mani_sita1]==-3.1415926){
            mani_status[mani_sita1]=-mani_status[mani_sita1];
        }
        mani_status[mani_m2]=(l1*sin(mani_status[mani_sita1])-y)/l2;
        if       (mani_status[mani_m2]<1.0001&&mani_status[mani_m2]>=1)       {mani_status[mani_m2]=1;}
        else if(mani_status[mani_m2]>-1.0001&&mani_status[mani_m2]<=-1)       {mani_status[mani_m2]=-1;}
        if(mani_status[mani_m2]>1||mani_status[mani_m2]<-1){
                m_error_flag   =   m2_fault;
        }
        else{
                if(mani_status[mani_m2]==0){
                    if(l1*cos(mani_status[mani_sita1])-y>0){
                        mani_status[mani_n]=0;
                    }
                    else{
                        mani_status[mani_n]=3.141593;
                    }
                }
                
                else{
                    mani_status[mani_n]=asin(mani_status[mani_m2]);
                }
                

            mani_status[mani_sita2]=3.141593-mani_status[mani_n]-mani_status[mani_sita1];

            mani_status[mani_sita3]=sita-mani_status[mani_sita1]-mani_status[mani_sita2]+2*3.141593;

             m_error_flag   =  no_fault;
    
            }
    }
}
	   
void set_pidp(PID_parameter *hpidp,int KP, int KI ,int KD){
    hpidp->kp=KP;
    hpidp->ki=KI;
    hpidp->kd=KD;
		  }

void pid_protect(PID_parameter *hpidp,PID_variables *hpidv){
			
	if(hpidv->target<=80&&hpidv->target>=-80)   {hpidp->kp=10;hpidp->ki=0.1;hpidp->kd=0.1;}
	
}	
int abs(int a){
	
	if(a>0){return a;}
	else{return -a;}
	
}

int nominus(int a){
	
	if(a>0){return a;}
	else {return 0;}

}

//-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE--------END---------------------------------------
//--------------------------------------------------------------------------------------------------------------------------

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
