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
double           target[arspace]={0};
double        mani_status[mani_arrspace]={0};
int           error_status[fault_Space]={0};
int           o_limit[10]={0};
PID_parameter drive_pidp_LF;
PID_parameter drive_pidp_LB;
PID_parameter drive_pidp_RF;
PID_parameter drive_pidp_RB;

PID_variables drive_pidv_L_F;
PID_variables drive_pidv_L_B;
PID_variables drive_pidv_R_F;
PID_variables drive_pidv_R_B;


unsigned char temp;
unsigned char data_L_X=0, data_L_Y=0,data_R_X=0,data_R_Y=0;
unsigned int i=0;
         int motor_rotate_speed=0 , target_cal=0;
unsigned int mani_prescaler=0;
unsigned int mani_lock_flag=1023;
unsigned int mani_tlock_flag=0;

int temp_X,temp_Y,temp_K,temp_A;
int m_reset_flag = 0;
//int m_error_flag = 0;

float a=0;

int w=0;//错误急停判断debug标志位
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
void pid_autoset(PID_parameter *hpidp,PID_variables *hpidv);
int ABS(int a);
int nominus(int a);
int pwm_limit(int pwm);
void mani_cul(double x,double y,double sita);
void delay_us(uint32_t nus);

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
	OLED_DrawBMP(0, 0,63, 7,gImage_windbreaker);//破风的logo
	HAL_Delay(1000);
	
   
   NRF24L01_Init();
//	printf("\r\ninit OK!\r\n");
//	temp = 4;
	while(NRF24L01_Check()){
		OLED_show_string(2, 0, "NRF24L01_test_fail");//无线通讯模块自检，并用oled显示
	}
	OLED_operate_gram(PEN_CLEAR);
	OLED_refresh_gram();
	OLED_show_string(2, 0, "NRF24L01_test_succeeded");
    
	RX_Mode();	//接收模式
	
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);	    // TIM1(pwm)
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_2);	    // TIM1(pwm)
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_1);	    // TIM1(pwm)
	HAL_TIM_PWM_Start_IT(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim11,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim12,TIM_CHANNEL_1);	    // TIM1(pwm)
	HAL_TIM_PWM_Start_IT(&htim12,TIM_CHANNEL_2);

	
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1); // 开启编码器
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
//	  controller_signal[OPTION]=2;
//	controller_signal[MODE]=0;
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  //-------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------USER-----CODE------BEGIN---WHILE----------------------------------
//--------------------------------------------------------------------------------------------------------------------------
	  	if(NRF24L01_RxPacket(controller_signal)==0){//提取信号
		data_L_X=controller_signal[DATA_L_X];
	    data_L_Y=controller_signal[DATA_L_Y];
		data_R_X=controller_signal[DATA_R_X];
	    data_R_Y=controller_signal[DATA_R_Y];
		}
		else{//没有收到信号时要输出0
			data_L_X=24;
			data_L_Y=24;
			data_R_X=23;
			data_R_Y=25;
			
		}
		if(controller_signal[MODE]==MOVI){//运动模式下的显示
			OLED_printf(0,1,"dLX=%d-dLY=%d",data_L_X,data_L_Y);
			OLED_printf(1,1,"dRX=%d-dRY=%d",data_R_X,data_R_Y);
			OLED_printf(2,1,"LFS=%d-RFS=%d",motor_speed[L_F],motor_speed[R_F]);
			OLED_printf(3,1,"LBS=%d-RBS=%d",motor_speed[L_B],motor_speed[R_B]);
			
  }
//		oled最后都注释一下
		else if(controller_signal[MODE]==MANI){//机械臂模式下的显示
			
			OLED_printf(0,1,"p=%u",(uint16_t)(mani_status[mani_sita1]*2000/3.141593+500));
			OLED_printf(1,1,"dx:%lf-dy:%lf",mani_status[mani_dx_target],mani_status[mani_dy_target]);
			if(error_status[m1_fault]==1)           {OLED_printf(2,0,"error_m1=%lf", mani_status[mani_m1]);}
			else if(error_status[m2_fault]==1)      {OLED_printf(2,0,"error_m2=%lf", mani_status[mani_m2]);}
			else{	
				OLED_printf(2,0,"s1=%lf-s2=%lf",mani_status[mani_sita1],mani_status[mani_sita2]);
				OLED_printf(3,0,"s3=%lf-S=%d",mani_status[mani_sita3],target[M_A]);//controller_signal[JAW]//target[M_A]
			}
			OLED_printf(4,1,"X=%lf-Y=%lf",target[M_X],target[M_Y]);
		}
		HAL_Delay(3);
		//DELAY_US(1);
  
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
			m_reset_flag=0;//机械臂模式初始化标志复位
			no_fault_happended//机械臂模式的清除错误标记
			if(controller_signal[OPTION]==2){//速度选择
				motor_rotate_speed=500;
//				set_pidp(&drive_pidp_LF,50,0.2 ,5);
//				set_pidp(&drive_pidp_LB,50,0.2 ,5);
//				set_pidp(&drive_pidp_RF,50,0.2 ,5);
//				set_pidp(&drive_pidp_RB,50,0.2 ,5);
			}
			else if (controller_signal[OPTION]==1){
				motor_rotate_speed=300;
//				set_pidp(&drive_pidp_LF,20,0.1 ,1);
//				set_pidp(&drive_pidp_LB,20,0.1 ,1);
//				set_pidp(&drive_pidp_RF,20,0.1 ,1);
//				set_pidp(&drive_pidp_RB,20,0.1 ,1);
			}
			else if(controller_signal[OPTION]==0){
				motor_rotate_speed=100;
//				set_pidp(&drive_pidp_LF,10,0.001 ,0.1);
//				set_pidp(&drive_pidp_LB,10,0.001 ,0.1);
//				set_pidp(&drive_pidp_RF,10,0.001 ,0.1);
//				set_pidp(&drive_pidp_RB,10,0.001 ,0.1);
			}
			else{
				motor_rotate_speed=0;
			}
			target_cal=motor_rotate_speed/25;
			//---------------------------------------------------------麦轮运动解算
			target[L_F]=target_cal*((int)data_L_X-24)+target_cal*((int)data_R_Y-25)/2;
			target[L_B]=target_cal*((int)data_L_Y-24)+target_cal*((int)data_R_Y-25)/2;
			target[R_F]=target_cal*((int)data_L_Y-24)-target_cal*((int)data_R_Y-25)/2;
			target[R_B]=target_cal*((int)data_L_X-24)-target_cal*((int)data_R_Y-25)/2;

			get_speed(motor_speed);//获取速度并放入数组
			
//-----------------------------------------------------------
			//------------------------pid计算
			input_pidv(&drive_pidv_L_F,(int)target[L_F],(int)motor_speed[L_F]);
			pid_autoset(&drive_pidp_LF,&drive_pidv_L_F);
			pwm_output[L_F]= pwm_limit(pid_cal(&drive_pidp_LF,&drive_pidv_L_F)); //LF
			drive_pidv_L_F.data_before=drive_pidv_L_F.data_now;
			drive_pidv_L_F.last_target=drive_pidv_L_F.target;
			
			input_pidv(&drive_pidv_R_F,(int)target[R_F],(int)motor_speed[R_F]);
			pid_autoset(&drive_pidp_RF,&drive_pidv_R_F);
			pwm_output[R_F]=pwm_limit( pid_cal(&drive_pidp_RF,&drive_pidv_R_F)); //RF
			drive_pidv_R_F.data_before=drive_pidv_R_F.data_now;
			drive_pidv_R_F.last_target=drive_pidv_R_F.target;
			
			input_pidv(&drive_pidv_L_B,(int)target[L_B],(int)motor_speed[L_B]);
			pid_autoset(&drive_pidp_LB,&drive_pidv_L_B);
			pwm_output[L_B]=pwm_limit(pid_cal(&drive_pidp_LB,&drive_pidv_L_B)); //LB
			drive_pidv_L_B.data_before=drive_pidv_L_B.data_now;
			drive_pidv_L_B.last_target=drive_pidv_L_B.target;
			
			input_pidv(&drive_pidv_R_B,(int)target[R_B],(int)motor_speed[R_B]);
			pid_autoset(&drive_pidp_RB,&drive_pidv_R_B);		
			pwm_output[R_B]=pwm_limit(pid_cal(&drive_pidp_RB,&drive_pidv_R_B)); //RB
			drive_pidv_R_B.data_before=drive_pidv_R_B.data_now;
			drive_pidv_R_B.last_target=drive_pidv_R_B.target;
//------------------------------------------------------------------------------------------
			PWM_output(pwm_output);//输出
			
			
			UART_SendData4(motor_speed[L_F],motor_speed[L_B],motor_speed[R_F],motor_speed[R_B]);//串口发送
			
		}
		else if(controller_signal[MODE]==MANI){//-------------------------manipulator
				temp_X=4*((int)data_L_X-24);
				temp_Y=4*((int)data_L_Y-24);
				temp_A=4*((int)data_R_X-23);
				temp_K=4*((int)data_R_Y-25);
				if(m_reset_flag ==0){//位置的初始化
					target[M_X]=200;
					target[M_Y]=100;
					target[M_A]=0  ;
					target[M_K]=0  ;
				mani_status[mani_L_X]=0;
				mani_status[mani_L_Y]=0;
				mani_status[mani_L_A]=0;
					m_reset_flag=1;//标志上拉
				}
				//-----------摇杆控制增减力度------
				//--------------------------------------------------x
				
				if(temp_X>90)                   {target[M_X]+=1  ;x_last_plus;}//记录行为
				else if(temp_X<=90&&temp_X>50)  {target[M_X]+=0.8  ;x_last_plus;}
				else if(temp_X<=50&&temp_X>10)  {target[M_X]+=0.5;x_last_plus;}
				else if(temp_X<-10&&temp_X>=-50){target[M_X]-=0.5;x_last_minus;}
				else if(temp_X<-50&&temp_X>=90) {target[M_X]-=0.8 ;x_last_minus;}
				else if(temp_X<-90)             {target[M_X]-=1  ;x_last_minus;}
					
				//---------------------------------------------------y
				
				if(temp_Y>90)                   {target[M_Y]+=1  ;y_last_plus;}
				else if(temp_Y<=90&&temp_Y>50)  {target[M_Y]+=0.8  ;y_last_plus;}
				else if(temp_Y<=50&&temp_Y>10)  {target[M_Y]+=0.5;y_last_plus;}
				else if(temp_Y<-10&&temp_Y>=-50){target[M_Y]-=0.5;y_last_minus;}
				else if(temp_Y<-50&&temp_Y>=90) {target[M_Y]-0.8  ;y_last_minus;}
				else if(temp_Y<-90)				{target[M_Y]-=1  ;y_last_minus;}

				//---------------------------------------------------k
				
				if(temp_K>90)                   {target[M_K]=70;}
				else if(temp_K<=90&&temp_K>50)  {target[M_K]=50;}
				else if(temp_K<=50&&temp_K>10)  {target[M_K]=40;}
				else if(temp_K<-10&&temp_K>=-50){target[M_K]=-40;}
				else if(temp_K<-50&&temp_K>=90) {target[M_K]=-50;}
				else if(temp_K<-90)             {target[M_K]=-70;}
				else  							{target[M_K]=0;}
				
				//----------------------------------------------------a
				
				if(temp_A>90)                   {target[M_A]+=0.8;a_last_plus;}
				else if(temp_A<=90&&temp_A>50)  {target[M_A]+=0.4;a_last_plus;}
				else if(temp_A<=50&&temp_A>10)  {target[M_A]+=0.1;a_last_plus;}
				else if(temp_A<-10&&temp_A>=-50){target[M_A]-=0.1;a_last_minus;}
				else if(temp_A<-50&&temp_A>=90) {target[M_A]-=0.4;a_last_minus;}
				else if(temp_A<-90)             {target[M_A]-=0.8;a_last_minus;}//0-180
				
				//---------------------------------------------------------
				//---------------------------------------------	//限制&错误急停(判断错误原因与动作并 节源)
					if(target[M_Y]<=0){target[M_Y]=0;}
					
					if((if_error_x_plus && if_x_plus)||(if_error_x_minus && if_x_minus)||target[M_Y]<=0)
						{target[M_X]=mani_status[mani_L_X];w++;}
						
					if((if_error_y_plus && if_y_plus)||(if_error_y_minus && if_y_minus))
						{target[M_Y]=mani_status[mani_L_Y];}
						
					if((if_error_a_plus && if_a_plus)||(if_error_a_minus && if_a_minus))
						{target[M_A]=mani_status[mani_L_A];}
				//------------------------
				//----------------------------------------------------------	
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];//记录上一次的信号
					
//				if(controller_signal[OPTION]==1&&o_limit[0]){//prot数组标记
//				target[M_X]=200 ;
//				target[M_Y]=100 ;
//				target[M_A]=0   ;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[0]=1;
//				}
//				else if(controller_signal[OPTION]==2&&o_limit[1]){//up
//				target[M_X]=300;
//				target[M_Y]=300;
//				target[M_A]=90 ;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[1]=1;
//				}
//				else if(controller_signal[OPTION]==3&&o_limit[2]){//mid
//				target[M_X]=50;
//				target[M_Y]=150;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[2]=1;
//				}
//				else if(controller_signal[OPTION]==4&&o_limit[3]){//down
//				target[M_X]=60;
//				target[M_Y]=60;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[3]=1;
//				}
//				else if(controller_signal[OPTION]==5&&o_limit[4]){//tou
//				target[M_X]=70;
//				target[M_Y]=70;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[4]=1;
//				}
//				else if(controller_signal[OPTION]==6&&o_limit[5]){//collect
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[5]=1;
//					
//				}
//				else if(controller_signal[OPTION]==7&&o_limit[6]){
//				target[M_X]=80;
//				target[M_Y]=80;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[6]=1;
//				}
//				else if(controller_signal[OPTION]==8&&o_limit[7]){
//				target[M_X]=90;
//				target[M_Y]=90;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[7]=1;
//				}
//				else if(controller_signal[OPTION]==9&&o_limit[8]){
//				target[M_X]=100;
//				target[M_Y]=100;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[8]=1;
//				}
//				else if(controller_signal[OPTION]==10&&o_limit[9]){
//				target[M_X]=200;
//				target[M_Y]=200;
//				target[M_A]=0;
//				for(int i=0;i<10;i++){o_limit[i]=0;}
//				o_limit[9]=1;
//				}
//---------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------机械臂点位（二进制标记）
				if(controller_signal[OPTION]==1&&(mani_lock_flag&1)){//up
				target[M_X]=215 ;
				target[M_Y]=340 ;
				target[M_A]=40   ;
				mani_lock_flag=1022;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended//清除错误状态标记
				}
				else if(controller_signal[OPTION]==2&&(mani_lock_flag&2)){//prot
				target[M_X]=252;
				target[M_Y]=169;
				target[M_A]=-90 ;
				mani_lock_flag=1021;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==3&&(mani_lock_flag&4)){//mid
				target[M_X]=216.3;
				target[M_Y]=262.9;
				target[M_A]=-90;
				mani_lock_flag=1019;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==4&&(mani_lock_flag&8)){//
				target[M_X]=377.7;
				target[M_Y]=62.7;
				target[M_A]=30;
				mani_lock_flag=1015;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==5&&(mani_lock_flag&16)){//place
				target[M_X]=130;
				target[M_Y]=310.9;
				target[M_A]=-90;
				mani_lock_flag=1007;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==6&&(mani_lock_flag&32)){//
				target[M_X]=305.2;
				target[M_Y]=235;
				target[M_A]=60;
				mani_lock_flag=991;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==7&&(mani_lock_flag&64)){//
				target[M_X]=200;
				target[M_Y]=100;
				target[M_A]=0;
				mani_lock_flag=959;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended
				}
				else if(controller_signal[OPTION]==8&&(mani_lock_flag&128)){//
				target[M_X]=216.3;
				target[M_Y]=262.9;
				target[M_A]=-90;
				mani_lock_flag=895;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended	
				}
				else if(controller_signal[OPTION]==9&&(mani_lock_flag&256)){//
				target[M_X]=200;
				target[M_Y]=100;
				target[M_A]=0;
				mani_lock_flag=767;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended	
				}
				else if(controller_signal[OPTION]==10&&(mani_lock_flag&512)){//place
				target[M_X]=216.3;
				target[M_Y]=262.9;
				target[M_A]=-90;
				mani_lock_flag=511;
				mani_status[mani_L_X]=target[M_X];
				mani_status[mani_L_Y]=target[M_Y];
				mani_status[mani_L_A]=target[M_A];
				no_fault_happended	
				}
				//-----------------------------------------------------------
	//			target[M_X]=200;
	//				
	//			target[M_Y]=200;//test
	//			
	//			target[M_A]=-90;
				mani_cul(target[M_X],target[M_Y],target[M_A]);
				
				get_speed(motor_speed);
				input_pidv(&drive_pidv_L_F,(int)target[M_K],(int)motor_speed[L_F]);
				pid_autoset(&drive_pidp_LF,&drive_pidv_L_F);
				pwm_output[L_F]= pwm_limit(pid_cal(&drive_pidp_LF,&drive_pidv_L_F)); //LF
				drive_pidv_L_F.data_before=drive_pidv_L_F.data_now;
				drive_pidv_L_F.last_target=drive_pidv_L_F.target;
				
				input_pidv(&drive_pidv_R_F,-(int)target[M_K],(int)motor_speed[R_F]);
				pid_autoset(&drive_pidp_RF,&drive_pidv_R_F);
				pwm_output[R_F]=pwm_limit( pid_cal(&drive_pidp_RF,&drive_pidv_R_F)); //RF
				drive_pidv_R_F.data_before=drive_pidv_R_F.data_now;
				drive_pidv_R_F.last_target=drive_pidv_R_F.target;
				
				input_pidv(&drive_pidv_L_B,-(int)target[M_K],(int)motor_speed[L_B]);
				pid_autoset(&drive_pidp_LB,&drive_pidv_L_B);
				pwm_output[L_B]=pwm_limit(pid_cal(&drive_pidp_LB,&drive_pidv_L_B)); //LB
				drive_pidv_L_B.data_before=drive_pidv_L_B.data_now;
				drive_pidv_L_B.last_target=drive_pidv_L_B.target;
				
				input_pidv(&drive_pidv_R_B,(int)target[M_K],(int)motor_speed[R_B]);
				pid_autoset(&drive_pidp_RB,&drive_pidv_R_B);		
				pwm_output[R_B]=pwm_limit(pid_cal(&drive_pidp_RB,&drive_pidv_R_B)); //RB
				drive_pidv_R_B.data_before=drive_pidv_R_B.data_now;
				drive_pidv_R_B.last_target=drive_pidv_R_B.target;
				
				PWM_output(pwm_output);
				//-------------------------一个简单的低通滤波
				pwm_output[M_S1]=(uint16_t)((P*mani_status[mani_sita1]+Q*mani_status[mani_L_sita1])*2000/3.141593+500);
				pwm_output[M_S2]=(uint16_t)((P*mani_status[mani_sita2]+Q*mani_status[mani_L_sita2])*2000/3.141593+500);
				pwm_output[M_S3]=(uint16_t)((P*mani_status[mani_sita3]+Q*mani_status[mani_L_sita3])*2000/3.141593+500);
				
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,pwm_output[M_S1]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,pwm_output[M_S2]);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,pwm_output[M_S3]);
				
////				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,(uint16_t)(mani_status[mani_sita1]*2000/3.141593+500));
////				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,(uint16_t)((3.141593-mani_status[mani_sita2])*2000/3.141593+500));
////				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,(uint16_t)(mani_status[mani_sita3]*2000/3.141593+500));
				//(uint16_t)(mani_status[mani_sita1]*2000/3.141593+500)
//				mani_prescaler=0;
				mani_status[mani_L_sita1]=mani_status[mani_sita1];
				mani_status[mani_L_sita2]=mani_status[mani_sita2];
				mani_status[mani_L_sita3]=mani_status[mani_sita3];

		}
		
//	if(controller_signal[JAW])     {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,500);}
//	else                           {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1,500);}//未测试具体数值
	
//	if(controller_signal[JAW])     {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,2000);}
//	else                           {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,2500);}//未测试具体数值
//	
//	if(controller_signal[JAW])     {__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,2000);}
//	else                           {__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,2500);}//未测试具体数值
//	
//	if(controller_signal[JAW])     {__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,500);}
//	else                           {__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1,500);}//未测试具体数值
	
	if(controller_signal[JAW])     {__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,2500);}
	else                           {__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,900);}//未测试具体数值
	
	
//	if(controller_signal[LIFT])    {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2,2500);}
//	else                           {__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 500);}//未测试具体数值
	
	
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
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint16_t)(ABS(pwm[L_F])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint16_t)(ABS(pwm[R_F])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint16_t)(ABS(pwm[L_B])));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,(uint16_t)(ABS(pwm[R_B])));

}
int pwm_limit(int pwm){//pwm限制
	
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

void input_pidv(PID_variables *hpidv,int Target,int Data_now){//pid变量输入
	
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
	if(hpidv->integral > 1000|| hpidv->integral <-1000||(hpidv->target<50&&hpidv->target>-50)){
		hpidv->integral=0;}
	
}

	
int pid_cal(PID_parameter *hpidp,PID_variables *hpidv){//--------------------------------pid_calculate
	int error,differential,pwm_control;
	error=hpidv->target - hpidv->data_now;
	differential = hpidv->data_now - hpidv->data_before;
	pwm_control = hpidp->kp*error- hpidp->kd*differential+ hpidp->ki*hpidv->integral;
	return pwm_control;
    }

void mani_cul(double x,double y,double sita){//运动学解算部分

    mani_status[mani_m1]=-(l2*l2-l1*l1-x*x-y*y)/(2*l1*sqrt(x*x+y*y));
    if(mani_status[mani_m1]>1||mani_status[mani_m1]<-1){
        error_m1;
		if(if_x_plus)      {error_x_plus;}
		else if(if_x_minus){error_x_minus;}
		if(if_y_plus)      {error_y_plus;}
		else if(if_y_minus){error_y_minus;}//返回错误信号和错误的状态和原因
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
			error_m2;
			if(if_x_plus)      {error_x_plus;}
			else if(if_x_minus){error_x_minus;}//同
			if(if_y_plus)      {error_y_plus;}
			else if(if_y_minus){error_y_minus;}
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
                
            // printf("%lf\n",k2);
            mani_status[mani_sita2]=3.141593-mani_status[mani_n]-mani_status[mani_sita1];
            // if(sita2<=0&&sita2!=-3.141593){
            //     sita2+=3.141593;
            // }
            // if(sita2==-3.141593){
            //     sita2=-sita2;
            // }
			
            mani_status[mani_sita3]=3*3.141593/2+sita*3.141593/180-mani_status[mani_sita1]-mani_status[mani_sita2];
			if(mani_status[mani_sita3]<0){
				error_a_minus;
				mani_status[mani_sita3]=mani_status[mani_L_sita3];
				}
			else if(mani_status[mani_sita3]>3.141593){
				error_a_plus;
				mani_status[mani_sita3]=mani_status[mani_L_sita3];
			}
            // printf("%lf\n",asin(-0.6));
            no_fault_happended
			
			
            }
    }
	
	//------------------------------------------------------------------
	
	
	
}
	   
void set_pidp(PID_parameter *hpidp,int KP, int KI ,int KD){
    hpidp->kp=KP;
    hpidp->ki=KI;
    hpidp->kd=KD;
		  }

void pid_autoset(PID_parameter *hpidp,PID_variables *hpidv){
//	hpidp->kp=20;hpidp->ki=0;hpidp->kd=4;
	if(ABS(hpidv->target)>450)                                  {hpidp->kp=20;hpidp->ki=0.01  ;hpidp->kd=5  ;}
	if(ABS(hpidv->target)>350&&ABS(hpidv->target)<=450)         {hpidp->kp=17;hpidp->ki=0.008 ;hpidp->kd=4  ;}
	if(ABS(hpidv->target)>250&&ABS(hpidv->target)<=350)         {hpidp->kp=14;hpidp->ki=0.004 ;hpidp->kd=3  ;}
	if(ABS(hpidv->target)>150&&ABS(hpidv->target)<=250)         {hpidp->kp=11;hpidp->ki=0.002 ;hpidp->kd=2  ;}
	if(ABS(hpidv->target)>80&&ABS(hpidv->target)<=150)          {hpidp->kp=8 ;hpidp->ki=0.001 ;hpidp->kd=1  ;}
	if(ABS(hpidv->target)>40&&ABS(hpidv->target)<=80)           {hpidp->kp=8 ;hpidp-> ki=0.0005;hpidp->kd=0.5;}
	if(hpidv->target<=40&&hpidv->target>=-40)                   {hpidp->kp=8 ;hpidp-> ki=0.0001;hpidp->kd=0.2;}
	
}	
int ABS(int a){//绝对值
	
	if(a>0){return a;}
	else{return -a;}
	
}

int nominus(int a){//非负，应该已经用不上了
	
	if(a>0){return a;}
	else {return 0;}

}
void delay_us(uint32_t nus)//毫秒延迟
{
  uint32_t temp;
  SysTick->LOAD = HAL_RCC_GetHCLKFreq()/1000000/8*nus;
  SysTick->VAL=0X00;//清空计数器
  SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
  do
  {
    temp=SysTick->CTRL;//读取当前倒计数值
  }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
  SysTick->CTRL=0x00; //关闭计数器
  SysTick->VAL =0X00; //清空计数器
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
