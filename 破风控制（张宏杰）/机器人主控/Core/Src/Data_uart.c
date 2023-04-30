#include "Data_uart.h"
#include "main.h"

/*************************************************************
//							匿名上位机
//				
//		使用uart1发送数据到匿名上位机
//		
**************************************************************/

uint8_t uart_flag=0x00;                             //用来指示是否接收到包头
uint8_t uart_mode=0;                                //识别命令
uint8_t uart_receive_temp[7];                       //暂存uart0接收的数据，待确认接收到包尾后再取出 
uint8_t uart_account=0x00;
uint8_t uart_datatosend[60];
uint8_t bluetooth_flag;

#define UART UART_1 //串口，使用需要初始化串口，调用uart_init(uart1,115200);

/*************************************************************
//							uart中断服务函数
//				
//		函数说明：为蓝牙接收功能
//
**************************************************************/

//void Data_uart_init()
//{
//	DMA_PORTx2BUFF_Init (DMA, (void *)&PTD_BYTE1_IN, temaddr, PTB21, DMA_BYTE1, 1, DMA_rising);   
//}





//void UART4_RX_TX_IRQHandler()
//{
//	uint8_t temp;
//	if((UART4->S1 & UART_S1_RDRF_MASK)!= 0)		//等待接收满了
//	{
//		//while (!(UART_S1_REG(UARTN[UART_4]) & UART_S1_RDRF_MASK));       //等待接收满了
//		temp = UART4->D ;
//		Data_Receive_Prepare(temp);
//	}
//}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数

void Data_Receive_Prepare(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
		{
			state = 5;
		}
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Data_Receive_Handler(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void Data_Receive_Handler(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
	{
		sum += *(data_buf+i);
	}
	if(!(sum==*(data_buf+num-1)))		
		return;		//判断sum，错误则返回
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
		return;		//判断帧头，错误则返回
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)		//对应匿名上位机中的“读取飞控”
		{
					//开启电机
		}
		if(*(data_buf+4)==0X02)		//对应匿名上位机中的“写入飞控”
		{
			
		}
		if(*(data_buf+4)==0XA1)		//对应匿名上位机中的“恢复默认值”
		{
					//关闭电机
		}
	}
}



void UART_SendData1(float datatosend)
{
	uint8_t _cnt=0;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xF1;
	uart_datatosend[_cnt++]=4;
	
	uart_datatosend[_cnt++]=BYTE3(datatosend);
	uart_datatosend[_cnt++]=BYTE2(datatosend);
	uart_datatosend[_cnt++]=BYTE1(datatosend);
	uart_datatosend[_cnt++]=BYTE0(datatosend);
		
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += uart_datatosend[i];
	uart_datatosend[_cnt++]=sum;		//校验数据位

	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
  
}
void UART_SendData2(float datatosend1,float datatosend2)
{
	uint8_t _cnt=0;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xF1;
	uart_datatosend[_cnt++]=8;
	
	uart_datatosend[_cnt++]=BYTE3(datatosend1);
	uart_datatosend[_cnt++]=BYTE2(datatosend1);
	uart_datatosend[_cnt++]=BYTE1(datatosend1);
	uart_datatosend[_cnt++]=BYTE0(datatosend1);	
	uart_datatosend[_cnt++]=BYTE3(datatosend2);
	uart_datatosend[_cnt++]=BYTE2(datatosend2);
	uart_datatosend[_cnt++]=BYTE1(datatosend2);
	uart_datatosend[_cnt++]=BYTE0(datatosend2);
		
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += uart_datatosend[i];
	uart_datatosend[_cnt++]=sum;		//校验数据位
	
	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
}

void UART_SendData3(float datatosend1,float datatosend2,float datatosend3)
{
	uint8_t _cnt=0;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xF1;
	uart_datatosend[_cnt++]=12;
	
	uart_datatosend[_cnt++]=BYTE3(datatosend1);
	uart_datatosend[_cnt++]=BYTE2(datatosend1);
	uart_datatosend[_cnt++]=BYTE1(datatosend1);
	uart_datatosend[_cnt++]=BYTE0(datatosend1);	
	uart_datatosend[_cnt++]=BYTE3(datatosend2);
	uart_datatosend[_cnt++]=BYTE2(datatosend2);
	uart_datatosend[_cnt++]=BYTE1(datatosend2);
	uart_datatosend[_cnt++]=BYTE0(datatosend2);	
	uart_datatosend[_cnt++]=BYTE3(datatosend3);
	uart_datatosend[_cnt++]=BYTE2(datatosend3);
	uart_datatosend[_cnt++]=BYTE1(datatosend3);
	uart_datatosend[_cnt++]=BYTE0(datatosend3);
		
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += uart_datatosend[i];
	uart_datatosend[_cnt++]=sum;		//校验数据位

	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
}
void UART_SendData4(float datatosend1,float datatosend2,float datatosend3,float datatosend4)
{
	uint8_t _cnt=0;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xAA;
	uart_datatosend[_cnt++]=0xF1;
	uart_datatosend[_cnt++]=16;
	
	uart_datatosend[_cnt++]=BYTE3(datatosend1);
	uart_datatosend[_cnt++]=BYTE2(datatosend1);
	uart_datatosend[_cnt++]=BYTE1(datatosend1);
	uart_datatosend[_cnt++]=BYTE0(datatosend1);	
	uart_datatosend[_cnt++]=BYTE3(datatosend2);
	uart_datatosend[_cnt++]=BYTE2(datatosend2);
	uart_datatosend[_cnt++]=BYTE1(datatosend2);
	uart_datatosend[_cnt++]=BYTE0(datatosend2);
		
	uart_datatosend[_cnt++]=BYTE3(datatosend3);
	uart_datatosend[_cnt++]=BYTE2(datatosend3);
	uart_datatosend[_cnt++]=BYTE1(datatosend3);
	uart_datatosend[_cnt++]=BYTE0(datatosend3);
		
	uart_datatosend[_cnt++]=BYTE3(datatosend4);
	uart_datatosend[_cnt++]=BYTE2(datatosend4);
	uart_datatosend[_cnt++]=BYTE1(datatosend4);
	uart_datatosend[_cnt++]=BYTE0(datatosend4);
		
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += uart_datatosend[i];
	uart_datatosend[_cnt++]=sum;		//校验数据位
	
	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
	
}



