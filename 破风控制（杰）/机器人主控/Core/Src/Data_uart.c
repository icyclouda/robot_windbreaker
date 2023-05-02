#include "Data_uart.h"
#include "main.h"

/*************************************************************
//							������λ��
//				
//		ʹ��uart1�������ݵ�������λ��
//		
**************************************************************/

uint8_t uart_flag=0x00;                             //����ָʾ�Ƿ���յ���ͷ
uint8_t uart_mode=0;                                //ʶ������
uint8_t uart_receive_temp[7];                       //�ݴ�uart0���յ����ݣ���ȷ�Ͻ��յ���β����ȡ�� 
uint8_t uart_account=0x00;
uint8_t uart_datatosend[60];
uint8_t bluetooth_flag;

#define UART UART_1 //���ڣ�ʹ����Ҫ��ʼ�����ڣ�����uart_init(uart1,115200);

/*************************************************************
//							uart�жϷ�����
//				
//		����˵����Ϊ�������չ���
//
**************************************************************/

//void Data_uart_init()
//{
//	DMA_PORTx2BUFF_Init (DMA, (void *)&PTD_BYTE1_IN, temaddr, PTB21, DMA_BYTE1, 1, DMA_rising);   
//}





//void UART4_RX_TX_IRQHandler()
//{
//	uint8_t temp;
//	if((UART4->S1 & UART_S1_RDRF_MASK)!= 0)		//�ȴ���������
//	{
//		//while (!(UART_S1_REG(UARTN[UART_4]) & UART_S1_RDRF_MASK));       //�ȴ���������
//		temp = UART4->D ;
//		Data_Receive_Prepare(temp);
//	}
//}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������

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
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void Data_Receive_Handler(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
	{
		sum += *(data_buf+i);
	}
	if(!(sum==*(data_buf+num-1)))		
		return;		//�ж�sum�������򷵻�
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		
		return;		//�ж�֡ͷ�������򷵻�
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)		//��Ӧ������λ���еġ���ȡ�ɿء�
		{
					//�������
		}
		if(*(data_buf+4)==0X02)		//��Ӧ������λ���еġ�д��ɿء�
		{
			
		}
		if(*(data_buf+4)==0XA1)		//��Ӧ������λ���еġ��ָ�Ĭ��ֵ��
		{
					//�رյ��
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
	uart_datatosend[_cnt++]=sum;		//У������λ

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
	uart_datatosend[_cnt++]=sum;		//У������λ
	
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
	uart_datatosend[_cnt++]=sum;		//У������λ

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
	uart_datatosend[_cnt++]=sum;		//У������λ
	
	HAL_UART_Transmit_IT(&huart1, uart_datatosend, _cnt);
	
}



