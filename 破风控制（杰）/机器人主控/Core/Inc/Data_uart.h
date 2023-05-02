#ifndef DATAUART_H
#define DATAUART_H

#include "main.h"
#include "usart.h"

//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
//������,float->string
union float_string{  
        uint8_t float_array[4];  
        float f;  
};

//������,int->string
union int_string{  
        uint8_t int_array[4];  
        int in_t; 
};
extern uint8_t bluetooth_flag;
extern uint8_t uart_flag;                             //����ָʾ�Ƿ���յ���ͷ
extern uint8_t uart_mode;                                //ʶ������
extern uint8_t uart_receive_temp[7];                       //�ݴ�uart0���յ����ݣ���ȷ�Ͻ��յ���β����ȡ�� 
extern uint8_t uart_account;
extern uint8_t uart_datatosend[60];
extern uint8_t bluetooth_flag;

void Data_Handler(void);
void Check_Handler(void);//
void UART_SendData1(float datatosend);
void UART_SendData2(float datatosend1,float datatosend2);
void UART_SendData3(float datatosend1,float datatosend2,float datatosend3);
void UART_SendData4(float datatosend1,float datatosend2,float datatosend3,float datatosend4);
void UART_SendData5(float datatosend1,float datatosend2,float datatosend3,float datatosend4,float datatosend5);
void Data_Receive_Prepare(uint8_t data);				//����Ԥ����
void Data_Receive_Handler(uint8_t *data_buf,uint8_t num);	//�Խ��յ����ݴ���


#endif

