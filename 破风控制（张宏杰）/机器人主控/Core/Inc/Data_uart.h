#ifndef DATAUART_H
#define DATAUART_H

#include "main.h"
#include "usart.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
//联合体,float->string
union float_string{  
        uint8_t float_array[4];  
        float f;  
};

//联合体,int->string
union int_string{  
        uint8_t int_array[4];  
        int in_t; 
};
extern uint8_t bluetooth_flag;
extern uint8_t uart_flag;                             //用来指示是否接收到包头
extern uint8_t uart_mode;                                //识别命令
extern uint8_t uart_receive_temp[7];                       //暂存uart0接收的数据，待确认接收到包尾后再取出 
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
void Data_Receive_Prepare(uint8_t data);				//接收预处理
void Data_Receive_Handler(uint8_t *data_buf,uint8_t num);	//对接收的数据处理


#endif

