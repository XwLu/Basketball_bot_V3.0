#ifndef __usart2_h
#define __usart2_h
#include"stm32f10x.h"
#include <stdio.h>
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口2接收	
#define USART2_REC_LEN  			200  	//定义最大接收字节数 200
void uart2_init(u32 bound);

#endif
