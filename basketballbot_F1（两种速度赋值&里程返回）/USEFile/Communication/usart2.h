#ifndef __usart2_h
#define __usart2_h
#include"stm32f10x.h"
#include <stdio.h>
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������2����	
#define USART2_REC_LEN  			200  	//�����������ֽ��� 200
void uart2_init(u32 bound);

#endif
