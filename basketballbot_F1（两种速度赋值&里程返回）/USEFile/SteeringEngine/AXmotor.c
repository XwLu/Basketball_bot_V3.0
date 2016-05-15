#include "AXmotor.h"

extern u8 pcneedLoad[2];

uint16_t axReadLoad(uint8_t bID) 
{
	unsigned int Load;
	gbpParameter[0] = P_PRESENT_LOAD_L ; //位置数据的起始地址 #define P_PRESENT_LOAD_L  (30) 参见数据手册
	gbpParameter[1] = 2; //读取长度
	TxPacket(bID, INST_READ, 2);
	if(RxPacket(DEFAULT_RETURN_PACKET_SIZE + gbpParameter[1]) != DEFAULT_RETURN_PACKET_SIZE + gbpParameter[1]){		 //?????????????
		Load = 0xffff;
	}
	else
	{
	    pcneedLoad[0]=gbpRxBuffer[6];
		pcneedLoad[1]=gbpRxBuffer[5];
		Load = ((unsigned int) gbpRxBuffer[6]) << 8;
		Load += gbpRxBuffer[5];
	}
	return Load;
}

void DigMotor_Move(uint8_t Act_Mode,uint8_t SpeedH,uint8_t SpeedL)
{
	uint16_t DigMotorSpeed,DigMotorNowPosi;
	unsigned delay_count; // wanguibin

	DigMotorSpeed= SpeedH*256+SpeedL;
	switch(Act_Mode)
	{
		case DigMotorOpen:
			axSendPosition(DigMotorNum1, 0x005b, DigMotorSpeed);
			axSendPosition(DigMotorNum2, 0x0039, DigMotorSpeed);
			for(delay_count = 0  ; delay_count <0xff;  delay_count++);		
			DigMotorNowPosi=axReadPosition(DigMotorNum1);
			USART_SendData(USART2,DigMotorNowPosi/256);//调试用的
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			USART_SendData(USART2,DigMotorNowPosi%256);//		 一样
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			DigMotorNowPosi=axReadPosition(DigMotorNum2);
			USART_SendData(USART2,DigMotorNowPosi/256);//调试用的
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			USART_SendData(USART2,DigMotorNowPosi%256);//		 一样
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
				/*	while(axReadPosition(DigMotorNum1)!=OpTagitPosi);*/
			break;

		case DigMotorClose:
			axSendPosition(DigMotorNum1, 0x0006, DigMotorSpeed);
			axSendPosition(DigMotorNum2, 0x0081, DigMotorSpeed);
			for(delay_count = 0  ; delay_count <0xff;  delay_count++);	
			DigMotorNowPosi=axReadPosition(DigMotorNum1);
			USART_SendData(USART2,DigMotorNowPosi/256);//调试用的
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			USART_SendData(USART2,DigMotorNowPosi%256);//		 一样
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			DigMotorNowPosi=axReadPosition(DigMotorNum2);
			USART_SendData(USART2,DigMotorNowPosi/256);//调试用的
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
			USART_SendData(USART2,DigMotorNowPosi%256);//		 一样
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
				/*	 while(axReadPosition(DigMotorNum1)!=ClTagitPosi)*/
			break;

		default :
			USART_SendData(USART2,0xbb);
			break;
			 }
}
