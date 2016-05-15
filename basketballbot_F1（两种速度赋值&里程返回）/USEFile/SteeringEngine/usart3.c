#include"stm32f10x.h"
#include "usart3.h"
#include "usart2.h"
#include "led.h"
#include "AX.h"

/*数字舵机的有关变量*/
//extern volatile uint8_t gbpParameter[];
//extern volatile uint8_t gbpRxBuffer[];
extern volatile unsigned char gbpRxInterruptBuffer[];
extern volatile unsigned char gbRxBufferWritePointer;

static void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
  
   /* Set the Vector Table base location at 0x08000000 */
   NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
  
   /* Configure the NVIC Preemption Priority Bits */  
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
   /* Enable the USART1 Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;       //通道设置为串口1中断
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		   //打开中断

   NVIC_Init(&NVIC_InitStructure); 						   //初始化
}



static void USART3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	  //  下位机与数字舵机通信
	

	//--------------------------------------------------------------PB12、PB13是控制串口3收发模式的管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
   //--------------------------------------------------------------------------------------------------------------

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
	USART_InitStructure.USART_BaudRate =1000000;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_Cmd(USART3, ENABLE);
 
}

void Uart3_PutChar(u8 ch)
{
  USART_SendData(USART3, (u8) ch);
  while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void axu3init(void)
{
  NVIC_Configuration();
  USART3_Configuration();
}


void USART3_IRQHandler(void) 
{												   		 //串口3用来直接与舵机通信，该中断用来读取舵机信息
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		u8 dat;	
		USART_ClearITPendingBit(USART3,USART_IT_RXNE); //清除中断标志.	    
		dat=USART_ReceiveData(USART3);
		gbpRxInterruptBuffer[(gbRxBufferWritePointer++)] = dat;
				
	}

}
