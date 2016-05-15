#include"stm32f10x.h"	
#include "bsp.h"
extern int flag;

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;


 
void RCC_Init(void)
{  	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);	   //����Ҫ��һ��������ϸ��ù���
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE ); 	  	//PC ���������Ƶ������ת ����һ��PC0 1���Ƶ��1��  2 3 ���Ƶ��2��  4 5 ���Ƶ��3��  6 7���Ƶ��4
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

}


void GPIO_init(void)
{
    
	//LED����ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);		  
	GPIO_SetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
//����ս���������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);		  
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	//---------------------------------------------------------------TIM2 CHI1 CHI2---PA0 PA1;
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//---------------------------------------------------------------TIM3 CHI1 CHI2---PA6 PA7;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

   //---------------------------------------------------------------TIM4 CHI1 CHI2---PB6 PB7;   
 	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);


    //����1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOA , &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init( GPIOA , &GPIO_InitStructure);

 
    //PC ���������Ƶ������ת ����һ�� PC0 1���Ƶ��1��  2 3 ���Ƶ��2��  4 5 ���Ƶ��3��  6 7���Ƶ��4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_3;	 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_ResetBits(GPIOB,GPIO_Pin_8);	rttrrtrtttttttttttttttttr

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	 
	GPIO_Init(GPIOA, &GPIO_InitStructure);   
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);	
		
//	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_8 |GPIO_Pin_9|GPIO_Pin_10 |GPIO_Pin_11;		
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		    //*************************TIM1�����������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOA, &GPIO_InitStructure);


    //*************************TIM8�����������
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8 |GPIO_Pin_9;		
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}


void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_Cmd(TIM3, DISABLE);  //ʹ��TIMx����
							 
}





//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
 
   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
 
    TIM_SetCompare2(TIM3,499);
	

}//void TIM1_Mode_Config(void)
//{ 
//    
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    TIM_OCInitTypeDef  TIM_OCInitStructure; 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//        
//	TIM_TimeBaseStructure.TIM_Prescaler = 71;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period = 5000;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OCInitStructure.TIM_Pulse =CCR1_Val;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);																 
//	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);	
//	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);	
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);	
//	TIM_Cmd(TIM1, ENABLE);	
//	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//		
//}


//void TIM8_Mode_Config(u16 arr,u16 psc)
//{ 
//    
//  GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
// 	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE );
//	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3������ӳ��  TIM3_CH2->PB5    
// 
//   //���ø�����Ϊ�����������,���TIM3 CH2��PWM���岨��	GPIOA.7,GPIOA.6
//GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8 |GPIO_Pin_9;		
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		   
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
//   //��ʼ��TIM3
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period = 5000-1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	
//	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//	TIM_OCInitStructure.TIM_Pulse =0;
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
//	TIM_OC1Init(TIM8, &TIM_OCInitStructure);																 
//	TIM_OCInitStructure.TIM_Pulse = 0;
//	TIM_OC2Init(TIM8, &TIM_OCInitStructure);	
//	TIM_OCInitStructure.TIM_Pulse = 0;
//	TIM_OC3Init(TIM8, &TIM_OCInitStructure);	
//	TIM_OCInitStructure.TIM_Pulse = 0;
//	TIM_OC4Init(TIM8, &TIM_OCInitStructure);	
//	TIM_Cmd(TIM8, ENABLE);	
//	TIM_CtrlPWMOutputs(TIM8, ENABLE);
//}



void TIM_Configuration (void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		     //-----TIM7
	NVIC_InitStructure.NVIC_IRQChannel =TIM7_IRQn;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 //ʹ�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;			//�����������ȼ�	2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		   //������Ӧ���ȼ�	  0
	NVIC_Init(&NVIC_InitStructure); 	
	NVIC_SetVectorTable (NVIC_VectTab_FLASH ,0x0);	 //���ô���Ĵ���
	
	TIM_TimeBaseStructure.TIM_Period =9999;   //���ü��������С��ÿ��2000�����Ͳ���һ�������¼�
	TIM_TimeBaseStructure.TIM_Prescaler =7199;	   //Ԥ��Ƶϵ��Ϊ36000-1������������ʱ��Ϊ72MHz/36000 = 2kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;   //����ʱ�ӷָ� TIM_CKD_DIV1=0x0000,���ָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;	  //���ü�����ģʽΪ���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure); //������Ӧ�õ�TIM7��
		
	TIM_UpdateRequestConfig( TIM7, TIM_UpdateSource_Regular);	
	TIM_Cmd(TIM7, ENABLE); //ʹ�ܼ�����
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);	//ʹ���ж�
	//TIM_ClearFlag(TIM7, TIM_FLAG_Update);//�����־λ
	

 }


void BSPINIT(void)
{
  RCC_Init();		                //����ʱ��ʹ��
	GPIO_init();	  
  TIM3_Int_Init(999,59);
	TIM3_PWM_Init(999,59);
  TIM_Configuration();	        //  TIM7���ж�����						
  flag=0;
}

