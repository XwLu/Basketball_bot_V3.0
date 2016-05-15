#include"stm32f10x.h"
#include "timer.h"
#include "led.h"
#include "usart.h"
#include "transform.h"
#include "mydata.h"

extern robot basketballbot;

void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

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

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
							 
}
//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		LED1=!LED1;
		move_base_control();
		}
}






void ENCODE_Init(void)			// TIM2 4�ı�����ģʽ
{
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_ICInitTypeDef TIM_ICInitStructure;
	
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);	   //����Ҫ��һ��������ϸ��ù���
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
		//--------------------------TIM2 CHI1 CHI2---PA0 PA1;
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		//--------------------------TIM4 CHI1 CHI2---PB6 PB7;
	 	GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		//--------------------�ж����ȼ�
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);


	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

		//---------------------------------------------------TIM2 CHI1 CHI2
  	TIM_DeInit(TIM2);
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  	TIM_TimeBaseStructure.TIM_Prescaler =0x0 ;  
  	TIM_TimeBaseStructure.TIM_Period =(4*ENCODER_PPR)-1;  	    	
 	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
  	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

  	TIM_ICStructInit(&TIM_ICInitStructure);
  	TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  	TIM_ICInit(TIM2, &TIM_ICInitStructure);


 //------------------------------------------------TIM4 CHI1 CHI2
  	TIM_DeInit(TIM4);
  	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;   
  	TIM_TimeBaseStructure.TIM_Period =  (4*ENCODER_PPR)-1 ;  	 	
 	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 
  	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

  	TIM_ICStructInit(&TIM_ICInitStructure);
  	TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  	TIM_ICInit(TIM4, &TIM_ICInitStructure);    
   

	  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	  TIM_ClearFlag(TIM4, TIM_FLAG_Update);

	  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	 
 
  	TIM_Cmd(TIM2, ENABLE);
	  TIM_Cmd(TIM4, ENABLE);  
}

void TIM2_IRQHandler(void)	   //----TIM2 PA0,1		���������ݶ�ȡ
{
	static long count1 = 0,rcount1 = 0;
	double angle;
	uint16_t buffer1=0;
	
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);  
  
	buffer1 = TIM_GetCounter(TIM2);

  if(buffer1==0)	       //�ж���ת��ת����ת���ϼ�����һ�����ں�buffer1=0����ת���¼�����һ�����ں�buffer1=3;
  	count1++; 
	 else 
  	count1--;
	if(count1>encoder_barrier)     	 //�������ĸ�ʽ��2500r��һȦ��2500����դ������2500������
  {
  	count1=0;
 	  rcount1++;	  		//��Ȧ��
  }		  
   if(count1<-encoder_barrier)
  {
  	count1=0;
   	rcount1--;	  		//��Ȧ��
  }
	angle=(count1*360.0/encoder_barrier);
  basketballbot.encoder_data.M1=(rcount1+angle/360)*diameter*pi;		 //�õ�ʵ�ʵ��ۼ�ת�����ȣ���λcm
}

void TIM4_IRQHandler(void)	   //----TIM4  PB6,7			���������ݶ�ȡ
{
	static long count2 = 0,rcount2 = 0;
	double angle;
	uint16_t buffer2=0;
	
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);  
  
	buffer2 = TIM_GetCounter(TIM4);

  if(buffer2==0)	       //�ж���ת��ת����ת���ϼ�����һ�����ں�buffer1=0����ת���¼�����һ�����ں�buffer1=3;
  	count2++; 
	 else 
  	count2--;
	if(count2>encoder_barrier)     	 //�������ĸ�ʽ��2500r��һȦ��2500����դ������2500������
  {
  	count2=0;
 	  rcount2++;	  		//��Ȧ��
  }		  
   if(count2<-encoder_barrier)
  {
  	count2=0;
   	rcount2--;	  		//��Ȧ��
  }
	angle=(count2*360.0/encoder_barrier);
  basketballbot.encoder_data.M2=(rcount2+angle/360)*diameter*pi;		 //�õ�ʵ�ʵ��ۼ�ת�����ȣ���λcm
}

