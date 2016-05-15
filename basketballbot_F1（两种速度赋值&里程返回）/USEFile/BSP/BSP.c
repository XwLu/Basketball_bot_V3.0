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
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);	   //很重要的一条必须加上复用功能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE ); 	  	//PC 口用来控制电机正反转 两个一组PC0 1控制电机1；  2 3 控制电机2；  4 5 控制电机3；  6 7控制电机4
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
    
	//LED引脚使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);		  
	GPIO_SetBits(GPIOC, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
//调试战舰的跑马灯
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


    //串口1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( GPIOA , &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init( GPIOA , &GPIO_InitStructure);

 
    //PC 口用来控制电机正反转 两个一组 PC0 1控制电机1；  2 3 控制电机2；  4 5 控制电机3；  6 7控制电机4
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
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		    //*************************TIM1复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOA, &GPIO_InitStructure);


    //*************************TIM8复用推挽输出
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8 |GPIO_Pin_9;		
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}


void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM3, DISABLE);  //使能TIMx外设
							 
}





//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
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

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	//使能定时器3时钟
// 	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE );
//	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
// 
//   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOA.7,GPIOA.6
//GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_6 |GPIO_Pin_7|GPIO_Pin_8 |GPIO_Pin_9;		
//	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;		   
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
//   //初始化TIM3
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
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 //使能优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;			//配置抢断优先级	2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		   //配置响应优先级	  0
	NVIC_Init(&NVIC_InitStructure); 	
	NVIC_SetVectorTable (NVIC_VectTab_FLASH ,0x0);	 //设置存入寄存器
	
	TIM_TimeBaseStructure.TIM_Period =9999;   //设置计数溢出大小，每计2000个数就产生一个更新事件
	TIM_TimeBaseStructure.TIM_Prescaler =7199;	   //预分频系数为36000-1，这样计数器时钟为72MHz/36000 = 2kHz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;   //设置时钟分割 TIM_CKD_DIV1=0x0000,不分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;	  //设置计数器模式为向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure); //将配置应用到TIM7中
		
	TIM_UpdateRequestConfig( TIM7, TIM_UpdateSource_Regular);	
	TIM_Cmd(TIM7, ENABLE); //使能计数器
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);	//使能中断
	//TIM_ClearFlag(TIM7, TIM_FLAG_Update);//清除标志位
	

 }


void BSPINIT(void)
{
  RCC_Init();		                //所有时钟使能
	GPIO_init();	  
  TIM3_Int_Init(999,59);
	TIM3_PWM_Init(999,59);
  TIM_Configuration();	        //  TIM7主中断设置						
  flag=0;
}

