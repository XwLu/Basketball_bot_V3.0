#include "stm32f10x.h"
#include "usart2.h"
#include "delay.h"
#include "motorcontrol.h"
#include "bsp.h"
#include "bsp_usart1.h"

 int step;                       //步进数，标志位
volatile int flag=0,hold=0;
/********************一号直流电机*********************************/
void direction(int a)        //正为正转，负为反转
{	 

    if(a>0)
	 {
			//ln1(1);                          //暂时去掉，蜂鸣器太吵
			ln2(0);
	 }
	if(a<0)
	 {

		//ln1(1);
		ln2(1);
	 }
}

void get_ball()              //得球
{                
 direction(1);                //上升
 step=4200;
 Stepper_motor();
 while(flag==0);  
 flag=0;  	//标志位请零
 
			USART1_SendChar(0xff);			
			USART1_SendChar(0xff);			
			USART1_SendChar(0x02);										
			USART1_SendChar(0x00);	
			USART1_SendChar(0x02);
			USART1_SendChar(0x01);
			USART1_SendChar(0x00);	
			USART1_SendChar(0x05);
	
			USART1_SendChar(0xff);			
			USART1_SendChar(0xff);			
			USART1_SendChar(0x02);										
			USART1_SendChar(0x00);	
			USART1_SendChar(0x02);
			USART1_SendChar(0x01);
			USART1_SendChar(0x00);	
			USART1_SendChar(0x05);
			
 TIM3_PWM_Init(999,9999);             //持球两秒
 step=100;
 hold=1;                       //保持持球位置，现在开始禁止关闭定时器，直到两秒以后
 Stepper_motor();
 delayms(2000);
 hold=0;
 flag=0;
	
 TIM3_PWM_Init(999,59);             //下降
 direction(-1);
 step=4190;                     //回来的时候貌似步数要小一些，防止架子打底盘，要经验算
 Stepper_motor();
 while(flag==0); 
 flag=0;
}

void get_hold_ball()              //得球并持球状态
{
 direction(1);              //上升
 step=1000;
 Stepper_motor();
 while(flag==0);  
 flag=0;  
	
 TIM3_PWM_Init(999,9999);           // 持球
 step=100;
 hold=1;                       //保持持球位置，现在开始禁止关闭定时器，直到收到getfromhold指令
 Stepper_motor();
}

void getfromhold()              //从持球位置得球
{
 TIM_Cmd(TIM3, DISABLE);
 TIM3_PWM_Init(999,59);              //上升 
 flag=0;                             //标志位清零
 hold=0;                             //解除持球
 direction(1); 
 step=3200;                          //step值需要测试
 Stepper_motor();
 while(flag==0);  
 flag=0;
	
 TIM3_PWM_Init(999,9999);           //保持铲球架静止的配置
 step=100;
 hold=1;                            //保持持球位置，现在开始禁止关闭定时器，直到收到getfromhold指令
 Stepper_motor();
 delayms(2000);
 TIM_Cmd(TIM3, DISABLE);
 hold=0;
 flag=0;
	
			USART1_SendChar(0xff);			
			USART1_SendChar(0xff);			
			USART1_SendChar(0x02);										
			USART1_SendChar(0x00);	
			USART1_SendChar(0x02);
			USART1_SendChar(0x01);
			USART1_SendChar(0x00);	
			USART1_SendChar(0x05);
	
			USART1_SendChar(0xff);			
			USART1_SendChar(0xff);			
			USART1_SendChar(0x02);										
			USART1_SendChar(0x00);	
			USART1_SendChar(0x02);
			USART1_SendChar(0x01);
			USART1_SendChar(0x00);	
			USART1_SendChar(0x05);
			
 TIM3_PWM_Init(999,59);
 direction(-1);                    //放下铲球架
 step=4000;                        //回来的时候貌似步数要小一些，防止架子打底盘，要经验算
 Stepper_motor();
 while(flag==0); 
 flag=0;
}

void high_lift()              //抬球
{
 direction(1); 
 step=4500;
 Stepper_motor();
 while(flag==0);  
 flag=0;
}

void high_down()              //放下捡球机构
{
 direction(-1); 
 step=4400;
 Stepper_motor();
 while(flag==0);  
 flag=0;
}


 void Stepper_motor()
	 {
     TIM_Cmd(TIM3, ENABLE);  //使能TIM3
   }


