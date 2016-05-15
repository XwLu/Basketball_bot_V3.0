#include "stm32f10x.h"
#include "usart2.h"
#include "delay.h"
#include "motorcontrol.h"
#include "bsp.h"
#include "bsp_usart1.h"

 int step;                       //����������־λ
volatile int flag=0,hold=0;
/********************һ��ֱ�����*********************************/
void direction(int a)        //��Ϊ��ת����Ϊ��ת
{	 

    if(a>0)
	 {
			//ln1(1);                          //��ʱȥ����������̫��
			ln2(0);
	 }
	if(a<0)
	 {

		//ln1(1);
		ln2(1);
	 }
}

void get_ball()              //����
{                
 direction(1);                //����
 step=4200;
 Stepper_motor();
 while(flag==0);  
 flag=0;  	//��־λ����
 
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
			
 TIM3_PWM_Init(999,9999);             //��������
 step=100;
 hold=1;                       //���ֳ���λ�ã����ڿ�ʼ��ֹ�رն�ʱ����ֱ�������Ժ�
 Stepper_motor();
 delayms(2000);
 hold=0;
 flag=0;
	
 TIM3_PWM_Init(999,59);             //�½�
 direction(-1);
 step=4190;                     //������ʱ��ò�Ʋ���ҪСһЩ����ֹ���Ӵ���̣�Ҫ������
 Stepper_motor();
 while(flag==0); 
 flag=0;
}

void get_hold_ball()              //���򲢳���״̬
{
 direction(1);              //����
 step=1000;
 Stepper_motor();
 while(flag==0);  
 flag=0;  
	
 TIM3_PWM_Init(999,9999);           // ����
 step=100;
 hold=1;                       //���ֳ���λ�ã����ڿ�ʼ��ֹ�رն�ʱ����ֱ���յ�getfromholdָ��
 Stepper_motor();
}

void getfromhold()              //�ӳ���λ�õ���
{
 TIM_Cmd(TIM3, DISABLE);
 TIM3_PWM_Init(999,59);              //���� 
 flag=0;                             //��־λ����
 hold=0;                             //�������
 direction(1); 
 step=3200;                          //stepֵ��Ҫ����
 Stepper_motor();
 while(flag==0);  
 flag=0;
	
 TIM3_PWM_Init(999,9999);           //���ֲ���ܾ�ֹ������
 step=100;
 hold=1;                            //���ֳ���λ�ã����ڿ�ʼ��ֹ�رն�ʱ����ֱ���յ�getfromholdָ��
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
 direction(-1);                    //���²����
 step=4000;                        //������ʱ��ò�Ʋ���ҪСһЩ����ֹ���Ӵ���̣�Ҫ������
 Stepper_motor();
 while(flag==0); 
 flag=0;
}

void high_lift()              //̧��
{
 direction(1); 
 step=4500;
 Stepper_motor();
 while(flag==0);  
 flag=0;
}

void high_down()              //���¼������
{
 direction(-1); 
 step=4400;
 Stepper_motor();
 while(flag==0);  
 flag=0;
}


 void Stepper_motor()
	 {
     TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
   }


