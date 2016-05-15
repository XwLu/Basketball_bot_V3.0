#include "stm32f10x.h"
#include "usart.h"
#include "usart2.h"
#include "delay.h"
#include "sys.h"
#include "led.h"
#include "string.h"
#include "queue.h"
#include "mydata.h"
#include "timer.h"

#define MAX_DATA_LEN 100
//����һ��������
robot basketballbot;

volatile Queue rx_queue;
static int noselect;
uint8_t radBufferRS485[99];  //��¼ת��

float xiansu(float v);
void handle_data(uint8_t buf[], int D,int len);
void start_dc(void);	       //����ٶȵĸ�ֵ
void send(void);
void init(void);

 int main(void)
 {	
	 uint8_t data;
	 static uint16_t checksum;
	 uint8_t  buffer[MAX_DATA_LEN] = {0};
	 int cur=0;
	 int device;
	 uint8_t datalen1;
	 short data_len;
	 enum Recstate								   		//״̬�� 
	{
		RECFF1,RECFF2,SENDID,RECID,RECLEN1,RECLEN2,RECSEL,RECCHECK
	} rs485Recstate = RECFF1;
	/*******************
	��ʼ��
	******************/
	init();
	
	 while(1)
	{
		if(queue_empty(rx_queue))
			            continue;

		data = (uint8_t)queue_get(&rx_queue);
  
		switch (rs485Recstate) 
		{
			case RECFF1:
				if (data == 0xff)	  
				{	
					rs485Recstate = RECFF2;	
					checksum=0;		
					cur = 0;								//У��λ����
				}
	
				break;
	
			case RECFF2:
				if (data == 0xff)
					rs485Recstate = RECID;
				else
					rs485Recstate = RECFF1;
				break;
	
	
			case RECID:				 					
			     if(data==0x01)	                       //�豸���0x01�����뺽��
			        {
							device=1;
					    checksum += data;
					    rs485Recstate = RECLEN1;
							
				      }
			else if(data==0x02)                       //�豸���0x02���� 
							{
							device=2;
					    checksum += data;
					    rs485Recstate = RECLEN1;
									
				      }
			else if(data==0x03)                        //�豸���0x03Ͷ�� 
				     {
							device=3;
					    checksum += data;
					    rs485Recstate = RECLEN1;
				      }
			else if(data==0x04) 			              //�豸���0x04 ���
				      {
				      device=4;
					    checksum += data;
					    rs485Recstate = RECLEN1;
				}
				else
					rs485Recstate = RECFF1;	  
				break;
	
			case RECLEN1:				 				
				checksum += data;
				datalen1 = data;
				rs485Recstate = RECLEN2;	  
				break;
		
			case RECLEN2:	
				checksum += data;			 				
				data_len = (short)datalen1 << 8	 | data;
				rs485Recstate = RECSEL;	  
				break;
	
			case RECSEL:
				checksum += data;
				buffer[cur++] = data;
				if(cur >= data_len)
					rs485Recstate = RECCHECK;
				break;
	
			case RECCHECK:
				checksum=checksum%255;
				
				if(data == checksum)
				{			
					handle_data(buffer,device, data_len);
					
  				send();				
					checksum=0;	
					rs485Recstate = RECFF1;	 
				}
				
				else
					rs485Recstate = RECFF1;
				break;
			 default:
					rs485Recstate = RECFF1;
			}	   	 
		}	
	
}
 
void init()
{
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	LED_Init();	//LED�˿ڳ�ʼ�� 	
	TIM3_Int_Init(719,999);  
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	uart2_init(9600);
	ENCODE_Init();
	queue_init(&rx_queue);	
}
float xiansu(float v)                        //��ֹ�ٶȹ���
{
   if(v>2) 
     {
	    return 2;
	 }
   if(v<-2) 
     {
	    return -2;
	 } 
   else
   {
        return v;
   }

}

void handle_data(uint8_t buf[], int D,int len)    
{
	 int i=0;
		switch(D)
		{ 
			case 1:				if(buf[0]==1) 
		            	  {
                     noselect = 0x01 ; basketballbot.Control_ID = Global; break;     //��ֵ��������ϵ�ٶ�
			              }
										else if(buf[0]==2)
										{
                     noselect = 0x02 ; basketballbot.Control_ID = Robot; break;     //��ֵ����������ϵ�ٶ�
			              }
										else if(buf[0]==3)
										{
                     noselect = 0x03 ; break;     //�������
			              }
			default:break;
		}
	if(noselect == 0x01 || noselect == 0x02)			
			{						
				for(i=0;i<12;i++)									     			//��¼�ٶ�
				{
					radBufferRS485[i] = buf[i+1] ;								
				}			
			}	
	    
} 

void start_dc(void)	    //����ٶȵĸ�ֵ
{	
     
 	  u8 byte1[4],byte2[4];
	  if(noselect == 0x01) 	//��������ϵ	
		{			
		byte1[0] = radBufferRS485[0];
		byte1[1] = radBufferRS485[1];
		byte1[2] = radBufferRS485[2];
		byte1[3] = radBufferRS485[3];	
    memcpy(&basketballbot.globel_expected_speed.x,byte1,sizeof(int));	  //x���ٶ�
		basketballbot.globel_expected_speed.x=xiansu(basketballbot.globel_expected_speed.x);
			
		byte1[0] = radBufferRS485[4];
		byte1[1] = radBufferRS485[5];
		byte1[2] = radBufferRS485[6];
		byte1[3] = radBufferRS485[7];		
   	memcpy(&basketballbot.globel_expected_speed.y,byte1,sizeof(int));   //y���ٶ�
		basketballbot.globel_expected_speed.y=xiansu(basketballbot.globel_expected_speed.y);
										   
		byte1[0] = radBufferRS485[8];
		byte1[1] = radBufferRS485[9];
		byte1[2] = radBufferRS485[10];
		byte1[3] = radBufferRS485[11];
    memcpy(&basketballbot.globel_expected_speed.z,byte1,sizeof(int));	 //���ٶ�
		basketballbot.globel_expected_speed.z=xiansu(basketballbot.globel_expected_speed.z);			
		}		
		
		else if(noselect == 0x02)  //����������ϵ
		{			
		byte2[0] = radBufferRS485[0];
		byte2[1] = radBufferRS485[1];
		byte2[2] = radBufferRS485[2];
		byte2[3] = radBufferRS485[3];	
    memcpy(&basketballbot.robot_expected_speed.x,byte2,sizeof(int));	  //x���ٶ�
		basketballbot.robot_expected_speed.x=xiansu(basketballbot.robot_expected_speed.x);
			
		byte2[0] = radBufferRS485[4];
		byte2[1] = radBufferRS485[5];
		byte2[2] = radBufferRS485[6];
		byte2[3] = radBufferRS485[7];		
   	memcpy(&basketballbot.robot_expected_speed.y,byte2,sizeof(int));   //y���ٶ�
		basketballbot.robot_expected_speed.y=xiansu(basketballbot.robot_expected_speed.y);
										   
		byte2[0] = radBufferRS485[8];
		byte2[1] = radBufferRS485[9];
		byte2[2] = radBufferRS485[10];
		byte2[3] = radBufferRS485[11];
    memcpy(&basketballbot.robot_expected_speed.z,byte2,sizeof(int));	 //���ٶ�
		basketballbot.robot_expected_speed.z=xiansu(basketballbot.robot_expected_speed.z);			
		}		
}

void send(void)
{
	uint16_t checksumsend2=0x10;
	char c_transition[4],i;
	float f_transition=0;
	if(USART_GetFlagStatus(USART1, USART_FLAG_TC)!=RESET)
	{	
		
		if(noselect == 0x01 || noselect == 0x02)
		 {
			start_dc();		
			LED1=0;
			noselect=0;
		 }
		else if(noselect == 0x03)
		 {
		  USART1_SendChar(0xff);
			USART1_SendChar(0xff);
			USART1_SendChar(0x01);
			USART1_SendChar(0x00);
			USART1_SendChar(0x0d);
			USART1_SendChar(0x02);
			//basketballbot.real_position.x
			f_transition = basketballbot.real_position.x;
			memcpy( c_transition, &f_transition,sizeof(float));
			for(i = 0;i < 4;i++)
			{
				USART1_SendChar(c_transition[i]);
				checksumsend2 += c_transition[i];
			}
			//basketballbot.real_position.y
			f_transition = basketballbot.real_position.y;
			memcpy( c_transition, &f_transition,sizeof(float));
			for(i = 0;i < 4;i++)
			{
				USART1_SendChar(c_transition[i]);
				checksumsend2 += c_transition[i];
			}
			
			checksumsend2=checksumsend2%255;
			USART1_SendChar(checksumsend2);
			noselect=0;
		 }			
	}
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);				
}
