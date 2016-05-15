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
//定义一个机器人
robot basketballbot;

volatile Queue rx_queue;
static int noselect;
uint8_t radBufferRS485[99];  //记录转角

float xiansu(float v);
void handle_data(uint8_t buf[], int D,int len);
void start_dc(void);	       //电机速度的赋值
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
	 enum Recstate								   		//状态机 
	{
		RECFF1,RECFF2,SENDID,RECID,RECLEN1,RECLEN2,RECSEL,RECCHECK
	} rs485Recstate = RECFF1;
	/*******************
	初始化
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
					cur = 0;								//校验位清零
				}
	
				break;
	
			case RECFF2:
				if (data == 0xff)
					rs485Recstate = RECID;
				else
					rs485Recstate = RECFF1;
				break;
	
	
			case RECID:				 					
			     if(data==0x01)	                       //设备编号0x01底盘与航迹
			        {
							device=1;
					    checksum += data;
					    rs485Recstate = RECLEN1;
							
				      }
			else if(data==0x02)                       //设备编号0x02捡球 
							{
							device=2;
					    checksum += data;
					    rs485Recstate = RECLEN1;
									
				      }
			else if(data==0x03)                        //设备编号0x03投球 
				     {
							device=3;
					    checksum += data;
					    rs485Recstate = RECLEN1;
				      }
			else if(data==0x04) 			              //设备编号0x04 测距
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	LED_Init();	//LED端口初始化 	
	TIM3_Int_Init(719,999);  
	uart_init(115200);	 //串口初始化为115200
	uart2_init(9600);
	ENCODE_Init();
	queue_init(&rx_queue);	
}
float xiansu(float v)                        //防止速度过快
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
                     noselect = 0x01 ; basketballbot.Control_ID = Global; break;     //赋值世界坐标系速度
			              }
										else if(buf[0]==2)
										{
                     noselect = 0x02 ; basketballbot.Control_ID = Robot; break;     //赋值机器人坐标系速度
			              }
										else if(buf[0]==3)
										{
                     noselect = 0x03 ; break;     //返回里程
			              }
			default:break;
		}
	if(noselect == 0x01 || noselect == 0x02)			
			{						
				for(i=0;i<12;i++)									     			//记录速度
				{
					radBufferRS485[i] = buf[i+1] ;								
				}			
			}	
	    
} 

void start_dc(void)	    //电机速度的赋值
{	
     
 	  u8 byte1[4],byte2[4];
	  if(noselect == 0x01) 	//世界坐标系	
		{			
		byte1[0] = radBufferRS485[0];
		byte1[1] = radBufferRS485[1];
		byte1[2] = radBufferRS485[2];
		byte1[3] = radBufferRS485[3];	
    memcpy(&basketballbot.globel_expected_speed.x,byte1,sizeof(int));	  //x轴速度
		basketballbot.globel_expected_speed.x=xiansu(basketballbot.globel_expected_speed.x);
			
		byte1[0] = radBufferRS485[4];
		byte1[1] = radBufferRS485[5];
		byte1[2] = radBufferRS485[6];
		byte1[3] = radBufferRS485[7];		
   	memcpy(&basketballbot.globel_expected_speed.y,byte1,sizeof(int));   //y轴速度
		basketballbot.globel_expected_speed.y=xiansu(basketballbot.globel_expected_speed.y);
										   
		byte1[0] = radBufferRS485[8];
		byte1[1] = radBufferRS485[9];
		byte1[2] = radBufferRS485[10];
		byte1[3] = radBufferRS485[11];
    memcpy(&basketballbot.globel_expected_speed.z,byte1,sizeof(int));	 //角速度
		basketballbot.globel_expected_speed.z=xiansu(basketballbot.globel_expected_speed.z);			
		}		
		
		else if(noselect == 0x02)  //机器人坐标系
		{			
		byte2[0] = radBufferRS485[0];
		byte2[1] = radBufferRS485[1];
		byte2[2] = radBufferRS485[2];
		byte2[3] = radBufferRS485[3];	
    memcpy(&basketballbot.robot_expected_speed.x,byte2,sizeof(int));	  //x轴速度
		basketballbot.robot_expected_speed.x=xiansu(basketballbot.robot_expected_speed.x);
			
		byte2[0] = radBufferRS485[4];
		byte2[1] = radBufferRS485[5];
		byte2[2] = radBufferRS485[6];
		byte2[3] = radBufferRS485[7];		
   	memcpy(&basketballbot.robot_expected_speed.y,byte2,sizeof(int));   //y轴速度
		basketballbot.robot_expected_speed.y=xiansu(basketballbot.robot_expected_speed.y);
										   
		byte2[0] = radBufferRS485[8];
		byte2[1] = radBufferRS485[9];
		byte2[2] = radBufferRS485[10];
		byte2[3] = radBufferRS485[11];
    memcpy(&basketballbot.robot_expected_speed.z,byte2,sizeof(int));	 //角速度
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
