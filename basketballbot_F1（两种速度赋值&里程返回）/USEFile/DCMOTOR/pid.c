#include"stm32f10x.h"
#include "pid.h"
#include "motorcontrol.h"

extern float angle1,angle3,angle5,angle7;
extern float angle_pid1[3],angle_pid2[3],angle_pid3[3],angle_pid4[3];
extern float AZ1, AZ2, AZ3;						// 每个万向轮目的角速度和当前角速度的差值
extern float destpid_angle1, destpid_angle2, destpid_angle3;
float Kp1=12.0f,Ti1=0.9f,Td1=0.00005f,Kp2=12.0f,Ti2=0.9f,Td2=0.00005f,Kp3=12.0f,Ti3=0.9f,Td3=0.00005f;
float angle_final1=0,angle_final2=0,angle_final3=0;


int numcount1=0,numcount2=0,numcount3=0,i=0;

void PID(void)
{
   
if(destpid_angle1>10||destpid_angle1<-10)  pid_pre1(angle_pid1,AZ1);
if(destpid_angle2>10||destpid_angle2<-10)  pid_pre2(angle_pid2,AZ2);
if(destpid_angle3>10||destpid_angle3<-10)	 pid_pre3(angle_pid3,AZ3);

}

void pid_data(float test_angle_pid[],int n)      
{
   #define Et1 test_angle_pid[0]
	 #define Et2 test_angle_pid[1]
	 #define Et3 test_angle_pid[2]

   	switch(n)
	{
		case 1:angle_final1 = Kp1*( (Et3-Et2) + Ti1*Et3*Tine + Td1*(Et3 - 2*Et2 + Et1)*Tined );break;
		case 2:angle_final2 = Kp2*( (Et3-Et2) + Ti2*Et3*Tine + Td2*(Et3 - 2*Et2 + Et1)*Tined );break;		 
		case 3:angle_final3 = Kp3*( (Et3-Et2) + Ti3*Et3*Tine + Td3*(Et3 - 2*Et2 + Et1)*Tined );break;
		

		default:break;
	}

}

void pid_pre1(float test_angle_pid[],float test_angle)
{
    test_angle_pid[1] = test_angle_pid[2];
	  test_angle_pid[0] = test_angle_pid[1];
		test_angle_pid[2] = test_angle;
    pid_data(angle_pid1,1);
	  angle_do1();
}


void pid_pre2(float test_angle_pid[],float test_angle)
{

  	test_angle_pid[1] = test_angle_pid[2];
	  test_angle_pid[0] = test_angle_pid[1];
		test_angle_pid[2] = test_angle;
	  pid_data(angle_pid2,2);
		angle_do2();
}


void pid_pre3(float test_angle_pid[],float test_angle)
{

  	test_angle_pid[1] = test_angle_pid[2];
	  test_angle_pid[0] = test_angle_pid[1];
		test_angle_pid[2] = test_angle;
	  pid_data(angle_pid3,3);
		angle_do3();
}

