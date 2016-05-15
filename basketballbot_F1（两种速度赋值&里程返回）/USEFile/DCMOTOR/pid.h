#include"stm32f10x.h"

#define Tine  0.01f
#define Tined 100.0f


void PID(void);
void pid_data(float test_angle_pid[],int n);
void pid_pre1(float test_angle_pid[],float test_angle);
void pid_pre2(float test_angle_pid[],float test_angle);
void pid_pre3(float test_angle_pid[],float test_angle);
void pid_pre4(float test_angle_pid[],float test_angle);
