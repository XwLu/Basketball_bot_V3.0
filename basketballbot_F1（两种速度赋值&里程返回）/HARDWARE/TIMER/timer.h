#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define pi 3.141592653
#define encoder_barrier 2000            //��������������Ķ�
#define diameter 10.16     //�����ֵ�ֱ��
#define ENCODER_PPR           (u16)(1)   // number of pulses per revolution 
#define ICx_FILTER      (u8) 1 // 6<-> 670nsec
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void ENCODE_Init(void);
#endif
