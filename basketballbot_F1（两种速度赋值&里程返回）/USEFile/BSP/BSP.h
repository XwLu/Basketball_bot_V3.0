#ifndef __bsp_H
#define	__bsp_H

#include"stm32f10x.h"

#define ON  0
#define OFF 1

//带参宏，可以像内联函数一样使用
#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_10);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_10)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_11);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_11)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_12);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_12)
#define LED4(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_5)
void BSPINIT(void);
void TIM8_Mode_Config(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);
#endif
