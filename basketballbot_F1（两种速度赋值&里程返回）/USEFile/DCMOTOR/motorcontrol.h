#include"stm32f10x.h"

void direction(int a);
void Stepper_motor(void);
void get_ball(void);
//void mid_lift(void);
void getfromhold(void);
void get_hold_ball(void);
//void mid_down(void);
void high_down(void);
void high_lift(void);
//步进电机
#define ln1(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_8);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define ln2(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_13);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_13)



