#ifndef __axmotor_H
#define	__axmotor_H

#include "stm32f10x.h"
#include "AX.h"

#define DigMotorNum1 0x01
#define DigMotorNum2 0x02
#define OpTagitPosi  0x0094
#define ClTagitPosi  0x0000
#define DigMotorOpen  0x02
#define DigMotorClose 0x01

void DigMotor_Move(uint8_t Act_Mode,uint8_t SpeedH,uint8_t SpeedL);	  //数字舵机动作模式，速度高八位，速度低八位
uint16_t axReadLoad(uint8_t bID);
uint16_t axReadTorque(uint8_t bID);	


#endif 
