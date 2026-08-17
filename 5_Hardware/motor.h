#ifndef __Motor_H__
#define __Motor_H__
#include "stm32f10x.h"                  // Device header

void Motor_Init(int16_t ARR,int16_t PSC);
void MotorA_SetSpeed(int16_t Speed);
void MotorB_SetSpeed(int16_t Speed);

#endif
