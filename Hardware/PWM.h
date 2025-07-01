#ifndef __PWM_H__
#define __PWM_H__
#include "stm32f10x.h"                  // Device header
void PWM_init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void TIM2_NVIC_Init(void);

//PWM的频率   =72MHz/PSC/ARR
//PWM的占空比 =CCR/ARR 

//默认1000hz 占空比为0
#define ARR 100-1
#define PSC 720-1 
#define CCR 0


#endif
