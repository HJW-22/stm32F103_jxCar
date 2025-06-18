#ifndef __Encoder_H__
#define __Encoder_H__
#include "stm32f10x.h"                  // Device header

void Encoder_TIM2_Init(void);
void Encoder_TIM3_Init(void);

int16_t Encoder_TIM2_Get(void);
int16_t Encoder_TIM3_Get(void);


#endif

