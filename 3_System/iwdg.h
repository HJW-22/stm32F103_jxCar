#ifndef __IWDG_H__
#define __IWDG_H__
#include "stm32f10x.h" 

void IWDG_Config(uint8_t prv ,uint16_t rlv);
void IWDG_Feed(void);


#endif
