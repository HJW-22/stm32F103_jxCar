#ifndef __RP_H__
#define __RP_H__

#include "stm32f10x.h"
#include "bsp_usart.h"

uint16_t RP_Getvalue(uint8_t RP_Number);
void RP_Init(void);

#endif

