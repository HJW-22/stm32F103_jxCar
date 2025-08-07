#ifndef __ESP8266_UART_H__
#define __ESP8266_UART_H__

#include "stm32f10x.h"    
#include <string.h>


void ESP8266_UART_Init(USART_TypeDef *USARTx, uint32_t Baudrate, uint16_t Preemption, uint16_t SubPriority);
uint8_t* ESP8266_GetRxFrame(void);
void ESP8266_RxRestart(void);
int16_t ESP8266_GetRXFrame_Len(void);

#endif

