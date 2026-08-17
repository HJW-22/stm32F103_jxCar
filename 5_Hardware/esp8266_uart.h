#ifndef __ESP8266_UART_H__
#define __ESP8266_UART_H__

#include "stm32f10x.h"
#include <string.h>

extern uint16_t esp8266_LoadingFinished_Flag;

void ESP8266_UART_Init(USART_TypeDef *usart, uint32_t baudrate, uint16_t preemption, uint16_t sub_priority);
uint8_t *ESP8266_GetRxFrame(void);
void ESP8266_RxRestart(void);
int16_t ESP8266_GetRXFrame_Len(void);

#endif
