#ifndef __ESP8266_UART_H__
#define __ESP8266_UART_H__

#include "stm32f10x.h"    
#include <string.h>

extern char Serial_RxPacket2[];  // 用于 USART2 的接收缓冲区  
extern uint8_t Serial_RxFlag2;    // 用于 USART2 的接收标志 
extern uint16_t esp8266_LoadingFinished_Flag;

void ESP8266_UART_Init(USART_TypeDef *USARTx, uint32_t Baudrate, uint16_t Preemption, uint16_t SubPriority);
uint8_t* ESP8266_GetRxFrame(void);
void ESP8266_RxRestart(void);
int16_t ESP8266_GetRXFrame_Len(void);

#endif

