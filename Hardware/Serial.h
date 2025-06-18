#ifndef __Serial_H
#define __Serial_H

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>  

//NVIC_PriorityGroup_2 2位响应级 2位抢占级  转换为数字都为0-4

//用不到注释即可
//#define usart1_flag
#define usart2_flag
#define usart3_flag


#ifdef usart1_flag
extern char Serial_RxPacket1[];  // 用于 USART1 的接收缓冲区 
extern uint8_t Serial_RxFlag1;    // 用于 USART1 的接收标志 
#define usart1_preemption 1
#define usart1_sub        1
#endif 

#ifdef usart2_flag
extern char Serial_RxPacket2[];  // 用于 USART2 的接收缓冲区  
extern uint8_t Serial_RxFlag2;    // 用于 USART2 的接收标志 
#define usart2_preemption 0
#define usart2_sub        0
#endif 
 

#ifdef usart3_flag
extern char Serial_RxPacket3[];  // 用于 USART3 的接收缓冲区
extern uint8_t Serial_RxFlag3;    // 用于 USART3 的接收标志
#define usart3_preemption 1
#define usart3_sub        1
#endif

// 发送缓冲区定义  
#define TX_BUFFER_SIZE 64  // 根据需要调整大小  

// 发送缓冲区  
static uint8_t TxBuffer[TX_BUFFER_SIZE];  

// USART3 TX DMA配置  
#define USART3_TX_DMA_CHANNEL   DMA1_Channel2  
#define USART3_RX_DMA_CHANNEL   DMA1_Channel3  



//USART部分
void Serial_Init(void);  
void Serial_SendByte(uint8_t Byte, USART_TypeDef *USARTx);  // 发送单个字节到指定USART  
void Serial_SendNumber(uint32_t Number, uint8_t Length, USART_TypeDef *USARTx); // 发送数字到指定USART  
void Serial_SendArray(uint8_t *Array, uint16_t Length, USART_TypeDef *USARTx); // 发送数组到指定USART  
void Serial_SendString(char *String, USART_TypeDef *USARTx);  // 发送字符串到指定USART  
void Serial_Printf(USART_TypeDef *USARTx, char *format, ...);  // 格式化发送字符串到指定USART  

//USART+DMA部分
void USART3_DMA_Init(void);
void USART3_DMA_Send(uint8_t* data, uint16_t size);
#endif
