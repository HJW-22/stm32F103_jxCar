#ifndef __Serial_H__
#define __Serial_H__

#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>  

//NVIC_PriorityGroup_2 2位响应级 2位抢占级  转换为数字都为0-4

// 发送缓冲区定义  
#define TX_BUFFER_SIZE 64  // 根据需要调整大小  
//用不到注释即可
#define USART1_FLAG
#define USART2_FLAG
//#define USART3_FLAG


#ifdef USART1_FLAG
extern char Serial_RxPacket1[];  // 用于 USART1 的接收缓冲区 
extern uint8_t Serial_RxFlag1;    // 用于 USART1 的接收标志 
#define usart1_preemption 1
#define usart1_sub        1
#endif 

#ifdef USART2_FLAG
extern char Serial_RxPacket2[];  // 用于 USART2 的接收缓冲区  
extern uint8_t Serial_RxFlag2;    // 用于 USART2 的接收标志 
#define usart2_preemption 1
#define usart2_sub        1
// USART3 TX DMA配置  
static uint8_t TxBuffer_USART2[TX_BUFFER_SIZE]; 
#define USART2_TX_DMA_CHANNEL   DMA1_Channel7  
#define USART2_RX_DMA_CHANNEL   DMA1_Channel6  
#endif 
 

#ifdef USART3_FLAG
extern char Serial_RxPacket3[];  // 用于 USART3 的接收缓冲区
extern uint8_t Serial_RxFlag3;    // 用于 USART3 的接收标志
#define usart3_preemption 1
#define usart3_sub        1
// USART3 TX DMA配置  
#define USART3_TX_DMA_CHANNEL   DMA1_Channel2  
#define USART3_RX_DMA_CHANNEL   DMA1_Channel3  
static uint8_t TxBuffer_USART3[TX_BUFFER_SIZE];  
#endif


//USART部分
void Serial_Init(void);  
void Serial_SendByte(uint8_t Byte, USART_TypeDef *USARTx);  // 发送单个字节到指定USART  
void Serial_SendArray(uint8_t *Array, uint16_t Length, USART_TypeDef *USARTx); // 发送数组到指定USART  
void Serial_SendString(char *String, USART_TypeDef *USARTx);  // 发送字符串到指定USART  
void Serial_Printf(USART_TypeDef *USARTx, char *format, ...);  // 格式化发送字符串到指定USART  
void Serial_SendNumber(int32_t Number, uint8_t Length, USART_TypeDef *USARTx); 


#ifdef USART2_FLAG
//USART+DMA部分
void USART2_DMA_Init(void);
void USART2_DMA_Send(uint8_t* data, uint16_t size);
#endif // USART2_FLAG

#ifdef USART3_FLAG
//USART+DMA部分
void USART3_DMA_Init(void);
void USART3_DMA_Send(uint8_t* data, uint16_t size);
#endif // USART3_FLAG

#endif
