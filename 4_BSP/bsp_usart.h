#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f10x.h"
#include <stdint.h>

#define BSP_USART_RX_BUFFER_SIZE 128U
#define RX_PACKET_MAX_LENGTH     99U

typedef enum {
    BSP_USART_1 = 0,
    BSP_USART_2,
    BSP_USART_3,
    BSP_USART_COUNT
} BSP_USART_Port;

/* Compatibility flags: Serial_Init() initialises enabled legacy ports. */
#define USART1_FLAG

extern char Serial_RxPacket1[RX_PACKET_MAX_LENGTH + 1U];
extern char Serial_RxPacket2[RX_PACKET_MAX_LENGTH + 1U];
extern char Serial_RxPacket3[RX_PACKET_MAX_LENGTH + 1U];
extern volatile uint8_t Serial_RxFlag1;
extern volatile uint8_t Serial_RxFlag2;
extern volatile uint8_t Serial_RxFlag3;

void BSP_USART_Init(BSP_USART_Port port, uint32_t baudrate, uint8_t preemption_priority, uint8_t sub_priority);
USART_TypeDef *BSP_USART_GetInstance(BSP_USART_Port port);
void BSP_USART_WriteByte(BSP_USART_Port port, uint8_t byte);
void BSP_USART_Write(BSP_USART_Port port, const uint8_t *data, uint16_t length);
void BSP_USART_WriteString(BSP_USART_Port port, const char *string);
uint16_t BSP_USART_Read(BSP_USART_Port port, uint8_t *data, uint16_t length);
uint16_t BSP_USART_Available(BSP_USART_Port port);
void BSP_USART_ClearRx(BSP_USART_Port port);
uint8_t BSP_USART_IsIdle(BSP_USART_Port port);
void BSP_USART_ClearIdle(BSP_USART_Port port);

/* Legacy API retained for existing application code. */
void Serial_Init(void);
void Serial_SendByte(uint8_t byte, USART_TypeDef *usart);
void Serial_SendArray(uint8_t *array, uint16_t length, USART_TypeDef *usart);
void Serial_SendString(char *string, USART_TypeDef *usart);
void Serial_Printf(USART_TypeDef *usart, char *format, ...);
void Serial_SendNumber(int32_t number, uint8_t length, USART_TypeDef *usart);

#endif
