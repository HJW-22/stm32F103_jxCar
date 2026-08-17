#include "esp8266_uart.h"
#include "bsp_usart.h"

#define ESP8266_UART_RX_BUF_SIZE BSP_USART_RX_BUFFER_SIZE

uint16_t esp8266_LoadingFinished_Flag = 0U;

static uint8_t esp8266_rx_frame[ESP8266_UART_RX_BUF_SIZE];
static uint16_t esp8266_rx_length;
static uint8_t esp8266_rx_finished;

void ESP8266_UART_Init(USART_TypeDef *usart, uint32_t baudrate, uint16_t preemption, uint16_t sub_priority)
{
    BSP_USART_Port port = (usart == USART1) ? BSP_USART_1 : (usart == USART2) ? BSP_USART_2
                                                                              : BSP_USART_3;
    BSP_USART_Init(port, baudrate, (uint8_t)preemption, (uint8_t)sub_priority);
    ESP8266_RxRestart();
}

uint8_t *ESP8266_GetRxFrame(void)
{
    uint8_t byte;

    while (BSP_USART_Read(BSP_USART_2, &byte, 1U) == 1U) {
        if (esp8266_rx_length < (ESP8266_UART_RX_BUF_SIZE - 1U)) {
            esp8266_rx_frame[esp8266_rx_length++] = byte;
        }
    }

    if ((esp8266_rx_length != 0U) && (BSP_USART_IsIdle(BSP_USART_2) != 0U)) {
        esp8266_rx_frame[esp8266_rx_length] = '\0';
        esp8266_rx_finished                 = 1U;
        BSP_USART_ClearIdle(BSP_USART_2);
    }

    return (esp8266_rx_finished != 0U) ? esp8266_rx_frame : NULL;
}

void ESP8266_RxRestart(void)
{
    BSP_USART_ClearRx(BSP_USART_2);
    esp8266_rx_length   = 0U;
    esp8266_rx_finished = 0U;
    esp8266_rx_frame[0] = '\0';
}

int16_t ESP8266_GetRXFrame_Len(void)
{
    (void)ESP8266_GetRxFrame();
    return (esp8266_rx_finished != 0U) ? (int16_t)esp8266_rx_length : 0;
}
