#include "bsp_usart.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    volatile uint8_t buffer[BSP_USART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t idle;
} BSP_USART_RxBuffer;

static BSP_USART_RxBuffer s_rx_buffers[BSP_USART_COUNT];
static uint16_t s_legacy_lengths[BSP_USART_COUNT];

char Serial_RxPacket1[RX_PACKET_MAX_LENGTH + 1U];
char Serial_RxPacket2[RX_PACKET_MAX_LENGTH + 1U];
char Serial_RxPacket3[RX_PACKET_MAX_LENGTH + 1U];
volatile uint8_t Serial_RxFlag1;
volatile uint8_t Serial_RxFlag2;
volatile uint8_t Serial_RxFlag3;

static const uint32_t s_usart_clocks[BSP_USART_COUNT]    = {RCC_APB2Periph_USART1, RCC_APB1Periph_USART2, RCC_APB1Periph_USART3};
static GPIO_TypeDef *const s_gpio_ports[BSP_USART_COUNT] = {GPIOA, GPIOA, GPIOB};
static const uint16_t s_tx_pins[BSP_USART_COUNT]         = {GPIO_Pin_9, GPIO_Pin_2, GPIO_Pin_10};
static const uint16_t s_rx_pins[BSP_USART_COUNT]         = {GPIO_Pin_10, GPIO_Pin_3, GPIO_Pin_11};

USART_TypeDef *BSP_USART_GetInstance(BSP_USART_Port port)
{
    static USART_TypeDef *const instances[BSP_USART_COUNT] = {USART1, USART2, USART3};
    return (port < BSP_USART_COUNT) ? instances[port] : NULL;
}

static IRQn_Type BSP_USART_GetIRQn(BSP_USART_Port port)
{
    static const IRQn_Type irqs[BSP_USART_COUNT] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
    return irqs[port];
}

static void BSP_USART_ReceiveByte(BSP_USART_Port port, uint8_t byte)
{
    BSP_USART_RxBuffer *rx = &s_rx_buffers[port];
    char *legacy_packet;
    volatile uint8_t *legacy_flag;
    uint16_t next = (uint16_t)((rx->head + 1U) % BSP_USART_RX_BUFFER_SIZE);

    if (rx->idle != 0U) {
        rx->idle               = 0U;
        s_legacy_lengths[port] = 0U;
        legacy_flag            = (port == BSP_USART_1) ? &Serial_RxFlag1 : (port == BSP_USART_2) ? &Serial_RxFlag2
                                                                                                 : &Serial_RxFlag3;
        *legacy_flag           = 0U;
    }

    if (next != rx->tail) {
        rx->buffer[rx->head] = byte;
        rx->head             = next;
    }

    legacy_packet = (port == BSP_USART_1) ? Serial_RxPacket1 : (port == BSP_USART_2) ? Serial_RxPacket2
                                                                                     : Serial_RxPacket3;
    if (s_legacy_lengths[port] < RX_PACKET_MAX_LENGTH) {
        legacy_packet[s_legacy_lengths[port]++] = (char)byte;
    }
}

static void BSP_USART_MarkIdle(BSP_USART_Port port)
{
    char *legacy_packet           = (port == BSP_USART_1) ? Serial_RxPacket1 : (port == BSP_USART_2) ? Serial_RxPacket2
                                                                                                     : Serial_RxPacket3;
    volatile uint8_t *legacy_flag = (port == BSP_USART_1) ? &Serial_RxFlag1 : (port == BSP_USART_2) ? &Serial_RxFlag2
                                                                                                    : &Serial_RxFlag3;

    legacy_packet[s_legacy_lengths[port]] = '\0';
    s_rx_buffers[port].idle               = 1U;
    *legacy_flag                          = 1U;
}

void BSP_USART_Init(BSP_USART_Port port, uint32_t baudrate, uint8_t preemption_priority, uint8_t sub_priority)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    USART_TypeDef *instance = BSP_USART_GetInstance(port);

    if ((instance == NULL) || (baudrate == 0U)) return;

    if (port == BSP_USART_1) {
        RCC_APB2PeriphClockCmd(s_usart_clocks[port] | RCC_APB2Periph_GPIOA, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(s_usart_clocks[port], ENABLE);
        RCC_APB2PeriphClockCmd((port == BSP_USART_2) ? RCC_APB2Periph_GPIOA : RCC_APB2Periph_GPIOB, ENABLE);
    }

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = s_tx_pins[port];
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(s_gpio_ports[port], &gpio);

    gpio.GPIO_Pin  = s_rx_pins[port];
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(s_gpio_ports[port], &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_Mode     = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(instance, &usart);
    USART_ITConfig(instance, USART_IT_RXNE, ENABLE);
    USART_ITConfig(instance, USART_IT_IDLE, ENABLE);
    USART_Cmd(instance, ENABLE);

    nvic.NVIC_IRQChannel                   = BSP_USART_GetIRQn(port);
    nvic.NVIC_IRQChannelPreemptionPriority = preemption_priority;
    nvic.NVIC_IRQChannelSubPriority        = sub_priority;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    BSP_USART_ClearRx(port);
}

void BSP_USART_WriteByte(BSP_USART_Port port, uint8_t byte)
{
    USART_TypeDef *instance = BSP_USART_GetInstance(port);
    if (instance == NULL) return;
    while (USART_GetFlagStatus(instance, USART_FLAG_TXE) == RESET) {}
    USART_SendData(instance, byte);
}

void BSP_USART_Write(BSP_USART_Port port, const uint8_t *data, uint16_t length)
{
    uint16_t i;
    if (data == NULL) return;
    for (i = 0U; i < length; ++i) BSP_USART_WriteByte(port, data[i]);
}

void BSP_USART_WriteString(BSP_USART_Port port, const char *string)
{
    if (string != NULL) BSP_USART_Write(port, (const uint8_t *)string, (uint16_t)strlen(string));
}

uint16_t BSP_USART_Available(BSP_USART_Port port)
{
    BSP_USART_RxBuffer *rx;
    if (port >= BSP_USART_COUNT) return 0U;
    rx = &s_rx_buffers[port];
    return (rx->head >= rx->tail) ? (rx->head - rx->tail) : (BSP_USART_RX_BUFFER_SIZE - rx->tail + rx->head);
}

uint16_t BSP_USART_Read(BSP_USART_Port port, uint8_t *data, uint16_t length)
{
    BSP_USART_RxBuffer *rx;
    uint16_t count = 0U;
    if ((port >= BSP_USART_COUNT) || (data == NULL)) return 0U;
    rx = &s_rx_buffers[port];
    while ((count < length) && (rx->tail != rx->head)) {
        data[count++] = rx->buffer[rx->tail];
        rx->tail      = (uint16_t)((rx->tail + 1U) % BSP_USART_RX_BUFFER_SIZE);
    }
    return count;
}

void BSP_USART_ClearRx(BSP_USART_Port port)
{
    if (port >= BSP_USART_COUNT) return;
    s_rx_buffers[port].head = 0U;
    s_rx_buffers[port].tail = 0U;
    s_rx_buffers[port].idle = 0U;
    s_legacy_lengths[port]  = 0U;
}

uint8_t BSP_USART_IsIdle(BSP_USART_Port port)
{
    return (port < BSP_USART_COUNT) ? s_rx_buffers[port].idle : 0U;
}

void BSP_USART_ClearIdle(BSP_USART_Port port)
{
    if (port < BSP_USART_COUNT) s_rx_buffers[port].idle = 0U;
}

static BSP_USART_Port BSP_USART_PortFromInstance(USART_TypeDef *usart)
{
    if (usart == USART1) return BSP_USART_1;
    if (usart == USART2) return BSP_USART_2;
    return BSP_USART_3;
}

void Serial_Init(void)
{
#ifdef USART1_FLAG
    BSP_USART_Init(BSP_USART_1, 128000U, 1U, 1U);
#endif
#ifdef USART2_FLAG
    BSP_USART_Init(BSP_USART_2, 115200U, 1U, 1U);
#endif
#ifdef USART3_FLAG
    BSP_USART_Init(BSP_USART_3, 115200U, 0U, 0U);
#endif
}

void Serial_SendByte(uint8_t byte, USART_TypeDef *usart)
{ BSP_USART_WriteByte(BSP_USART_PortFromInstance(usart), byte); }
void Serial_SendArray(uint8_t *array, uint16_t length, USART_TypeDef *usart)
{ BSP_USART_Write(BSP_USART_PortFromInstance(usart), array, length); }
void Serial_SendString(char *string, USART_TypeDef *usart)
{ BSP_USART_WriteString(BSP_USART_PortFromInstance(usart), string); }

void Serial_Printf(USART_TypeDef *usart, char *format, ...)
{
    char string[100];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(string, sizeof(string), format, args);
    va_end(args);
    Serial_SendString(string, usart);
}

void Serial_SendNumber(int32_t number, uint8_t length, USART_TypeDef *usart)
{
    char string[16];
    (void)length;
    (void)snprintf(string, sizeof(string), "%ld", (long)number);
    Serial_SendString(string, usart);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) BSP_USART_ReceiveByte(BSP_USART_1, (uint8_t)USART_ReceiveData(USART1));
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        (void)USART1->SR;
        (void)USART1->DR;
        BSP_USART_MarkIdle(BSP_USART_1);
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) BSP_USART_ReceiveByte(BSP_USART_2, (uint8_t)USART_ReceiveData(USART2));
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        (void)USART2->SR;
        (void)USART2->DR;
        BSP_USART_MarkIdle(BSP_USART_2);
    }
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) BSP_USART_ReceiveByte(BSP_USART_3, (uint8_t)USART_ReceiveData(USART3));
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {
        (void)USART3->SR;
        (void)USART3->DR;
        BSP_USART_MarkIdle(BSP_USART_3);
    }
}
