#include "esp8266_uart.h"

/* UART收发缓冲大小 */
#define ESP8622_UART_RX_BUF_SIZE            128
#define ESP8622_UART_TX_BUF_SIZE            64
uint16_t esp8266_LoadingFinished_Flag =0;

/*台式(tcp)--->esp8266(串口)--->stm32
台式--->tcp(透传)--->esp8622---->串口(AT响应)----->stm32--->返回--->关闭透传



透传:
AT+CIPMODE=1      // 开启透传模式
AT+CIPSEND        // 进入透传 
>                 // 收到">"后进入透传状态  
示例:\
targetA = 111;
AT+CIPMODE=1      // 开启透传模式
AT+CIPSEND        // 进入透传 
>targetA                // 收到">"后进入透传状态  



AT响应：
+IPD,<len>:<data>
示例：+IPD,10:hello12345

返回:
AT+CIPSEND=<length>  // 指定发送长度
>                   // 输入待发送数据
实例AT+CIPSEND=2  // 指定发送长度
>OK                  // 输入待发送数据


关闭透传
AT+CIPMODE=0      // 开启透传模式

*/
typedef struct
{
    uint8_t buf[ESP8622_UART_RX_BUF_SIZE];              /* 帧接收缓冲 */
    struct
    {
        uint16_t len    : 15;                               /* 帧接收长度，sta[14:0] */
        uint16_t finsh  : 1;                                /* 帧接收完成标志，sta[15] */
    } sta;                                                  /* 帧状态信息 */
} ESP8622_UART_RX_Frame;            




/* 全局变量 */
static volatile ESP8622_UART_RX_Frame esp8266_rx_frame = {0};

// typedef struct ESP8622_InitTypeDef {
//     RST_EN_Typedef RST_EN;
// };


// typedef enum {
//     RST_Disable,
//     RST_ENABLE,
// } RST_EN_Typedef;

typedef enum {
    AFIO_Disable,
    AFIO_ENABLE,
} AFIO_EN_Typedef;

typedef struct
{
    USART_TypeDef *USARTx;
    uint16_t Baudrate;
    uint16_t Preemption;
    uint16_t SubPriority;
    AFIO_EN_Typedef AFIO_EN; //此功能没有扩展
    // 需要什么可以加
} Serial_InitTypeDef;

// ESP8266_Init() --> ESP8266_UART_Init -->标准库
void ESP8266_UART_Init(USART_TypeDef *USARTx, uint32_t Baudrate, uint16_t Preemption, uint16_t SubPriority)
{
    Serial_InitTypeDef this = {
        .USARTx = USARTx,
        .Baudrate = Baudrate,
        .Preemption = Preemption,
        .SubPriority = SubPriority,
        .AFIO_EN = AFIO_Disable
    };

    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_Mode = GPIO_Mode_AF_PP
    };
    
    USART_InitTypeDef USART_InitStruct = {
        .USART_BaudRate = Baudrate,
        .USART_WordLength = USART_WordLength_8b,
        .USART_StopBits = USART_StopBits_1,
        .USART_Parity = USART_Parity_No,
        .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
        .USART_HardwareFlowControl = USART_HardwareFlowControl_None
    };

    NVIC_InitTypeDef NVIC_InitStruct = {
        .NVIC_IRQChannelCmd = ENABLE,
        .NVIC_IRQChannelPreemptionPriority = Preemption,
        .NVIC_IRQChannelSubPriority = SubPriority
    };

    if (USARTx == USART1) {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        USART_Init(USART1, &USART_InitStruct);
        USART_Cmd(USART1, ENABLE);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
        USART_ITConfig(USART1, USART_IT_ORE, ENABLE);


        NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
        NVIC_Init(&NVIC_InitStruct);
    } 
    else if (USARTx == USART2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        USART_Init(USART2, &USART_InitStruct);
        USART_Cmd(USART2, ENABLE);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
        USART_ITConfig(USART2, USART_IT_ORE, ENABLE);

        NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
        NVIC_Init(&NVIC_InitStruct);
    }
}

uint8_t Serial_RxFlag2; // 串口2接收标志  
char Serial_RxPacket2[100]; // 串口2接收缓冲区  
/**
 * @brief  USART2中断服务函数（ATK-MW8266D专用）
 * @note   处理ESP8266的多行响应和帧结束判断
 */
void USART2_IRQHandler(void)
{

    if (USART_GetITStatus(USART2, USART_IT_ORE) != RESET)
    {
        volatile uint8_t temp = USART2->DR;  // 直接寄存器操作更高效
        uint32_t sr = USART2->SR;  // 必须读取SR
        uint8_t dr = USART2->DR;   // 必须读取DR
        (void)sr; (void)dr;        // 避免编译器警告
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        static uint8_t RxState = 0;
        static uint8_t pRxPacket = 0;
        uint8_t ch = USART_ReceiveData(USART2);
        
        if (esp8266_LoadingFinished_Flag == 0)
        {
            /* 存储到帧缓冲区 */
            if(esp8266_rx_frame.sta.len < ESP8622_UART_RX_BUF_SIZE-1)
            {
                esp8266_rx_frame.buf[esp8266_rx_frame.sta.len] = ch;
                            esp8266_rx_frame.sta.len++;  
            }else
            {
                esp8266_rx_frame.sta.len = 0;
                esp8266_rx_frame.buf[esp8266_rx_frame.sta.len]=ch;
                            esp8266_rx_frame.sta.len++;  
            }
            USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        }
        else
        {
            uint8_t RxData = ch;
            
            if (RxState == 0)
            {
                if (RxData == '@' && Serial_RxFlag2 == 0)
                {
                    RxState = 1;
                    pRxPacket = 0;
                    Serial_RxPacket2[pRxPacket] = '@';
                    pRxPacket ++;
                }
            }
            else if (RxState == 1)
            {
                if (RxData == '\r')
                {
                    RxState = 2;
                }
                else
                {
                    Serial_RxPacket2[pRxPacket] = RxData;
                    pRxPacket ++;
                }
            }
            else if (RxState == 2)
            {
                if (RxData == '\n')
                {
                    RxState = 0;
                    Serial_RxPacket2[pRxPacket] = '\0';
                    Serial_RxFlag2 = 1;
                }
            } 
            USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        }
    
    }
   
    if (USART_GetITStatus(USART2,USART_IT_IDLE) !=RESET)
    {
        USART_ReceiveData(USART2);
        USART_ClearITPendingBit(USART2, USART_IT_IDLE);
        esp8266_rx_frame.sta.finsh=1;
    }
    
}


/**
 * @brief  获取接收到的完整帧
 * @retval 帧数据指针（NULL表示无有效数据）
 */
uint8_t* ESP8266_GetRxFrame(void)
{
    if(esp8266_rx_frame.sta.finsh)
    {
         esp8266_rx_frame.buf[esp8266_rx_frame.sta.len] = '\0';
        return (uint8_t*)esp8266_rx_frame.buf;
    }
    return NULL;
}

/**
 * @brief  重启接收过程
 */
void ESP8266_RxRestart(void)
{
    esp8266_rx_frame.sta.len = 0;
    esp8266_rx_frame.sta.finsh = 0;
}


/**
 * @brief  获取接收到的完整帧的长度
 * @retval 0   : 未接收到一帧数据
 *         其他 : 接收到的一帧数据的长度
 */
int16_t ESP8266_GetRXFrame_Len(void)
{
   if (esp8266_rx_frame.sta.finsh == 1)
    {
        return esp8266_rx_frame.sta.len;
    }
    else
    {
        return 0;
    }

}
