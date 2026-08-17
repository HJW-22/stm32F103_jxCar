#include "bsp_usart.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>  
#include <string.h>  // 添加这一行 
#include "oled.h"
#define RX_PACKET_MAX_LENGTH 99  // 预留一个字节给字符串结束符  


char Serial_RxPacket1[RX_PACKET_MAX_LENGTH + 1];  // +1 用于存储字符串结束符   
// char Serial_RxPacket2[RX_PACKET_MAX_LENGTH + 1]; // 串口2接收缓冲区  
char Serial_RxPacket3[RX_PACKET_MAX_LENGTH + 1]; // 串口3接收缓冲区  

uint8_t Serial_RxFlag1;
// uint8_t Serial_RxFlag2; // 串口2接收标志  
uint8_t Serial_RxFlag3; // 串口3接收标志 

/**
  * @brief  初始化串口 UAST1与UAST2(优先级在Serial.h选择)
  * @param  无
  * @retval 无
  */
void Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    #ifdef USART1_FLAG
    
	  //------------------- 串口1初始化 --------------------
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  
	  
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //TX
	   GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10; //RX
	  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	  
	  USART_InitStructure.USART_BaudRate = 128000;                          //波特率
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //8位数据位
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;              //停止位1
	  USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     //发送接收模式
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //无硬件流控制
  
	  USART_Init(USART1, &USART_InitStructure);
	  USART_Cmd(USART1, ENABLE);
  
	  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	  
  
	
	  NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=usart1_preemption;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority=usart1_sub;
	  NVIC_Init(&NVIC_InitStructure);

	#endif // USART1_FLAG

	#ifdef USART2_FLAG
	 //------------------- 串口2初始化 --------------------
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //TX
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3; //RX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;                          //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;              //停止位1
	USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     //发送接收模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //无硬件流控制


	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);


	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	

	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=usart2_preemption;  //抢占级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=usart2_sub; //响应级
	NVIC_Init(&NVIC_InitStructure);
  	#endif



	
	#ifdef USART3_FLAG
    //------------------- 串口3初始化 --------------------
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //TX
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11; //RX
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_0); // 设置为高电平（3.3V） 
	GPIO_ResetBits(GPIOB, GPIO_Pin_1); // 设置为低电平(GND)

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200;                          //波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;         //8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;              //停止位1
    USART_InitStructure.USART_Parity = USART_Parity_No;                 //无奇偶校验
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     //发送接收模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //无硬件流控制

    
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);


    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

    // GPIO_StructInit(&GPIO_InitStructure);
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_0;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);

    // GPIO_SetBits(GPIOB, GPIO_Pin_0); // 设置为高电平（3.3V） 
    // GPIO_ResetBits(GPIOB, GPIO_Pin_1); // 设置为低电平(GND)



    NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //抢占级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; //响应级
    NVIC_Init(&NVIC_InitStructure);
	#endif // DEBUG
}
void Serial_Printf(USART_TypeDef *USARTx,  char *format, ...)  
{  
    char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
    // 发送格式化后的字符串  
    Serial_SendString(String, USARTx);  
}  

// 发送字节，增加串口选择参数  
void Serial_SendByte(uint8_t Byte, USART_TypeDef *USARTx)  
{  
    USART_SendData(USARTx, Byte);  
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); //  
}  

// 发送数组  
void Serial_SendArray(uint8_t *Array,uint16_t Length, USART_TypeDef *USARTx)  
{  
    uint16_t i;  
    for (i = 0; i < Length; i++)  
    {  
        Serial_SendByte(Array[i], USARTx);  
    }  
}  

// 发送字符串  
void Serial_SendString(char *String, USART_TypeDef *USARTx){  
    uint8_t i;  
    for (i = 0; String[i] != '\0'; i++)  
    {  
        Serial_SendByte(String[i], USARTx);  
    }  
}  


//x*y 数学函数
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(int32_t Number, uint8_t Length, USART_TypeDef *USARTx)  
{  
    char buffer[12]; // 足够大的缓冲以存放最大32位整型数字，包含负号和结束符 '\0'  
    uint8_t i = 0;

    // 处理负数
    if (Number < 0) {
        Serial_SendByte('-', USARTx); // 先发送负号
        Number = -Number; // 转为正数
    }

    // 将数字转成字符串形式
    uint32_t temp = Number; // 使用临时变量，避免直接修改 Number
    for (i = 0; i < Length; i++) {
        buffer[Length - i - 1] = (temp % 10) + '0'; // 从低位开始取出数字
        temp /= 10;
    }

    // 如果需要，清理前导零
    if (temp > 0) {
        // 如果有剩余的值，大于 Length，表示超出范围，比如 1000 要输出 1000，而不是06
        while (temp > 0 && i < Length) {
            buffer[Length - i - 1] = (temp % 10) + '0';
            temp /= 10;
            i++;
        }
    }

    // 确保以 '\0' 结尾
    buffer[Length] = '\0'; 

    // 发送整个数字字符串
    Serial_SendString(buffer, USARTx);
}



#ifdef USART1_FLAG

void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag1 == 0)
			{
				RxState = 1;
				pRxPacket = 0;
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
				Serial_RxPacket1[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		else if (RxState == 2)
		{
			if (RxData == '\n')
			{
				RxState = 0;
				Serial_RxPacket1[pRxPacket] = '\0';
				Serial_RxFlag1 = 1;
			}
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}


void USART1_DMA_Init(void) {  
	// 使能DMA时钟  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
	
	DMA_InitTypeDef DMA_InitStructure;  
	
	// TX DMA配置  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  //外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer_USART1;  //缓冲区基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //指定外设是源还是目的地destination
	DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;  //缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //设置源(外设)是否自动增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //设置目的地(缓冲区)是否自动增加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据的类型
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  //缓冲区数据的类型
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //模式循环还是正常
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //内存到内存模式
	
	// 初始化DMA  
	DMA_Init(USART1_TX_DMA_CHANNEL, &DMA_InitStructure);  
	
	// 使能USART1的DMA发送  
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
}


void USART1_DMA_Send(uint8_t* data, uint16_t size) {  
	// 确保不超过缓冲区大小  
	size = (size > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : size;  
	
	// 复制数据到缓冲区  
	memcpy(TxBuffer_USART1, data, size);  
	
	// 关闭DMA  
	DMA_Cmd(USART1_TX_DMA_CHANNEL, DISABLE);  
	
	// 设置传输数据量  
	DMA_SetCurrDataCounter(USART1_TX_DMA_CHANNEL, size);  
	
	// 更新内存地址  
	USART1_TX_DMA_CHANNEL->CMAR = (uint32_t)TxBuffer_USART1;  
	
	// 使能DMA  
	DMA_Cmd(USART1_TX_DMA_CHANNEL, ENABLE);  
}



#endif // DEBUG


#ifdef USART2_FLAG

void USART2_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART2);
		
		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag2 == 0)
			{
				RxState = 1;
				pRxPacket = 0;
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




void USART2_DMA_Init(void) {  
	// 使能DMA时钟  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
	
	DMA_InitTypeDef DMA_InitStructure;  
	
	// TX DMA配置  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;  //外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer_USART2;  //缓冲区基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //指定外设是源还是目的地destination
	DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;  //缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //设置源(外设)是否自动增加
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //设置目的地(缓冲区)是否自动增加
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据的类型
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  //缓冲区数据的类型
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //模式循环还是正常
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //内存到内存模式
	
	// 初始化DMA  
	DMA_Init(USART2_TX_DMA_CHANNEL, &DMA_InitStructure);  
	
	// 使能USART2的DMA发送  
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  
}


void USART2_DMA_Send(uint8_t* data, uint16_t size) {  
	// 确保不超过缓冲区大小  
	size = (size > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : size;  
	
	// 复制数据到缓冲区  
	memcpy(TxBuffer_USART2, data, size);  
	
	// 关闭DMA  
	DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);  
	
	// 设置传输数据量  
	DMA_SetCurrDataCounter(USART2_TX_DMA_CHANNEL, size);  
	
	// 更新内存地址  
	DMA1_Channel7->CMAR = (uint32_t)TxBuffer_USART2;  
	
	// 使能DMA  
	DMA_Cmd(USART2_TX_DMA_CHANNEL, ENABLE);  
}

#endif // USART2_FLAG


#ifdef USART3_FLAG

void USART3_IRQHandler(void)
{

	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART3);
		
		if (RxState == 0)
		{
			if (RxData == '@' && Serial_RxFlag3 == 0)
			{
				RxState = 1;
				pRxPacket = 0;
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
				Serial_RxPacket3[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
		else if (RxState == 2)
		{
			if (RxData == '\n')
			{
				RxState = 0;
				Serial_RxPacket3[pRxPacket] = '\0';
				Serial_RxFlag3 = 1;
			}
		}
		
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
} 



void USART3_DMA_Init(void) {  
    // 使能DMA时钟  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  
    
    DMA_InitTypeDef DMA_InitStructure;  
    
    // TX DMA配置  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;  //外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer_USART3;  //缓冲区基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //指定外设是源还是目的地destination
    DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;  //缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //设置源(外设)是否自动增加
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //设置目的地(缓冲区)是否自动增加
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据的类型
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  //缓冲区数据的类型
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //模式循环还是正常
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;   //内存到内存模式
    
    // 初始化DMA  
    DMA_Init(USART3_TX_DMA_CHANNEL, &DMA_InitStructure);  
    
    // 使能USART3的DMA发送  
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  
}  

// DMA发送函数  
void USART3_DMA_Send(uint8_t* data, uint16_t size) {  
    // 确保不超过缓冲区大小  
    size = (size > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : size;  
    
    // 复制数据到缓冲区  
    memcpy(TxBuffer_USART3, data, size);  
    
    // 关闭DMA  
    DMA_Cmd(USART3_TX_DMA_CHANNEL, DISABLE);  
    
    // 设置传输数据量  
    DMA_SetCurrDataCounter(USART3_TX_DMA_CHANNEL, size);  
    
    // 更新内存地址  
    DMA1_Channel2->CMAR = (uint32_t)TxBuffer_USART3;  
    
    // 使能DMA  
    DMA_Cmd(USART3_TX_DMA_CHANNEL, ENABLE);  
}  

#endif // USART3_FLAG


