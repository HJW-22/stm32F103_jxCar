#include "stm32f10x.h"    // Device header
#include "Delay.h"
void Key_Init(void)
{
   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15 |GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);


    	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line=EXTI_Line15|EXTI_Line12;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // 设置响应的中断通道  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 设置抢占优先级  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 设置响应优先级  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能这个通道  
    NVIC_Init(&NVIC_InitStructure);  
    
}



