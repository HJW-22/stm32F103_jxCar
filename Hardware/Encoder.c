#include "stm32f10x.h"                  // Device header


void Encoder_TIM4_Init(void)
{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); //打开TIM2通用定时器外设
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//打开GPIOA的外设
	
		//声明结构体
		GPIO_InitTypeDef GPIO_InitSrtucture;
		GPIO_InitSrtucture.GPIO_Mode=GPIO_Mode_IPU; //上拉输入模式
		GPIO_InitSrtucture.GPIO_Pin=GPIO_Pin_6 |GPIO_Pin_7;
		GPIO_InitSrtucture.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitSrtucture);
	
			
		//TIM2 结构体初始化
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; //声明结构体
		TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟
		TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
		TIM_TimeBaseInitStructure.TIM_Period=65536 - 1;
		TIM_TimeBaseInitStructure.TIM_Prescaler=1 - 1; 
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0; //重复计数器，高级计数器专属
		TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	
		//捕获模式声明结构体
		TIM_ICInitTypeDef TIM_ICInitSrtucture;
		TIM_ICStructInit(&TIM_ICInitSrtucture);//声明默认的结构体值，预防冲突
		TIM_ICInitSrtucture.TIM_Channel=TIM_Channel_1;//通道1
		TIM_ICInitSrtucture.TIM_ICFilter=0xF; //分频器的频率
		TIM_ICInitSrtucture.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿触发
		TIM_ICInit(TIM4,&TIM_ICInitSrtucture);

		TIM_ICStructInit(&TIM_ICInitSrtucture); // 重新初始化  
		
		TIM_ICInitSrtucture.TIM_Channel=TIM_Channel_2;
		TIM_ICInitSrtucture.TIM_ICFilter=0xF;
		TIM_ICInitSrtucture.TIM_ICPolarity=TIM_ICPolarity_Rising;
		TIM_ICInit(TIM4,&TIM_ICInitSrtucture);
		
		
		TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
		
		
		TIM_Cmd(TIM4,ENABLE);
		
		
}

void Encoder_TIM3_Init(void)
{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //打开TIM2通用定时器外设
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//打开GPIOA的外设
	
	 	//声明结构体
		GPIO_InitTypeDef GPIO_InitSrtucture;
		GPIO_InitSrtucture.GPIO_Mode=GPIO_Mode_IPU; //上拉输入模式
		GPIO_InitSrtucture.GPIO_Pin=GPIO_Pin_7 |GPIO_Pin_6;
		GPIO_InitSrtucture.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitSrtucture);
			
		//TIM2 结构体初始化
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; //声明结构体
		TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟
		TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
		TIM_TimeBaseInitStructure.TIM_Period=65536 - 1;
		TIM_TimeBaseInitStructure.TIM_Prescaler=1 - 1; 
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0; //重复计数器，高级计数器专属
		TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	
		//捕获模式声明结构体
		TIM_ICInitTypeDef TIM_ICInitSrtucture;
		TIM_ICStructInit(&TIM_ICInitSrtucture);//声明默认的结构体值，预防冲突
		TIM_ICInitSrtucture.TIM_Channel=TIM_Channel_1;//通道1
		TIM_ICInitSrtucture.TIM_ICFilter=0xF; //分频器的频率
		TIM_ICInitSrtucture.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升沿触发
		TIM_ICInit(TIM3,&TIM_ICInitSrtucture);

		TIM_ICStructInit(&TIM_ICInitSrtucture); // 重新初始化  
		
		TIM_ICInitSrtucture.TIM_Channel=TIM_Channel_2;
		TIM_ICInitSrtucture.TIM_ICFilter=0xF;
		TIM_ICInitSrtucture.TIM_ICPolarity=TIM_ICPolarity_Rising;
		TIM_ICInit(TIM3,&TIM_ICInitSrtucture);
		
		
		TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
		
		
		TIM_Cmd(TIM3,ENABLE);
		
		
}


int16_t Encoder_TIM4_Get(void)
{
			int16_t Temp ;
			Temp =TIM_GetCounter(TIM4);
			TIM_SetCounter(TIM4,0);
			return  Temp;
}

int16_t Encoder_TIM3_Get(void)
{
			int16_t Temp ;
			Temp =TIM_GetCounter(TIM3);
			TIM_SetCounter(TIM3,0);
			return  -Temp;
}

