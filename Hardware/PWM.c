#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void PWM_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); //打开TIM1通用定时器外设
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //打开GPIOA外设
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
		
	
	
	TIM_InternalClockConfig(TIM1);  //使用内部时钟 72MHz 默认为（可以不设置）

	//TIM2 结构体初始化 1000频率
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; //声明结构体
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; //时钟
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=ARR;
	TIM_TimeBaseInitStructure.TIM_Prescaler=PSC; 
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0; //重复计数器，高级计数器专属
	TIM_TimeBaseInit( TIM1,&TIM_TimeBaseInitStructure);


	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse= CCR;                //CCR
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能	
	TIM_Cmd(TIM1,ENABLE); //使能定时器1
	// 高级定时器重要配置  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);  // 使能PWM输出  



}
void TIM1_NVIC_Init(){
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);//使能TIM中断源，即寄存器值1操作
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //优先级配置为2，即抢占和从优先级为3:3
	//中断结构体初始化
	NVIC_InitTypeDef NVIC_InitStructure; //声明结构体
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn; //选择中断
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//打开中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //抢占优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1; //从优先级
	NVIC_Init(&NVIC_InitStructure); 
}


void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM1,Compare);
}


void PWM_SetCompare4(uint16_t Compare)
{                                                                                                                                                                                                                                                                                                                                                                                                                
	TIM_SetCompare4(TIM1,Compare);
}

