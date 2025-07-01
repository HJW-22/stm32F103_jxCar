#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "stdlib.h"

void Motor_Init(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //打开GPIOA外设
	
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;   
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOB,&GPIO_InitStructure);
		PWM_init();
		TIM2_NVIC_Init();
}


void MotorB_SetSpeed(int16_t Speed)
{
		if(Speed >0)
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_13);//反转
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
			
			PWM_SetCompare1(Speed);
		}else if(Speed<0){
			
			GPIO_SetBits(GPIOB,GPIO_Pin_12); //正转
			GPIO_ResetBits(GPIOB,GPIO_Pin_13);
			Speed=abs(Speed);
			PWM_SetCompare1(Speed);
		}else 
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_13);//停转
			GPIO_ResetBits(GPIOB,GPIO_Pin_12);
			
			PWM_SetCompare1(Speed);
		}			


}
void MotorA_SetSpeed(int16_t Speed)
{
		if(Speed >0)
		{
			GPIO_SetBits(GPIOB,GPIO_Pin_14); //正转
			GPIO_ResetBits(GPIOB,GPIO_Pin_15);
			
			PWM_SetCompare2(Speed);
		}else if(Speed<0){
		GPIO_SetBits(GPIOB,GPIO_Pin_15);   // 反转  
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);  
		Speed=abs(Speed);
		PWM_SetCompare2(Speed);
		}else 
		{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);//停转
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);
		PWM_SetCompare2(Speed);
		}			


}






