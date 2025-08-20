#include "RP.h"


void RP_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);


    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitTypeDef GPIO_InitStruct=
    {
        .GPIO_Mode = GPIO_Mode_AIN,
        .GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4,
        .GPIO_Speed = GPIO_Speed_50MHz
    };


    GPIO_Init(GPIOA,&GPIO_InitStruct);
    // GPIO_Init(GPIOB,&GPIO_InitStruct);




    ADC_InitTypeDef ADC_InitStruct =
    {
        .ADC_ContinuousConvMode=DISABLE,
        .ADC_DataAlign =ADC_DataAlign_Right,
        .ADC_ExternalTrigConv=ADC_ExternalTrigConv_None,
        .ADC_Mode = ADC_Mode_Independent,
        .ADC_NbrOfChannel =1,
        .ADC_ScanConvMode =DISABLE
    };
    ADC_Init(ADC1,&ADC_InitStruct);
    
    // ADC_InitStruct.ADC_NbrOfChannel=2;

    // ADC_Init(ADC2,&ADC_InitStruct);

    ADC_Cmd(ADC1,ENABLE);
    // ADC_Cmd(ADC2,ENABLE);

    ADC_ResetCalibration(ADC2);
	while (ADC_GetResetCalibrationStatus(ADC2) == SET);
	ADC_StartCalibration(ADC2);
	while (ADC_GetCalibrationStatus(ADC2) == SET);

    // ADC_ResetCalibration(ADC1);
	// while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	// ADC_StartCalibration(ADC1);
	// while (ADC_GetCalibrationStatus(ADC1) == SET);
}



uint16_t RP_Getvalue(uint8_t RP_Number)
{
    switch (RP_Number)
    {
    // case  1:
    // ADC_RegularChannelConfig(ADC2,ADC_Channel_5,1,ADC_SampleTime_55Cycles5);
    // break;
    // case  2:
    // ADC_RegularChannelConfig(ADC2,ADC_Channel_4,1,ADC_SampleTime_55Cycles5);
    // break;
    case  3:
    ADC_RegularChannelConfig(ADC1,ADC_Channel_4,1,ADC_SampleTime_55Cycles5);
    break;
    case  4:
    ADC_RegularChannelConfig(ADC1,ADC_Channel_5,1,ADC_SampleTime_55Cycles5);
    break;
    default:
        Serial_Printf(USART1,"RP数据类型错误");
        break;
    }
    // if (RP_Number ==1 || RP_Number ==2)
    // {
    // ADC_SoftwareStartConvCmd(ADC2, ENABLE);
	// while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	// return ADC_GetConversionValue(ADC2);
    // }else
    // {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
    // }
}



    // pidA_inner.kp =RP_Getvalue(1)/4095.0*1;
    // pidA_inner.ki =RP_Getvalue(2)/4095.0*1;
    // pidA_inner.kp =RP_Getvalue(3)/4095.0*10;
    // TargetA=RP_Getvalue(4)/4095.0* 1000-100;
