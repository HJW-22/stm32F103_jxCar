#include "stm32f10x.h"                 
#include "MPU6050Reg.h"
#include "MPU6050.h"
#include "Delay.h"
#include "Serial.h"
#include <string.h>  // 添加这一行 
#include <math.h>
  
#define MPU6050_ADDRESS 0xD0
#define MPU6050_TIMEOUT 10000

volatile uint8_t DMA_Ready = 1;

// 在全局变量声明处添加
__attribute__((aligned(4))) MPU6050_DataTypeDef MPU6050_Data;
volatile MPU6050_State_t MPU6050_State = MPU_IDLE;
MPU6050_AngleData MPU6050_Angle;


// 定义全局超时变量  
volatile uint16_t I2C_Timeout = MPU6050_TIMEOUT;


// 只在一个源文件中定义全局变量  
float angleX = 0.0f;  
float angleY = 0.0f;  
float angleZ = 0.0f;  
float dt = 0.003f;    

#define MPU6050_ADDRESS 0xD0

// 重写MyI2C_CheckEvent函数  
void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)  
{  
  uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}  

void MPU6050_WriteReg(uint8_t RegAddress,int8_t Data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, Data);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}

void MPU6050_ReadReg(MPU6050_DataTypeDef* DataStruct)
{
    uint8_t i, Data[14]; 

	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, MPU6050_ACCEL_XOUT_H);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(I2C2, ENABLE);  
    for ( i = 0; i < 13; i++)
    {
   MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[i] = I2C_ReceiveData(I2C2);
  
    }

    I2C_AcknowledgeConfig(I2C2,DISABLE);
    I2C_GenerateSTOP(I2C2,ENABLE); 

     MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[13] = I2C_ReceiveData(I2C2);

		

     // 组装数据  
    DataStruct->AccX = (Data[0] << 8) | Data[1];  
    DataStruct->AccY = (Data[2] << 8) | Data[3];  
    DataStruct->AccZ = (Data[4] << 8) | Data[5];  
    
    DataStruct->Temp = (Data[6] << 8) | Data[7];  
    
    DataStruct->GyroX = (Data[8] << 8) | Data[9];  
    DataStruct->GyroY = (Data[10] << 8) | Data[11];  
    DataStruct->GyroZ = (Data[12] << 8) | Data[13]; 
    
}

uint8_t MPU6050_GetID(void)
{
    uint8_t id = 0;
    
    // 发送起始条件
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
    
    // 发送设备地址(写模式)
    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    
    // 发送要读取的寄存器地址(WHO_AM_I)
    I2C_SendData(I2C2, MPU6050_WHO_AM_I);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    
    // 发送重复起始条件
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
    
    // 发送设备地址(读模式)
    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    
    // 禁用应答(因为只读取一个字节)
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    
    // 发送停止条件
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    // 等待数据接收完成
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
    
    // 读取数据
    id = I2C_ReceiveData(I2C2);
    
    // 重新启用应答(为后续操作做准备)
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    
    return id;
}



// 初始化典型配置  
void MPU6050_Init(void)  
{  
    // MyI2C_Init();  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStructurn;
    I2C_InitStructurn.I2C_Ack=I2C_Ack_Enable;
    I2C_InitStructurn.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    I2C_InitStructurn.I2C_ClockSpeed=400000;
    I2C_InitStructurn.I2C_DutyCycle=I2C_DutyCycle_16_9;
    I2C_InitStructurn.I2C_Mode=I2C_Mode_I2C;
    I2C_InitStructurn.I2C_OwnAddress1=0x00;
    I2C_Init(I2C2,&I2C_InitStructurn);

    I2C_Cmd(I2C2,ENABLE);


    // 复位MPU6050  
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);  
    Delay_ms(100);  // 延时等待复位完成  
    
    // 唤醒并选择时钟源  
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLKSEL);  
    
    // 配置陀螺仪量程 ±500°/s  
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);  
    
    // 配置加速度计量程 ±4g  
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);  
    
    // 配置采样率 (可选)  
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);  // 采样率 = 陀螺仪输出率 / (1 + 7) = 1kHz  
}  



// 修改后的DMA读取函数
void MPU6050_DMA_Read(void) {
    if (MPU6050_State != MPU_IDLE) return;
    
    MPU6050_State = MPU_READ_REQUESTED;
    
    // 1. 发送起始条件+设备地址+寄存器地址
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    
    I2C_SendData(I2C2, MPU6050_ACCEL_XOUT_H);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED); 
    
    // 2. 重新发送起始条件，切换到读模式
    I2C_GenerateSTART(I2C2, ENABLE);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
    MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
 
    // 3. 配置并启动DMA传输
    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel7, sizeof(MPU6050_DataTypeDef));
    I2C_DMALastTransferCmd(I2C2, ENABLE);  // 重要！最后一次传输生成NACK
    DMA_Cmd(DMA1_Channel7, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);
    
    // 4. 设置状态为等待DMA完成
    MPU6050_State = MPU_DATA_READY;
}




// 检查数据是否就绪
uint8_t MPU6050_DMA_IsDataReady(void) {
    return DMA_Ready;
}

// 数据转换函数（可选）  
void MPU6050_ConvertData(MPU6050_DataTypeDef* RawData,   
                          float* AccData,   
                          float* GyroData,   
                          float* Temperature)  
{  
    // 根据初始化时的量程进行转换  
    // 加速度计转换（假设±4g量程）  
    AccData[0] = (float)RawData->AccX / 8192.0f;  // ±4g 时灵敏度  
    AccData[1] = (float)RawData->AccY / 8192.0f;  
    AccData[2] = (float)RawData->AccZ / 8192.0f;  
    
    // 陀螺仪转换（假设±500°/s量程）  
    GyroData[0] = (float)RawData->GyroX / 65.5f;  // ±500°/s 时灵敏度  
    GyroData[1] = (float)RawData->GyroY / 65.5f;  
    GyroData[2] = (float)RawData->GyroZ / 65.5f;  
    
    // 温度转换（MPU6050手册中的转换公式）  
    *Temperature = (float)RawData->Temp / 340.0f + 36.53f;  
}  
void sendFloat(float value, USART_TypeDef *USARTx) {
    char buffer[20]; // 创建一个足够大的缓冲区来存放数字
    snprintf(buffer, sizeof(buffer), "%.2f", value); // 转换为字符串，保留两位小数
    Serial_Printf(USARTx,"angleY的值是:");
    Serial_SendString(buffer, USARTx); // 正确地发送字符串
    Serial_SendString("\n", USARTx); // 发送换行符
}

void MPU6050_CalculateAngle(MPU6050_DataTypeDef* DataStruct)  
{  
    // 角度计算  
    // float gyroXrate = (float)DataStruct->GyroX / 65.5f; // 角速度转换  
    float gyroYrate = (float)DataStruct->GyroY / 65.5f;  
    
    // 积分计算角度  
    // angleX += gyroXrate * dt;  // 积分获得X轴角度  
    angleY += gyroYrate * dt;  // 积分获得Y轴角度  

    // 使用加速度计进行一定的校正  
    // float accXangle = atan2(DataStruct->AccY, DataStruct->AccZ) * 180 / M_PI; // 计算俯仰角  
    float accYangle = atan2(-DataStruct->AccX, sqrt(DataStruct->AccY * DataStruct->AccY + DataStruct->AccZ * DataStruct->AccZ)) * 180 / M_PI; // 计算偏航角  

    // 简单互补滤波法  
    // angleX = 0.98f * angleX + 0.02f * accXangle;   // 融合角度  
    angleY = 0.98f * angleY + 0.02f * accYangle;  

    //  sendFloat(angleY, USART1); // 发送格式化后的浮点数
    
    // MPU6050_Angle.X_Angle=angleX;
    MPU6050_Angle.Y_Angle=angleY;
  
}  



void DMA1_Channel7_IRQHandler(void) {
    if(DMA_GetITStatus(DMA1_IT_TC7)) {
        DMA_ClearITPendingBit(DMA1_IT_TC7);
        I2C_GenerateSTOP(I2C2, ENABLE); // 发送停止信号
        MPU6050_State = MPU_DATA_READY; // 更新状态
    }
}


void MPU6050_DMA_Init(void)  
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Channel5);
    
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_BufferSize=sizeof(MPU6050_DataTypeDef);
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M=DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)&MPU6050_Data;
    DMA_InitStructure.DMA_MemoryDataSize=DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(I2C2->DR);
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority=DMA_Priority_High;
    

    DMA_Init(DMA1_Channel5,&DMA_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;

    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    I2C_DMACmd(I2C2, ENABLE);

}
