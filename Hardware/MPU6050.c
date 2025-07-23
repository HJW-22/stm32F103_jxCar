#include "stm32f10x.h"
#include "MPU6050Reg.h"
#include "MPU6050.h"
#include "Delay.h"
#include "Serial.h"
#include <string.h> // 添加这一行
#include <math.h>

#define MPU6050_ADDRESS 0xD0
#define MPU6050_TIMEOUT 10000

uint8_t DMA_Ready = 1;

// 在全局变量声明处添加
MPU6050_OriginalData MPU6050_Data;
MPU6050_StateData     MPU6050_State = MPU_IDLE;
MPU6050_AngleData   MPU6050_Angle;

// 定义全局超时变量
uint16_t I2C_Timeout = MPU6050_TIMEOUT;

// 只在一个源文件中定义全局变量
float angleX = 0.0f;
float angleY = 0.0f;
float angleZ = 0.0f;
float dt     = 0.003f;

#define MPU6050_ADDRESS 0xD0


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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStructurn;
    I2C_InitStructurn.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructurn.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructurn.I2C_ClockSpeed          = 400000;
    I2C_InitStructurn.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    I2C_InitStructurn.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructurn.I2C_OwnAddress1         = 0x00;
    I2C_Init(I2C2, &I2C_InitStructurn);

    I2C_Cmd(I2C2, ENABLE);

    // 复位MPU6050
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET);
    Delay_ms(100); // 延时等待复位完成

    // 唤醒并选择时钟源
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLKSEL);

    // 配置陀螺仪量程 ±500°/s
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);

    // 配置加速度计量程 ±4g
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);

    // 配置采样率 (可选)
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07); // 采样率 = 陀螺仪输出率 / (1 + 7) = 1kHz
}


void MPU6050_CalculateAngle(MPU6050_OriginalData *MPU6050_Original,MPU6050_AngleData *MPU6050_Angle)
{
// 常量定义
#define DT        0.003f // 采样时间3ms
#define GYRO_SENS 65.5f  // 陀螺仪灵敏度
#define ALPHA     0.96f  // 互补滤波系数
    static float angleY      = 0.0f;
    static float gyroYoffset = 0.0f; // 需通过校准获得

    // 陀螺仪角度积分（需校准零偏）
    float gyroYrate = ((float)DataStruct->GyroY - gyroYoffset) / GYRO_SENS;
    angleY += gyroYrate * DT;

    // 加速度计角度计算（实际是Roll角）
    float accRoll = atan2(-DataStruct->AccX,
                          sqrt(DataStruct->AccY * DataStruct->AccY +
                               DataStruct->AccZ * DataStruct->AccZ)) *
                    180 / M_PI;

    // 互补滤波融合
    angleY = ALPHA * angleY + (1 - ALPHA) * accRoll;

    // 角度限幅
    if (angleY > 180.0f) angleY = 180.0f;
    if (angleY < -180.0f) angleY = -180.0f;

    MPU6050_Angle.Y_Angle = angleY;
}








// 检查数据是否就绪
uint8_t MPU6050_DMA_IsDataReady(void)
{
    return DMA_Ready;
}

// 修改后的DMA读取函数
void MPU6050_DMA_Read(void)
{
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
    I2C_DMALastTransferCmd(I2C2, ENABLE); // 重要！最后一次传输生成NACK
    DMA_Cmd(DMA1_Channel7, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);

    // 4. 设置状态为等待DMA完成
    MPU6050_State = MPU_DATA_READY;
}


void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC7)) {
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
    DMA_InitStructure.DMA_BufferSize         = sizeof(MPU6050_OriginalData);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&MPU6050_Data;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2C2->DR);
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;

    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;

    NVIC_Init(&NVIC_InitStructure);

    DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);

    I2C_DMACmd(I2C2, ENABLE);
}
