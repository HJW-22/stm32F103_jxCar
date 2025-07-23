#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f10x.h"                  // Device header

#define M_PI 3.14159265358979323846  

typedef struct   
{  
    int16_t AccX;  
    int16_t AccY;  
    int16_t AccZ;  
    int16_t Temp;  
    int16_t GyroX;  
    int16_t GyroY;  
    int16_t GyroZ;  
} MPU6050_OriginalData;  

typedef struct   
{  
    float X_Angle;
    float Y_Angle;
    float Z_Angle;
} MPU6050_AngleData;  

typedef enum {
    MPU_IDLE,           //空闲状态
    MPU_READ_REQUESTED, //读取MPU6050请求
    MPU_DMA_READING,    //读取MPU6050寄存器中
    MPU_DATA_READY      //准备被DMA读取
} MPU6050_StateData;

typedef enum {
    MPU_ALL,           
    MPU_TEMP, 
    MPU_Y,   
    MPU_X,
    MPU_Z
} MPU6050_GetOriginalType;

typedef enum {
    MPU_ALL,           
    MPU_TEMP, 
    MPU_Y,   
    MPU_X,
    MPU_Z
} MPU6050_GetCalculateType;

extern MPU6050_StateData    MPU6050_State;
extern MPU6050_OriginalData MPU6050_Data;
extern MPU6050_AngleData    MPU6050_Angle;

void MPU6050_Init(void); 
uint8_t MPU6050_GetID(void);
void MPU6050_WriteReg(uint8_t RegAddress,int8_t Data);
void MPU6050_ReadReg(MPU6050_OriginalData* DataStruct);
void MPU6050_CalculateAngle(MPU6050_OriginalData *MPU6050_Original,MPU6050_AngleData *MPU6050_Angle);



void MPU6050_DMA_Init(void); 
void MPU6050_DMA_Read(void);
uint8_t MPU6050_DMA_IsDataReady(void);


#endif


					



