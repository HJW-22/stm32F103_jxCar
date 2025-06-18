#ifndef __MPU6050_H
#define __MPU6050_H

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
} MPU6050_DataTypeDef;  

typedef enum {
    MPU_IDLE,
    MPU_READ_REQUESTED,
    MPU_DATA_READY
} MPU6050_State_t;

extern volatile MPU6050_State_t MPU6050_State;
extern MPU6050_DataTypeDef MPU6050_Data;

void MPU6050_DMA_Read(MPU6050_DataTypeDef* data);
uint8_t MPU6050_DMA_IsDataReady(void);
void MPU6050_WriteReg(uint8_t RegAddress,int8_t Data);
void MPU6050_ReadReg(MPU6050_DataTypeDef* DataStruct);
void MPU6050_Init(void); 
void MPU6050_ConvertData(MPU6050_DataTypeDef* RawData,float* AccData,float* GyroData,float* Temperature);  
void MPU6050_CalculateAngle(MPU6050_DataTypeDef* DataStruct);
uint8_t MPU6050_GetID(void);

#endif


					



