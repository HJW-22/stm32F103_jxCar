#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f10x.h"                  // Device header


typedef struct MPU6050_raw
{
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    uint16_t Temp;
}MPU6050_raw;


typedef struct MPU6050
{
    float yaw;
    float roll;
    float pitch;
}MPU6050;



void MPU6050_Init(I2C_TypeDef *I2Cx,uint8_t AFIO_EN);
void MPU6050_Get_Angle(MPU6050* this);
float MPU6050_GetTemp(void);
uint8_t MPU6050_ID(void);
void MPU6050_Get_Angle_Plus(MPU6050* this);


#endif


					



