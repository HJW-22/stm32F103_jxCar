#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f10x.h" // Device header

typedef struct MPU6050_raw {
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    uint16_t Temp;
} MPU6050_raw;

typedef struct MPU6050 {
    float yaw;
    float roll;
    float pitch;
} MPU6050;

/* MPU6050 is fixed to I2C2 on PB10/PB11 at 100 kHz. */
void MPU6050_Init(void);
uint8_t MPU6050_IsOnline(void);
void MPU6050_Get_Raw(MPU6050_raw *this);
void MPU6050_Get_Angle(MPU6050 *this);
float MPU6050_GetTemp(void);
uint8_t MPU6050_ID(void);
void MPU6050_Get_Angle_Plus(MPU6050 *this);

#endif
