#ifndef __I2C_H__
#define __I2C_H__
#include "stm32f10x.h"
#define SI2C_delay_time 0																			//设置硬件I2C的延时速度
// #define I2C(obj) (Pthis_I2C = &obj)																	//Pthis全局指针宏定义
typedef struct I2C_Private I2C_Private;		

typedef struct I2C_BUS
{
	//严禁使用该指针!(私有变量) It is strictly forbidden to use this pointer
	I2C_Private* Private;

	//常用函数                 			                                                              //用户API函数接口
	void (*Write_Reg)(struct I2C_BUS* bus,uint8_t RegAddress, uint16_t Data);											 //写寄存器函数(默认8bit操作),write register by I2C bus
	uint16_t (*Read_Reg)(struct I2C_BUS* bus,uint8_t RegAddress);													     //读寄存器函数(默认8bit操作),read register by I2C bus

	//扩展函数
	uint8_t (*ScanAdress)(struct I2C_BUS* bus);																		 //扫描IIC地址并返回,Scan IIC address and return it
	uint8_t (*AckTest)(struct I2C_BUS* bus);																			 //响应接口(1 success,0 failed),it can check that if our I2C bus is init succese
	uint8_t Mode16bit;																				 //将IIC升级为16位操作(置1为升级),boost reg operation to 16bit
	void (*Rest_Speed)(struct I2C_BUS* bus,uint32_t Speed);																 //硬件I2C重新设置速度,you can reset your Hardware I2C Speed	
	void (*Write_Reg_continue)(struct I2C_BUS* bus,uint8_t RegAddress,uint16_t Count,uint8_t* Data);//连续写寄存器函数,continue write register by I2C bus，为了适应移植
	void (*Read_Reg_continue)(struct I2C_BUS* bus,uint8_t RegAddress,uint16_t Count,uint8_t* Data); //连续读寄存器函数,continue read register by I2C bus，为了适应移植

}I2C_BUS;

// extern I2C_BUS* Pthis_I2C;

//初始化函数
I2C_BUS Create_SI2C(GPIO_TypeDef* GPIOx,uint16_t SCL,uint16_t SDA,uint8_t Address);					//创建软件I2C对象,create a softwere I2C
I2C_BUS Create_HI2C(I2C_TypeDef* I2Cx,uint8_t Address);												//创建硬件I2C对象,create a hardware I2C


#endif
