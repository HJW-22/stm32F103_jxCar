#include "stm32f10x.h"                  // Device header
#include "_I2C.h"
#include "stdlib.h"

// #define SI2C_CHRONOLOGY_DELAY_FLAG


typedef struct I2C_Private 
{
    GPIO_TypeDef* GPIOx;
    I2C_TypeDef* I2Cx;
    int16_t SDA;
    int16_t SCL;
    int16_t I2C_Add;
    int8_t  Hard_I2C_EN;    
}I2C_Private;



I2C_BUS* Pthis_I2C = 0;//全局指针

#ifdef  SI2C_CHRONOLOGY_DELAY_FLAG
#define SI2C_CHRONOLOGY_DELAY_TIME 1 // 纳秒(us)为单位延时
uint8_t SI2C_Delay = SI2C_CHRONOLOGY_DELAY_TIME;
#endif

//------------------------软件I2C基本时序部分------------------------
void SI2C_I2C_W_SCL(uint8_t BitValue)
{
    if (BitValue != Bit_RESET)
        Pthis_I2C->Private->GPIOx->BSRR=Pthis_I2C->Private->SCL;
    else
        Pthis_I2C->Private->GPIOx->BRR=Pthis_I2C->Private->SCL;
#ifdef SI2C_CHRONOLOGY_DELAY_FLAG
    Delay_us(SI2C_Delay); 
#endif // SI2C_CHRONOLOGY_DELAY_FLAG
}

void SI2C_I2C_W_SDA(uint8_t BitValue)
{
     if (BitValue != Bit_RESET)
        Pthis_I2C->Private->GPIOx->BSRR=Pthis_I2C->Private->SDA;
    else
        Pthis_I2C->Private->GPIOx->BRR=Pthis_I2C->Private->SDA;
#ifdef SI2C_CHRONOLOGY_DELAY_FLAG
    Delay_us(SI2C_Delay);
#endif // SI2C_CHRONOLOGY_DELAY_FLAG
}

uint8_t SI2C_I2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue =  GPIO_ReadInputDataBit(Pthis_I2C->Private->GPIOx, Pthis_I2C->Private->SDA);
    return BitValue;
}

uint8_t SI2C_I2C_R_SCL(void)
{
    uint8_t BitValue;
     BitValue =  GPIO_ReadInputDataBit(Pthis_I2C->Private->GPIOx, Pthis_I2C->Private->SCL);
    return BitValue;
}


void SI2C_I2C_Start(void)
{
    SI2C_I2C_W_SDA(1);
    SI2C_I2C_W_SCL(1);
    SI2C_I2C_W_SDA(0);
    SI2C_I2C_W_SCL(0);
}

void SI2C_I2C_Stop(void)
{
    SI2C_I2C_W_SDA(0);
    SI2C_I2C_W_SCL(1);
    SI2C_I2C_W_SDA(1);
}

void SI2C_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        SI2C_I2C_W_SDA(!!(Byte & (0x80 >> i)));
        SI2C_I2C_W_SCL(1);
        SI2C_I2C_W_SCL(0);
    }
}

//------------------------软件I2C测试(ACK)部分------------------------
uint8_t SI2C_ReceiveByte(){
	uint8_t i, Byte = 0x00;
	SI2C_I2C_W_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		SI2C_I2C_W_SCL(1);
		if (SI2C_I2C_R_SDA() == 1){Byte |= (0x80 >> i);}
		SI2C_I2C_W_SCL(0);
	}
	return Byte;
}
//0应答ACK , 1 is Nack
void SI2C_WriteAck(uint8_t AckBit){
	SI2C_I2C_W_SDA(AckBit);
	SI2C_I2C_W_SCL(1);
	SI2C_I2C_W_SCL(0);
}

//1成功   0失败(failed)
uint8_t SI2C_ReceiveAck(){//receive ask
	uint8_t AckBit;
	SI2C_I2C_W_SDA(1);
	SI2C_I2C_W_SCL(1);
	AckBit = SI2C_I2C_R_SDA();
	SI2C_I2C_W_SCL(0);
	return AckBit;
}
//1成功   0失败(failed)
uint8_t SI2C_ACK_Test(){
	uint8_t Ack;
	SI2C_I2C_Start();
	SI2C_I2C_SendByte(Pthis_I2C->Private->I2C_Add);
	Ack = SI2C_ReceiveAck();
	SI2C_I2C_Stop();
	return Ack;
}

uint8_t _I2C_AdressScan(){
	uint8_t address;
	if(Pthis_I2C->Private->Hard_I2C_EN){
    //没有实现
	}else{//软件IIC
		for (address = 1; address < 128; address++) {//一般地址只有7位所以是128
			SI2C_I2C_Start();
			SI2C_I2C_SendByte(address << 1);
			if (!SI2C_ReceiveAck()) {//如果扫描到地址
				SI2C_I2C_Stop();//结束通信
				return address;//返回地址(10进制)
			}
			SI2C_I2C_Stop();
		}
	}
	return 0;
}

//------------------------硬件I2C基本时序部分(与软件I2C结合加入判断)------------------------
void HI2C_Rest_Speed(uint32_t Speed){
	I2C_InitTypeDef I2C_InitStructure;
 	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
 	I2C_InitStructure.I2C_ClockSpeed = Speed;		
 	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
 	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
 	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
 	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
 	I2C_Init(Pthis_I2C->Private->I2Cx, &I2C_InitStructure);
}

// 重写MyI2C_CheckEvent函数
void HI2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout;
    Timeout = 10000;
    while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS) {
        Timeout--;
        if (Timeout == 0) {
            break;
        }
    }
}


void _I2C_WriteReg(uint8_t RegAddress, uint16_t Data)
{
    if (Pthis_I2C->Private->Hard_I2C_EN)
    {
        I2C_GenerateSTART(Pthis_I2C->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(Pthis_I2C->Private->I2Cx, Pthis_I2C->Private->I2C_Add,I2C_Direction_Transmitter);
        HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(Pthis_I2C->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING);

        I2C_SendData(Pthis_I2C->Private->I2Cx, Data);
        HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTOP(Pthis_I2C->Private->I2Cx, ENABLE);
    }
    else{
        SI2C_I2C_Start();
      
		SI2C_I2C_SendByte(Pthis_I2C->Private->I2C_Add);
		SI2C_ReceiveAck();

		SI2C_I2C_SendByte(RegAddress);
		SI2C_ReceiveAck();

		if(Pthis_I2C->Mode16bit == 1){//如果是16位操作模式
			SI2C_I2C_SendByte((uint8_t)Data>>8);//发送高位
			SI2C_I2C_SendByte((uint8_t)(Data&0x00FF));//发送低位
		}else{
			SI2C_I2C_SendByte((uint8_t)Data);
		}
		SI2C_ReceiveAck();
		SI2C_I2C_Stop();
    }
    
}

uint16_t _I2C_ReadReg(uint8_t RegAddress) 
{
    uint16_t Data = 0;
    if (Pthis_I2C->Private->Hard_I2C_EN){
    I2C_GenerateSTART(Pthis_I2C->Private->I2Cx, ENABLE);
    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(Pthis_I2C->Private->I2Cx, Pthis_I2C->Private->I2C_Add, I2C_Direction_Transmitter);
    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

    I2C_SendData(Pthis_I2C->Private->I2Cx, RegAddress);
    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    I2C_GenerateSTART(Pthis_I2C->Private->I2Cx, ENABLE);
    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    I2C_Send7bitAddress(Pthis_I2C->Private->I2Cx, Pthis_I2C->Private->I2C_Add, I2C_Direction_Receiver);
    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    I2C_AcknowledgeConfig(Pthis_I2C->Private->I2Cx, ENABLE);
    I2C_GenerateSTOP(Pthis_I2C->Private->I2Cx, ENABLE);

    HI2C_WaitEvent(Pthis_I2C->Private->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);				
	Data = I2C_ReceiveData(Pthis_I2C->Private->I2Cx);											
		
	I2C_AcknowledgeConfig(Pthis_I2C->Private->I2Cx, ENABLE);	
    return Data;
   }else
   {
    SI2C_I2C_Start();
    SI2C_I2C_SendByte(Pthis_I2C->Private->I2C_Add);
    SI2C_ReceiveAck();
    SI2C_I2C_SendByte(RegAddress);
    SI2C_ReceiveAck();

    SI2C_I2C_Start();
    SI2C_I2C_SendByte(Pthis_I2C->Private->I2C_Add | 0x01);//|0x01读命令
    SI2C_ReceiveAck();
    if(Pthis_I2C->Mode16bit == 1){//如果是16位操作模式
        Data = (uint16_t)SI2C_ReceiveByte()<<8;
        Data |= SI2C_ReceiveByte();
    }else{
        Data = SI2C_ReceiveByte();
    }
    SI2C_WriteAck(1);//直接写1结束这次通信
    SI2C_I2C_Stop();
    return Data;
   }
}

uint16_t _I2C_Read_Reg_continue()
{



}


uint16_t _I2C_Write_Reg_continue()
{



}

//------------------------I2C创建部分------------------------
/*
GPIOx:选择你的GPIO口
SCL:时钟线(必须同一个GPIO口)
SDA:数据线(必须同一个GPIO口)
Address:一般是地址没有进行移位过的
*/
I2C_BUS Create_SI2C(GPIO_TypeDef* GPIOx,uint16_t SCL,uint16_t SDA,uint8_t Address){
	struct I2C_BUS this;
	this.Private = malloc(sizeof(I2C_Private));//开辟堆区，这里没有进行内存释放，不过单片机复位之后堆区会重新初始化别害怕
	this.Private->I2C_Add = Address<<1;
	this.Private->GPIOx = GPIOx;
	this.Private->SCL = SCL;
	this.Private->SDA = SDA;
	this.Private->I2Cx = 0;
	this.Private->Hard_I2C_EN = 0;
	
	this.ScanAdress = _I2C_AdressScan;
	this.AckTest = SI2C_ACK_Test;
	this.Mode16bit = 0;
	this.Write_Reg = _I2C_WriteReg;
	this.Read_Reg = _I2C_ReadReg;
	this.Rest_Speed = 0;
	// this.Read_Reg_continue = I2C_Read_Reg_continue;
	// this.Write_Reg_continue = I2C_Write_Reg_continue;
	this.Read_Reg_continue = 0;
	this.Write_Reg_continue = 0;

	if(GPIOx==GPIOA)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	else if(GPIOx==GPIOB)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	else if(GPIOx==GPIOC)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	else if(GPIOx==GPIOD)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	else if(GPIOx==GPIOE)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	else if(GPIOx==GPIOF)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	else if(GPIOx==GPIOG)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);
	
	GPIO_InitTypeDef GPIO_Init_Struct;
	GPIO_Init_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_Struct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init_Struct.GPIO_Pin = SCL|SDA;
	GPIO_Init(GPIOx,&GPIO_Init_Struct);

	GPIO_WriteBit(GPIOx, SCL, (BitAction)1);
	GPIO_WriteBit(GPIOx, SDA, (BitAction)1);

	return this;
}
/*
I2Cx:I2C1 I2C2选择硬件I2C
Address:一般是地址没有进行移位过的7位地址
*/
I2C_BUS Create_HI2C(I2C_TypeDef* I2Cx,uint8_t Address){
	struct I2C_BUS this;
	this.Private = malloc(sizeof(I2C_Private));//给私有成员开辟堆区
	this.Private->I2C_Add = Address<<1;
	this.Private->GPIOx = 0;
	this.Private->SCL = 0;
	this.Private->SDA = 0;
	this.Private->I2Cx = I2Cx;
	this.Private->Hard_I2C_EN = 1;
	
	this.ScanAdress = _I2C_AdressScan;
	this.AckTest = SI2C_ACK_Test;
	this.Mode16bit = 0;
	this.Write_Reg = _I2C_WriteReg;
	this.Read_Reg = _I2C_ReadReg;
	this.Rest_Speed = HI2C_Rest_Speed;
	// this.Read_Reg_continue = I2C_Read_Reg_continue;
	// this.Write_Reg_continue = I2C_Write_Reg_continue;
	this.Read_Reg_continue = 0;
	this.Write_Reg_continue = 0;

	if(I2Cx==I2C1)RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	else RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//!!!!!! OD mode!!!!!!!!开漏模式
	if(I2Cx==I2C1)GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	else GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//Mode
	I2C_InitStructure.I2C_ClockSpeed = 200000;//I2C速度设置
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;	
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2Cx, &I2C_InitStructure);

	I2C_Cmd(I2Cx, ENABLE);

	return this;
}
