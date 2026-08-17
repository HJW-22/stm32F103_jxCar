#include "stm32f10x.h" // Device header
#include "bsp_i2c.h"
#include "stdlib.h"

// #define SI2C_CHRONOLOGY_DELAY_FLAG

typedef struct I2C_Private {
    GPIO_TypeDef *GPIOx;
    I2C_TypeDef *I2Cx;
    int16_t SDA;
    int16_t SCL;
    int16_t I2C_Add;
    int8_t Hard_I2C_EN;
} I2C_Private;

// I2C_BUS *Pthis_I2C = 0; // 全局指针

#ifdef SI2C_CHRONOLOGY_DELAY_FLAG
#define SI2C_CHRONOLOGY_DELAY_TIME 1 // 纳秒(us)为单位延时
uint8_t SI2C_Delay = SI2C_CHRONOLOGY_DELAY_TIME;
#endif

//------------------------软件I2C基本时序部分------------------------
static void SI2C_I2C_W_SCL(I2C_BUS* bus, uint8_t BitValue)
{
    if (BitValue != Bit_RESET)
        bus->Private->GPIOx->BSRR = bus->Private->SCL;
    else
        bus->Private->GPIOx->BRR = bus->Private->SCL;
#ifdef SI2C_CHRONOLOGY_DELAY_FLAG
    Delay_us(SI2C_Delay);
#endif
}

static void SI2C_I2C_W_SDA(I2C_BUS* bus, uint8_t BitValue)
{
    if (BitValue != Bit_RESET)
        bus->Private->GPIOx->BSRR = bus->Private->SDA;
    else
        bus->Private->GPIOx->BRR = bus->Private->SDA;
#ifdef SI2C_CHRONOLOGY_DELAY_FLAG
    Delay_us(SI2C_Delay);
#endif
}

static uint8_t SI2C_I2C_R_SDA(I2C_BUS* bus)
{
    return GPIO_ReadInputDataBit(bus->Private->GPIOx, bus->Private->SDA);
}

static void SI2C_I2C_Start(I2C_BUS* bus)
{
    SI2C_I2C_W_SDA(bus, 1);
    SI2C_I2C_W_SCL(bus, 1);
    SI2C_I2C_W_SDA(bus, 0);
    SI2C_I2C_W_SCL(bus, 0);
}

static void SI2C_I2C_Stop(I2C_BUS* bus)
{
    SI2C_I2C_W_SDA(bus, 0);
    SI2C_I2C_W_SCL(bus, 1);
    SI2C_I2C_W_SDA(bus, 1);
}

static void SI2C_I2C_SendByte(I2C_BUS* bus, uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        SI2C_I2C_W_SDA(bus, !!(Byte & (0x80 >> i)));
        SI2C_I2C_W_SCL(bus, 1);
        SI2C_I2C_W_SCL(bus, 0);
    }
}

//------------------------软件I2C测试(ACK)部分------------------------
static uint8_t SI2C_ReceiveByte(I2C_BUS* bus)
{
    uint8_t i, Byte = 0x00;
    SI2C_I2C_W_SDA(bus, 1);
    for (i = 0; i < 8; i++) {
        SI2C_I2C_W_SCL(bus, 1);
        if (SI2C_I2C_R_SDA(bus)) { Byte |= (0x80 >> i); }
        SI2C_I2C_W_SCL(bus, 0);
    }
    return Byte;
}

static void SI2C_WriteAck(I2C_BUS* bus, uint8_t AckBit)
{
    SI2C_I2C_W_SDA(bus, AckBit);
    SI2C_I2C_W_SCL(bus, 1);
    SI2C_I2C_W_SCL(bus, 0);
}

static uint8_t SI2C_ReceiveAck(I2C_BUS* bus)
{
    uint8_t AckBit;
    SI2C_I2C_W_SDA(bus, 1);
    SI2C_I2C_W_SCL(bus, 1);
    AckBit = SI2C_I2C_R_SDA(bus);
    SI2C_I2C_W_SCL(bus, 0);
    return AckBit;
}

//------------------------硬件I2C基本时序部分------------------------
static void HI2C_Rest_Speed(I2C_BUS* bus, uint32_t Speed)
{
    I2C_InitTypeDef I2C_InitStructure = {
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_ClockSpeed = Speed,
        .I2C_DutyCycle = I2C_DutyCycle_2,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
        .I2C_OwnAddress1 = 0x00
    };
    I2C_Init(bus->Private->I2Cx, &I2C_InitStructure);
}

static void HI2C_WaitEvent(I2C_BUS* bus, uint32_t I2C_EVENT)
{
    uint32_t Timeout = 10000;
    while (I2C_CheckEvent(bus->Private->I2Cx, I2C_EVENT) != SUCCESS) {
        if ((Timeout--) == 0) break;
    }
}

//------------------------I2C公共接口部分------------------------
static uint8_t _I2C_AdressScan(I2C_BUS* bus)
{
    if (bus->Private->Hard_I2C_EN) {
        // 硬件I2C地址扫描实现
        return 0;
    } else {
        uint8_t address;
        for (address = 1; address < 128; address++) {
            SI2C_I2C_Start(bus);
            SI2C_I2C_SendByte(bus, address << 1);
            if (!SI2C_ReceiveAck(bus)) {
                SI2C_I2C_Stop(bus);
                return address;
            }
            SI2C_I2C_Stop(bus);
        }
    }
    return 0;
}

static uint8_t SI2C_ACK_Test(I2C_BUS* bus)
{
    uint8_t Ack;
    SI2C_I2C_Start(bus);
    SI2C_I2C_SendByte(bus, bus->Private->I2C_Add);
    Ack = SI2C_ReceiveAck(bus);
    SI2C_I2C_Stop(bus);
    return Ack;
}

static void _I2C_WriteReg(I2C_BUS* bus, uint8_t RegAddress, uint16_t Data)
{
    if (bus->Private->Hard_I2C_EN) {
        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(bus->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        if (bus->Mode16bit) {
            I2C_SendData(bus->Private->I2Cx, (uint8_t)(Data >> 8));
            HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }
        
        I2C_SendData(bus->Private->I2Cx, (uint8_t)(Data & 0xFF));
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTOP(bus->Private->I2Cx, ENABLE);
    } else {
        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add);
        SI2C_ReceiveAck(bus);
        SI2C_I2C_SendByte(bus, RegAddress);
        SI2C_ReceiveAck(bus);

        if (bus->Mode16bit) {
            SI2C_I2C_SendByte(bus, (uint8_t)Data >> 8);
            SI2C_ReceiveAck(bus);
            SI2C_I2C_SendByte(bus, (uint8_t)(Data & 0x00FF));
        } else {
            SI2C_I2C_SendByte(bus, (uint8_t)Data);
        }
        SI2C_ReceiveAck(bus);
        SI2C_I2C_Stop(bus);
    }
}

uint16_t _I2C_ReadReg(I2C_BUS* bus, uint8_t RegAddress)
{
    uint16_t Data = 0;
    if (bus->Private->Hard_I2C_EN) {
        // 硬件I2C读取
        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(bus->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Receiver);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

        I2C_AcknowledgeConfig(bus->Private->I2Cx, DISABLE);
        I2C_GenerateSTOP(bus->Private->I2Cx, ENABLE);

        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_RECEIVED);
        Data = I2C_ReceiveData(bus->Private->I2Cx);

        // 16位模式读取第二字节
        if (bus->Mode16bit) {
            HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_RECEIVED);
            Data = (Data << 8) | I2C_ReceiveData(bus->Private->I2Cx);
        }

        I2C_AcknowledgeConfig(bus->Private->I2Cx, ENABLE);
    } else {
        // 软件I2C读取
        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add);
        SI2C_ReceiveAck(bus);
        SI2C_I2C_SendByte(bus, RegAddress);
        SI2C_ReceiveAck(bus);

        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add | 0x01); // 读命令
        SI2C_ReceiveAck(bus);
        
        if (bus->Mode16bit) {
            Data = (uint16_t)SI2C_ReceiveByte(bus) << 8;
            Data |= SI2C_ReceiveByte(bus);
        } else {
            Data = SI2C_ReceiveByte(bus);
        }
        
        SI2C_WriteAck(bus, 1); // 发送NACK结束通信
        SI2C_I2C_Stop(bus);
    }
    return Data;
}

void _I2C_Write_Reg_continue(I2C_BUS* bus, uint8_t RegAddress, uint16_t count, uint8_t *pData)
{
    if (pData == NULL || count == 0) return;
    
    if (bus->Private->Hard_I2C_EN) {
        // 硬件I2C连续写入
        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(bus->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        for (uint16_t i = 0; i < count; i++) {
            I2C_SendData(bus->Private->I2Cx, pData[i]);
            HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }
        
        I2C_GenerateSTOP(bus->Private->I2Cx, ENABLE);
    } else {
        // 软件I2C连续写入
        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add);
        SI2C_ReceiveAck(bus);
        SI2C_I2C_SendByte(bus, RegAddress);
        SI2C_ReceiveAck(bus);

        for (uint16_t i = 0; i < count;) {
            if (bus->Mode16bit && (i+1) < count) {
                SI2C_I2C_SendByte(bus, pData[i]);
                SI2C_ReceiveAck(bus);
                SI2C_I2C_SendByte(bus, pData[i+1]);
                SI2C_ReceiveAck(bus);
                i += 2;
            } else {
                SI2C_I2C_SendByte(bus, pData[i]);
                SI2C_ReceiveAck(bus);
                i++;
            }
        }
        SI2C_I2C_Stop(bus);
    }
}

void _I2C_Read_Reg_continue(I2C_BUS* bus, uint8_t RegAddress, uint16_t count, uint8_t *pData)
{
    if (pData == NULL || count == 0) return;
    
    if (bus->Private->Hard_I2C_EN) {
        // 硬件I2C连续读取
        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Transmitter);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);

        I2C_SendData(bus->Private->I2Cx, RegAddress);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        I2C_GenerateSTART(bus->Private->I2Cx, ENABLE);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_MODE_SELECT);

        I2C_Send7bitAddress(bus->Private->I2Cx, bus->Private->I2C_Add, I2C_Direction_Receiver);
        HI2C_WaitEvent(bus, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

        I2C_AcknowledgeConfig(bus->Private->I2Cx, ENABLE);

        for (uint16_t i = 0; i < count; i++) {
            if (i == count - 1) {
                I2C_AcknowledgeConfig(bus->Private->I2Cx, DISABLE); // 最后一个字节发NACK
            }
            HI2C_WaitEvent(bus, I2C_EVENT_MASTER_BYTE_RECEIVED);
            pData[i] = I2C_ReceiveData(bus->Private->I2Cx);
        }

        I2C_GenerateSTOP(bus->Private->I2Cx, ENABLE);
    } else {
        // 软件I2C连续读取
        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add);
        SI2C_ReceiveAck(bus);
        SI2C_I2C_SendByte(bus, RegAddress);
        SI2C_ReceiveAck(bus);

        SI2C_I2C_Start(bus);
        SI2C_I2C_SendByte(bus, bus->Private->I2C_Add | 0x01); // 读命令
        SI2C_ReceiveAck(bus);
        
        for (uint16_t i = 0; i < count; i++) {
            pData[i] = SI2C_ReceiveByte(bus);
            if (bus->Mode16bit && (i + 1) < count) {
                pData[i + 1] = SI2C_ReceiveByte(bus);
                i++;
            }
        }
        
        SI2C_WriteAck(bus, 1); // 发送NACK结束通信
        SI2C_I2C_Stop(bus);
    }
} 

//------------------------I2C创建部分------------------------
/*
GPIOx:选择你的GPIO口
SCL:时钟线(必须同一个GPIO口)
SDA:数据线(必须同一个GPIO口)
Address:一般是地址没有进行移位过的
*/
I2C_BUS Create_SI2C(GPIO_TypeDef *GPIOx, uint16_t SCL, uint16_t SDA, uint8_t Address)
{
    I2C_BUS bus = {0};
    bus.Private = malloc(sizeof(I2C_Private));
    if (!bus.Private) return bus;
    
    bus.Private->GPIOx = GPIOx;
    bus.Private->SCL = SCL;
    bus.Private->SDA = SDA;
    bus.Private->I2C_Add = Address << 1;
    bus.Private->Hard_I2C_EN = 0;
    
    bus.ScanAdress = _I2C_AdressScan;
    bus.AckTest = SI2C_ACK_Test;
    bus.Write_Reg = _I2C_WriteReg;
    bus.Read_Reg = _I2C_ReadReg;
    bus.Rest_Speed = NULL;
    bus.Write_Reg_continue = _I2C_Write_Reg_continue;
    bus.Read_Reg_continue = _I2C_Read_Reg_continue;
    bus.Mode16bit = 0;

    if (GPIOx == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (GPIOx == GPIOD)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if (GPIOx == GPIOE)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if (GPIOx == GPIOF)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if (GPIOx == GPIOG)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    GPIO_InitTypeDef GPIO_Init_Struct;
    GPIO_Init_Struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init_Struct.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init_Struct.GPIO_Pin   = SCL | SDA;
    GPIO_Init(GPIOx, &GPIO_Init_Struct);

    GPIO_WriteBit(GPIOx, SCL, (BitAction)1);
    GPIO_WriteBit(GPIOx, SDA, (BitAction)1);

    return bus;
}
/*
I2Cx:I2C1 I2C2选择硬件I2C
Address:一般是地址没有进行移位过的7位地址
*/
I2C_BUS Create_HI2C(I2C_TypeDef *I2Cx, uint8_t Address,uint8_t AFIO_EN)
{
    I2C_BUS bus = {0};
    bus.Private = malloc(sizeof(I2C_Private));
    if (!bus.Private) return bus;
    
    bus.Private->I2Cx = I2Cx;
    bus.Private->I2C_Add = Address << 1;
    bus.Private->Hard_I2C_EN = 1;

    bus.ScanAdress = _I2C_AdressScan;
    bus.AckTest    = SI2C_ACK_Test;
    bus.Mode16bit  = 0;
    bus.Write_Reg  = _I2C_WriteReg;
    bus.Read_Reg   = _I2C_ReadReg;
    bus.Rest_Speed = HI2C_Rest_Speed;
    bus.Read_Reg_continue = _I2C_Read_Reg_continue;
    bus.Write_Reg_continue = _I2C_Write_Reg_continue;
  

    if (I2Cx == I2C1)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    else
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

     if (AFIO_EN)
    {
        if (I2Cx == I2C1)
        {
            GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
        }
        else
        {
            //F10x没有这个功能
            //GPIO_PinRemapConfig(I2C2,ENABLE);     
        }
        
    }
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //!!!!!! OD mode!!!!!!!!开漏模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (I2Cx == I2C1)
    {
        if (AFIO_EN)
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else 
        {
            GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        } 
    }
    else
    {
        if (AFIO_EN)
        {
            // 重映射后的引脚：PB10, PB11
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10  | GPIO_Pin_11;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else 
        {
            // 默认引脚：PB10, PB11 (I2C2默认就是这些引脚)
            GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10  | GPIO_Pin_11;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        } 
    }
  
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C; // Mode
    I2C_InitStructure.I2C_ClockSpeed          = 400000;       // I2C速度设置
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_Init(I2Cx, &I2C_InitStructure);

    I2C_Cmd(I2Cx, ENABLE);

    return bus;
}



