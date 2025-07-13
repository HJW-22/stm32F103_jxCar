#include "stm32f10x.h"
#include "OLED_Font.h"

#define OLED_ADDRESS 0x78 // OLED I2C地址

/*引脚初始化*/
void OLED_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    // 启用I2C1引脚重映射(PB8-SCL, PB9-SDA)
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD; // 复用开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable; // 使能应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = 400000; // 400kHz
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_16_9;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C; // I2C模式
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;

    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE); // 使能I2C1
}

/* 等待I2C事件 */
uint8_t I2C_WaitEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    uint32_t timeout = 10000;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT)) {
        if ((timeout--) == 0) return 0;
    }
    return 1;
}

/* I2C发送数据 */
void OLED_I2C_Write(uint8_t address, uint8_t *data, uint8_t len)
{
    // 发送起始条件
    I2C_GenerateSTART(I2C1, ENABLE);
    if (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) return;

    // 发送设备地址
    I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
    if (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;

    // 发送数据
    while (len--) {
        I2C_SendData(I2C1, *data++);
        if (!I2C_WaitEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    }

    // 发送停止条件
    I2C_GenerateSTOP(I2C1, ENABLE);
}

/* OLED写命令 */
void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {0x00, Command}; // 0x00表示命令
    OLED_I2C_Write(OLED_ADDRESS, buf, 2);
}

/* OLED写数据 */
void OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2] = {0x40, Data}; // 0x40表示数据
    OLED_I2C_Write(OLED_ADDRESS, buf, 2);
}

/**
 * @brief  OLED设置光标位置
 * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
 * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
 * @retval 无
 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                 // 设置Y位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); // 设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));        // 设置X位置低4位
}

/**
 * @brief  OLED清屏
 * @param  无
 * @retval 无
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++) {
            OLED_WriteData(0x00);
        }
    }
}

/**
 * @brief  清除 OLED 指定行
 * @param  Line 行位置，范围：1~4
 * @retval 无
 */
void OLED_ClearLine(uint8_t Line)
{
    uint8_t i, j;
    // 每行高度为2页（16像素高）
    for (j = 0; j < 2; j++) {
        OLED_SetCursor((Line - 1) * 2 + j, 0);
        for (i = 0; i < 128; i++) // 128列全部清除
        {
            OLED_WriteData(0x00); // 写入0x00清除像素
        }
    }
}

/**
 * @brief  OLED显示一个字符
 * @param  Line 行位置，范围：1~4
 * @param  Column 列位置，范围：1~16
 * @param  Char 要显示的一个字符，范围：ASCII可见字符
 * @retval 无
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8); // 设置光标位置在上半部分
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]); // 显示上半部分内容
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8); // 设置光标位置在下半部分
    for (i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]); // 显示下半部分内容
    }
}

/**
 * @brief  OLED显示字符串
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  String 要显示的字符串，范围：ASCII可见字符
 * @retval 无
 */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/**
 * @brief  OLED次方函数
 * @retval 返回值等于X的Y次方
 */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  OLED显示数字（十进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~4294967295
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示数字（十进制，带符号数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：-2147483648~2147483647
 * @param  Length 要显示数字的长度，范围：1~10
 * @retval 无
 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0) {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    } else {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示数字（十六进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
 * @param  Length 要显示数字的长度，范围：1~8
 * @retval 无
 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++) {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10) {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        } else {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
 * @brief  OLED显示数字（二进制，正数）
 * @param  Line 起始行位置，范围：1~4
 * @param  Column 起始列位置，范围：1~16
 * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
 * @param  Length 要显示数字的长度，范围：1~16
 * @retval 无
 */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/* OLED初始化 */
void OLED_Init(void)
{
    // 硬件初始化
    OLED_I2C_Init();

    // 软件初始化
    OLED_WriteCommand(0xAE); // 关闭显示

    OLED_WriteCommand(0xD5); // 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8); // 设置多路复用率
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3); // 设置显示偏移
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40); // 设置显示开始行

    OLED_WriteCommand(0xA1); // 设置左右方向
    OLED_WriteCommand(0xC8); // 设置上下方向

    OLED_WriteCommand(0xDA); // 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81); // 设置对比度控制
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9); // 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB); // 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4); // 设置整个显示打开/关闭
    OLED_WriteCommand(0xA6); // 设置正常/倒转显示

    OLED_WriteCommand(0x8D); // 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF); // 开启显示

    OLED_Clear(); // 清屏
}
