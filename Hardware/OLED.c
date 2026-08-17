#include "stm32f10x.h"
#include "OLED_Font.h"
#include "Delay.h"
#include "OLED.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "_I2C.h"
#include "stm32f10x_dma.h"

I2C_BUS OLED_I2C;


// 包装函数实现
void OLED_Write_Wrapper(uint8_t RegAddress, uint16_t Data) {
    OLED_I2C.Write_Reg(&OLED_I2C, RegAddress, Data);
}

void OLED_Write_Continue_Wrapper(uint8_t RegAddress, uint16_t Count, uint8_t* Data) {
    OLED_I2C.Write_Reg_continue(&OLED_I2C, RegAddress, Count, Data);
}

#define OLED_WriteCommand(RegAddress,Data)           OLED_Write_Wrapper(RegAddress,Data)
#define OLED_WriteData(RegAddress,count,pData)       OLED_Write_Continue_Wrapper(RegAddress,count,pData)


#define OLED_ADDRESS 0x3C // OLED I2C地址

uint8_t OLED_DisplayBuf[8][128];



// #define OLED_CHRONOLOGY_DELAY_FLAG

// #ifdef OLED_CHRONOLOGY_DELAY_FLAG
// #define OLED_CHRONOLOGY_DELAY_TIME 1 // 纳秒(us)为单位延时
// uint8_t OLED_Delay = OLED_CHRONOLOGY_DELAY_TIME;
// #endif

// #define OLED_GPIO_GROUP GPIOB
// #define OLED_GPIO_CLK   RCC_APB2Periph_GPIOB
// #define OLED_I2C_SCL    GPIO_Pin_8
// #define OLED_I2C_SDA    GPIO_Pin_9


/*
//------------------------软件I2C基本时序部分------------------------
void OLED_I2C_W_SCL(uint8_t BitValue)
{
    GPIO_WriteBit(OLED_GPIO_GROUP, OLED_I2C_SCL, (BitAction)BitValue);
#ifdef OLED_CHRONOLOGY_DELAY_FLAG
    Delay_us(OLED_Delay);
#endif // OLED_CHRONOLOGY_DELAY_FLAG
}

void OLED_I2C_W_SDA(uint8_t BitValue)
{
    GPIO_WriteBit(OLED_GPIO_GROUP, OLED_I2C_SDA, (BitAction)BitValue);
#ifdef OLED_CHRONOLOGY_DELAY_FLAG
    Delay_us(OLED_Delay);
#endif // OLED_CHRONOLOGY_DELAY_FLAG
}

uint8_t OLED_I2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(OLED_GPIO_GROUP, OLED_I2C_SDA);
    return BitValue;
}

uint8_t OLED_I2C_R_SCL(void)
{
    uint8_t BitValue;
    BitValue = GPIO_ReadInputDataBit(OLED_GPIO_GROUP, OLED_I2C_SCL);
    return BitValue;
}

void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(OLED_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD; // 开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = OLED_I2C_SCL | OLED_I2C_SDA;
    GPIO_Init(OLED_GPIO_GROUP, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, OLED_I2C_SCL | OLED_I2C_SDA);
}

void OLED_I2C_Start(void)
{
    OLED_I2C_W_SDA(1);
    OLED_I2C_W_SCL(1);
    OLED_I2C_W_SDA(0);
    OLED_I2C_W_SCL(0);
}

void OLED_I2C_Stop(void)
{
    OLED_I2C_W_SDA(0);
    OLED_I2C_W_SCL(1);
    OLED_I2C_W_SDA(1);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++) {
        OLED_I2C_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_I2C_W_SCL(1);
        OLED_I2C_W_SCL(0);
    }
    OLED_I2C_W_SCL(1);
    OLED_I2C_W_SCL(0);
}

void OLED_WriteCommand(uint8_t address, uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_ADDRESS);
    OLED_I2C_SendByte(address);
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

void OLED_WriteData(uint8_t address, uint8_t *Data, uint8_t Count)
{
    uint8_t i;
    OLED_I2C_Start();
    OLED_I2C_SendByte(OLED_ADDRESS);
    OLED_I2C_SendByte(address);
    for (i = 0; i < Count; i++) {
        OLED_I2C_SendByte(Data[i]);
    }
    OLED_I2C_Stop();
}
*/




//------------------------软件I2C通讯部分------------------------

/* OLED初始化 */
void OLED_Init(I2C_TypeDef *I2Cx,uint8_t AFIO_EN)
{
    // 硬件初始化
    OLED_I2C = Create_HI2C(I2C1,OLED_ADDRESS,1);//创建软件IIC
    Delay_ms(100);      // 重要延时，等待OLED电源稳定
    
    // 软件初始化（SSD1306命令配置）
    OLED_WriteCommand(0x00, 0xAE); // 关闭显示（DISPLAYOFF）

    /* 基础显示设置 */
    OLED_WriteCommand(0x00, 0x20);  // 设置内存地址模式（Memory Addressing Mode）
    OLED_WriteCommand(0x00, 0x00);  // 水平地址模式（Horizontal Addressing Mode）
    OLED_WriteCommand(0x00, 0xB0);  // 设置页起始地址（Page Start Address）
    OLED_WriteCommand(0x00, 0xC8);  // 设置COM输出扫描方向（COM Output Scan Direction）：反向（从下到上）
    OLED_WriteCommand(0x00, 0x00);  // 设置列地址低4位（Lower Column Start Address）
    OLED_WriteCommand(0x00, 0x10);  // 设置列地址高4位（Higher Column Start Address）
    OLED_WriteCommand(0x00, 0x40);  // 设置显示起始行（Display Start Line）：0
    OLED_WriteCommand(0x00, 0x81);  // 设置对比度控制（Contrast Control）
    OLED_WriteCommand(0x00, 0xFF);  // 对比度值（最大亮度，0xFF）
    OLED_WriteCommand(0x00, 0xA1);  // 设置段重映射（Segment Re-map）：列地址127映射到SEG0（水平翻转）
    OLED_WriteCommand(0x00, 0xA6);  // 设置正常显示（Normal Display，非反色）
    OLED_WriteCommand(0x00, 0xA8);  // 设置多路复用比率（Multiplex Ratio）
    OLED_WriteCommand(0x00, 0x3F);  // 默认值0x3F（对应64行）
    OLED_WriteCommand(0x00, 0xA4);  // 禁用全局显示（Disable Entire Display On）
    OLED_WriteCommand(0x00, 0xD3);  // 设置显示偏移（Display Offset）
    OLED_WriteCommand(0x00, 0x00);  // 无偏移（Vertical Shift = 0）

    /* 时序和电源配置 */
    OLED_WriteCommand(0x00, 0xD5);  // 设置显示时钟分频（Display Clock Divide Ratio/Oscillator Frequency）
    OLED_WriteCommand(0x00, 0xF0);  // 默认分频比（0xF0）
    OLED_WriteCommand(0x00, 0xD9);  // 设置预充电周期（Pre-charge Period）
    OLED_WriteCommand(0x00, 0x22);  // Phase1 = 2 DCLK, Phase2 = 2 DCLK
    OLED_WriteCommand(0x00, 0xDA);  // 设置COM引脚配置（COM Pins Hardware Configuration）
    OLED_WriteCommand(0x00, 0x12);  // 序列模式（Sequential COM Pin Config），禁用左右复用（Alternative COM Pin Disable）
    OLED_WriteCommand(0x00, 0xDB);  // 设置VCOMH电压（VCOMH Deselect Level）
    OLED_WriteCommand(0x00, 0x20);  // 默认值0x20（~0.77 * VCC）

    /* 电源管理 */
    OLED_WriteCommand(0x00, 0x8D);  // 设置电荷泵（Charge Pump Setting）
    OLED_WriteCommand(0x00, 0x14);  // 启用电荷泵（Enable Charge Pump）
    OLED_WriteCommand(0x00, 0xAF);  // 开启显示（DISPLAYON）

    /* 初始化后处理 */
    OLED_Clear();   // 清空显存（全写0）
    OLED_Update();  // 更新显示，防止初始化后花屏
}

/**
 * @brief  OLED设置光标位置
 * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
 * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
 * @retval 无
 */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0x00, 0xB0 | Y);                 // 设置Y位置
    OLED_WriteCommand(0x00, 0x10 | ((X & 0xF0) >> 4)); // 设置X位置高4位
    OLED_WriteCommand(0x00, 0x00 | (X & 0x0F));        // 设置X位置低4位
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
 * @brief  显存数组清零
 * @param  无
 * @retval 无
 */
void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) {
        for (i = 0; i < 128; i++) {
            OLED_DisplayBuf[j][i] = 0x00;
        }
    }
}

/**
 * @brief 显存数组部分清零
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Width 指定图像的宽度   范围:0~128
 * @param  Height 指定图像的高度   范围:0~64
 * @retval 无
 */
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;

    for (j = Y; j < Y + Height; j++) // 遍历指定页
    {
        for (i = X; i < X + Width; i++) // 遍历指定列
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63) // 超出屏幕的内容不显示
            {
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8)); // 将显存数组指定数据清零
            }
        }
    }
}

/**
 * @brief  显存数组全部取反
 * @param  无
 * @retval 无
 */
void OLED_Reverse(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++) // 遍历8页
    {
        for (i = 0; i < 128; i++) // 遍历128列
        {
            OLED_DisplayBuf[j][i] ^= 0xFF; // 将显存数组数据全部取反
        }
    }
}

/**
 * @brief 显存数组部分取反
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Width 指定图像的宽度   范围:0~128
 * @param  Height 指定图像的高度   范围:0~64
 * @retval 无
 */
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t i, j;

    for (j = Y; j < Y + Height; j++) // 遍历指定页
    {
        for (i = X; i < X + Width; i++) // 遍历指定列
        {
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63) // 超出屏幕的内容不显示
            {
                OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8); // 将显存数组指定数据取反
            }
        }
    }
}

/**
 * @brief  显存数组写入到OLED硬件
 * @param  无
 * @retval 无
 */
void OLED_Update(void)
{
    uint8_t j;
    /*遍历每一页*/
    for (j = 0; j < 8; j++) {
        /*设置光标位置为每一页的第一列*/
        OLED_SetCursor(j, 0);
        /*连续写入128个数据，将显存数组的数据写入到OLED硬件*/
        OLED_WriteData(0x40, 128, OLED_DisplayBuf[j]);
    }
}

/**
 * @brief 显存数组部分写入到OLED硬件
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Width 指定图像的宽度   范围:0~128
 * @param  Height 指定图像的高度   范围:0~64
 * @retval 无
 */
void OLED_UpdateArea(int16_t X, int16_t Y, int16_t Width, int16_t Height)
{
    int16_t j;
    int16_t page, page1;

    page  = Y / 8; // 获取页区
    page1 = (Y + Height - 1) / 8 + 1;
    if (Y < 0) {
        page -= 1;
        page1 -= 1;
    }

    /*遍历指定区域涉及的相关页*/
    for (j = page; j < page1; j++) {
        if (X >= 0 && X <= 127 && j >= 0 && j <= 7) // 超出屏幕的内容不显示
        {
            /*设置光标位置为相关页的指定列*/
            OLED_SetCursor(j, X);
            /*连续写入Width个数据，将显存数组的数据写入到OLED硬件*/
            OLED_WriteData(0x40, Width, &OLED_DisplayBuf[j][X]);
        }
    }
}

/**
 * @brief  OLED显示一个字符
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Char 指定要显示的字符，范围：ASCII码可见字符
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowChar(int16_t X, int16_t Y, char Char, int16_t FontSize)
{
    if (FontSize == OLED_8X16) // 字体为宽8像素，高16像素
    {
        /*将ASCII字模库OLED_F8x16的指定数据以8*16的图像格式显示*/
        OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
    } else if (FontSize == OLED_6X8) // 字体为宽6像素，高8像素
    {
        /*将ASCII字模库OLED_F6x8的指定数据以6*8的图像格式显示*/
        OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
    }
}

/**
 * @brief  OLED显示字符串
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  String 指定要显示的字符串，范围：ASCII码可见字符或中文字符组成的字符串
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void  OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{

    uint16_t i = 0;
    char SingleChar[5];
    uint8_t CharLength = 0;
    uint16_t XOFFset   = 0;
    uint16_t pIndex;
    while (String[i] != '\0') {
#ifdef OLED_CHARSET_UTF8
        if ((String[i] & 0x80) == 0x00) {
            CharLength    = 1;
            SingleChar[0] = String[i++];
            SingleChar[1] = '\0';
        } else if ((String[i] & 0xE0) == 0xC0) // 第一个字节为110xxxxx
        {
            CharLength    = 2;                // 字符为2字节
            SingleChar[0] = String[i++];      // 将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
            if (String[i] == '\0') { break; } // 意外情况，跳出循环，结束显示
            SingleChar[1] = String[i++];      // 将第二个字节写入SingleChar第1个位置，随后i指向下一个字节
            SingleChar[2] = '\0';             // 为SingleChar添加字符串结束标志位
        } else if ((String[i] & 0xF0) == 0xE0) {
            CharLength    = 3;
            SingleChar[0] = String[i++];
            if (String[i] == '\0') break;
            SingleChar[1] = String[i++];
            if (SingleChar[i] == '\0') break;
            SingleChar[2] = String[i++];
            SingleChar[3] = '\0';
        } else if ((String[i] & 0xF8) == 0xF0) {
            CharLength    = 4;
            SingleChar[0] = String[i++];
            if (String[i] == '\0') break;
            SingleChar[1] = String[i++];
            if (SingleChar[i] == '\0') break;
            SingleChar[2] = String[i++];
            if (SingleChar[i] == '\0') break;
            SingleChar[3] = String[i++];
            SingleChar[4] = '\0';
        } else {
            i++;
            continue;
        }
#endif
#ifdef OLED_CHARSET_GB2312 // 定义字符集为GB2312
        /*此段代码的目的是，提取GB2312字符串中的一个字符，转存到SingleChar子字符串中*/
        /*判断GB2312字节的最高位标志位*/
        if ((String[i] & 0x80) == 0x00) // 最高位为0
        {
            CharLength    = 1;           // 字符为1字节
            SingleChar[0] = String[i++]; // 将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
            SingleChar[1] = '\0';        // 为SingleChar添加字符串结束标志位
        } else                           // 最高位为1
        {
            CharLength    = 2;                // 字符为2字节
            SingleChar[0] = String[i++];      // 将第一个字节写入SingleChar第0个位置，随后i指向下一个字节
            if (String[i] == '\0') { break; } // 意外情况，跳出循环，结束显示
            SingleChar[1] = String[i++];      // 将第二个字节写入SingleChar第1个位置，随后i指向下一个字节
            SingleChar[2] = '\0';             // 为SingleChar添加字符串结束标志位
        }
#endif
        if (CharLength == 1) {
            OLED_ShowChar(X + XOFFset, Y, SingleChar[0], FontSize);
            XOFFset += FontSize;
        } else {
            for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex++) {
                if (strcmp(OLED_CF16x16[pIndex].Index, SingleChar) == 0) {
                    break;
                }
            }
            if (FontSize == OLED_8X16) {
                OLED_ShowImage(X + XOFFset, Y, 16, 16, OLED_CF16x16[pIndex].Data);
                XOFFset += 16;
            }

            else if (FontSize == OLED_6X8) {
                OLED_ShowChar(X + XOFFset, Y, '?', OLED_6X8);
                XOFFset += OLED_6X8;
            }
        }
    }
}

/**
 * @brief  OLED显示数字
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:0~4294967295
 * @param  Length 指定数字的长度   范围:0~10
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    for (i = 0; i < Length; i++) // 遍历数字的每一位
    {
        /*调用OLED_ShowChar函数，依次显示每个数字*/
        /*Number / OLED_Pow(10, Length - i - 1) % 10 可以十进制提取数字的每一位*/
        /*+ '0' 可将数字转换为字符格式*/
        OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
    }
}

/**
 * @brief  OLED显示带符号数字
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:-2147483648~2147483647
 * @param  Length 指定数字的长度   范围:0~10
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    uint32_t Number1;

    if (Number >= 0) // 数字大于等于0
    {
        OLED_ShowChar(X, Y, '+', FontSize); // 显示+号
        Number1 = Number;                   // Number1直接等于Number
    } else                                  // 数字小于0
    {
        OLED_ShowChar(X, Y, '-', FontSize); // 显示-号
        Number1 = -Number;                  // Number1等于Number取负
    }

    for (i = 0; i < Length; i++) // 遍历数字的每一位
    {
        /*调用OLED_ShowChar函数，依次显示每个数字*/
        /*Number1 / OLED_Pow(10, Length - i - 1) % 10 可以十进制提取数字的每一位*/
        /*+ '0' 可将数字转换为字符格式*/
        OLED_ShowChar(X + (i + 1) * FontSize, Y, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
    }
}

/**
 * @brief  OLED显示十六进制数
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:0x00000000~0xFFFFFFFF
 * @param  Length 指定数字的长度   范围:0~8
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++) // 遍历数字的每一位
    {
        /*以十六进制提取数字的每一位*/
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;

        if (SingleNumber < 10) // 单个数字小于10
        {
            /*调用OLED_ShowChar函数，显示此数字*/
            /*+ '0' 可将数字转换为字符格式*/
            OLED_ShowChar(X + i * FontSize, Y, SingleNumber + '0', FontSize);
        } else // 单个数字大于10
        {
            /*调用OLED_ShowChar函数，显示此数字*/
            /*+ 'A' 可将数字转换为从A开始的十六进制字符*/
            OLED_ShowChar(X + i * FontSize, Y, SingleNumber - 10 + 'A', FontSize);
        }
    }
}

/**
 * @brief  OLED显示二进制数
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:0x00000000~0xFFFFFFFF
 * @param  Length 指定数字的长度   范围:0~16
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint8_t i;
    for (i = 0; i < Length; i++) // 遍历数字的每一位
    {
        /*调用OLED_ShowChar函数，依次显示每个数字*/
        /*Number / OLED_Pow(2, Length - i - 1) % 2 可以二进制提取数字的每一位*/
        /*+ '0' 可将数字转换为字符格式*/
        OLED_ShowChar(X + i * FontSize, Y, Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
    }
}

/**
 * @brief  OLED显示浮点数
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:-4294967295.0~4294967295.0
 * @param  IntLength 指定数字的长度   范围:0~10
 * @param  FraLength 指定数字的小数位长度   范围:0~9，小数进行四舍五入显示
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @retval 无
 */
void OLED_ShowFloatNum(int16_t X, int16_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    uint32_t PowNum, IntNum, FraNum;

    if (Number >= 0) // 数字大于等于0
    {
        OLED_ShowChar(X, Y, '+', FontSize); // 显示+号
    } else                                  // 数字小于0
    {
        OLED_ShowChar(X, Y, '-', FontSize); // 显示-号
        Number = -Number;                   // Number取负
    }

    /*提取整数部分和小数部分*/
    IntNum = Number;                  // 直接赋值给整型变量，提取整数
    Number -= IntNum;                 // 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
    PowNum = OLED_Pow(10, FraLength); // 根据指定小数的位数，确定乘数
    FraNum = round(Number * PowNum);  // 将小数乘到整数，同时四舍五入，避免显示误差
    IntNum += FraNum / PowNum;        // 若四舍五入造成了进位，则需要再加给整数

    /*显示整数部分*/
    OLED_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);

    /*显示小数点*/
    OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);

    /*显示小数部分*/
    OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);
}

/**
 * @brief  OLED显示无符号浮点数
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Number 指定要显示的数字   范围:0.0~4294967295.0
 * @param  IntLength 整数部分长度   范围:0~10
 * @param  FraLength 小数部分长度   范围:0~9（四舍五入）
 * @param  FontSize 字体大小   范围:OLED_8X16或OLED_6X8
 * @retval 无
 */
void OLED_ShowUnsignedFloatNum(int16_t X, int16_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    // 参数校验
    if (Number < 0) Number = 0; // 强制非负

    // 计算10^FraLength（避免重复计算）
    uint32_t PowNum = OLED_Pow(10, FraLength);

    // 分离整数和小数部分（优化浮点运算）
    uint32_t IntNum  = (uint32_t)Number;
    float fractional = Number - (float)IntNum;
    uint32_t FraNum  = round(fractional * PowNum);

    // 处理四舍五入进位
    if (FraNum >= PowNum) {
        FraNum -= PowNum;
        IntNum++;
    }

    // 显示整数部分
    OLED_ShowNum(X, Y, IntNum, IntLength, FontSize);

    // 如果有小数部分
    if (FraLength > 0) {
        // 显示小数点
        OLED_ShowChar(X + IntLength * FontSize, Y, '.', FontSize);

        // 显示小数部分（自动补前导零）
        OLED_ShowNum(X + (IntLength + 1) * FontSize, Y, FraNum, FraLength, FontSize);
    }
}
/**
 * @brief  OLED显示图像
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  Width 指定图像的宽度   范围:0~128
 * @param  Height 指定图像的高度   范围:0~64
 * @param  Image 指定要显示的图像
 * @retval 无
 */
void OLED_ShowImage(int16_t X, int16_t Y, int16_t Width, int16_t Height, const uint8_t *Image)
{
    int16_t j, i    ;
    int16_t page, shift;

    OLED_ClearArea(X, Y, Width, Height); // 区域置0

    // 页区循环,向上取整操作
    for (j = 0; j < (Height - 1) / 8 + 1; j++) {
        // 宽度循环
        for (i = 0; i < Width; i++) {
            // 舍弃过多部分
            if (X + i >= 0 && X + i <= 127) {
                page  = Y / 8; // 获取页区
                shift = Y % 8; // 获取移位值
                // 如果为负值 +1
                if (Y < 0) {
                    page -= 1;
                    shift += 8;
                }
            }
            // 先循环上半部分
            if (page + j >= 0 && page + j <= 7) {
                OLED_DisplayBuf[page + j][X + i] |= Image[j * Width + i] << (shift);
            }
            // 再循环上半部分
            if (page + j + 1 >= 0 && page + j + 1 <= 7) {
                OLED_DisplayBuf[page + j + 1][X + i] |= Image[j * Width + i] << (8 - shift);
            }
        }
    }
}

/**
 * @brief  OLED使用printf函数打印格式化字符串（支持ASCII码和中文混合写入）
 * @param  X 起始行位置   范围:0~127
 * @param  Y 起始列位置   范围:0~63
 * @param  FontSize 要显示的文字格式   范围:OLED_8X16		宽8像素，高16像素
 *                                        OLED_6X8		  宽6像素，高8像素
 * @param  format 指定要显示的格式化字符串   范围:ASCII码可见字符或中文字符组成的字符串
 * @param  ... 格式化字符串参数列表 范围:无
 * @retval 无
 */
void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    char String[256];                        // 定义字符数组
    va_list arg;                             // 定义可变参数列表数据类型的变量arg
    va_start(arg, format);                   // 从format开始，接收参数列表到arg变量
    vsprintf(String, format, arg);           // 使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);                             // 结束变量arg
    OLED_ShowString(X, Y, String, FontSize); // OLED显示字符数组（字符串）
}


uint8_t OLED_ID(void)
{
  return 1;
}




// OLED DMA部分
volatile uint8_t OLED_DMA_TransferComplete = 0;



// DMA初始化
void OLED_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    // 开启DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    // 配置DMA通道（I2C1 TX用DMA1 Channel6）
    DMA_DeInit(DMA1_Channel6);
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)OLED_DisplayBuf; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  // 传输方向 寄存器->外设
    DMA_InitStructure.DMA_BufferSize = 1025;             // 动态设置
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设是否为递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;           //存储器是否为递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据类型 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据类型 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;          //模式  CIRC                  1位 循环(ADC) 标准
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;    //优先级 PL                   2位 低 中 高 最高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;           //存储器到存储器模式 MEM2MEM   1位 非 启
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    
    // 启用DMA传输完成中断
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    
    // 配置NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 启用I2C DMA请求
    I2C_DMACmd(I2C1, ENABLE);
}

// DMA中断服务函数
void DMA1_Channel6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA1_IT_TC6))
    {
        DMA_ClearITPendingBit(DMA1_IT_TC6 | DMA1_IT_TE6 | DMA1_IT_HT6);
        I2C_GenerateSTOP(I2C1, ENABLE);//关闭I2C1总线
        OLED_DMA_TransferComplete = 0;  // 只设置标志位，不操作DMA通道
    }
}

// 带DMA的OLED刷新
void OLED_Update_DMA(void)
{
    while (OLED_DMA_TransferComplete);
    // 等待I2C就绪
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    // 启动I2C传输
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    // 发送设备地址+写
    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // 发送控制字节(0x40表示数据)
    I2C_SendData(I2C1, 0x40);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // 配置DMA
    DMA_Cmd(DMA1_Channel6, DISABLE);
    // 配置并启动DMA
    OLED_DMA_TransferComplete = 1;
    DMA_SetCurrDataCounter(DMA1_Channel6,1025);
    DMA_Cmd(DMA1_Channel6, ENABLE);
}


void OLED_UpdateArea_DMA(int16_t X, int16_t Y, int16_t Width, int16_t Height)
{
    // 参数检查
    if(X < 0 || X > 127 || Y < 0 || Y > 7) return;
    if(Width <= 0 || Width > 128) Width = 128 - X;
    if(Height <= 0 || Height > 8) Height = 8 - Y;
    
    // 等待上次DMA传输完成
    while(OLED_DMA_TransferComplete == 0);
    
    // 等待I2C空闲
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    
    // 启动I2C传输
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
    // 发送设备地址+写
    I2C_Send7bitAddress(I2C1, 0x78, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // 发送控制字节(0x40表示数据)
    I2C_SendData(I2C1, 0x40);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // 配置DMA
    DMA_Cmd(DMA1_Channel6, DISABLE);
    
    // 设置DMA源地址(显存区域)
    DMA1_Channel6->CMAR = (uint32_t)&OLED_DisplayBuf[Y][X];
    
    // 设置传输数据量(宽度×页数)
    uint16_t transfer_size = Width * Height;
    DMA_SetCurrDataCounter(DMA1_Channel6, transfer_size);
    
    // 标记DMA传输状态
    OLED_DMA_TransferComplete = 0;
    
    // 启动DMA传输
    DMA_Cmd(DMA1_Channel6, ENABLE);
}
