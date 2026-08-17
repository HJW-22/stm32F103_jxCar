#ifndef __OLED_H__
#define __OLED_H__
#include "stm32f10x.h" // Device header

/*FontSize参数取值*/
/*此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度*/
#define OLED_8X16 8
#define OLED_6X8  6

/*IsFilled参数数值*/
#define OLED_UNFILLED 0
#define OLED_FILLED   1

/* OLED is fixed to I2C1 remapped to PB8/PB9 at 400 kHz. */
void OLED_Init(void);

void OLED_Clear(void);
void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

void OLED_Reverse(void);
void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height);

void OLED_Update(void);
void OLED_UpdateArea(int16_t X, int16_t Y, int16_t Width, int16_t Height);
void OLED_SetWindow(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);

void OLED_ShowChar(int16_t X, int16_t Y, char Char, int16_t FontSize);
void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize);
void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);
void OLED_ShowFloatNum(int16_t X, int16_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);
void OLED_ShowImage(int16_t X, int16_t Y, int16_t Width, int16_t Height, const uint8_t *Image);
void OLED_ShowUnsignedFloatNum(int16_t X, int16_t Y, float Number, uint8_t IntLength, uint8_t FraLength, uint8_t FontSize);

// uint8_t OLED_ID(void);

/* DMA transfer state: 1 = no transfer pending, 0 = transfer in progress. */
extern volatile uint8_t OLED_DMA_TransferComplete;

/* I2C1 TX uses DMA1 Channel6.  Initialise it after OLED_Init(). */
void OLED_DMA_Init(void);
void OLED_Update_DMA(void);
void DMA1_Channel6_IRQHandler(void);

#endif
