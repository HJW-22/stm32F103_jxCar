#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f10x.h"

/* Fixed hardware-I2C interface. STM32 peripherals are fixed resources, so
 * this API uses explicit peripherals rather than allocated function objects. */

/* I2C1: remap=0 selects PB6/PB7, remap=1 selects PB8/PB9.
 * I2C2 always uses PB10/PB11 and ignores remap. */
void BSP_I2C_Init(I2C_TypeDef *I2Cx, uint8_t remap, uint32_t clock_hz);

/* Blocking register transactions; address7 is an unshifted 7-bit address.
 * Both functions return 1 on success and 0 after a bounded timeout. */
uint8_t BSP_I2C_MemWrite(I2C_TypeDef *I2Cx, uint8_t address7,
                         uint8_t reg, const uint8_t *data, uint16_t count);
uint8_t BSP_I2C_MemRead(I2C_TypeDef *I2Cx, uint8_t address7,
                        uint8_t reg, uint8_t *data, uint16_t count);

/* Returns 1 when the bus is idle, otherwise 0 after a bounded timeout. */
uint8_t BSP_I2C_WaitIdle(I2C_TypeDef *I2Cx);

#endif
