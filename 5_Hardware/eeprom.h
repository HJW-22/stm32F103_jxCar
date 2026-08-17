#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "stm32f10x.h" 

void EEPROM_Write(uint16_t *data,uint32_t address,uint16_t length);


#endif
