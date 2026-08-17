#include  "eeprom.h"
#include  "bsp_usart.h"

#define EEPROM_ERASE_ADDRESS 0x0800F000

// 
// eeprom_data = {kpA , kiA, kdA}
// eeprom_data = {kpB , kiB, kdB}

void EEPROM_Write(uint16_t *data,uint32_t address,uint16_t length)
{
    FLASH_Status this;

    //解锁
    FLASH_Unlock();


    this = FLASH_ErasePage(EEPROM_ERASE_ADDRESS);

    if (this !=FLASH_COMPLETE)
    {
        Serial_Printf(USART1,"扇区擦写失败");
        return;
    }
    
    
    for (uint16_t i = 0; i < length; i++)
    {
        this  = FLASH_ProgramHalfWord(address,data[i]);
         if (this !=FLASH_COMPLETE){
            Serial_Printf(USART1,"扇区写入失败");
         }
        address += 2;
    }
    
    //上锁
    FLASH_Lock();
}


