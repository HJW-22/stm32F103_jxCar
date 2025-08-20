#ifndef __ESP8266_H__
#define __ESP8266_H__

#include "stm32f10x.h"    
#include "esp8266_uart.h"
/* 错误代码 */
#define ATK_MW8266D_EOK         0   /* 没有错误 */
#define ATK_MW8266D_ERROR       1   /* 通用错误 */
#define ATK_MW8266D_ETIMEOUT    2   /* 超时错误 */
#define ATK_MW8266D_EINVAL      3   /* 参数错误 */




typedef enum 
{
    ESP8266_INIT_EOK =0,
    ESP8266_ATTEST_ERROR =1,
    ESP8266_JOINWIFIAP_ERROR =2,
    ESP8266_CONNECTTCPSERVER_ERROR =3,
    ESP8266_ENTERUNVARNISHED_ERROR =4,

} ESP8266_INIT_ERROR;



ESP8266_INIT_ERROR ESP8266_Init(uint32_t baudrate);
void ESP8266_ERROR_Handling(ESP8266_INIT_ERROR this);





#endif
