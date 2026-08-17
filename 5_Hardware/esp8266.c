#include <stdio.h>
#include "esp8266_uart.h"
#include "esp8266.h"
#include "bsp_usart.h"
#include "delay.h"




/**
 * @brief       ATK-MW8266D发送AT指令
 * @param       cmd    : 待发送的AT指令
 *              ack    : 等待的响应
 *              timeout: 等待超时时间
 * @retval      ATK_MW8266D_EOK     : 函数执行成功
 *              ATK_MW8266D_ETIMEOUT: 等待期望应答超时，函数执行失败
 */
uint8_t atk_mw8266d_send_at_cmd(char *cmd, char *ack, uint32_t timeout)
{
    uint8_t *ret = NULL;
    
    ESP8266_RxRestart();
    Serial_Printf(USART2,"%s\r\n", cmd);
    
    if ((ack == NULL) || (timeout == 0))
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        while (timeout > 0)
        {
            ret = ESP8266_GetRxFrame();
            if (ret != NULL)
            {
                if (strstr((const char *)ret, ack) != NULL)
                {
                    return ATK_MW8266D_EOK;
                }
                else
                {
                    ESP8266_RxRestart();
                }
            }
            timeout--;
            Delay_ms(1);
        }
        
        return ATK_MW8266D_ETIMEOUT;
    }
}

/**
 * @brief       ATK-MW8266D AT指令测试
 * @param       无
 * @retval      ATK_MW8266D_EOK  : AT指令测试成功
 *              ATK_MW8266D_ERROR: AT指令测试失败
 */
uint8_t atk_mw8266d_at_test(void)
{
    uint8_t ret;
    uint8_t i;
    
    for (i=0; i<10; i++)
    {
        ret = atk_mw8266d_send_at_cmd("AT", "OK", 500);
        if (ret == ATK_MW8266D_EOK)
        {
            return ATK_MW8266D_EOK;
        }
    }
    
    return ATK_MW8266D_ERROR;
}




/**
 * @brief       设置ATK-MW8266D工作模式
 * @param       mode: 1，Station模式
 *                    2，AP模式
 *                    3，AP+Station模式
 * @retval      ATK_MW8266D_EOK   : 工作模式设置成功
 *              ATK_MW8266D_ERROR : 工作模式设置失败
 *              ATK_MW8266D_EINVAL: mode参数错误，工作模式设置失败
 */
uint8_t atk_mw8266d_set_mode(uint8_t mode)
{
    uint8_t ret;
    
    switch (mode)
    {
        case 1:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=1", "OK", 500);    /* Station模式 */
            break;
        }
        case 2:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=2", "OK", 500);    /* AP模式 */
            break;
        }
        case 3:
        {
            ret = atk_mw8266d_send_at_cmd("AT+CWMODE=3", "OK", 500);    /* AP+Station模式 */
            break;
        }
        default:
        {
            return ATK_MW8266D_EINVAL;
        }
    }
    
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}

/**
 * @brief       ATK-MW8266D软件复位
 * @param       无
 * @retval      ATK_MW8266D_EOK  : 软件复位成功
 *              ATK_MW8266D_ERROR: 软件复位失败
 */
uint8_t atk_mw8266d_sw_reset(void)
{
    uint8_t ret;
    
    ret = atk_mw8266d_send_at_cmd("AT+RST", "OK", 500);
    if (ret == ATK_MW8266D_EOK)
    {
        Delay_ms(1000);
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}
/**
 * @brief       ATK-MW8266D设置回显模式
 * @param       cfg: 0，关闭回显
 *                   1，打开回显
 * @retval      ATK_MW8266D_EOK  : 设置回显模式成功
 *              ATK_MW8266D_ERROR: 设置回显模式失败
 */
uint8_t atk_mw8266d_ate_config(uint8_t cfg)
{
    uint8_t ret;
    
    switch (cfg)
    {
        case 0:
        {
            ret = atk_mw8266d_send_at_cmd("ATE0", "OK", 500);   /* 关闭回显 */
            break;
        }
        case 1:
        {
            ret = atk_mw8266d_send_at_cmd("ATE1", "OK", 500);   /* 打开回显 */
            break;
        }
        default:
        {
            return ATK_MW8266D_EINVAL;
        }
    }
    
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}
/**
 * @brief       ATK-MW8266D连接WIFI
 * @param       ssid: WIFI名称
 *              pwd : WIFI密码
 * @retval      ATK_MW8266D_EOK  : WIFI连接成功
 *              ATK_MW8266D_ERROR: WIFI连接失败
 */
uint8_t atk_mw8266d_join_ap(char *ssid, char *pwd)
{
    uint8_t ret;
    char cmd[64];
    
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, pwd);
    ret = atk_mw8266d_send_at_cmd(cmd, "WIFI GOT IP", 10000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}

/**
 * @brief       ATK-MW8266D获取IP地址
 * @param       buf: IP地址，需要16字节内存空间
 * @retval      ATK_MW8266D_EOK  : 获取IP地址成功
 *              ATK_MW8266D_ERROR: 获取IP地址失败
 */
uint8_t atk_mw8266d_get_ip(char *buf)
{
    uint8_t ret;
    char *p_start;
    char *p_end;
    
    ret = atk_mw8266d_send_at_cmd("AT+CIFSR", "OK", 500);
    if (ret != ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_ERROR;
    }
    
    p_start = strstr((const char *)ESP8266_GetRxFrame(), "\"");
    p_end = strstr(p_start + 1, "\"");
    *p_end = '\0';
    sprintf(buf, "%s", p_start + 1);
    
    return ATK_MW8266D_EOK;
}

/**
 * @brief       ATK-MW8266D连接TCP服务器
 * @param       server_ip  : TCP服务器IP地址
 *              server_port: TCP服务器端口号
 * @retval      ATK_MW8266D_EOK  : 连接TCP服务器成功
 *              ATK_MW8266D_ERROR: 连接TCP服务器失败
 */
uint8_t atk_mw8266d_connect_tcp_server(char *server_ip, char *server_port)
{
    uint8_t ret;
    char cmd[64];
    
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%s", server_ip, server_port);
    ret = atk_mw8266d_send_at_cmd(cmd, "CONNECT", 5000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}


/**
 * @brief       ATK-MW8266D进入透传
 * @param       无
 * @retval      ATK_MW8266D_EOK  : 进入透传成功
 *              ATK_MW8266D_ERROR: 进入透传失败
 */
uint8_t atk_mw8266d_enter_unvarnished(void)
{
    uint8_t ret;
    
    ret  = atk_mw8266d_send_at_cmd("AT+CIPMODE=1", "OK", 500);
    ret += atk_mw8266d_send_at_cmd("AT+CIPSEND", ">", 500);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}

/**
 * @brief       ATK-MW8266D退出透传
 * @param       无
 * @retval      无
 */
void atk_mw8266d_exit_unvarnished(void)
{
    Serial_Printf(USART2,"+++");
}

/**
 * @brief       ATK-MW8266D连接原子云服务器
 * @param       id : 原子云设备编号
 *              pwd: 原子云设备密码
 * @retval      ATK_MW8266D_EOK  : 连接原子云服务器成功
 *              ATK_MW8266D_ERROR: 连接原子云服务器失败
 */
uint8_t atk_mw8266d_connect_atkcld(char *id, char *pwd)
{
    uint8_t ret;
    char cmd[64];
    
    sprintf(cmd, "AT+ATKCLDSTA=\"%s\",\"%s\"", id, pwd);
    ret = atk_mw8266d_send_at_cmd(cmd, "CLOUD CONNECTED", 10000);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}

/**
 * @brief       ATK-MW8266D断开原子云服务器连接
 * @param       无
 * @retval      ATK_MW8266D_EOK  : 断开原子云服务器连接成功
 *              ATK_MW8266D_ERROR: 断开原子云服务器连接失败
 */
uint8_t atk_mw8266d_disconnect_atkcld(void)
{
    uint8_t ret;
    
    ret = atk_mw8266d_send_at_cmd("AT+ATKCLDCLS", "CLOUD DISCONNECT", 500);
    if (ret == ATK_MW8266D_EOK)
    {
        return ATK_MW8266D_EOK;
    }
    else
    {
        return ATK_MW8266D_ERROR;
    }
}



ESP8266_INIT_ERROR ESP8266_Init(uint32_t baudrate)
{
    //AT正确的话3,4,6一般不需要报错信息

    // 1. 硬件初始化（UART和GPIO）
    ESP8266_UART_Init(USART2, baudrate, 1, 1);
    
    // 2. 测试AT指令
    if(atk_mw8266d_at_test() != ATK_MW8266D_EOK) {
        return ESP8266_ATTEST_ERROR;
    }
    
 
    // 4. 设置WiFi模式（Station模式）
    atk_mw8266d_set_mode(1);
  
     //3.复位
    atk_mw8266d_sw_reset();
    
    // 5. 连接WiFi
    if(atk_mw8266d_join_ap("ZTE-kZzeDz", "Hjwa3b9c") != ATK_MW8266D_EOK) {
        return ESP8266_JOINWIFIAP_ERROR;
    }
    
    //6.设置单连接
    atk_mw8266d_send_at_cmd("AT+CIPMUX=0","OK",500);

    //7. 连接TCP服务器
    if(atk_mw8266d_connect_tcp_server("192.168.5.64", "8086") != ATK_MW8266D_EOK) {
        return ESP8266_CONNECTTCPSERVER_ERROR;
    }
		Delay_ms(800);
    //8. 进入透传模式
    if(atk_mw8266d_enter_unvarnished() != ATK_MW8266D_EOK) {
        return ESP8266_ENTERUNVARNISHED_ERROR;
    }

    //9. tcp透传测试
    Serial_Printf(USART2,"test\n\r");
    esp8266_LoadingFinished_Flag = 1;

    // Delay_ms(800);
    // //9. 关闭tcp透传
    // atk_mw8266d_exit_unvarnished();
    
    
    
    return ESP8266_INIT_EOK;
}


void ESP8266_ERROR_Handling(ESP8266_INIT_ERROR this)
{
    esp8266_LoadingFinished_Flag = 1;
switch (this)
{
    case  0:
    Serial_Printf(USART1,"测试成功");
    
    break;
    case  1:
    Serial_Printf(USART1,"AT测试错误");
    
    break;
    case  2:
    Serial_Printf(USART1,"加入WIFI错误");
    
    break;
    case  3:
    Serial_Printf(USART1,"加入tcp服务器错误");
    
    break;
    case  4:
    Serial_Printf(USART1,"加入tcp透传错误");
    
    break;

default:
    Serial_Printf(USART1,"未知类型");
    break;
}




}


