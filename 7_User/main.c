#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <math.h> //绝对值浮点数
#include "motor.h"
#include "delay.h"
#include "oled.h"
#include "encoder.h"
#include "bsp_usart.h"
#include "mpu6050.h"
#include "pid.h"
#include "oled_font.h"
#include "timer.h"
#include "iwdg.h"
#include "esp8266.h"
#include "rp.h"

MPU6050 MPU6050_Data; // 创建一个结构体用来储存欧拉角

// 程序逻辑编写
// 一类代码 基础的内部外设,比如定时器,看门狗
// 二类代码 基础的外部外设,比如RP,key,led
// 三类代码 普通的内部外设与外部外设联动,比如电机的编码器,pwm输出,oled的i2c,蓝牙的usrt
// 四类代码 进阶的内部外设与外部外设联动,比如wifi,mpu6050,

// 最主要的特征
// 一类代码, 基本上可以使用hal库来编写(虽然我的程序是标准库),不需要任何的外部因素,比如key的抖动,rp的可调大小
// 二类代码, 基本上不用考虑很多,但是跟三类代码的区别是不用使用内部的一些外设,只要gpio,顶多中断,从复杂度上来看少很多
// 三类代码, 占用了一些外设,要考量的就很多了,但是跟四类代码相比,少了错误的判断,因为能通过外部观测来看,比如oled出错就是显示不出来
// 四类代码, 加入了错误机制,wifi的纠错码,mpu6050的id读取

// 代码结构编写
// 一类代码完全可以使用hal库
// 二类代码需要一些值,比如arr,psc,部分外设面向对象
// 三类代码,面向对象
// 四类代码,面向结果,面向对象

// 主程序的pid调参,可以使用tcp,oled俩种方式,互相不冲突
// 使用了#define方式,调整调参对象,以下是用户设置

/* 128x64 数字符号为8x16 中文为16x16
--------------OLED显示页面一--------------
    电机X
    实:+0000 P:+xx.x
    目:+0000 I:+xx.x
    输:+0000 D:+xx.x
--------------OLED显示页面二--------------
    MPU6050姿态角
    俯仰角:+xxx.xx
    翻滚角:+xxx.xx
    偏航角:+xxx.xx
*/

/*
tcp发送格式 主机--->单片机     @开头 \r\n结尾
tcp接收格式 单片机--->主机     自己开发
*/

/*尚未完成的地方


USART的面向对象编程
tcp发送和oled  编写过程太过复杂,如果不熟悉整个程序,难以编写
pid的优化,有很多冗余
下面的调试有一些问题,需要优化
tcp的dma没有写
mpu的dma没有写

*/

// 电机A调试还是电机B调试(仅串口2发送无oled)
#define MOTORA_DEBUG
// #define MOTORB_DEBUG

// 双环pid选项内环还是外环调试(用于调参串口2)
// #define INNER_DEBUG        //内部
#define OUTER_DEBUG // 外部

// PID control mode
#define ANGLE_MODE

/* Bring-up safety switches: keep both motors electrically stopped until every
 * peripheral (especially IMU and ESP8266 TCP link) has been verified. */
#define MOTOR_CONTROL_ENABLED 0U
/* Keep this off during IMU validation; PID samples are meaningful only when
 * MOTOR_CONTROL_ENABLED is 1. */
#define PID_TELEMETRY_ENABLED 0U

// 俩者不可共存
#ifdef MOTORA_DEBUG
#ifdef MOTORB_DEBUG
#error "MOTORA_DEBUG and MOTORB_DEBUG cannot be defined at the same time!"
#endif
#endif

// 俩者不可共存
#ifdef INNER_DEBUG
#ifdef OUTER_DEBUG
#error "INNER_DEBUG and OUTER_DEBUG cannot be defined at the same time!"
#endif
#endif

#ifdef ANGLE_MODE
/* PID data only; hardware I/O remains explicit in the control task. */
PID_t pidA_inner, pidB_inner;
PID_t pidA_outer, pidB_outer;
#endif

//   ------------------主函数逻辑判断------------------
uint8_t page1_flag       = 1;
uint8_t page1_firstEntry = 1;

uint8_t page2_flag       = 0;
uint8_t page2_firstEntry = 1;

float Mechanical_Median = 3;

//   ------------------MPU6050变量------------------
uint8_t mpu6050_online;

float AccData[3];
float GyroData[3];
float Temperature;
int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t mpu6050_count;
uint8_t mpu6050_timingFlag;
uint8_t mpu6050_errorFlag;

//   ------------------定时器计次位------------------
uint16_t pid_count                = 0;
uint8_t oled_rxClear_flag         = 0;
uint8_t oled_BufClear_flag        = 0;
uint8_t oled_BufDisplay_flag      = 0;
uint8_t oled_BufDisplayOne_flag   = 0;
uint8_t oled_BufDisplayThree_flag = 0;
uint8_t oled_BufDisplayTwo_flag   = 0;

float angle_temp = 0;

//   ------------------PID调试变量------------------
float TargetA, ActualA, OutA;
float TargetB, ActualB, OutB;

/* Latest PID samples for the six-channel serial tuning waveform:
 * angle target/actual/output followed by speed target/actual/output. */
volatile float angle_target;
volatile float angle_actual;
volatile float angle_out;
volatile float speed_target;
volatile float speed_actual;
volatile float speed_out;

uint8_t fallDown_flag   = 0;
uint8_t angle_errorFlag = 0;
uint8_t pid_timingFlag  = 0;

float Angle_Get()
{
    return -angle_temp;
}
// 往前倒+,往后- oled方向

void Main_Config_OLED()
{
    OLED_Init();
    // if (OLED_ID() == 0)
    // {
    //    while (1);
    // }
    OLED_DMA_Init();
    OLED_ShowString((128 - 96) / 2, (64 - 16) / 2, "系统初始化中", OLED_8X16);
    OLED_Update_DMA();
    Delay_ms(10);
}

void Main_Config_ESP8266()
{
    ESP8266_INIT_ERROR ret = ESP8266_INIT_EOK;
    ret                    = ESP8266_Init(115200);
    if (ret != ESP8266_INIT_EOK) {
        ESP8266_ERROR_Handling(ret);
    }
}

void Main_Config_MPU6050()
{
    MPU6050_Init();
    mpu6050_online = MPU6050_IsOnline();
}

void Main_Config()
{
    Main_Config_OLED();
    Main_Config_MPU6050();
    Serial_Init();
    Motor_Init(10, 72);
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
    Timer_Init();
    Main_Config_ESP8266();
#ifdef ANGLE_MODE
#ifdef INNER_DEBUG
    PID_Init(&pidA_inner, 0.2f, 0.0f, 0.9f, 100.0f, 100.0f);
    PID_Init(&pidB_inner, 0.2f, 0.0f, 0.9f, 100.0f, 100.0f);
#endif
#ifdef OUTER_DEBUG
    PID_Init(&pidA_inner, 0.2f, 0.0f, 0.9f, 50.0f, 50.0f);
    PID_Init(&pidB_inner, 0.2f, 0.0f, 0.9f, 50.0f, 50.0f);
    PID_Init(&pidA_outer, 2.0f, 0.0f, 0.0f, 5000.0f, 10000.0f);
    PID_Init(&pidB_outer, 2.0f, 0.0f, 0.0f, 5000.0f, 10000.0f);
#endif
#endif
}

void OLED_PIDDisplay()
{
    // 测试数据发送
    OLED_Clear();
// 处理修改显示
#ifdef MOTORA_DEBUG
    OLED_ShowString(0, 0, "电机A", OLED_8X16);
#endif
#ifdef MOTORB_DEBUG
    OLED_ShowString(0, 0, "电机B", OLED_8X16);
#endif
    OLED_ShowString(0, 16, "实:", OLED_8X16);
    OLED_ShowString(0, 32, "目:", OLED_8X16);
    OLED_ShowString(0, 48, "输:", OLED_8X16);

#ifdef INNER_DEBUG
    OLED_ShowString(72, 0, "内环", OLED_8X16);
#endif
#ifdef OUTER_DEBUG
    OLED_ShowString(72, 0, "外环", OLED_8X16);
#endif
    OLED_ShowString(72, 16, "P:", OLED_8X16);
    OLED_ShowString(72, 32, "I:", OLED_8X16);
    OLED_ShowString(72, 48, "D:", OLED_8X16);
    OLED_Update_DMA();
    // OLED_Update();
}

void OLED_PIDCycleDisplay()
{
    if (oled_BufDisplayOne_flag) {
#ifdef MOTORA_DEBUG
        OLED_ShowSignedNum(24, 16, ActualA, 4, OLED_8X16);
        OLED_ShowSignedNum(24, 32, TargetA, 4, OLED_8X16);
#endif

#ifdef MOTORB_DEBUG
        OLED_ShowSignedNum(24, 16, ActualB, 4, OLED_8X16);
        OLED_ShowSignedNum(24, 32, TargetB, 4, OLED_8X16);
#endif
        oled_BufDisplayOne_flag = 0;
    }
    if (oled_BufDisplayTwo_flag) {
#ifdef MOTORA_DEBUG
        OLED_ShowSignedNum(24, 48, OutA, 4, OLED_8X16);
#endif

#ifdef MOTORB_DEBUG
        OLED_ShowSignedNum(24, 48, OutB, 4, OLED_8X16);
#endif

#ifdef INNER_DEBUG
        OLED_ShowFloatNum(88, 16, pidA_inner.kp, 2, 1, OLED_8X16);
#endif
#ifdef OUTER_DEBUG
        OLED_ShowFloatNum(88, 16, pidA_outer.kp, 2, 1, OLED_8X16);
#endif
        oled_BufDisplayTwo_flag = 0;
    }
    if (oled_BufDisplayThree_flag) {
#ifdef INNER_DEBUG
        OLED_ShowFloatNum(88, 32, pidA_inner.ki, 2, 1, OLED_8X16);
        OLED_ShowFloatNum(88, 48, pidA_inner.kd, 2, 1, OLED_8X16);
#endif

#ifdef OUTER_DEBUG
        OLED_ShowFloatNum(88, 32, pidA_outer.ki, 2, 1, OLED_8X16);
        OLED_ShowFloatNum(88, 48, pidA_outer.kd, 2, 1, OLED_8X16);
#endif
        oled_BufDisplayThree_flag = 0;
    }
    if (oled_BufDisplay_flag) {
        // OLED_UpdateArea(40,16,40,48);
        // OLED_UpdateArea(104,16,24,48);
        OLED_Update_DMA();
        oled_BufDisplay_flag = 0;
    }
}

void OLED_MPU6050Display()
{
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050姿态角", OLED_8X16);
    OLED_ShowString(0, 16, "俯仰角:", OLED_8X16);
    OLED_ShowString(0, 32, "翻滚角:", OLED_8X16);
    OLED_ShowString(0, 48, "偏航角:", OLED_8X16);
    // OLED_UpdateArea(0, 0, 104, 16);
    // OLED_UpdateArea(0, 16, 56, 48);
    OLED_Update_DMA();
}

void OLED_MPU6050CycleDisplay()
{
    if (oled_BufDisplayOne_flag) {
        OLED_ShowFloatNum(54, 16, MPU6050_Data.pitch, 3, 2, OLED_8X16);
        oled_BufDisplayOne_flag = 0;
    }
    if (oled_BufDisplayTwo_flag) {
        OLED_ShowFloatNum(54, 32, MPU6050_Data.roll, 3, 2, OLED_8X16);
        oled_BufDisplayTwo_flag = 0;
    }
    if (oled_BufDisplayThree_flag) {
        OLED_ShowFloatNum(54, 48, MPU6050_Data.yaw, 3, 2, OLED_8X16);
        oled_BufDisplayThree_flag = 0;
    }
    if (oled_BufDisplay_flag) {
        // 局部更新数据区域（宽度71保证不超128）
        OLED_Update_DMA();
        // OLED_UpdateArea(54, 16, 74, 48);
        oled_BufDisplay_flag = 0;
    }
}

// 使用OLED测试串口是否正常使用 最好发2位 不然OLED挤不下一行
void OLED_SerialSend()
{
#ifdef USART1_FLAG
    Serial_Printf(USART1, "串口1初始化成功！\r\n");
#endif // USART1_FLAG

#ifdef USART2_FLAG
    Serial_Printf(USART2, "串口2初始化成功！\r\n");
#endif // USART2_FLAG

#ifdef USART3_FLAG
    Serial_Printf(USART3, "串口3初始化成功！\n");
#endif // USART3_FLAG
}

void OLED_PIDCycleSend()
{
#if PID_TELEMETRY_ENABLED
#ifdef MOTORA_DEBUG
    Serial_Printf(USART2, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                  (double)angle_target, (double)angle_actual, (double)angle_out,
                  (double)speed_target, (double)speed_actual, (double)speed_out);
#endif
#ifdef MOTORB_DEBUG
    Serial_Printf(USART2, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                  (double)angle_target, (double)angle_actual, (double)angle_out,
                  (double)speed_target, (double)speed_actual, (double)speed_out);
#endif
#endif
}

void OLED_MPU6050CycleSend()
{
#if PID_TELEMETRY_ENABLED
    Serial_Printf(USART2, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", TargetA, ActualA, OutA, MPU6050_Data.roll, MPU6050_Data.pitch, MPU6050_Data.yaw); // 串口发送数据
#endif
}

void Serial_change()
{
    // 每3秒清除一次
    if (oled_rxClear_flag) {
        oled_rxClear_flag = 0;
    }
    if (!Serial_RxFlag1 && !Serial_RxFlag2) {
        return; // 没有新数据，直接返回
    }

    if (Serial_RxFlag2) {
        char *ptr = Serial_RxPacket2; // 指向接收缓冲区

        /* TCP transparent-mode commands use an '@' prefix, for example:
         * @kp=1.0\r\n  or  @target=100\r\n.
         * Ignore unrelated server text instead of replying with an error. */
        if (ptr[0] == '@') {
            ptr++; // 跳过 '@'

            if (strncmp(ptr, "macd=", 5) == 0) {
                float macd;
                if (sscanf(ptr + 5, "%f", &macd) == 1) {
                    Mechanical_Median = macd; // 更新 机械中值
                    Serial_Printf(USART2, "OK:MACD updated\r\n");
                }
            } else if (strncmp(ptr, "angle_kp=", 9) == 0 || strncmp(ptr, "kp=", 3) == 0) {
                float kp_value;
                char *value = (ptr[0] == 'a') ? ptr + 9 : ptr + 3;
                if (sscanf(value, "%f", &kp_value) == 1) {
                    pidA_outer.kp = kp_value;
                    pidB_outer.kp = kp_value;
                    Serial_Printf(USART2, "OK:ANGLE_KP updated\r\n");
                }
            } else if (strncmp(ptr, "angle_ki=", 9) == 0 || strncmp(ptr, "ki=", 3) == 0) {
                float ki_value;
                char *value = (ptr[0] == 'a') ? ptr + 9 : ptr + 3;
                if (sscanf(value, "%f", &ki_value) == 1) {
                    pidA_outer.ki = ki_value;
                    pidB_outer.ki = ki_value;
                    Serial_Printf(USART2, "OK:ANGLE_KI updated\r\n");
                }
            } else if (strncmp(ptr, "angle_kd=", 9) == 0 || strncmp(ptr, "kd=", 3) == 0) {
                float kd_value;
                char *value = (ptr[0] == 'a') ? ptr + 9 : ptr + 3;
                if (sscanf(value, "%f", &kd_value) == 1) {
                    pidA_outer.kd = kd_value;
                    pidB_outer.kd = kd_value;
                    Serial_Printf(USART2, "OK:ANGLE_KD updated\r\n");
                }
            } else if (strncmp(ptr, "speed_kp=", 9) == 0) {
                float kp_value;
                if (sscanf(ptr + 9, "%f", &kp_value) == 1) {
                    pidA_inner.kp = kp_value;
                    pidB_inner.kp = kp_value;
                    Serial_Printf(USART2, "OK:SPEED_KP updated\r\n");
                }
            } else if (strncmp(ptr, "speed_ki=", 9) == 0) {
                float ki_value;
                if (sscanf(ptr + 9, "%f", &ki_value) == 1) {
                    pidA_inner.ki = ki_value;
                    pidB_inner.ki = ki_value;
                    Serial_Printf(USART2, "OK:SPEED_KI updated\r\n");
                }
            } else if (strncmp(ptr, "speed_kd=", 9) == 0) {
                float kd_value;
                if (sscanf(ptr + 9, "%f", &kd_value) == 1) {
                    pidA_inner.kd = kd_value;
                    pidB_inner.kd = kd_value;
                    Serial_Printf(USART2, "OK:SPEED_KD updated\r\n");
                }
            }
            // 解析 target
            else if (strncmp(ptr, "target=", 7) == 0) {
                float target_value;
                if (sscanf(ptr + 7, "%f", &target_value) == 1) {
                    TargetA = target_value; // 更新目标值
                    TargetB = target_value; // 更新目标值
                    Serial_Printf(USART2, "OK:Target updated\r\n");
                }
            } else {
                Serial_Printf(USART2, "ERROR:Unknown command; use @kp=, @ki=, @kd=, @macd=, or @target=\r\n");
            }
        }

        Serial_RxFlag2 = 0; // 清除接收标志
    }

    if (Serial_RxFlag1) {
        // 将接收到的数据通过串口回显
        if (strcmp(Serial_RxPacket1, "TargetA add") == 0) {
            TargetA += 10;
        } else if (strcmp(Serial_RxPacket1, "TargetA lower") == 0) {
            TargetA -= 10;
        }

        else if (strcmp(Serial_RxPacket1, "TargetB add") == 0) {
            TargetB += 10;
        } else if (strcmp(Serial_RxPacket1, "TargetB lower") == 0) {
            TargetB -= 10;
        } else if (strcmp(Serial_RxPacket1, "MotorA forward") == 0) {
            TargetA += 330 * 4;
            // TargetA += 100;
        } else if (strcmp(Serial_RxPacket1, "MotorA backward") == 0) {
            TargetA += -330 * 4;
            // TargetA += -100;
        } else if (strcmp(Serial_RxPacket1, "page1") == 0) {
            page1_firstEntry = 1;
            page1_flag       = 1;
            page2_flag       = 0;
        } else if (strcmp(Serial_RxPacket1, "page2") == 0) {
            page2_firstEntry = 1;
            page1_flag       = 0;
            page2_flag       = 1;
        }
        Serial_RxFlag1 = 0; // 重置接收标志
    }
}

#ifdef ANGLE_MODE
void MotorControlLoop_Angle(void)
{
#ifdef INNER_DEBUG
    float speed_a = (float)Encoder_TIM4_Get();
    float speed_b = (float)Encoder_TIM3_Get();
    OutA          = PID_Update(&pidA_inner, TargetA, speed_a);
    OutB          = PID_Update(&pidB_inner, TargetB, speed_b);
    ActualA       = speed_a;
    ActualB       = speed_b;
#endif
#ifdef OUTER_DEBUG
    float angle_a_actual = Angle_Get();
    float angle_b_actual = angle_a_actual;
    float speed_a_actual = (float)Encoder_TIM4_Get();
    float speed_b_actual = (float)Encoder_TIM3_Get();
    float angle_a_out;
    float angle_b_out;
    float speed_a_target;
    float speed_b_target;
    float speed_a_out;
    float speed_b_out;

    angle_target = Mechanical_Median;
    angle_actual = angle_a_actual;
    angle_a_out  = PID_Update(&pidA_outer, angle_target, angle_actual);
    angle_b_out  = PID_Update(&pidB_outer, Mechanical_Median, angle_b_actual);
    angle_out    = angle_a_out;

    speed_a_target = angle_a_out + TargetA;
    speed_b_target = angle_b_out + TargetB;
    speed_target   = speed_a_target;
    speed_actual   = speed_a_actual;
    speed_a_out    = PID_Update(&pidA_inner, speed_target, speed_actual);
    speed_b_out    = PID_Update(&pidB_inner, speed_b_target, speed_b_actual);
    speed_out      = speed_a_out;

    OutA    = speed_a_out;
    OutB    = speed_b_out;
    ActualA = angle_a_actual;
    ActualB = angle_b_actual;
#endif
    MotorA_SetSpeed((int16_t)OutA);
    MotorB_SetSpeed((int16_t)OutB);
}
#endif

int main(void)
{
    Delay_ms(10);
    Main_Config();
    // 20ms看门
    // IWDG_Config(IWDG_Prescaler_16,100);
    while (1) {
        if (page1_flag) {
            if (page1_firstEntry) {
                OLED_PIDDisplay();
                OLED_SerialSend();
                page1_firstEntry = 0;
            } else {
                OLED_PIDCycleDisplay();
            }
        } else if (page2_flag) {
            if (page2_firstEntry) {
                OLED_MPU6050Display();
                OLED_SerialSend();
                page2_firstEntry = 0;
            } else {
                OLED_MPU6050CycleDisplay();
                OLED_MPU6050CycleSend();
            }
        }
        // 串口改变目标值
        Serial_change();
        /* Reassert the safe state during peripheral bring-up. */
#if !MOTOR_CONTROL_ENABLED
        MotorA_SetSpeed(0);
        MotorB_SetSpeed(0);
#endif
        IWDG_Feed();
        OLED_PIDCycleSend();
    }
}

// 使用 static 关键字使 Count 保持其值
void TIM1_UP_IRQHandler(void)
{
    static uint16_t sys_cnt = 0;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        sys_cnt++;
        if (sys_cnt % 10 == 0) {
            // 软件i2c 0.00085
            // 硬件i2c(400k) 0.00124
            if (mpu6050_online) {
                MPU6050_Get_Angle_Plus(&MPU6050_Data);
                angle_temp = MPU6050_Data.roll;
            }

#ifdef SETLOACTION_MODE
            MotorControlLoop_SetLoaction();
#endif // SETLOACTION_MODE

#if MOTOR_CONTROL_ENABLED
            MotorControlLoop_Angle();
#endif // MOTOR_CONTROL_ENABLED
        }
        if (sys_cnt % 100 == 0) oled_BufDisplay_flag = 1;
        if (sys_cnt % 24 == 0) oled_BufDisplayOne_flag = 1;
        if (sys_cnt % 48 == 0) oled_BufDisplayTwo_flag = 1;
        if (sys_cnt % 72 == 0) oled_BufDisplayThree_flag = 1;
        // 测试可行性,1s 闪烁led(pc13),占用RTC口,需要在主程序初始化LED_Init();
        if (sys_cnt % 1000 == 0) {
            // if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET )
            // {
            //     GPIO_WriteBit(GPIOC, GPIO_Pin_13 ,Bit_RESET); // 翻转 PA0
            // }else
            // {
            //     GPIO_WriteBit(GPIOC, GPIO_Pin_13 ,Bit_SET); // 翻转 PA0
            // }
            // void LED_Init()
            // {
            //     PWR_BackupAccessCmd(ENABLE);//允许修改RTC 和后备寄存器

            //     RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。

            //     BKP_TamperPinCmd(DISABLE);//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用

            //     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
            //     GPIO_InitTypeDef GPIO_InitStart;
            //     GPIO_InitStart.GPIO_Pin=GPIO_Pin_13;
            //     GPIO_InitStart.GPIO_Mode=GPIO_Mode_Out_PP;
            //     GPIO_InitStart.GPIO_Speed=GPIO_Speed_50MHz;
            //     GPIO_Init(GPIOC,&GPIO_InitStart);
            // }
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 根据您的定时器和情况调整
    }
}
