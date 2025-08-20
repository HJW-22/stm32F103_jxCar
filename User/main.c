#include "stm32f10x.h" 
#include <string.h>
#include <math.h> //绝对值浮点数
#include "Motor.h"
#include "Delay.h"
#include "OLED.h"
#include "Encoder.h"
#include "Serial.h"
#include "MPU6050.h"
#include "PID_Positional.h"
#include "OLED_Font.h"
#include "Timer.h"
#include "IWDG.h"
#include "esp8266.h"
#include "RP.h"


MPU6050 MPU6050_Data;	//创建一个结构体用来储存欧拉角


//程序逻辑编写
//一类代码 基础的内部外设,比如定时器,看门狗
//二类代码 基础的外部外设,比如RP,key,led
//三类代码 普通的内部外设与外部外设联动,比如电机的编码器,pwm输出,oled的i2c,蓝牙的usrt
//四类代码 进阶的内部外设与外部外设联动,比如wifi,mpu6050,

//最主要的特征
//一类代码, 基本上可以使用hal库来编写(虽然我的程序是标准库),不需要任何的外部因素,比如key的抖动,rp的可调大小
//二类代码, 基本上不用考虑很多,但是跟三类代码的区别是不用使用内部的一些外设,只要gpio,顶多中断,从复杂度上来看少很多
//三类代码, 占用了一些外设,要考量的就很多了,但是跟四类代码相比,少了错误的判断,因为能通过外部观测来看,比如oled出错就是显示不出来
//四类代码, 加入了错误机制,wifi的纠错码,mpu6050的id读取


//代码结构编写
//一类代码完全可以使用hal库
//二类代码需要一些值,比如arr,psc,部分外设面向对象
//三类代码,面向对象
//四类代码,面向结果,面向对象


//主程序的pid调参,可以使用tcp,oled俩种方式,互相不冲突
//使用了#define方式,调整调参对象,以下是用户设置

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


//电机A调试还是电机B调试(仅串口2发送无oled)
#define MOTORA_DEBUG
// #define MOTORB_DEBUG

//双环pid选项内环还是外环调试(用于调参串口2)
//#define INNER_DEBUG        //内部
#define OUTER_DEBUG     //外部




//PID_Init
//PID_Init_Angle

// #define SETLOACTION_MODE
#define ANGLE_MODE




//俩者不可共存 
#ifdef MOTORA_DEBUG
    #ifdef MOTORB_DEBUG
        #error "MOTORA_DEBUG and MOTORB_DEBUG cannot be defined at the same time!"
    #endif
#endif

//俩者不可共存 
#ifdef INNER_DEBUG
    #ifdef OUTER_DEBUG
        #error "INNER_DEBUG and OUTER_DEBUG cannot be defined at the same time!"
    #endif
#endif

//俩者不可共存 
#ifdef SETLOACTION_MODE
    #ifdef ANGLE_MODE
        #error "SETLOACTION_MODE and ANGLE_MODE cannot be defined at the same time!"
    #endif
#endif


#ifdef SETLOACTION_MODE
    PID_Params pidA,pidB;
#endif // SETLOACTION_MODE

#ifdef ANGLE_MODE
    PID_AngleParam pidA_inner,pidB_inner;
    PID_AngleParam pidA_outer,pidB_outer;
#endif




//   ------------------主函数逻辑判断------------------
uint8_t page1_flag       = 1;
uint8_t page1_firstEntry = 1;

uint8_t page2_flag       = 0;
uint8_t page2_firstEntry = 1;

float Mechanical_Median = 3;

//   ------------------MPU6050变量------------------
uint8_t ID;

float AccData[3];
float GyroData[3];
float Temperature;
int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t mpu6050_count;
uint8_t mpu6050_timingFlag;
uint8_t mpu6050_errorFlag;

//   ------------------定时器计次位------------------
uint16_t pid_count = 0;
uint8_t oled_rxClear_flag=0;
uint8_t oled_BufClear_flag=0;
uint8_t oled_BufDisplay_flag=0;
uint8_t oled_BufDisplayOne_flag=0;
uint8_t oled_BufDisplayThree_flag=0;
uint8_t oled_BufDisplayTwo_flag=0;


float angle_temp=0;


//   ------------------PID调试变量------------------
 float TargetA, ActualA, OutA;
 float TargetB, ActualB, OutB;

 uint8_t fallDown_flag=0;
 uint8_t angle_errorFlag=0;
 uint8_t pid_timingFlag=0;

float Angle_Get()
{
    return -angle_temp;
}
//往前倒+,往后- oled方向





void Main_Config_OLED()
{
    OLED_Init(I2C1,1);
    // if (OLED_ID() == 0)
    // {
    //    while (1);
    // }
    OLED_DMA_Init();
    OLED_ShowString( (128-96)/2, (64-16)/2, "系统初始化中", OLED_8X16);
    OLED_Update_DMA();
    Delay_ms(10);
}

void Main_Config_ESP8266()
{
    ESP8266_INIT_ERROR ret= ESP8266_INIT_EOK;
    ret = ESP8266_Init(115200);
    if (ret !=ESP8266_INIT_EOK)
    {
        ESP8266_ERROR_Handling(ret);
    }
    
}

void Main_Config_MPU6050()
{
   MPU6050_Init(I2C2,0);
    // if (MPU6050_ID() == 0)
    // {
    //    while (1);
    // }
}


void Main_Config()
{
    Main_Config_OLED();
    Main_Config_ESP8266();
    Main_Config_MPU6050();
    
    //串口初始化
    Serial_Init(); 
    //电机初始化(10K)
    Motor_Init(10,72);
    
    //一些人机交互 RP有硬件设计有问题不要使用
    // RP_Init();
    // LED_Init();

    //最后初始化,若先进入中断,此时外设尚未初始化,那就会出错
    Timer_Init();

    #ifdef SETLOACTION_MODE
    // 初始化电机A的PID
    PID_Init(&pidA, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    // 初始化电机B的PID
    PID_Init(&pidB, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    #endif // SETLOACTION_MODE
    
    #ifdef ANGLE_MODE
    #ifdef INNER_DEBUG
    PID_Init_Angle(MOTOR_A,1,&pidA_inner,NULL,Encoder_TIM4_Get,0.2,0,0.9,-100,100,MotorA_SetSpeed);
    PID_Init_Angle(MOTOR_B,1,&pidB_inner,NULL,Encoder_TIM3_Get,0.2,0,0.9,-100,100,MotorB_SetSpeed);
    #endif // INNER_DEBUG
   
    //一般来说如果单环的参数不可以使用到双环内
    #ifdef OUTER_DEBUG
    // PID_Init_Angle(MOTOR_A,&pidA_inner,Angle_Get,200,0,2500,-7200,7200,MotorA_SetSpeed);
    // PID_Init_Angle(MOTOR_B,&pidB_inner,Angle_Get,200,0,2500,-7200,7200,MotorB_SetSpeed);

    PID_Init_Angle(MOTOR_A,1,&pidA_inner,NULL,Encoder_TIM4_Get,0.2,0,0.9,-50,50,MotorA_SetSpeed);
    PID_Init_Angle(MOTOR_B,1,&pidB_inner,NULL,Encoder_TIM3_Get,0.2,0,0.9,-50,50,MotorB_SetSpeed);
    PID_Init_Angle(MOTOR_A,0,&pidA_outer,Angle_Get,NULL,2,0,0,-10000,10000,NULL);
    PID_Init_Angle(MOTOR_B,0,&pidB_outer,Angle_Get,NULL,2,0,0,-10000,10000,NULL);
    #endif // OUTER_DEBUG
   
    #endif
}

void OLED_PIDDisplay()
{
    // 测试数据发送
    OLED_Clear();
//处理修改显示
#ifdef MOTORA_DEBUG
    OLED_ShowString(0, 0,  "电机A",OLED_8X16);
#endif 
#ifdef MOTORB_DEBUG
    OLED_ShowString(0, 0,  "电机B",OLED_8X16);
#endif
	OLED_ShowString(0, 16, "实:",OLED_8X16);
	OLED_ShowString(0, 32, "目:",OLED_8X16);
	OLED_ShowString(0, 48, "输:",OLED_8X16);

#ifdef INNER_DEBUG
    OLED_ShowString(72, 0,  "内环",OLED_8X16);
#endif 
#ifdef OUTER_DEBUG
    OLED_ShowString(72, 0,  "外环",OLED_8X16);
#endif
    OLED_ShowString(72, 16, "P:",OLED_8X16);
    OLED_ShowString(72, 32, "I:",OLED_8X16);
    OLED_ShowString(72, 48, "D:",OLED_8X16);
    OLED_Update_DMA();
    // OLED_Update();
}

void OLED_PIDCycleDisplay()
{
     if (oled_BufDisplayOne_flag)
    {
        #ifdef MOTORA_DEBUG
        OLED_ShowSignedNum(24, 16, ActualA, 4,OLED_8X16);
        OLED_ShowSignedNum(24, 32, TargetA, 4,OLED_8X16);
        #endif 
       
        #ifdef MOTORB_DEBUG
        OLED_ShowSignedNum(24, 16, ActualB, 4,OLED_8X16);
        OLED_ShowSignedNum(24, 32, TargetB, 4,OLED_8X16);
        #endif 
        oled_BufDisplayOne_flag=0;
    }
     if (oled_BufDisplayTwo_flag)
    {
        #ifdef MOTORA_DEBUG
        OLED_ShowSignedNum(24, 48, OutA, 4,OLED_8X16);
        #endif 

        #ifdef MOTORB_DEBUG
        OLED_ShowSignedNum(24, 48, OutB, 4,OLED_8X16);
        #endif 

        #ifdef INNER_DEBUG
        OLED_ShowFloatNum(88, 16, pidA_inner.kp, 2,1,OLED_8X16);
        #endif 
        #ifdef OUTER_DEBUG
        OLED_ShowFloatNum(88, 16, pidA_outer.kp, 2,1,OLED_8X16);
        #endif
        oled_BufDisplayTwo_flag=0;
    }
     if (oled_BufDisplayThree_flag)
    {
        #ifdef INNER_DEBUG
        OLED_ShowFloatNum(88, 32, pidA_inner.ki, 2,1,OLED_8X16);
        OLED_ShowFloatNum(88, 48, pidA_inner.kd, 2,1,OLED_8X16);
        #endif 

        #ifdef OUTER_DEBUG
        OLED_ShowFloatNum(88, 32, pidA_outer.ki, 2,1,OLED_8X16);
        OLED_ShowFloatNum(88, 48, pidA_outer.kd, 2,1,OLED_8X16);
        #endif 
        oled_BufDisplayThree_flag=0;
    }
     if (oled_BufDisplay_flag)
    {
        // OLED_UpdateArea(40,16,40,48);
        // OLED_UpdateArea(104,16,24,48);
        OLED_Update_DMA();
        oled_BufDisplay_flag=0;
    }
}

void OLED_MPU6050Display()
{
    OLED_Clear();
    OLED_ShowString(0, 0,  "MPU6050姿态角", OLED_8X16);
    OLED_ShowString(0, 16, "俯仰角:", OLED_8X16);
    OLED_ShowString(0, 32, "翻滚角:", OLED_8X16);
    OLED_ShowString(0, 48, "偏航角:", OLED_8X16);
    // OLED_UpdateArea(0, 0, 104, 16);
    // OLED_UpdateArea(0, 16, 56, 48);
    OLED_Update_DMA();
}

void OLED_MPU6050CycleDisplay()
{
    if (oled_BufDisplayOne_flag)
    {
        OLED_ShowFloatNum(54, 16, MPU6050_Data.pitch, 3, 2, OLED_8X16);
        oled_BufDisplayOne_flag=0;
    }
    if (oled_BufDisplayTwo_flag)
    {
        OLED_ShowFloatNum(54, 32, MPU6050_Data.roll, 3, 2, OLED_8X16);
        oled_BufDisplayTwo_flag=0;
    }
    if (oled_BufDisplayThree_flag)
    {
        OLED_ShowFloatNum(54, 48, MPU6050_Data.yaw, 3, 2, OLED_8X16);
        oled_BufDisplayThree_flag=0;
    }
    if (oled_BufDisplay_flag)
    {
        // 局部更新数据区域（宽度71保证不超128）
        OLED_Update_DMA();
        // OLED_UpdateArea(54, 16, 74, 48);
        oled_BufDisplay_flag=0;
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
    #ifdef MOTORA_DEBUG
    Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",pidA_inner.target,pidA_inner.actual,pidA_inner.output,pidA_outer.target,pidA_outer.actual,pidA_outer.output);//串口发送数据
    #endif
    #ifdef MOTORB_DEBUG
    Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",pidB_inner.target,pidB_inner.actual,pidB_inner.output,pidB_outer.target,pidB_outer.actual,pidB_outer.output);//串口发送数据
    #endif

}

void OLED_MPU6050CycleSend()
{
    Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",TargetA,ActualA,OutA,MPU6050_Data.roll,MPU6050_Data.pitch,MPU6050_Data.yaw);//串口发送数据
}

void Serial_change()
{
    //每3秒清除一次
    if(oled_rxClear_flag)
    {
        oled_rxClear_flag=0;
        
    }
    if (!Serial_RxFlag2 && !Serial_RxFlag2) 
    {
        return;  // 没有新数据，直接返回
    }
   

    if (Serial_RxFlag2) 
    {
        char *ptr = Serial_RxPacket2;  // 指向接收缓冲区
        
        // 检查帧头帧尾
        if (ptr[0] == '@') 
        {
            ptr++;  // 跳过 '@'
            

            // 解析 kp
            if (strncmp(ptr, "macd=", 5) == 0) 
            {
                float macd;
                if (sscanf(ptr + 5, "%f", &macd) == 1) 
                {
                    Mechanical_Median = macd;  // 更新 机械中值
                    Serial_Printf(USART2, "OK:MACD updated\r\n");
                }
            }else 
            // 解析 kp
            if (strncmp(ptr, "kp=", 3) == 0) 
            {
                float kp_value;
                if (sscanf(ptr + 3, "%f", &kp_value) == 1) 
                {
                    pidA_outer.kp = kp_value;  // 更新 KP
                    pidB_outer.kp = kp_value;  // 更新 KP
                    Serial_Printf(USART2, "OK:KP updated\r\n");
                }
            }
            // 解析 ki
            else if (strncmp(ptr, "ki=", 3) == 0) 
            {
                float ki_value;
                if (sscanf(ptr + 3, "%f", &ki_value) == 1) 
                {
                    pidA_outer.ki = ki_value;  // 更新 KI
                    pidB_outer.ki = ki_value;  // 更新 KI
                    Serial_Printf(USART2, "OK:KI updated\r\n");
                }
            }
             // 解析 kd
            else if (strncmp(ptr, "kd=", 3) == 0) 
            {
                float kd_value;
                if (sscanf(ptr + 3, "%f", &kd_value) == 1) 
                {
                    pidA_outer.kd = kd_value;  // 更新 KI
                    pidB_outer.kd = kd_value;  // 更新 KI

                    Serial_Printf(USART2, "OK:KD updated\r\n");
                }
            }
            // 解析 target
            else if (strncmp(ptr, "target=", 7) == 0) 
            {
                float target_value;
                if (sscanf(ptr + 7, "%f", &target_value) == 1) 
                {
                    TargetA = target_value;  // 更新目标值
                    TargetB = target_value;  // 更新目标值
                    Serial_Printf(USART2, "OK:Target updated\r\n");
                }
            }
            else 
            {
                Serial_Printf(USART2, "ERROR:Invalid parameter\r\n");
            }
        }
        else 
        {
            Serial_Printf(USART2, "ERROR:Invalid frame format\r\n");
        }
        
        Serial_RxFlag2 = 0;  // 清除接收标志
    }




    if (Serial_RxFlag1) {
        // 将接收到的数据通过串口回显
        if (strcmp(Serial_RxPacket1, "TargetA add") == 0) {
            TargetA += 10;
        }
        else if (strcmp(Serial_RxPacket1, "TargetA lower") == 0) {
            TargetA -= 10;
        }

        else if (strcmp(Serial_RxPacket1, "TargetB add") == 0) {
            TargetB += 10;
        }
        else if (strcmp(Serial_RxPacket1, "TargetB lower") == 0) {
            TargetB -= 10;
        }
        else if (strcmp(Serial_RxPacket1, "MotorA forward") == 0) {
            TargetA += 330*4;
            //TargetA += 100;
        }
        else if (strcmp(Serial_RxPacket1, "MotorA backward") == 0) {
           TargetA += -330*4;
            //TargetA += -100;
        }
        else if (strcmp(Serial_RxPacket1, "page1") == 0) {
            page1_firstEntry = 1;
            page1_flag       = 1;
            page2_flag       = 0;
        }
        else if (strcmp(Serial_RxPacket1, "page2") == 0) {
            page2_firstEntry = 1;
            page1_flag       = 0;
            page2_flag       = 1;
        }
        Serial_RxFlag1 = 0; // 重置接收标志
    }
}




#ifdef SETLOACTION_MODE
void MotorControlLoop_SetLoaction() {
    // 读取实际值（例如编码器）
    pidA.actual += Encoder_TIM3_Get();
    pidB.actual += Encoder_TIM2_Get();

    ActualA=pidA.actual;
    ActualB=pidB.actual;

    // 更新目标值（例如来自遥控器）
    pidA.target = TargetA;
    pidB.target = TargetB;

    //计算PID输出
    PID_SetLocation_Optimization(&pidA);
    PID_SetLocation_Optimization(&pidB);

    // 应用输出到电机
    MotorA_SetSpeed(pidA.output);
    MotorB_SetSpeed(pidB.output);

    OutA=pidA.output;
    OutB=pidB.output;
}
#endif // SETLOACTION_MODE

#ifdef DUALCONTROL_MODE
void MotorControlLoop_Dual() {
    #ifdef INNER_DEBUG
    pidA_outer.target = TargetA;
    pidB_outer.target = TargetB;

    pidA_inner.target =PID_DualLoopControl(&pidA_outer);
    pidB_inner.target =PID_DualLoopControl(&pidB_outer);

    PID_DualLoopControl(&pidA_inner);
    PID_DualLoopControl(&pidB_inner);

    ActualA=pidA_outer.actual;
    ActualB=pidB_outer.actual;
   
    OutA=pidA_outer.output;
    OutB=pidB_outer.output;
    #endif
   
    #ifdef OUTER_DEBUG

    pidA_inner.target = TargetA;
    pidB_inner.target = TargetB;

    PID_DualLoopControl(&pidA_inner);
    PID_DualLoopControl(&pidB_inner);
   

    ActualA=pidA_inner.actual;
    ActualB=pidB_inner.actual;

    OutA=pidA_inner.output;
    OutB=pidB_inner.output;

    #endif

}
#endif //DUALCONTROL_MODE


#ifdef ANGLE_MODE
void MotorControlLoop_Angle() {

    #ifdef INNER_DEBUG
    pidA_inner.target = TargetA;
    pidB_inner.target = TargetB;

    PID_Cascade(&pidA_inner);
    PID_Cascade(&pidB_inner);


    ActualA=pidA_inner.actual;
    ActualB=pidB_inner.actual;

    OutA=pidA_inner.output;
    OutB=pidB_inner.output;
    #endif

    #ifdef OUTER_DEBUG
    pidA_outer.target = Mechanical_Median;
    pidB_outer.target = Mechanical_Median;

    pidA_inner.target =PID_Cascade(&pidA_outer)+TargetA;
    pidB_inner.target =PID_Cascade(&pidB_outer)+TargetB;

    PID_Cascade(&pidA_inner);
    PID_Cascade(&pidB_inner);

    ActualA=pidA_outer.actual;
    ActualB=pidB_outer.actual;
   
    OutA=pidA_outer.output;
    OutB=pidB_outer.output;
    #endif


}
#endif //ANGLE_MODE




int main(void){
   Delay_ms(10);
    Main_Config();
    //20ms看门
    //IWDG_Config(IWDG_Prescaler_16,100);
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
        //串口改变目标值
        Serial_change();
        IWDG_Feed();
        OLED_PIDCycleSend();
        
    }
}


// 使用 static 关键字使 Count 保持其值
void TIM1_UP_IRQHandler(void)
{
    static uint16_t sys_cnt=0;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) 
    {
        sys_cnt++;
        if(sys_cnt % 10 == 0){
			//软件i2c 0.00085
            //硬件i2c(400k) 0.00124
            MPU6050_Get_Angle_Plus(&MPU6050_Data);
            angle_temp=MPU6050_Data.roll;

            #ifdef SETLOACTION_MODE
            MotorControlLoop_SetLoaction();
            #endif // SETLOACTION_MODE

            #ifdef ANGLE_MODE
            // MotorControlLoop_Angle();
            #endif // ANGLE_MODE
        }
        if(sys_cnt % 100 == 0)oled_BufDisplay_flag=1;
        if(sys_cnt % 24 == 0)oled_BufDisplayOne_flag=1;
        if(sys_cnt % 48 == 0)oled_BufDisplayTwo_flag=1;
        if(sys_cnt % 72 == 0)oled_BufDisplayThree_flag=1;
        //测试可行性,1s 闪烁led(pc13),占用RTC口,需要在主程序初始化LED_Init();
        if(sys_cnt % 1000 == 0)
        {
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
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);// 根据您的定时器和情况调整
    }
}



