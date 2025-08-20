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







//电机A调试还是电机B调试(仅串口2发送无oled)
#define MOTORA_DEBUG
// #define MOTORB_DEBUG

//双环pid选项内环还是外环调试(用于调参串口2)

// #define INNER_DEBUG        //内部
#define OUTER_DEBUG     //外部




//PID_Init
//PID_Init_BicyclicParams
//PID_Init_Angle

// #define SETLOACTION_MODE
// #define DUALCONTROL_MODE
#define ANGLE_MODE

// void sendATCommand(char* cmd, uint16_t timeout) {
//   // 发送命令
//   Serial_Printf(USART2, "%s\r\n", cmd);  // 确保AT命令以\r\n结尾
  
//   while (!Serial_RxFlag2) {
//     if ((timeout--)==0) {
//       Serial_Printf(USART1, "CMD %s timeout!\r\n", cmd);
//       return;
//     }
//   }
//     // 打印接收到的响应
//     Serial_Printf(USART1, "[DEBUG] CMD: %s\r\nResponse: %s\r\n", cmd, Serial_RxPacket2);
    
//     // 解析响应判断是否成功
//     if (strstr(Serial_RxPacket2, "OK") || strstr(Serial_RxPacket2, "CONNECT")) {
//         Serial_Printf(USART1, "[INFO] Command success\r\n");
//     } else {
//         Serial_Printf(USART1, "[WARN] Command may have failed\r\n");
//     }
// }
// void WIFI_Init() {
//   sendATCommand("AT", 10000);      // 设置为STA模式
//   sendATCommand("AT+CWMODE=1", 10000);      // 设置为STA模式
//   sendATCommand("AT+CWJAP=\"Xiaomi_D351\",\"Hjwa3b9c\"", 50000);  // 连接WiFi
//   sendATCommand("AT+CIPSTART=\"TCP\",\"192.168.32.32\",8080", 20000);  // 连接服务器
// }




/* 128x64 数字符号为8x16 中文为16x16
--------------OLED显示页面一--------------
    电机X     
    实际:+0000 P:xx.x
    目标:+0000 I:xx.x
    输出:+0000 D:xx.x
--------------OLED显示页面二--------------
    MPU6050姿态角
    俯仰角:+xxx.xx
    翻滚角:+xxx.xx
    偏航角:+xxx.xx
--------------OLED显示页面三--------------


*/

//统一采用电机A调试 俩者不可共存 
#ifdef MOTORA_DEBUG
    #ifdef MOTORB_DEBUG
        #error "MOTORA_DEBUG and MOTORB_DEBUG cannot be defined at the same time!"
    #endif
#endif


#ifdef INNER_DEBUG
    #ifdef OUTER_DEBUG
        #error "INNER_DEBUG and OUTER_DEBUG cannot be defined at the same time!"
    #endif
#endif


#if (defined(SETLOACTION_MODE) + defined(DUALCONTROL_MODE) + defined(ANGLE_MODE)) > 1
    #error "Only one of SETLOACTION_MODE, DUALCONTROL_MODE, or ANGLE_MODE can be defined!"
#endif

#ifdef SETLOACTION_MODE
    PID_Params pidA,pidB;
#endif // SETLOACTION_MODE

#ifdef DUALCONTROL_MODE
    PID_BicyclicParams pidA_inner,pidB_inner;
    PID_BicyclicParams pidA_outer,pidB_outer;
#endif // DUALCONTROL_MODE

#ifdef ANGLE_MODE
    PID_AngleParam pidA_inner,pidB_inner;
    PID_AngleParam pidA_outer,pidB_outer;
#endif


/*typec方向左B2TIM2 右A1TIM3
电机的插线反了 即PWMA代表的是电机B
即PWMB代表的是电机A
下面的互换就可以
MotorB
A8青色 PWMA
GPIO_Pin_13 AIN1
GPIO_Pin_12 AIN2
MotorA
A11橙色 PWMB
GPIO_Pin_15 BIN1
GPIO_Pin_14 BIN2
*/

/*串口说明 串口1发送 串口2修改
串口2用来发送PID的值 即Target Actual Out 图形化调试  为PB8与PB9
串口1用来修改PID的值 发送格式为@XXXX 加回车  具体串口会写  为PA3与PA2
*/


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



void LED_Init()
{
    PWR_BackupAccessCmd(ENABLE);//允许修改RTC 和后备寄存器

	RCC_LSEConfig(RCC_LSE_OFF);//关闭外部低速外部时钟信号功能 后，PC13 PC14 PC15 才可以当普通IO用。

	BKP_TamperPinCmd(DISABLE);//关闭入侵检测功能，也就是 PC13，也可以当普通IO 使用

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO_InitTypeDef GPIO_InitStart;
    GPIO_InitStart.GPIO_Pin=GPIO_Pin_13;
    GPIO_InitStart.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStart.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStart);
}


void Main_Config()
{
    ESP8266_INIT_ERROR ret= ESP8266_INIT_EOK ;
    OLED_Init();
    OLED_DMA_Init();
    OLED_ShowString( (128-96)/2, (64-16)/2, "系统初始化中", OLED_8X16);
    // OLED_Update();
    OLED_Update_DMA();
    Delay_ms(800);
    Serial_Init(); 
    
    ret = ESP8266_Init(115200);
    if (ret !=ESP8266_INIT_EOK)
    {
        ESP8266_ERROR_Handling(ret);
    }
    
    
    Motor_Init();
    Encoder_TIM4_Init();
    Encoder_TIM3_Init();
    // USART2_DMA_Init();
    //LED_Init();

    MPU6050_Init(GPIOB,GPIO_Pin_10,GPIO_Pin_11);
    RP_Init();
    Timer_Init();


    #ifdef SETLOACTION_MODE
    // 初始化电机A的PID
    PID_Init(&pidA, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    // 初始化电机B的PID
    PID_Init(&pidB, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    #endif // SETLOACTION_MODE
    
    #ifdef DUALCONTROL_MODE

   
    #ifdef INNER_DEBUG
    PID_Init_BicyclicParams(MOTOR_A,&pidA_inner,Encoder_TIM3_Get,0.4,0.15,0,-30,30,MotorA_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_B,&pidB_inner,Encoder_TIM4_Get,0.4,0.1,0,-20,20,MotorB_SetSpeed);
    #endif // INNER_DEBUG

    //一般来说如果单环的参数不可以使用到双环内
    #ifdef OUTER_DEBUG
    PID_Init_BicyclicParams(MOTOR_A,&pidA_inner,Encoder_TIM3_Get,0.4,0.15,0,-30,30,MotorA_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_B,&pidB_inner,Encoder_TIM4_Get,0.4,0.1,0,-20,20,MotorB_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_A,&pidA_outer,NULL,0.05,0,0.05,-100,100,NULL);
    PID_Init_BicyclicParams(MOTOR_B,&pidB_outer,NULL,0.4,0,0.3,-100,100,NULL);
    #endif // OUTER_DEBUG

    #endif // DUALCONTROL_MODE

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
    OLED_ShowString(0, 0,  "调试电机A",OLED_8X16);
#endif 
#ifdef MOTORB_DEBUG
    OLED_ShowString(0, 0,  "调试电机B",OLED_8X16);
#endif
	OLED_ShowString(0, 16, "实际:",OLED_8X16);
	OLED_ShowString(0, 32, "目标:",OLED_8X16);
	OLED_ShowString(0, 48, "输出:",OLED_8X16);

    OLED_ShowString(88, 16, "P:",OLED_8X16);
    OLED_ShowString(88, 32, "I:",OLED_8X16);
    OLED_ShowString(88, 48, "D:",OLED_8X16);
    OLED_Update_DMA();
    // OLED_Update();
}

void OLED_PIDCycleDisplay()
{
     if (oled_BufDisplayOne_flag)
    {
        OLED_ShowSignedNum(40, 16, ActualA, 4,OLED_8X16);
        OLED_ShowSignedNum(40, 32, TargetA, 4,OLED_8X16);
        oled_BufDisplayOne_flag=0;
    }
     if (oled_BufDisplayTwo_flag)
    {
        OLED_ShowSignedNum(40, 48, OutA, 4,OLED_8X16);
        OLED_ShowUnsignedFloatNum(104, 16, pidA_outer.kp, 1,1,OLED_8X16);
        oled_BufDisplayTwo_flag=0;
    }
     if (oled_BufDisplayThree_flag)
    {
        OLED_ShowUnsignedFloatNum(104, 32, pidA_outer.ki, 1,1,OLED_8X16);
        OLED_ShowUnsignedFloatNum(104, 48, pidA_outer.kd, 1,1,OLED_8X16);
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
//    static char buffer[32];

//    #ifdef MOTORA_DEBUG
//    int len =snprintf(buffer,sizeof(buffer),"%.2f,%.2f,%.2f\r\n", TargetA, ActualA, OutA);
//    #endif // MotorA_Debug
//    
//    #ifdef MOTORB_DEBUG
//    int len =snprintf(buffer,sizeof(buffer),"%.2f,%.2f,%.2f\r\n", TargetA, ActualA, OutA);
//    #endif // MotorB_Debug
//  
//    // 确保长度不超过缓冲区大小  
//    if (len >= sizeof(buffer)) {  
//        // 如果数据太长，可以做一些处理  
//        // 例如：截断 or 报错  
//        len = sizeof(buffer) - 1; // 截断并保留结尾符  
//    }  
//    
//    // 发送格式化数据  
//    // USART2_DMA_Send((uint8_t *)buffer, len);  
    // Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",pidA_inner.target,pidA_inner.actual,pidA_inner.output,pidA_outer.target,pidA_outer.actual,pidA_outer.output);//串口发送数据
    Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",pidB_inner.target,pidB_inner.actual,pidB_inner.output,pidB_outer.target,pidB_outer.actual,pidB_outer.output);//串口发送数据

    // Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",TargetA,ActualA,OutA,MPU6050_Data.roll,MPU6050_Data.pitch,MPU6050_Data.yaw);//串口发送数据

}
//0.0016
void OLED_MPU6050CycleSend()
{
    // Serial_Printf(USART2,"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",TargetA,ActualA,OutA,MPU6050_Data.roll,MPU6050_Data.pitch,MPU6050_Data.yaw);//串口发送数据
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
               
            }
        }

        // pidA_inner.kp =RP_Getvalue(1)/4095.0*1;
        // pidA_inner.ki =RP_Getvalue(2)/4095.0*1;
        // pidA_inner.kp =RP_Getvalue(3)/4095.0*10;
        // TargetA=RP_Getvalue(4)/4095.0* 1000-100;

        

        if (pid_timingFlag)
        {
            // OLED_MPU6050CycleSend();
            pid_timingFlag=0;
        }
		// Serial_SendByte('s',USART2);
		// Serial_Printf(USART2,"AT\r\n");
        // Serial_SendByte(0x55,USART2);
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
        if (sys_cnt==0);
        sys_cnt++;
        if(sys_cnt % 10 == 0){

			//软件i2c 0.00085
            //硬件i2c(400k) 0.00124
            MPU6050_Get_Angle_Plus(&MPU6050_Data);
            angle_temp=MPU6050_Data.roll;
            #ifdef SETLOACTION_MODE
            MotorControlLoop_SetLoaction();
            #endif // SETLOACTION_MODE

            #ifdef DUALCONTROL_MODE
            MotorControlLoop_Dual();
            #endif // DUALCONTROL_MODE

            #ifdef ANGLE_MODE
            MotorControlLoop_Angle();
            #endif // ANGLE_MODE
            pid_timingFlag=1;
        }
        if(sys_cnt % 100 == 0)oled_BufDisplay_flag=1;
        if(sys_cnt % 24 == 0)oled_BufDisplayOne_flag=1;
        if(sys_cnt % 48 == 0)oled_BufDisplayTwo_flag=1;
        if(sys_cnt % 72 == 0)oled_BufDisplayThree_flag=1;
        //测试可行性,1s 闪烁led(pc13)
        if(sys_cnt % 1000 == 0)
        {
            // if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET )
            // {
            //     GPIO_WriteBit(GPIOC, GPIO_Pin_13 ,Bit_RESET); // 翻转 PA0
            // }else
            // {
            //     GPIO_WriteBit(GPIOC, GPIO_Pin_13 ,Bit_SET); // 翻转 PA0
            // } 
        
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);// 根据您的定时器和情况调整
    }
}



