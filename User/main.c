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

//PID_Init
#define MOTORA_DEBUG
// #define MOTORB_DEBUG

//PID_Init_BicyclicParams
#define SETLOACTION_MODE
// #define DUALCONTROL_MODE


//统一采用电机A调试 俩者不可共存 
#ifdef MOTORA_DEBUG
    #ifdef MOTORB_DEBUG
        #error "MOTORA_DEBUG and MOTORB_DEBUG cannot be defined at the same time!"
    #endif
#endif

//定位置PID  双环PID(外环位置,内环速度)
#ifdef SETLOACTION_MODE
    #ifdef DUALCONTROL_MODE
        #error "MOTORA_DEBUG and MOTORB_DEBUG cannot be defined at the same time!"
    #endif
#endif

#ifdef SETLOACTION_MODE
    PID_Params pidA,pidB;
#endif // SETLOACTION_MODE

#ifdef DUALCONTROL_MODE
    PID_BicyclicParams pidA_inner,pidB_inner;
    PID_BicyclicParams pidA_outer,pidB_outer;
#endif // DUALCONTROL_MODE



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
串口1用来发送PID的值 即Target Actual Out 图形化调试  为PB8与PB9
串口2用来修改PID的值 发送格式为@XXXX 加回车  具体串口会写  为PA3与PA2
*/


//   ------------------主函数逻辑判断------------------
uint8_t page1_flag       = 1;
uint8_t page1_firstEntry = 1;

uint8_t page2_flag       = 0;
uint8_t page2_firstEntry = 1;


//   ------------------MPU6050变量------------------
uint8_t ID;

float AccData[3];
float GyroData[3];
float Temperature;
int16_t AX, AY, AZ, GX, GY, GZ;
uint16_t mpu6050_count =0;


//   ------------------定时器计次位------------------
uint16_t pid_count = 0;
uint16_t oled_rxClear_count=0;
uint8_t oled_rxClear_flag=0;




//   ------------------PID调试变量------------------
 float TargetA, ActualA, OutA;
 float TargetB, ActualB, OutB;




void Main_Config()
{
    Motor_Init();
    Encoder_TIM2_Init();
    Encoder_TIM3_Init();
    MPU6050_Init();
    Serial_Init();
    OLED_Init();
    USART3_DMA_Init();
    MPU6050_DMA_Init();

    
    #ifdef SETLOACTION_MODE
    // 初始化电机A的PID
    PID_Init(&pidA, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    // 初始化电机B的PID
    PID_Init(&pidB, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    #endif // SETLOACTION_MODE
    
    
    #ifdef DUALCONTROL_MODE
    PID_Init_BicyclicParams(MOTOR_A,&pidA_inner,Encoder_TIM3_Get,0.4,0.15,0,-30,30,MotorA_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_A,&pidA_outer,NULL,0.05,0,0.05,-100,100,NULL);

    PID_Init_BicyclicParams(MOTOR_B,&pidB_inner,Encoder_TIM2_Get,0.4,0.1,0,-20,20,MotorB_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_B,&pidB_outer,NULL,0.4,0,0.3,-100,100,NULL);
    #endif // DUALCONTROL_MODE
  
}

void OLED_PIDDisplay()
{
    // 测试数据发送
    OLED_Clear();
    OLED_ShowString(1, 1, "OA:");
    OLED_ShowString(1, 9, "OB:");
    OLED_ShowString(2, 1, "TA:");
    OLED_ShowString(2, 9, "TB:");
    OLED_ShowString(3, 1, "AA:");
    OLED_ShowString(3, 9, "AB:");
}

void OLED_PIDCycleDisplay()
{
    // 主程序循环
    OLED_ShowNum(1, 4, OutA, 4);
    OLED_ShowNum(1, 12, OutB, 4);
    OLED_ShowNum(2, 4, TargetA, 4);
    OLED_ShowNum(2, 12, TargetB, 4);
    OLED_ShowNum(3, 4, ActualA, 4);
    OLED_ShowNum(3, 12, ActualB, 4);
}

void OLED_MPU6050Display()
{
    OLED_Clear();
    OLED_ShowString(1, 1, "AX:");
    OLED_ShowString(1, 9, "AY:");
    OLED_ShowString(2, 1, "AZ:");
    OLED_ShowString(2, 9, "GX:");
    OLED_ShowString(3, 1, "GY:");
    OLED_ShowString(3, 9, "GZ:");

    // 测试程序 i2c是否正确运行
    /*
    OLED_ShowString(1, 1, "ID:");
    ID = MPU6050_GetID();
    OLED_ShowHexNum(1, 4, ID, 2);
    Delay_ms(2000);
    */
}

void OLED_MPU6050CycleDisplay()
{

    // 取消注释这些行来显示加速度计、温度、陀螺仪数据   
    /*  
    OLED_ShowSignedNum(1, 4, (int32_t)(AccData[0] * 100), 4); // X轴加速度  
    OLED_ShowSignedNum(1, 12, (int32_t)(AccData[1] * 100), 4); // Y轴加速度  
    OLED_ShowSignedNum(2, 4, (int32_t)(AccData[2] * 100), 4); // Z轴加速度  
    OLED_ShowSignedNum(4, 7, (int32_t)(Temperature * 100), 4); // 温度  
    OLED_ShowSignedNum(2, 12, (int32_t)(GyroData[0] * 100), 4); // X轴陀螺仪  
    OLED_ShowSignedNum(3, 4, (int32_t)(GyroData[1] * 100), 4); // Y轴陀螺仪  
    OLED_ShowSignedNum(3, 12, (int32_t)(GyroData[2] * 100), 4); // Z轴陀螺仪  
    */  

   // 显示角度，这里假设使用GyroX、GyroY和GyroZ来代表角度  
   OLED_ShowSignedNum(2,12,(uint32_t)(MPU6050_Data.GyroX),4);
   OLED_ShowSignedNum(3,4,(uint32_t)(MPU6050_Data.GyroY),4);
   OLED_ShowSignedNum(3,12,(uint32_t)(MPU6050_Data.GyroZ),4);
}

// 使用OLED测试串口是否正常使用 最好发2位 不然OLED挤不下一行
void OLED_SerialDisplay()
{
    Serial_Printf(USART2, "串口2初始化成功！\n");
    Serial_Printf(USART3, "串口3初始化成功！\n");

    // 测试程序
    /*
    OLED_ShowString(4,1,"TX1");
    OLED_ShowString(4,5,"TX2");
    OLED_ShowString(4,10,"TX3");
    if(Serial_RxFlag1)
    {
         // 将接收到的数据通过串口回显
         OLED_ShowString(4,3," ");
         OLED_ShowString(4,3,Serial_RxPacket1);
         Serial_RxFlag1 = 0;  // 重置接收标志
    }
    if(Serial_RxFlag2)
    {
         // 将接收到的数据通过串口回显
         OLED_ShowString(4,8,"  ");
         OLED_ShowString(4,8,Serial_RxPacket2);
         Serial_RxFlag2 = 0;  // 重置接收标志
    }

   if(Serial_RxFlag3)
    {
         // 将接收到的数据通过串口回显
         OLED_ShowString(4,13," ");
         OLED_ShowString(4,13,Serial_RxPacket3);
         Serial_RxFlag3 = 0;  // 重置接收标志
    }
    */
}

void OLED_SerialCycleDisplay()
{
    static char buffer[32];

    #ifdef MOTORA_DEBUG
    int len =snprintf(buffer,sizeof(buffer),"%.2f,%.2f,%.2f\r\n", TargetA, ActualA, OutA);
    #endif // MotorA_Debug
    
    #ifdef MOTORB_DEBUG
    int len =snprintf(buffer,sizeof(buffer),"%.2f,%.2f,%.2f\r\n", TargetA, ActualA, OutA);
    #endif // MotorB_Debug
  
    // 确保长度不超过缓冲区大小  
    if (len >= sizeof(buffer)) {  
        // 如果数据太长，可以做一些处理  
        // 例如：截断 or 报错  
        len = sizeof(buffer) - 1; // 截断并保留结尾符  
    }  
    
    // 发送格式化数据  
    USART3_DMA_Send((uint8_t *)buffer, len);  
}

void Serial_change()
{
    //每3秒清除一次
    if(oled_rxClear_flag)
    {
        oled_rxClear_flag=0;
        OLED_ClearLine(4);  // 清除第4行
    }
    if (!Serial_RxFlag2) {
        return; // 没有新数据时直接返回
    }
    if (Serial_RxFlag2) {
        // 将接收到的数据通过串口回显
        OLED_ClearLine(4);
        OLED_ShowString(4, 1, Serial_RxPacket2);

        if (strcmp(Serial_RxPacket2, "TargetA add") == 0) {
            TargetA += 10;
        }
        else if (strcmp(Serial_RxPacket2, "TargetA lower") == 0) {
            TargetA -= 10;
        }

        else if (strcmp(Serial_RxPacket2, "TargetB add") == 0) {
            TargetB += 10;
        }
        else if (strcmp(Serial_RxPacket2, "TargetB lower") == 0) {
            TargetB -= 10;
        }
        else if (strcmp(Serial_RxPacket2, "MotorA forward") == 0) {
            TargetA += 330*4;
            //TargetA += 100;
        }
        else if (strcmp(Serial_RxPacket2, "MotorA backward") == 0) {
           TargetA += -330*4;
            //TargetA += -100;
        }
        else if (strcmp(Serial_RxPacket2, "page1") == 0) {
            page1_firstEntry = 1;
            page1_flag       = 1;
            page2_flag       = 0;
        }
        else if (strcmp(Serial_RxPacket2, "page2") == 0) {
            page2_firstEntry = 1;
            page1_flag       = 0;
            page2_flag       = 1;
        }
        Serial_RxFlag2 = 0; // 重置接收标志
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
   
/*
   pidA_inner.target = TargetA;
   pidB_inner.target = TargetB;

   PID_DualLoopControl(&pidA_inner);
   PID_DualLoopControl(&pidB_inner);
   

   ActualA=pidA_inner.actual;
   ActualB=pidB_inner.actual;

    OutA=pidA_inner.output;
    OutB=pidB_inner.output;

*/
}
#endif //DUALCONTROL_MODE



int main(void){
    Delay_ms(100);
    Main_Config();
    while (1) {
        if (page1_flag) {
            if (page1_firstEntry) {
                OLED_PIDDisplay();
                OLED_SerialDisplay();
                page1_firstEntry = 0;
            } else {
                OLED_PIDCycleDisplay();
                OLED_SerialCycleDisplay();
            }
        } else if (page2_flag) {
            if (page2_firstEntry) {
                OLED_MPU6050Display();
                page2_firstEntry = 0;
            } else {
               OLED_MPU6050CycleDisplay();
            }
        }


        switch (MPU6050_State) {
            case MPU_READ_REQUESTED:
                MPU6050_DMA_Read();
                MPU6050_State = MPU_DMA_READING; // 进入DMA等待状态
                break;
        
            case MPU_DATA_READY:
                MPU6050_CalculateAngle(&MPU6050_Data);
                // 执行控制逻辑（如PID）
                MPU6050_State = MPU_IDLE; // 处理完成后回到空闲
                break;
        
            case MPU_DMA_READING: // 无需操作，等待DMA中断
            case MPU_IDLE:        // 无操作
            default:
                break;
        }

        // 串口改变目标值
        Serial_change();
    }
}


// 使用 static 关键字使 Count 保持其值
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) {
        if(++mpu6050_count == 3)//3ms
        {
            mpu6050_count=0;
            // 仅触发读取请求，不阻塞中断
            // if (MPU6050_State == MPU_IDLE) {
            //     MPU6050_State = MPU_READ_REQUESTED;
    
            // }
            MPU6050_ReadReg(&MPU6050_Data);
            MPU6050_CalculateAngle(&MPU6050_Data);
        }
        if (++pid_count == 30)//30ms
        {

        pid_count = 0; // 重置计数器，以便下次计算
        #ifdef SETLOACTION_MODE
        MotorControlLoop_SetLoaction();
        #endif // SETLOACTION_MODE

        #ifdef DUALCONTROL_MODE
        MotorControlLoop_Dual();
        #endif // DUALCONTROL_MODE
        }
        if(++oled_rxClear_count==3000)
        {
            oled_rxClear_count=0;
            oled_rxClear_flag=1;
        }
        // 确保清除中断标志，避免重复触发
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 根据您的定时器和情况调整
    }
}


