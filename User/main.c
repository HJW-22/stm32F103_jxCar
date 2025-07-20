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


#define MOTORA_DEBUG
// #define MOTORB_DEBUG


//PID_Init
//PID_Init_BicyclicParams
//PID_Init_Angle

// #define SETLOACTION_MODE
//#define DUALCONTROL_MODE
#define ANGLE_MODE


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
uint16_t oled_rxClear_count=0;
uint8_t oled_rxClear_flag=0;


uint16_t oled_BufClear_count=0;
uint8_t oled_BufClear_flag=0;


uint16_t oled_BufDisplay_count=0;
uint8_t oled_BufDisplay_flag=0;

uint16_t oled_BufDisplayOne_count=0;
uint8_t oled_BufDisplayOne_flag=0;

uint16_t oled_BufDisplayThree_count=0;
uint8_t oled_BufDisplayThree_flag=0;

uint16_t oled_BufDisplayTwo_count=0;
uint8_t oled_BufDisplayTwo_flag=0;


float angle_temp=0;


//   ------------------PID调试变量------------------
 float TargetA, ActualA, OutA;
 float TargetB, ActualB, OutB;

 uint8_t fallDown_flag=0;
 uint8_t angle_errorFlag=0;
 uint8_t pid_timingFlag=0;

int16_t Angle_Get()
{
return ((-angle_temp));
}

void Main_Config()
{
    Serial_Init(); 
    Motor_Init();
    Encoder_TIM4_Init();
    Encoder_TIM3_Init();
    MPU6050_Init();
    OLED_Init();
    USART2_DMA_Init();
    Timer_Init();
//    MPU6050_DMA_Init();

    #ifdef SETLOACTION_MODE
    // 初始化电机A的PID
    PID_Init(&pidA, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    // 初始化电机B的PID
    PID_Init(&pidB, 0.07,0.02,0.1,PID_VERSION_VARIABLE_INTEGRAL,PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,0.9);
    #endif // SETLOACTION_MODE
    
    #ifdef DUALCONTROL_MODE
    PID_Init_BicyclicParams(MOTOR_A,&pidA_inner,Encoder_TIM3_Get,0.4,0.15,0,-30,30,MotorA_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_A,&pidA_outer,NULL,0.05,0,0.05,-100,100,NULL);

    PID_Init_BicyclicParams(MOTOR_B,&pidB_inner,Encoder_TIM4_Get,0.4,0.1,0,-20,20,MotorB_SetSpeed);
    PID_Init_BicyclicParams(MOTOR_B,&pidB_outer,NULL,0.4,0,0.3,-100,100,NULL);
    #endif // DUALCONTROL_MODE

    #ifdef ANGLE_MODE
    PID_Init_Angle(MOTOR_A,&pidA_inner,Angle_Get,3.5,0.01,1,-100,100,MotorA_SetSpeed);
    // PID_Init_Angle(MOTOR_A,&pidA_outer,NULL,0.1,0,20,-100,100,NULL);

    PID_Init_Angle(MOTOR_B,&pidB_inner,Angle_Get,3.5,0.01,1,-100,100,MotorB_SetSpeed);
    // PID_Init_Angle(MOTOR_B,&pidB_outer,NULL,0.1,0,20,-100,100,NULL);
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
    OLED_Update();
}

void OLED_PIDCycleDisplay()
{
    //此函数需要10ms 要缩短到3ms以内
    //100ms显示1次
    // if (oled_BufClear_flag)
    // {
    //     OLED_ClearArea(40,16,40,48);
    //     OLED_ClearArea(104,16,24,48);
    //     oled_BufClear_flag=0;
    // }

     if (oled_BufDisplayOne_flag)
    {
        OLED_ShowSignedNum(40, 16, ActualA, 4,OLED_8X16);
        OLED_ShowSignedNum(40, 32, TargetA, 4,OLED_8X16);
        oled_BufDisplayOne_flag=0;
    }
     if (oled_BufDisplayTwo_flag)
    {
        OLED_ShowSignedNum(40, 48, OutA, 4,OLED_8X16);
        OLED_ShowUnsignedFloatNum(104, 16, pidA_inner.kp, 1,1,OLED_8X16);
        oled_BufDisplayTwo_flag=0;
    }
     if (oled_BufDisplayThree_flag)
    {
        OLED_ShowUnsignedFloatNum(104, 32, pidA_inner.ki, 1,1,OLED_8X16);
        OLED_ShowUnsignedFloatNum(104, 48, pidA_inner.kd, 1,1,OLED_8X16);
        oled_BufDisplayThree_flag=0;
    }
    if (oled_BufDisplay_flag)
    {
        OLED_UpdateArea(40,16,40,48);
        OLED_UpdateArea(104,16,24,48);
        oled_BufDisplay_flag=0;
    }
   
  
}

void OLED_MPU6050Display()
{
    OLED_Clear();
    OLED_ShowString(1, 1,  "MPU6050姿态角", OLED_8X16);
    OLED_ShowString(1, 16, "俯仰角:", OLED_8X16);
    OLED_ShowString(1, 32, "翻滚角:", OLED_8X16);
    OLED_ShowString(1, 48, "偏航角:", OLED_8X16);
    OLED_Update();
}

void OLED_MPU6050CycleDisplay()
{
    // 清空数值区域（原X:56~127 → 现57~128，但最大127所以调整为57~127）
    OLED_ClearArea(57, 16, 71, 48);
    
    // 显示浮点数（格式：+xxx.xx，保留2位小数）
    OLED_ShowFloatNum(57, 16, MPU6050_Angle.X_Angle, 3, 2, OLED_8X16);
    OLED_ShowFloatNum(57, 32, MPU6050_Angle.Y_Angle, 3, 2, OLED_8X16);
    OLED_ShowFloatNum(57, 48, MPU6050_Angle.Z_Angle, 3, 2, OLED_8X16);
    
    // 局部更新数据区域（宽度71保证不超128）
    OLED_UpdateArea(57, 16, 71, 48);
}

// 使用OLED测试串口是否正常使用 最好发2位 不然OLED挤不下一行
void OLED_SerialDisplay()
{

    #ifdef USART1_FLAG
    Serial_Printf(USART1, "串口1初始化成功！\n");
    #endif // USART1_FLAG

    #ifdef USART2_FLAG
    Serial_Printf(USART2, "串口2初始化成功！\n");
    #endif // USART2_FLAG

    #ifdef USART3_FLAG
    Serial_Printf(USART3, "串口3初始化成功！\n");
    #endif // USART3_FLAG

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
    USART2_DMA_Send((uint8_t *)buffer, len);  
}

void Serial_change()
{
    //每3秒清除一次
    if(oled_rxClear_flag)
    {
        oled_rxClear_flag=0;
        
    }
    if (!Serial_RxFlag1) {
        return; // 没有新数据时直接返回
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


#ifdef ANGLE_MODE
void MotorControlLoop_Angle() {
    // pidA_outer.target = TargetA;
    // pidB_outer.target = TargetB;

    // pidA_inner.target =PID_Angle(&pidA_outer);
    // pidB_inner.target =PID_Angle(&pidB_outer);

    // PID_Angle(&pidA_inner);
    // PID_Angle(&pidB_inner);

    // ActualA=pidA_outer.actual;
    // ActualB=pidB_outer.actual;
   
    // OutA=pidA_outer.output;
    // OutB=pidB_outer.output;
   

    pidA_inner.target = TargetA;
    pidB_inner.target = TargetB;

    PID_Angle(&pidA_inner);
    PID_Angle(&pidB_inner);


    ActualA=pidA_inner.actual;
    ActualB=pidB_inner.actual;

    OutA=pidA_inner.output;
    OutB=pidB_inner.output;


}
#endif //ANGLE_MODE




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

        if (mpu6050_timingFlag)
        {
            MPU6050_ReadReg(&MPU6050_Data);
            MPU6050_CalculateAngle(&MPU6050_Data);
            angle_temp=MPU6050_Angle.Y_Angle;
            if (angle_temp < -50 || angle_temp > 50) // 角度异常
            {
                MPU6050_Angle.Y_Angle=0;
                MotorA_SetSpeed(0);
                MotorB_SetSpeed(0);
                PID_Angle_Clear(&pidA_inner);
                PID_Angle_Clear(&pidB_inner);
                PID_Angle_Clear(&pidA_outer);
                PID_Angle_Clear(&pidA_outer); 
            }
            mpu6050_timingFlag=0;
        }
        if (pid_timingFlag)
        {
            #ifdef SETLOACTION_MODE
            MotorControlLoop_SetLoaction();
            #endif // SETLOACTION_MODE

            #ifdef DUALCONTROL_MODE
            MotorControlLoop_Dual();
            #endif // DUALCONTROL_MODE

            #ifdef ANGLE_MODE
            MotorControlLoop_Angle();
            #endif // ANGLE_MODE
            pid_timingFlag=0;
        }

        

        // switch (MPU6050_State) {
        //     case MPU_READ_REQUESTED:
        //         MPU6050_DMA_Read();
        //         MPU6050_State = MPU_DMA_READING; // 进入DMA等待状态
        //         break;
        
        //     case MPU_DATA_READY:
        //         MPU6050_CalculateAngle(&MPU6050_Data);
        //         // 执行控制逻辑（如PID）
        //         MPU6050_State = MPU_IDLE; // 处理完成后回到空闲
        //         break;
        
        //     case MPU_DMA_READING: // 无需操作，等待DMA中断
        //     case MPU_IDLE:        // 无操作
        //     default:
        //         break;
        // }

        // 串口改变目标值
        Serial_change();
    }
}


// 使用 static 关键字使 Count 保持其值
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) 
    {
        if(++mpu6050_count == 3)//3ms
        {
            mpu6050_count=0;
            mpu6050_timingFlag=1;
            // 仅触发读取请求，不阻塞中断
            // if (MPU6050_State == MPU_IDLE) {
            //     MPU6050_State = MPU_READ_REQUESTED;
            // }
        }
        if(++pid_count == 5)//5ms
        {
            pid_count = 0; // 重置计数器，以便下次计算
            pid_timingFlag=1;
        }
        if(++oled_rxClear_count==3000)//3000ms
        {
            oled_rxClear_count=0;
            oled_rxClear_flag=1;
        }
        if(++oled_BufClear_count==50)//50ms
        {
            oled_BufClear_count=0;
            oled_BufClear_flag=1;
        }
        if(++oled_BufDisplay_count==20)//100ms
        {
            oled_BufDisplay_count=0;
            oled_BufDisplay_flag=1;
        }
        if(++oled_BufDisplayOne_count==6)//57ms
        {
            oled_BufDisplayOne_count=0;
            oled_BufDisplayOne_flag=1;
        }
        if(++oled_BufDisplayTwo_count==12)//67ms
        {
            oled_BufDisplayTwo_count=0;
            oled_BufDisplayTwo_flag=1;
        }
        if(++oled_BufDisplayThree_count==18)//77ms
        {
            oled_BufDisplayThree_count=0;
            oled_BufDisplayThree_flag=1;
        }
        // 确保清除中断标志，避免重复触发
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);// 根据您的定时器和情况调整
    }
}
/*
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
*/


