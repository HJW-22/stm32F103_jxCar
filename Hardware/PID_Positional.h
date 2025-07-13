#ifndef __PID_Positional_H__
#define __PID_Positional_H__

#include "stm32f10x.h"                  // Device header
//死区阈值
#define DEADZONE_THRESHOLD 20

//输出补偿
#define OUTPUT_OFFSET 0

//积分的最大值 
#define ERRORINT_LIMIT_THRESHOLDS 500  

// 积分分离阈值较大的阈值  
#define ERROR0_LIMIT_THRESHOLDS_HIGH 50  

#define MOTOR_A    1
#define MOTOR_B    2


//------------------  PID定位置式位置选择版本  --------------------


// 定义 PID_I 优化版本枚举类型  主要用于积分累加,过冲
typedef enum {  
    PID_VERSION_INTEGRAL_SEPARATION, // 积分分离版本  
    PID_VERSION_VARIABLE_INTEGRAL,  // 变速积分版本  
    PID_VERSION_INTEGRAL_NON   
} PIDVersion_I;  

// 定义 PID_D 优化版本枚举类型  解决纯微分环节对高频噪声敏感的问题
typedef  enum {  
    PID_VERSION_DIFFERENTIAL_FIRST,   //微分先行 
    PID_VERSION_DIFFERENTIAL_INCOMPLETE,  //不完全微分
    PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE,  //全部  
    PID_VERSION_DIFFERENTIAL_NON
} PIDVersion_D; 


// PID配置结构体（推荐）
typedef struct {
    float kp;
    float ki;
    float kd;
    PIDVersion_I i_version;      // 积分版本选择
    PIDVersion_D d_version;      // 微分版本选择
    //不完全的微分变量的取值
    //difOut = (1-a)*Kd*(Actual-Actual1)+a*difOut;
    //difOut = (1-a)*Kd*(Error0-Error1)+a*difOut;
    float alpha;              // 不完全微分滤波系数 (0~1)
    // 输入输出接口
    float target;             // 目标值（可外部写入）
    float actual;             // 实际值（需外部更新）
    float output;             // 输出值（计算结果）
    // 内部状态（无需外部操作）
    struct {
        float error[2];       // 当前和上一次误差 [0]当前 [1]上一次
        float integral;       // 积分项
        float last_actual;    // 上一次实际值（用于微分先行）
        float dif_filter;     // 微分滤波缓存（用于不完全微分）
    } state;
} PID_Params;

// PID双环控制(内环速度外环位置)配置结构体（推荐）
typedef struct {
    uint8_t name;
    //内环参数
    float kp;
    float ki;
    float kd;
    
    float target;
    float actual;
    float output;

    float outMin;
    float outMax;
    // 内部状态（无需外部操作）
    struct {
        float error[2];       // 当前和上一次误差 [0]当前 [1]上一次
        float integral;       // 积分项
        float speed;
    } state;
    void (*SetPWM)(int16_t output);// PWM 输出函数指针  
    int16_t (*GetPWM)(void);
    
} PID_BicyclicParams;


// PID双环控制(内环角度外环位置)配置结构体（推荐）
typedef struct {
    uint8_t name;
    //内环参数
    float kp;
    float ki;
    float kd;
    
    float target;
    float actual;
    float output;

    float outMin;
    float outMax;
    // 内部状态（无需外部操作）
    struct {
        float error[2];       // 当前和上一次误差 [0]当前 [1]上一次
        float integral;       // 积分项
        float speed;
    } state;
    void (*SetPWM)(int16_t output);// PWM 输出函数指针  
    int16_t (*GetPWM)(void);
    
} PID_AngleParam;






void PID_Init(PID_Params *pid ,float kp,float ki,float kd,PIDVersion_I i_mode,PIDVersion_D d_mode ,float alpha);
void PID_SetLocation_Optimization(PID_Params *pid);
void PID_Init_Angle( uint8_t name,PID_AngleParam *pid, int16_t (*GetPWM)(void), float kp, float ki, float kd,  float outMin,float outMax,void (*SetPWM)(int16_t output));
int16_t PID_Angle(PID_AngleParam *pid);


void PID_Init_BicyclicParams(
    uint8_t name,
    PID_BicyclicParams *pid, 
    int16_t (*GetPWM)(void),
    float kp, float ki, float kd, 
    float outMin,float outMax,
    void (*SetPWM)(int16_t output)
    );  
int16_t PID_DualLoopControl(PID_BicyclicParams *pid);



void PID_Angle_Clear(PID_AngleParam *pid);

#endif // !_PID_POSITIONAL__H
		
		
		
		
		
		