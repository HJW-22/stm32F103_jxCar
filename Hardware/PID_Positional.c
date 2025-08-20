#include "stm32f10x.h" // Device header
#include <string.h>
#include "PID_Positional.h"
#include "Motor.h"
#include <math.h> //绝对值浮点数

int16_t motorA_speed;
int16_t motorB_speed;


//限幅函数
float constrain(float value, float min, float max) 
{
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}


void PID_Init(PID_Params *pid ,float kp,float ki,float kd,PIDVersion_I i_mode,PIDVersion_D d_mode ,float alpha)  
{
    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;
    pid->i_version=i_mode;
    pid->d_version=d_mode;
    pid->alpha=alpha;

    pid->target=0;
    pid->actual=0;
    pid->output=0;
    
    memset(&pid->state,0,sizeof(pid->state));
}


void PID_Init_BicyclicParams( uint8_t name,PID_BicyclicParams *pid,  float (*GetPWM)(void), float kp, float ki, float kd,  float outMin,float outMax,void (*SetPWM)(int16_t output))
{  

    pid->name=name;
    // 设置函数指针  
    pid->GetPWM = GetPWM; // 初始化获取 PWM 的函数  
    pid->SetPWM = SetPWM; // 初始化设置 PWM 的函数 
    // 初始化内环参数  
    pid->kp = kp;  
    pid->ki = ki;  
    pid->kd = kd;   
    
    pid->outMax=outMax;
    pid->outMin=outMin;

    // 初始化目标值、实际值和输出  
    pid->target = 0;   
    pid->actual = 0;  
    pid->output = 0;  

    // 清零状态  
    memset(&pid->state, 0, sizeof(pid->state));  
}  

// 定位置式位置PID优化版本(积分分离^变速积分^无(积分限幅))&&输出偏移&&输入死区&&(微分先行^不完全微分^全部^无)
void PID_SetLocation_Optimization(PID_Params *pid)
{
     /* 复制参数
    pid->kp;
    pid->ki;
    pid->output;
    pid->i_version;
    pid->d_version;
    pid->kd;
    pid->alpha;
    pid->target;
    pid->actual; 
    pid->output; 
    pid->state.error[0];
    pid->state.error[1];
    pid->state.integral;
    pid->state.last_actual;
    pid->state.dif_filter;
    */
    pid->state.error[1] =pid->state.error[0];
    pid->state.error[0] =pid->target - pid->actual;

    //一切需要调控的都已经参数化
    if(fabs(pid->state.error[0])<DEADZONE_THRESHOLD){pid->output=0; return;}else{
    switch (pid->i_version) {  
    /*变速积分的说明
    变速积分总体来说是积分分离的升级板,除了满足积分分离的作用,另一个作用是为了防止在用手快速转动电机时,
    那一瞬间的误差TargetB - ActualB假如是100,但是由于设置的阈值不对,稳态误差无法消除,那么ki的作用就没有用了,
    所以需要创建一个函数,这个函数要在误差小的时候ki大,误差越大积分越来越弱*/
        case PID_VERSION_VARIABLE_INTEGRAL:  // 变速积分版本 
        {  
            float C1;
            C1=1/(1*fabs(pid->state.error[0])+1);
            pid->state.integral+=C1*pid->state.error[0];
            break; 
        }  
    /*积分分离的说明
    积分分离的作用主要体现在当Target在瞬间调控非常大的,误差ErrorIntA += Error0A和KiA*ErrorIntA都是非常大的,
    但是ki的作用是消除稳态误差,但是现在的情况跟kp一样是线性的,就算将kp给设置为0,电机的速度因为ki的设置,一样是可以运行的
    这样性质就变了,ki的本质是消除稳态误差而不是带动电机,所以将 TargetB - ActualB的值先判断
    是否小于某值(即电机内部的摩擦力或者是电池供电又或者是外部的摩擦力导致的稳态误差(比例与外内部消耗持平)),
    大于某值(即pwm的设置太小的电机带动不了导致积分累加,直到电机驱动)*/
        case PID_VERSION_INTEGRAL_SEPARATION:// 积分分离版本
        {
            if (fabs(pid->state.error[0])<ERROR0_LIMIT_THRESHOLDS_HIGH) {
                pid->state.integral += pid->state.error[0];
            } else {
                pid->state.integral = 0;
            }
            
            break; 
        }
        case PID_VERSION_INTEGRAL_NON:       // 不使用(有积分限幅)
        {
            pid->state.integral += pid->state.error[0];
            if(pid->state.integral>ERRORINT_LIMIT_THRESHOLDS){pid->state.integral=ERRORINT_LIMIT_THRESHOLDS;}
            if(pid->state.integral<-ERRORINT_LIMIT_THRESHOLDS){pid->state.integral=-ERRORINT_LIMIT_THRESHOLDS;}
            break; 
        }
    }
    switch (pid->d_version) {  
        case PID_VERSION_DIFFERENTIAL_FIRST:
        {
             pid->state.dif_filter=-pid->kd * ( pid->actual - pid->state.last_actual);
            pid->output = pid->kp * pid->state.error[0] + pid->ki * pid->state.integral+ pid->state.dif_filter;
            break; 
        }
        case PID_VERSION_DIFFERENTIAL_INCOMPLETE:
        {
             pid->state.dif_filter = (1-pid->alpha)*pid->kd*(pid->state.error[0]-pid->state.error[1])+pid->alpha* pid->state.dif_filter;
            pid->output = pid->kp * pid->state.error[0] + pid->ki * pid->state.integral+ pid->state.dif_filter;
            break; 
        }
        case PID_VERSION_DIFFERENTIAL_FIRST_AND_INCOMPLETE:
        {
            float raw_differential = -pid->kd * (pid->actual - pid->state.last_actual);
            pid->state.last_actual = pid->actual;
            pid->state.dif_filter = (1-pid->alpha)*raw_differential + pid->alpha*pid->state.dif_filter;
            pid->output = pid->kp * pid->state.error[0] + pid->ki * pid->state.integral + pid->state.dif_filter;
            break; 
        }
        case PID_VERSION_DIFFERENTIAL_NON:
        {
            pid->output = pid->kp * pid->state.error[0] + pid->ki * pid->state.integral + pid->kd * (pid->state.error[0] - pid->state.error[1]);
            break; 
        }
    }
    }    
    pid->output += (pid->output > 0) ? OUTPUT_OFFSET 
                                    : -OUTPUT_OFFSET;

    // 输出限幅
    if (pid->output > 100) { pid->output = 100; }
    if (pid->output < -100) { pid->output = -100; }

}


// 双环控制函数(没有如何优化,只有限幅)
int16_t PID_DualLoopControl(PID_BicyclicParams *pid) 
{
    // 获取实际值
    if(pid->GetPWM != NULL) {
        pid->actual = pid->GetPWM();
        if(pid->name == MOTOR_A){
        motorA_speed +=pid->actual;
        }else
        {
            motorB_speed +=pid->actual; 
        }
    } else {
        // 外环使用全局位置变量
        pid->actual = (pid->name == MOTOR_A) ? motorA_speed : motorB_speed;
    }

    // PID计算
    pid->state.error[1] = pid->state.error[0];
    pid->state.error[0] = pid->target - pid->actual;
    
    // 积分项处理
    if (pid->ki != 0) {
        pid->state.integral += pid->state.error[0];
    }
    
    // PID计算
    pid->output = pid->kp * pid->state.error[0] 
                + pid->ki * pid->state.integral 
                + pid->kd * (pid->state.error[0] - pid->state.error[1]);
    
    // 输出限幅
    pid->output = constrain(pid->output, pid->outMin, pid->outMax);

    // 应用输出
    if(pid->SetPWM != NULL) {
        pid->SetPWM(pid->output);
    }
    
    return pid->output;
}


// 双环PID控制函数（角度环+速度环）
float PID_Cascade(PID_AngleParam *pid) {
    // 1. 获取实际值（区分内外环）
    float actual_value;
    float actual_test;
    if (pid->is_inner_loop) {
        // 内环（速度环）：从编码器获取实际速度 
        actual_test = pid->GetCounter();  // 编码器反馈的速度值（RPM或脉冲/秒）
        pid->actual += actual_test;
    } else {
        // 外环（角度环）：从IMU获取实际角度
        actual_value = pid->GetAngle();  // IMU反馈的角度值（度或弧度）
        pid->actual = actual_value;
    }
    

    // 2. 计算误差
    pid->state.error[1] = pid->state.error[0];
    pid->state.error[0] = pid->target - pid->actual;


    // 3. 积分项处理（抗饱和）
    if (pid->ki != 0) {
                // 在PID计算前，检查误差是否值得积分
    if (fabsf(pid->state.error[0]) > 0.1f) {  // 死区阈值
        pid->state.integral += pid->state.error[0];
    }
        // 积分限幅（限制在输出范围的20%）
      // 改为动态比例限制（如输出范围的50%）
    float max_integral = 0.5f * (pid->outMax - pid->outMin) / (pid->ki + 1e-6f);  // +1e-6避免除零
    pid->state.integral = constrain(pid->state.integral, -max_integral, max_integral);



    }

    // 4. 微分项滤波（一阶低通）
    float raw_derivative = pid->state.error[0] - pid->state.error[1];
    pid->state.derivative = 0.3 * raw_derivative + 0.7 * pid->state.derivative;

    // 5. PID计算
    pid->output = pid->kp * pid->state.error[0] 
                + pid->ki * pid->state.integral 
                + pid->kd * pid->state.derivative;

    // // 6. 死区处理（避免电机抖动）
    // if (fabs(pid->state.error[0]) < pid->dead_zone) {
    //     pid->output = 0;
    // }

    // 7. 输出限幅
    pid->output = constrain(pid->output, pid->outMin, pid->outMax);

    // 8. 应用输出（仅内环直接控制电机）
    if (pid->is_inner_loop && pid->SetPWM != NULL) {
        pid->SetPWM(pid->output);
    }
    
    return pid->output;
}






void PID_Init_Angle( uint8_t name,uint16_t is_inner_loop, PID_AngleParam *pid, float (*GetAngle)(void),int16_t (*GetCounter)(void) ,float kp, float ki, float kd,  float outMin,float outMax,void (*SetPWM)(int16_t output))
{  
    pid->is_inner_loop=is_inner_loop;
    pid->name=name;
    // 设置函数指针  
    pid->GetAngle = GetAngle; // 初始化获取 PWM 的函数  
    pid->GetCounter = GetCounter; // 初始化设置 PWM 的函数 
    pid->SetPWM = SetPWM; // 初始化设置 PWM 的函数 
    // 初始化内环参数  
    pid->kp = kp;  
    pid->ki = ki;  
    pid->kd = kd;   
    
    pid->outMax=outMax;
    pid->outMin=outMin;

    // 初始化目标值、实际值和输出  
    pid->target = 0;   
    pid->actual = 0;  
    pid->output = 0;  

    // 清零状态  
    memset(&pid->state, 0, sizeof(pid->state));  
}  

void PID_Angle_Clear(PID_AngleParam *pid)
{
    // 初始化目标值、实际值和输出  
    pid->target = 0;   
    pid->actual = 0;  
    pid->output = 0;  
    motorA_speed=0;
    motorB_speed=0;
    // 清零状态  
    memset(&pid->state, 0, sizeof(pid->state));  
}







