#include "pid.h"
#include <string.h>

static float PID_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Reset(PID_t *pid)
{
    if (pid == 0) return;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_out   = 0.0f;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float max_out)
{
    if (pid == 0) return;
    memset(pid, 0, sizeof(*pid));
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->max_integral = (max_i >= 0.0f) ? max_i : -max_i;
    pid->max_out      = (max_out >= 0.0f) ? max_out : -max_out;
}

float PID_Update(PID_t *pid, float target, float actual)
{
    float error;
    float derivative;
    float output;

    if (pid == 0) return 0.0f;
    error           = target - actual;
    pid->integral   = PID_Clamp(pid->integral + error, pid->max_integral);
    derivative      = error - pid->prev_error;
    output          = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prev_error = error;
    pid->last_out   = PID_Clamp(output, pid->max_out);
    return pid->last_out;
}
