#ifndef PID_H
#define PID_H

/* Pure positional PID controller: no hardware callbacks or I/O. */
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_integral;
    float max_out;
    float integral;
    float prev_error;
    float last_out;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float max_out);
float PID_Update(PID_t *pid, float target, float actual);
void PID_Reset(PID_t *pid);

#endif
