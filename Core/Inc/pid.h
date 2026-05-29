#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float out_limit;
    float integral_limit;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float integral_limit);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float target, float measure, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
