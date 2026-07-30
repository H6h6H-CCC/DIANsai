#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float integral_limit;
    float output_limit;
    uint8_t has_last_error;
} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit);
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);
float PID_Update(PID_t *pid, float error, float dt_s);
float PID_UpdateWithRate(PID_t *pid, float error, float error_rate, float dt_s);
void PID_Reset(PID_t *pid);

#endif /* PID_H */
