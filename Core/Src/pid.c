#include "pid.h"

static float PID_Clamp(float value, float limit)
{
    if (limit <= 0.0f) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_limit = out_limit;
    pid->integral_limit = integral_limit;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float PID_Update(PID_t *pid, float target, float measure, float dt_s)
{
    float error = target - measure;
    float derivative = 0.0f;
    float output;

    if (dt_s > 0.0f) {
        pid->integral += error * dt_s;
        pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
        derivative = (error - pid->last_error) / dt_s;
    }

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->last_error = error;

    return PID_Clamp(output, pid->out_limit);
}
