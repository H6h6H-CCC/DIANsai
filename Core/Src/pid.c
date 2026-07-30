#include "pid.h"

static float PID_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    PID_Reset(pid);
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float PID_Update(PID_t *pid, float error, float dt_s)
{
    float error_rate = 0.0f;

    if ((pid->has_last_error != 0U) && (dt_s > 0.0f))
    {
        error_rate = (error - pid->last_error) / dt_s;
    }

    return PID_UpdateWithRate(pid, error, error_rate, dt_s);
}

float PID_UpdateWithRate(PID_t *pid, float error, float error_rate, float dt_s)
{
    float output;

    if (dt_s > 0.0f)
    {
        pid->integral += error * dt_s;
    }
    pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
    pid->last_error = error;
    pid->has_last_error = 1U;

    output = pid->kp * error
           + pid->ki * pid->integral
           + pid->kd * error_rate;
    return PID_Clamp(output, pid->output_limit);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->has_last_error = 0U;
}
