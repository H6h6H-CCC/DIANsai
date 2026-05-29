#include "balance_control.h"
#include "angle_sensor.h"
#include "encoder.h"
#include "moter.h"
#include "pid.h"

#define BALANCE_DT_S          0.005f
#define BALANCE_PWM_LIMIT     1000.0f
#define BALANCE_I_LIMIT       100.0f
#define BALANCE_MIN_ANGLE     151.5f
#define BALANCE_MAX_ANGLE     179.5f
#define BALANCE_SPEED_ANGLE_LIMIT 1.3f
#define BALANCE_SPEED_I_LIMIT 30.0f

static PID_t s_angle_pid;
static PID_t s_speed_pid;
static float s_target_angle = 165.0f;
static float s_target_speed = 0.0f;
static uint8_t s_enable = 1U;
static volatile int16_t s_last_pwm = 0;

static float Balance_AngleError(float target, float measure)
{
    float error = target - measure;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

void Balance_Init(void)
{
    PID_Init(&s_angle_pid, 235.0f, 0.7f, 11.0f, BALANCE_PWM_LIMIT, BALANCE_I_LIMIT);
    PID_Init(&s_speed_pid, 0.00115f, 0.0f, 0.0f, BALANCE_SPEED_ANGLE_LIMIT, BALANCE_SPEED_I_LIMIT);
    s_target_angle = 163.8f;
    s_target_speed = 0.0f;
    s_enable = 1U;
}

void Balance_Update10ms(void)
{
    float angle;
    float target_angle;
    float speed;
    float angle_offset;
    float error;
    float abs_error;
    int16_t pwm;

    if ((s_enable == 0U) || (AngleSensor_IsReady() == 0U)) {
        Moter_A(0);
        Moter_B(0);
        s_last_pwm = 0;
        PID_Reset(&s_angle_pid);
        PID_Reset(&s_speed_pid);
        return;
    }

    angle = AngleSensor_GetAngle();
    speed = ((float)Encoder3_GetLastDelta() + (float)Encoder4_GetLastDelta()) * 0.5f;
    angle_offset = PID_Update(&s_speed_pid, s_target_speed, speed, BALANCE_DT_S);
    target_angle = s_target_angle + angle_offset;
    error = Balance_AngleError(target_angle, angle);
    abs_error = (error < 0.0f) ? -error : error;

    if ((angle < BALANCE_MIN_ANGLE) || (angle > BALANCE_MAX_ANGLE)) {
        Moter_A(0);
        Moter_B(0);
        s_last_pwm = 0;
        PID_Reset(&s_angle_pid);
        PID_Reset(&s_speed_pid);
        return;
    }

    pwm = -(int16_t)PID_Update(&s_angle_pid, 0.0f, -error, BALANCE_DT_S);

    Moter_A(pwm);
    Moter_B(pwm);
    s_last_pwm = pwm;
}

void Balance_SetTargetAngle(float angle_deg)
{
    s_target_angle = angle_deg;
}

void Balance_SetPid(float kp, float ki, float kd)
{
    PID_Init(&s_angle_pid, kp, ki, kd, BALANCE_PWM_LIMIT, BALANCE_I_LIMIT);
}

void Balance_Enable(uint8_t enable)
{
    s_enable = enable;
    if (enable == 0U) {
        Moter_A(0);
        Moter_B(0);
        s_last_pwm = 0;
        PID_Reset(&s_angle_pid);
        PID_Reset(&s_speed_pid);
    }
}

int16_t Balance_GetLastPwm(void)
{
    return s_last_pwm;
}
