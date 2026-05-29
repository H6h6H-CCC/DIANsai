#include "balance_control.h"
#include "angle_sensor.h"
#include "moter.h"
#include "pid.h"

#define BALANCE_DT_S          0.01f
#define BALANCE_PWM_LIMIT     700.0f
#define BALANCE_I_LIMIT       100.0f
#define BALANCE_SAFE_ANGLE    50.0f
#define BALANCE_KP_BOOST_ANGLE 30.0f

static PID_t s_angle_pid;
static float s_target_angle = 165.0f;
static uint8_t s_enable = 1U;

static float Balance_AngleError(float target, float measure)
{
    float error = target - measure;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

void Balance_Init(void)
{
    PID_Init(&s_angle_pid, 1000.0f, 0.0f, 0.8f, BALANCE_PWM_LIMIT, BALANCE_I_LIMIT);
    s_target_angle = 165.0f;
    s_enable = 1U;
}

void Balance_Update10ms(void)
{
    float angle;
    float error;
    float abs_error;
    float old_kp;
    int16_t pwm;

    if ((s_enable == 0U) || (AngleSensor_IsReady() == 0U)) {
        Moter_A(0);
        Moter_B(0);
        PID_Reset(&s_angle_pid);
        return;
    }

    angle = AngleSensor_GetAngle();
    error = Balance_AngleError(s_target_angle, angle);
    abs_error = (error < 0.0f) ? -error : error;

    if (abs_error > BALANCE_SAFE_ANGLE) {
        Moter_A(0);
        Moter_B(0);
        PID_Reset(&s_angle_pid);
        return;
    }

    old_kp = s_angle_pid.kp;
    if (abs_error > BALANCE_KP_BOOST_ANGLE) {
        s_angle_pid.kp = old_kp * 2.0f;
    }

    pwm = -(int16_t)PID_Update(&s_angle_pid, 0.0f, -error, BALANCE_DT_S);
    s_angle_pid.kp = old_kp;

    Moter_A(pwm);
    Moter_B(pwm);
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
        PID_Reset(&s_angle_pid);
    }
}
