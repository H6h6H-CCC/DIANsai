#include "balance_control.h"
#include "angle_sensor.h"
#include "encoder.h"
#include "moter.h"
#include "pid.h"

#define BALANCE_DT_S          0.005f
#define BALANCE_PWM_LIMIT     1000.0f
#define BALANCE_I_LIMIT       100.0f
#define BALANCE_MIN_ANGLE     149.5f
#define BALANCE_MAX_ANGLE     179.0f
#define BALANCE_SPEED_ANGLE_LIMIT 4.0f
#define BALANCE_SPEED_I_LIMIT 30.0f
#define BALANCE_POS_SPEED_LIMIT 8.0f
#define BALANCE_POS_I_LIMIT   200.0f

static PID_t s_angle_pid;
static PID_t s_speed_pid;
static PID_t s_position_pid;
static float s_target_angle = 163.7f;
static float s_target_speed = 0.0f;
static float s_target_position = 0.0f;
static uint8_t s_enable = 1U;
static uint8_t s_hold_position_enable = 1U;
static volatile int16_t s_last_pwm = 0;

static float Balance_AngleError(float target, float measure)
{
    float error = target - measure;

    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return error;
}

static float Balance_GetPosition(void)
{
    return ((float)Encoder3_GetTotal() + (float)Encoder4_GetTotal()) * 0.5f;
}

void Balance_Init(void)
{
    PID_Init(&s_angle_pid, 250.0f, 1.0f, 11.0f, BALANCE_PWM_LIMIT, BALANCE_I_LIMIT);
    PID_Init(&s_speed_pid, 0.0011f, 0.000001f, 0.000000001f, BALANCE_SPEED_ANGLE_LIMIT, BALANCE_SPEED_I_LIMIT);
    PID_Init(&s_position_pid, 0.00001f, 0.0f, 0.0f, BALANCE_POS_SPEED_LIMIT, BALANCE_POS_I_LIMIT);
    s_target_angle = 163.7f;
    s_target_speed = 0.0f;
    s_target_position = Balance_GetPosition();
    s_hold_position_enable = 1U;
    s_enable = 1U;
}

void Balance_Update10ms(void)
{
    float angle;
    float target_angle;
    float speed;
    float position;
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
        PID_Reset(&s_position_pid);
        return;
    }

    angle = AngleSensor_GetAngle();
    position = Balance_GetPosition();
    speed = ((float)Encoder3_GetLastDelta() + (float)Encoder4_GetLastDelta()) * 0.5f;
    if (s_hold_position_enable != 0U) {
        s_target_speed = PID_Update(&s_position_pid, s_target_position, position, BALANCE_DT_S);
    } else {
        s_target_speed = 0.0f;
    }
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
        PID_Reset(&s_position_pid);
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

void Balance_HoldPositionEnable(uint8_t enable)
{
    s_hold_position_enable = enable;
    s_target_speed = 0.0f;
    PID_Reset(&s_position_pid);
    PID_Reset(&s_speed_pid);
    if (enable != 0U) {
        s_target_position = Balance_GetPosition();
    }
}

void Balance_ResetHoldPosition(void)
{
    s_target_position = Balance_GetPosition();
    s_target_speed = 0.0f;
    PID_Reset(&s_position_pid);
    PID_Reset(&s_speed_pid);
}

uint8_t Balance_IsHoldPositionEnabled(void)
{
    return s_hold_position_enable;
}

float Balance_GetTargetAngle(void)
{
    return s_target_angle;
}

float Balance_GetMinAngle(void)
{
    return BALANCE_MIN_ANGLE;
}

float Balance_GetMaxAngle(void)
{
    return BALANCE_MAX_ANGLE;
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
        PID_Reset(&s_position_pid);
    } else {
        Balance_ResetHoldPosition();
    }
}

int16_t Balance_GetLastPwm(void)
{
    return s_last_pwm;
}
