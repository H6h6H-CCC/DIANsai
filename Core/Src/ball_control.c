#include "ball_control.h"

#include "bsp_servo.h"
#include "pid.h"

#define BALL_INNER_PERIOD_MS       10U
#define BALL_OUTER_PERIOD_MS       50U
#define BALL_OUTER_DT_S            0.050f
#define BALL_INNER_DT_S            0.010f
#define BALL_POSITION_TIMEOUT_MS   150U
#define BALL_ANGLE_TIMEOUT_MS      50U

#define BALL_SERVO_MIN_US          800U
#define BALL_SERVO_MAX_US          2300U
#define BALL_SERVO_CENTER_US       1730U

#define BALL_MAX_TARGET_ANGLE_DEG  5.0f
#define BALL_MAX_SERVO_DELTA_US    1230.0f

/* 初始参数偏保守，现场按位置环再角度环的顺序调节。 */
#define BALL_POSITION_KP           0.40f
#define BALL_POSITION_KI           0.00f
#define BALL_POSITION_KD           0.05f
#define BALL_ANGLE_KP              60.0f
#define BALL_ANGLE_KI              0.0f
#define BALL_ANGLE_KD              2.0f

static PID_t position_pid;
static PID_t angle_pid;

static volatile float position_cm;
static volatile float pipe_angle_deg;
static volatile float pipe_gyro_dps;
static volatile uint32_t position_time_ms;
static volatile uint32_t angle_time_ms;
static volatile uint8_t position_pending;
static volatile uint8_t position_valid;
static volatile uint8_t angle_valid;

static float target_position_cm;
static float target_angle_deg;
static uint32_t last_outer_time_ms;
static uint32_t last_inner_time_ms;
static uint16_t servo_center_us;
static uint16_t servo_pulse_us;
static uint8_t control_enabled;

static uint16_t BallControl_ClampPulse(float pulse_us)
{
    if (pulse_us > (float)BALL_SERVO_MAX_US) return BALL_SERVO_MAX_US;
    if (pulse_us < (float)BALL_SERVO_MIN_US) return BALL_SERVO_MIN_US;
    return (uint16_t)(pulse_us + 0.5f);
}

static void BallControl_OutputCenter(void)
{
    servo_pulse_us = servo_center_us;
    BSP_ServoSetPulse(BSP_SERVO_1, servo_pulse_us);
}

void BallControl_Init(void)
{
    PID_Init(&position_pid,
             BALL_POSITION_KP, BALL_POSITION_KI, BALL_POSITION_KD,
             20.0f, BALL_MAX_TARGET_ANGLE_DEG);
    PID_Init(&angle_pid,
             BALL_ANGLE_KP, BALL_ANGLE_KI, BALL_ANGLE_KD,
             500.0f, BALL_MAX_SERVO_DELTA_US);

    target_position_cm = 0.0f;
    target_angle_deg = 0.0f;
    servo_center_us = BALL_SERVO_CENTER_US;
    position_pending = 0U;
    position_valid = 0U;
    angle_valid = 0U;
    control_enabled = 0U;
    last_outer_time_ms = 0U;
    last_inner_time_ms = 0U;
    BallControl_OutputCenter();
}

void BallControl_SetEnabled(uint8_t enabled)
{
    control_enabled = (enabled != 0U);
    PID_Reset(&position_pid);
    PID_Reset(&angle_pid);
    target_angle_deg = 0.0f;

    if (control_enabled == 0U)
    {
        BallControl_OutputCenter();
    }
}

void BallControl_SetTargetPosition(float target_cm)
{
    target_position_cm = target_cm;
    PID_Reset(&position_pid);
}

void BallControl_SetServoCenter(uint16_t center_us)
{
    if (center_us < BALL_SERVO_MIN_US) center_us = BALL_SERVO_MIN_US;
    if (center_us > BALL_SERVO_MAX_US) center_us = BALL_SERVO_MAX_US;
    servo_center_us = center_us;
    BallControl_OutputCenter();
}

void BallControl_SetPositionPid(float kp, float ki, float kd)
{
    PID_SetGains(&position_pid, kp, ki, kd);
    PID_Reset(&position_pid);
}

void BallControl_SetAnglePid(float kp, float ki, float kd)
{
    PID_SetGains(&angle_pid, kp, ki, kd);
    PID_Reset(&angle_pid);
}

void BallControl_SetPosition(float new_position_cm, uint32_t timestamp_ms)
{
    position_cm = new_position_cm;
    position_time_ms = timestamp_ms;
    position_valid = 1U;
    position_pending = 1U;
}

void BallControl_InvalidatePosition(void)
{
    position_valid = 0U;
    position_pending = 0U;
}

void BallControl_SetPipeAngle(float angle_deg,
                              float gyro_dps,
                              uint32_t timestamp_ms)
{
    pipe_angle_deg = angle_deg;
    pipe_gyro_dps = gyro_dps;
    angle_time_ms = timestamp_ms;
    angle_valid = 1U;
}

void BallControl_Process(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (control_enabled == 0U)
    {
        return;
    }

    if ((position_valid != 0U) &&
        ((uint32_t)(now_ms - position_time_ms) <= BALL_POSITION_TIMEOUT_MS) &&
        (position_pending != 0U) &&
        ((last_outer_time_ms == 0U) ||
         ((uint32_t)(now_ms - last_outer_time_ms) >= BALL_OUTER_PERIOD_MS)))
    {
        last_outer_time_ms = now_ms;
        position_pending = 0U;
        /* 实车方向：P>0 时输出负目标角度，使钢球回到 P=0。 */
        target_angle_deg = PID_Update(&position_pid,
                                      target_position_cm - position_cm,
                                      BALL_OUTER_DT_S);
    }
    else if ((position_valid == 0U) ||
             ((uint32_t)(now_ms - position_time_ms) > BALL_POSITION_TIMEOUT_MS))
    {
        target_angle_deg = 0.0f;
        PID_Reset(&position_pid);
    }

    elapsed_ms = (uint32_t)(now_ms - last_inner_time_ms);
    if (elapsed_ms < BALL_INNER_PERIOD_MS)
    {
        return;
    }
    last_inner_time_ms = now_ms;

    if ((angle_valid == 0U) ||
        ((uint32_t)(now_ms - angle_time_ms) > BALL_ANGLE_TIMEOUT_MS))
    {
        PID_Reset(&angle_pid);
        BallControl_OutputCenter();
        return;
    }

    /* 目标角度在两次外环更新间保持不变，因此误差变化率取 -陀螺仪角速度。 */
    /* 正roll表示车尾抬高；舵机脉宽减小时车尾抬高，因此减去PID输出。 */
    servo_pulse_us = BallControl_ClampPulse(
        (float)servo_center_us
        - PID_UpdateWithRate(&angle_pid,
                             target_angle_deg - pipe_angle_deg,
                             -pipe_gyro_dps,
                             BALL_INNER_DT_S));
    BSP_ServoSetPulse(BSP_SERVO_1, servo_pulse_us);
}

float BallControl_GetTargetAngle(void)
{
    return target_angle_deg;
}

uint16_t BallControl_GetServoPulse(void)
{
    return servo_pulse_us;
}
