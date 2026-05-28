#include "rolllll.h"
#include "Emm_V5.h"
#include "shijue.h"
#include "tim.h"
#include <stdlib.h>
#include "doji.h"

#define ROLL_DEADBAND_PX 6.0f
#define ROLL_DEADBAND_P9_PX 6.0f
#define ROLL_CENTER_COORD 160.0f
#define ROLL_EDGE_COORD   280.0f
#define ROLL_SAFE_HALF_SIZE 130.0f
#define ROLL_CMD_LIMIT 500
#define ROLL_CH1_GLOBAL_BIAS 30L
#define ROLL_TARGET9_INSET 8.0f
#define ROLL_TARGET137_INSET 6.0f
#define ROLL_TARGET9_X_OUTPUT_BIAS 100L
#define ROLL_TARGET9_Y_OUTPUT_BIAS (-100L)
#define ROLL_TARGET1_X_OUTPUT_BIAS 100L
#define ROLL_TARGET1_Y_OUTPUT_BIAS (-30L)

static RollCascadePid_t g_roll_ctrl_x;
static RollCascadePid_t g_roll_ctrl_y;
static float g_base_pos_kp = 0.0f;
static float g_base_pos_ki = 0.0f;
static float g_base_pos_kd = 0.0f;
static float g_base_vel_kp = 0.0f;
static float g_base_vel_ki = 0.0f;
static float g_base_vel_kd = 0.0f;
static float g_base_ang_kp = 0.0f;
static float g_base_ang_ki = 0.0f;
static float g_base_ang_kd = 0.0f;
static float g_default_pos_kp = 0.0f;
static float g_default_pos_ki = 0.0f;
static float g_default_pos_kd = 0.0f;
static float g_default_pos_out_limit = 0.0f;
static float g_default_pos_integral_limit = 0.0f;
static float g_default_vel_kp = 0.0f;
static float g_default_vel_ki = 0.0f;
static float g_default_vel_kd = 0.0f;
static float g_default_vel_out_limit = 0.0f;
static float g_default_vel_integral_limit = 0.0f;
static float g_default_ang_kp = 0.0f;
static float g_default_ang_ki = 0.0f;
static float g_default_ang_kd = 0.0f;
static float g_default_ang_out_limit = 0.0f;
static float g_default_ang_integral_limit = 0.0f;
static float g_corner_pos_kp = 0.0f;
static float g_corner_pos_ki = 0.0f;
static float g_corner_pos_kd = 0.0f;
static float g_corner_pos_out_limit = 0.0f;
static float g_corner_pos_integral_limit = 0.0f;
static float g_corner_vel_kp = 0.0f;
static float g_corner_vel_ki = 0.0f;
static float g_corner_vel_kd = 0.0f;
static float g_corner_vel_out_limit = 0.0f;
static float g_corner_vel_integral_limit = 0.0f;
static float g_corner_ang_kp = 0.0f;
static float g_corner_ang_ki = 0.0f;
static float g_corner_ang_kd = 0.0f;
static float g_corner_ang_out_limit = 0.0f;
static float g_corner_ang_integral_limit = 0.0f;
static uint8_t g_pid_profile_mode = 0U; /* 0=default,1=center,2=corner */
float g_roll_target_x = 0.0f;
float g_roll_target_y = 0.0f;
static const uint16_t k_roll_pair_vel = 500;
static const uint8_t k_roll_pair_acc = 50;
static float g_target_vel_x = 0.0f;
static float g_target_vel_y = 0.0f;
static float g_target_angle_x = 0.0f;
static float g_target_angle_y = 0.0f;
static int32_t g_abs_cmd_x = 0;
static int32_t g_abs_cmd_y = 0;
static uint16_t g_last_servo_pos_x = 1380U;
static uint16_t g_last_servo_pos_y = 1400U;

static uint8_t roll_target_matches_point(uint8_t point)
{
    float cx;
    float cy;
    if ((point < 1U) || (point > 9U))
    {
        return 0U;
    }
    cx = (float)g_shijue_centers[point - 1U].x;
    cy = (float)g_shijue_centers[point - 1U].y;
    if (point == 1U)
    {
        cx += ROLL_TARGET137_INSET;
        cy += ROLL_TARGET137_INSET;
    }
    else if (point == 3U)
    {
        cx -= ROLL_TARGET137_INSET;
        cy += ROLL_TARGET137_INSET;
    }
    else if (point == 7U)
    {
        cx += ROLL_TARGET137_INSET;
        cy -= ROLL_TARGET137_INSET;
    }
    else if (point == 9U)
    {
        cx += ROLL_TARGET9_INSET;
        cy += ROLL_TARGET9_INSET;
    }
    if ((g_roll_target_x > (cx - 0.5f)) && (g_roll_target_x < (cx + 0.5f)) &&
        (g_roll_target_y > (cy - 0.5f)) && (g_roll_target_y < (cy + 0.5f)))
    {
        return 1U;
    }
    return 0U;
}

static uint8_t roll_is_center_target(void)
{
    return roll_target_matches_point(5U);
}

static uint8_t roll_is_corner_target(void)
{
    if (roll_target_matches_point(1U) || roll_target_matches_point(3U) ||
        roll_target_matches_point(7U) || roll_target_matches_point(9U))
    {
        return 1U;
    }
    return 0U;
}

static void roll_apply_center_pid_profile(void)
{
    /* 主人指定：目标为中心点时启用这套参数 */
    g_base_pos_kp = 0.0f;
    g_base_pos_ki = 0.0f;
    g_base_pos_kd = 0.0f;
    g_roll_ctrl_x.pos_pid.out_limit = 80.0f;
    g_roll_ctrl_y.pos_pid.out_limit = 80.0f;
    g_roll_ctrl_x.pos_pid.integral_limit = 40.0f;
    g_roll_ctrl_y.pos_pid.integral_limit = 40.0f;

    g_base_vel_kp = 0.0f;
    g_base_vel_ki = 0.0f;
    g_base_vel_kd = 0.0f;
    g_roll_ctrl_x.vel_pid.out_limit = 2500.0f;
    g_roll_ctrl_y.vel_pid.out_limit = 2500.0f;
    g_roll_ctrl_x.vel_pid.integral_limit = 120.0f;
    g_roll_ctrl_y.vel_pid.integral_limit = 120.0f;

    g_base_ang_kp = 0.0f;
    g_base_ang_ki = 0.0f;
    g_base_ang_kd = 0.0f;
    g_roll_ctrl_x.angle_pid.out_limit = 1000.0f;
    g_roll_ctrl_y.angle_pid.out_limit = 1000.0f;
    g_roll_ctrl_x.angle_pid.integral_limit = 200.0f;
    g_roll_ctrl_y.angle_pid.integral_limit = 200.0f;
}

static void roll_apply_corner_pid_profile(void)
{
    g_base_pos_kp = g_corner_pos_kp;
    g_base_pos_ki = g_corner_pos_ki;
    g_base_pos_kd = g_corner_pos_kd;
    g_roll_ctrl_x.pos_pid.out_limit = g_corner_pos_out_limit;
    g_roll_ctrl_y.pos_pid.out_limit = g_corner_pos_out_limit;
    g_roll_ctrl_x.pos_pid.integral_limit = g_corner_pos_integral_limit;
    g_roll_ctrl_y.pos_pid.integral_limit = g_corner_pos_integral_limit;

    g_base_vel_kp = g_corner_vel_kp;
    g_base_vel_ki = g_corner_vel_ki;
    g_base_vel_kd = g_corner_vel_kd;
    g_roll_ctrl_x.vel_pid.out_limit = g_corner_vel_out_limit;
    g_roll_ctrl_y.vel_pid.out_limit = g_corner_vel_out_limit;
    g_roll_ctrl_x.vel_pid.integral_limit = g_corner_vel_integral_limit;
    g_roll_ctrl_y.vel_pid.integral_limit = g_corner_vel_integral_limit;

    g_base_ang_kp = g_corner_ang_kp;
    g_base_ang_ki = g_corner_ang_ki;
    g_base_ang_kd = g_corner_ang_kd;
    g_roll_ctrl_x.angle_pid.out_limit = g_corner_ang_out_limit;
    g_roll_ctrl_y.angle_pid.out_limit = g_corner_ang_out_limit;
    g_roll_ctrl_x.angle_pid.integral_limit = g_corner_ang_integral_limit;
    g_roll_ctrl_y.angle_pid.integral_limit = g_corner_ang_integral_limit;
}

static void roll_restore_default_pid_profile(void)
{
    g_base_pos_kp = g_default_pos_kp;
    g_base_pos_ki = g_default_pos_ki;
    g_base_pos_kd = g_default_pos_kd;
    g_roll_ctrl_x.pos_pid.out_limit = g_default_pos_out_limit;
    g_roll_ctrl_y.pos_pid.out_limit = g_default_pos_out_limit;
    g_roll_ctrl_x.pos_pid.integral_limit = g_default_pos_integral_limit;
    g_roll_ctrl_y.pos_pid.integral_limit = g_default_pos_integral_limit;

    g_base_vel_kp = g_default_vel_kp;
    g_base_vel_ki = g_default_vel_ki;
    g_base_vel_kd = g_default_vel_kd;
    g_roll_ctrl_x.vel_pid.out_limit = g_default_vel_out_limit;
    g_roll_ctrl_y.vel_pid.out_limit = g_default_vel_out_limit;
    g_roll_ctrl_x.vel_pid.integral_limit = g_default_vel_integral_limit;
    g_roll_ctrl_y.vel_pid.integral_limit = g_default_vel_integral_limit;

    g_base_ang_kp = g_default_ang_kp;
    g_base_ang_ki = g_default_ang_ki;
    g_base_ang_kd = g_default_ang_kd;
    g_roll_ctrl_x.angle_pid.out_limit = g_default_ang_out_limit;
    g_roll_ctrl_y.angle_pid.out_limit = g_default_ang_out_limit;
    g_roll_ctrl_x.angle_pid.integral_limit = g_default_ang_integral_limit;
    g_roll_ctrl_y.angle_pid.integral_limit = g_default_ang_integral_limit;
}

static void roll_update_pid_profile_by_target(void)
{
    uint8_t center_target = roll_is_center_target();
    uint8_t corner_target = roll_is_corner_target();
    uint8_t wanted_mode = 0U;
    if (center_target)
    {
        wanted_mode = 1U;
    }
    else if (corner_target)
    {
        wanted_mode = 2U;
    }

    if (wanted_mode == g_pid_profile_mode)
    {
        return;
    }

    if (wanted_mode == 1U)
    {
        roll_apply_center_pid_profile();
    }
    else if (wanted_mode == 2U)
    {
        roll_apply_corner_pid_profile();
    }
    else
    {
        roll_restore_default_pid_profile();
    }
    g_pid_profile_mode = wanted_mode;
}

static uint8_t roll_is_out_of_safe_zone(float x, float y)
{
    float min_c = ROLL_CENTER_COORD - ROLL_SAFE_HALF_SIZE;
    float max_c = ROLL_CENTER_COORD + ROLL_SAFE_HALF_SIZE;
    if ((x < min_c) || (x > max_c) || (y < min_c) || (y > max_c))
    {
        return 1U;
    }
    return 0U;
}

static void roll_apply_kp_boost_by_zone(float measure_x, float measure_y)
{
    uint8_t out_zone = roll_is_out_of_safe_zone(measure_x, measure_y);
    float kp_scale = out_zone ? 2.5f : 1.0f;
    float ki_scale = out_zone ? 0.0f : 1.0f;
    float kd_scale = out_zone ? 0.5f : 1.0f;
    float ref_pos_kp = out_zone ? g_corner_pos_kp : g_base_pos_kp;
    float ref_pos_ki = out_zone ? g_corner_pos_ki : g_base_pos_ki;
    float ref_pos_kd = out_zone ? g_corner_pos_kd : g_base_pos_kd;
    float ref_vel_kp = out_zone ? g_corner_vel_kp : g_base_vel_kp;
    float ref_vel_ki = out_zone ? g_corner_vel_ki : g_base_vel_ki;
    float ref_vel_kd = out_zone ? g_corner_vel_kd : g_base_vel_kd;
    float ref_ang_kp = out_zone ? g_corner_ang_kp : g_base_ang_kp;
    float ref_ang_ki = out_zone ? g_corner_ang_ki : g_base_ang_ki;
    float ref_ang_kd = out_zone ? g_corner_ang_kd : g_base_ang_kd;

    g_roll_ctrl_x.pos_pid.kp = ref_pos_kp * kp_scale;
    g_roll_ctrl_y.pos_pid.kp = ref_pos_kp * kp_scale;
    g_roll_ctrl_x.vel_pid.kp = ref_vel_kp * kp_scale;
    g_roll_ctrl_y.vel_pid.kp = ref_vel_kp * kp_scale;
    g_roll_ctrl_x.angle_pid.kp = ref_ang_kp * kp_scale;
    g_roll_ctrl_y.angle_pid.kp = ref_ang_kp * kp_scale;

    g_roll_ctrl_x.pos_pid.ki = ref_pos_ki * ki_scale;
    g_roll_ctrl_y.pos_pid.ki = ref_pos_ki * ki_scale;
    g_roll_ctrl_x.vel_pid.ki = ref_vel_ki * ki_scale;
    g_roll_ctrl_y.vel_pid.ki = ref_vel_ki * ki_scale;
    g_roll_ctrl_x.angle_pid.ki = ref_ang_ki * ki_scale;
    g_roll_ctrl_y.angle_pid.ki = ref_ang_ki * ki_scale;

    g_roll_ctrl_x.pos_pid.kd = ref_pos_kd * kd_scale;
    g_roll_ctrl_y.pos_pid.kd = ref_pos_kd * kd_scale;
    g_roll_ctrl_x.vel_pid.kd = ref_vel_kd * kd_scale;
    g_roll_ctrl_y.vel_pid.kd = ref_vel_kd * kd_scale;
    g_roll_ctrl_x.angle_pid.kd = ref_ang_kd * kd_scale;
    g_roll_ctrl_y.angle_pid.kd = ref_ang_kd * kd_scale;
}

static float roll_clamp(float value, float limit)
{
    if (limit <= 0.0f) {
        return value;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void roll_apply_edge_gain(float measure_x, float measure_y)
{
    float dx = measure_x - ROLL_CENTER_COORD;
    float dy = measure_y - ROLL_CENTER_COORD;
    float adx = (dx >= 0.0f) ? dx : -dx;
    float ady = (dy >= 0.0f) ? dy : -dy;
    float edge_dist = (adx > ady) ? adx : ady;
    float edge_ref = ROLL_EDGE_COORD - ROLL_CENTER_COORD;
    float edge_ratio;
    float kp_scale;
    float ki_scale;
    float kd_scale;

    if (edge_ref <= 0.0f) {
        edge_ref = 1.0f;
    }
    edge_ratio = edge_dist / edge_ref;
    if (edge_ratio < 0.0f) {
        edge_ratio = 0.0f;
    } else if (edge_ratio > 1.0f) {
        edge_ratio = 1.0f;
    }

    /* 越靠�? kp/ki降到0.5�? kd升到2�?*/
    kp_scale = 1.0f - 0.5f * edge_ratio;
    ki_scale = 1.0f - 0.5f * edge_ratio;
    kd_scale = 1.0f + 1.0f * edge_ratio;

    g_roll_ctrl_x.pos_pid.kp = g_base_pos_kp * kp_scale;
    g_roll_ctrl_y.pos_pid.kp = g_base_pos_kp * kp_scale;
    g_roll_ctrl_x.pos_pid.ki = g_base_pos_ki * ki_scale;
    g_roll_ctrl_y.pos_pid.ki = g_base_pos_ki * ki_scale;
    g_roll_ctrl_x.pos_pid.kd = g_base_pos_kd * kd_scale;
    g_roll_ctrl_y.pos_pid.kd = g_base_pos_kd * kd_scale;

    g_roll_ctrl_x.vel_pid.kp = g_base_vel_kp * kp_scale;
    g_roll_ctrl_y.vel_pid.kp = g_base_vel_kp * kp_scale;
    g_roll_ctrl_x.vel_pid.ki = g_base_vel_ki * ki_scale;
    g_roll_ctrl_y.vel_pid.ki = g_base_vel_ki * ki_scale;
    g_roll_ctrl_x.vel_pid.kd = g_base_vel_kd * kd_scale;
    g_roll_ctrl_y.vel_pid.kd = g_base_vel_kd * kd_scale;

    g_roll_ctrl_x.angle_pid.kp = g_base_ang_kp * kp_scale;
    g_roll_ctrl_y.angle_pid.kp = g_base_ang_kp * kp_scale;
    g_roll_ctrl_x.angle_pid.ki = g_base_ang_ki * ki_scale;
    g_roll_ctrl_y.angle_pid.ki = g_base_ang_ki * ki_scale;
    g_roll_ctrl_x.angle_pid.kd = g_base_ang_kd * kd_scale;
    g_roll_ctrl_y.angle_pid.kd = g_base_ang_kd * kd_scale;
}

void RollPid_Init(RollPid_t *pid, float kp, float ki, float kd, float out_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->out_limit = out_limit;
    pid->integral_limit = integral_limit;
}

void RollPid_Reset(RollPid_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

float RollPid_Update(RollPid_t *pid, float target, float measure, float dt_s)
{
    float error = target - measure;
    float derivative;
    float out;

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    pid->integral += error * dt_s;
    pid->integral = roll_clamp(pid->integral, pid->integral_limit);

    derivative = (error - pid->last_error) / dt_s;
    out = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    out = roll_clamp(out, pid->out_limit);

    pid->last_error = error;
    return out;
}

void RollCascade_Init(RollCascadePid_t *ctrl,
                      float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                      float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                      float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit)
{
    RollPid_Init(&ctrl->pos_pid, pos_kp, pos_ki, pos_kd, pos_out_limit, pos_integral_limit);
    RollPid_Init(&ctrl->vel_pid, vel_kp, vel_ki, vel_kd, vel_out_limit, vel_integral_limit);
    /* 先保留角度环参数，当前两环模式不启用 */
    /* RollPid_Init(&ctrl->angle_pid, ang_kp, ang_ki, ang_kd, ang_out_limit, ang_integral_limit); */
    RollPid_Init(&ctrl->angle_pid, ang_kp, ang_ki, ang_kd, ang_out_limit, ang_integral_limit);
}

void RollCascade_Reset(RollCascadePid_t *ctrl)
{
    RollPid_Reset(&ctrl->pos_pid);
    RollPid_Reset(&ctrl->vel_pid);
    /* 先保留角度环状态重置，当前两环模式不启�?*/
    /* RollPid_Reset(&ctrl->angle_pid); */
    RollPid_Reset(&ctrl->angle_pid);
}

int16_t RollCascade_Update(RollCascadePid_t *ctrl,
                           float target_pos, float measure_pos,
                           float measure_vel, float measure_angle, float dt_s)
{
    float target_vel = RollPid_Update(&ctrl->pos_pid, target_pos, measure_pos, dt_s);
    float target_angle = RollPid_Update(&ctrl->vel_pid, target_vel, measure_vel, dt_s);
    float out;

    /* 三环保留：目标角�?-> 角度�?-> 电机输出 */
    /* out = RollPid_Update(&ctrl->angle_pid, target_angle, measure_angle, dt_s); */

    /* 当前两环：速度环输出直接作为电机输�?*/
    (void)measure_angle;
    out = target_angle;

    if (out > 32767.0f) {
        out = 32767.0f;
    } else if (out < -32768.0f) {
        out = -32768.0f;
    }

    return (int16_t)out;
}

void RollCtrl_Init(float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                   float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                   float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit)
{
    RollCascade_Init(&g_roll_ctrl_x,
                     pos_kp, pos_ki, pos_kd, pos_out_limit, pos_integral_limit,
                     vel_kp, vel_ki, vel_kd, vel_out_limit, vel_integral_limit,
                     ang_kp, ang_ki, ang_kd, ang_out_limit, ang_integral_limit);
    RollCascade_Init(&g_roll_ctrl_y,
                     pos_kp, pos_ki, pos_kd, pos_out_limit, pos_integral_limit,
                     vel_kp, vel_ki, vel_kd, vel_out_limit, vel_integral_limit,
                     ang_kp, ang_ki, ang_kd, ang_out_limit, ang_integral_limit);

    g_base_pos_kp = pos_kp;
    g_base_pos_ki = pos_ki;
    g_base_pos_kd = pos_kd;
    g_base_vel_kp = vel_kp;
    g_base_vel_ki = vel_ki;
    g_base_vel_kd = vel_kd;
    g_base_ang_kp = ang_kp;
    g_base_ang_ki = ang_ki;
    g_base_ang_kd = ang_kd;

    g_default_pos_kp = pos_kp;
    g_default_pos_ki = pos_ki;
    g_default_pos_kd = pos_kd;
    g_default_pos_out_limit = pos_out_limit;
    g_default_pos_integral_limit = pos_integral_limit;
    g_default_vel_kp = vel_kp;
    g_default_vel_ki = vel_ki;
    g_default_vel_kd = vel_kd;
    g_default_vel_out_limit = vel_out_limit;
    g_default_vel_integral_limit = vel_integral_limit;
    g_default_ang_kp = ang_kp;
    g_default_ang_ki = ang_ki;
    g_default_ang_kd = ang_kd;
    g_default_ang_out_limit = ang_out_limit;
    g_default_ang_integral_limit = ang_integral_limit;

    g_corner_pos_kp = pos_kp;
    g_corner_pos_ki = pos_ki;
    g_corner_pos_kd = pos_kd;
    g_corner_pos_out_limit = pos_out_limit;
    g_corner_pos_integral_limit = pos_integral_limit;
    g_corner_vel_kp = vel_kp;
    g_corner_vel_ki = vel_ki;
    g_corner_vel_kd = vel_kd;
    g_corner_vel_out_limit = vel_out_limit;
    g_corner_vel_integral_limit = vel_integral_limit;
    g_corner_ang_kp = ang_kp;
    g_corner_ang_ki = ang_ki;
    g_corner_ang_kd = ang_kd;
    g_corner_ang_out_limit = ang_out_limit;
    g_corner_ang_integral_limit = ang_integral_limit;
    g_pid_profile_mode = 0U;
}

void RollCtrl_SetCornerPidProfile(float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                                  float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                                  float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit)
{
    g_corner_pos_kp = pos_kp;
    g_corner_pos_ki = pos_ki;
    g_corner_pos_kd = pos_kd;
    g_corner_pos_out_limit = pos_out_limit;
    g_corner_pos_integral_limit = pos_integral_limit;

    g_corner_vel_kp = vel_kp;
    g_corner_vel_ki = vel_ki;
    g_corner_vel_kd = vel_kd;
    g_corner_vel_out_limit = vel_out_limit;
    g_corner_vel_integral_limit = vel_integral_limit;

    g_corner_ang_kp = ang_kp;
    g_corner_ang_ki = ang_ki;
    g_corner_ang_kd = ang_kd;
    g_corner_ang_out_limit = ang_out_limit;
    g_corner_ang_integral_limit = ang_integral_limit;

    if (g_pid_profile_mode == 2U)
    {
        roll_apply_corner_pid_profile();
    }
}

void RollCtrl_Reset(void)
{
    RollCascade_Reset(&g_roll_ctrl_x);
    RollCascade_Reset(&g_roll_ctrl_y);
    g_target_vel_x = 0.0f;
    g_target_vel_y = 0.0f;
    g_target_angle_x = 0.0f;
    g_target_angle_y = 0.0f;
    g_abs_cmd_x = 0;
    g_abs_cmd_y = 0;
    g_last_servo_pos_x = 1380U;
    g_last_servo_pos_y = 1400U;
}

int16_t RollCtrl_CalcX(float measure_x, float measure_vx, float measure_angle_x, float dt_s)
{
    return RollCascade_Update(&g_roll_ctrl_x, g_roll_target_x, measure_x, measure_vx, measure_angle_x, dt_s);
}

int16_t RollCtrl_CalcY(float measure_y, float measure_vy, float measure_angle_y, float dt_s)
{
    return RollCascade_Update(&g_roll_ctrl_y, g_roll_target_y, measure_y, measure_vy, measure_angle_y, dt_s);
}

void RollCtrl_UpdatePos(float measure_x, float measure_y, float dt_s)
{
    if (g_shijue_error_flag != 0U)
    {
        return;
    }
    roll_update_pid_profile_by_target();
    roll_apply_kp_boost_by_zone(measure_x, measure_y);
    /* roll_apply_edge_gain(measure_x, measure_y); */
    g_target_vel_x = RollPid_Update(&g_roll_ctrl_x.pos_pid, g_roll_target_x, measure_x, dt_s);
    g_target_vel_y = RollPid_Update(&g_roll_ctrl_y.pos_pid, g_roll_target_y, measure_y, dt_s);
}

void RollCtrl_UpdateVel(float measure_vx, float measure_vy, float dt_s)
{
    if (g_shijue_error_flag != 0U)
    {
        return;
    }
    g_target_angle_x = RollPid_Update(&g_roll_ctrl_x.vel_pid, g_target_vel_x, measure_vx, dt_s);
    g_target_angle_y = RollPid_Update(&g_roll_ctrl_y.vel_pid, g_target_vel_y, measure_vy, dt_s);
}

void RollCtrl_UpdateAngleOutput(float measure_angle_x, float measure_angle_y, float dt_s)
{
    float out_x_f;
    float out_y_f;
    int16_t out_x;
    int16_t out_y;
    int32_t cmd_x;
    int32_t cmd_y;
    uint8_t dir_x;
    uint8_t dir_y;
    uint32_t pulse_x;
    uint32_t pulse_y;

    if (g_shijue_error_flag != 0U)
    {
        return;
    }

    /* 三环保留：姿态环输出 */
    /* out_x_f = RollPid_Update(&g_roll_ctrl_x.angle_pid, g_target_angle_x, measure_angle_x, dt_s); */
    /* out_y_f = RollPid_Update(&g_roll_ctrl_y.angle_pid, g_target_angle_y, measure_angle_y, dt_s); */

    /* 当前两环：速度环输出直接驱动电�?*/
    (void)measure_angle_x;
    (void)measure_angle_y;
    (void)dt_s;
    out_x_f = g_target_angle_x;
    out_y_f = g_target_angle_y;

    if (out_x_f > 32767.0f) {
        out_x_f = 32767.0f;
    } else if (out_x_f < -32768.0f) {
        out_x_f = -32768.0f;
    }
    if (out_y_f > 32767.0f) {
        out_y_f = 32767.0f;
    } else if (out_y_f < -32768.0f) {
        out_y_f = -32768.0f;
    }

    out_x = (int16_t)out_x_f; /* 本周期增�?du_x */
    out_y = (int16_t)out_y_f; /* 本周期增�?du_y */

    cmd_x = g_abs_cmd_x + (int32_t)out_x;
    cmd_y = g_abs_cmd_y + (int32_t)out_y;
    if (cmd_x > 2147483647L) {
        cmd_x = 2147483647L;
    } else if (cmd_x < -2147483647L) {
        cmd_x = -2147483647L;
    }
    if (cmd_y > 2147483647L) {
        cmd_y = 2147483647L;
    } else if (cmd_y < -2147483647L) {
        cmd_y = -2147483647L;
    }
    g_abs_cmd_x = cmd_x;
    g_abs_cmd_y = cmd_y;

    dir_x = (g_abs_cmd_x >= 0) ? 1 : 0;
    dir_y = (g_abs_cmd_y >= 0) ? 1 : 0;
    pulse_x = (uint32_t)abs((int)g_abs_cmd_x);
    pulse_y = (uint32_t)abs((int)g_abs_cmd_y);

    Emm_V5_MMCL_Pos_Control(3, dir_x, k_roll_pair_vel, k_roll_pair_acc, pulse_x, true, true);
    Emm_V5_MMCL_Pos_Control(4, dir_y, k_roll_pair_vel, k_roll_pair_acc, pulse_y, true, true);
    Emm_V5_Multi_Motor_Cmd_UART5(0);
}

void RollCtrl_UpdateAngleOutput_duoji(float measure_angle_x, float measure_angle_y, float dt_s)
{
    float out_x_f;
    float out_y_f;
    float err_x;
    float err_y;
    int16_t out_x;
    int16_t out_y;
    int32_t cmd_x;
    int32_t cmd_y;
    int32_t servo_cmd_x;
    int32_t servo_cmd_y;
    uint16_t pos_x;
    uint16_t pos_y;
    float deadband_px;

    if (g_shijue_error_flag != 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_last_servo_pos_x);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_last_servo_pos_y);
        return;
    }

    /* 三环保留：姿态环输出 */
    /* out_x_f = RollPid_Update(&g_roll_ctrl_x.angle_pid, g_target_angle_x, measure_angle_x, dt_s); */
    /* out_y_f = RollPid_Update(&g_roll_ctrl_y.angle_pid, g_target_angle_y, measure_angle_y, dt_s); */

    /* 当前两环：速度环输出直接驱动电�?*/
    (void)measure_angle_x;
    (void)measure_angle_y;
    (void)dt_s;
    out_x_f = g_target_angle_x;
    out_y_f = g_target_angle_y;

    /* Ŀ�긽��������Ĭ��6���أ�Ŀ��9Ϊ6 */
    err_x = g_roll_target_x - (float)g_shijue_x;
    err_y = g_roll_target_y - (float)g_shijue_y;
    deadband_px = ROLL_DEADBAND_PX;
    if (roll_target_matches_point(9U))
    {
        deadband_px = ROLL_DEADBAND_P9_PX;
    }
    if ((err_x <= deadband_px) && (err_x >= -deadband_px))
    {
        out_x_f = 0.0f;
    }
    if ((err_y <= deadband_px) && (err_y >= -deadband_px))
    {
        out_y_f = 0.0f;
    }

    /* 边界保护：以中心(160,160)为原点，260x260安全�?=> x/y在[30,290] */
    if (((float)g_shijue_x > (ROLL_CENTER_COORD + ROLL_SAFE_HALF_SIZE)) && (out_x_f > 0.0f))
    {
        out_x_f = 0.0f;
    }
    if (((float)g_shijue_x < (ROLL_CENTER_COORD - ROLL_SAFE_HALF_SIZE)) && (out_x_f < 0.0f))
    {
        out_x_f = 0.0f;
    }
    if (((float)g_shijue_y > (ROLL_CENTER_COORD + ROLL_SAFE_HALF_SIZE)) && (out_y_f > 0.0f))
    {
        out_y_f = 0.0f;
    }
    if (((float)g_shijue_y < (ROLL_CENTER_COORD - ROLL_SAFE_HALF_SIZE)) && (out_y_f < 0.0f))
    {
        out_y_f = 0.0f;
    }

    if (out_x_f > 32767.0f) {
        out_x_f = 32767.0f;
    } else if (out_x_f < -32768.0f) {
        out_x_f = -32768.0f;
    }
    if (out_y_f > 32767.0f) {
        out_y_f = 32767.0f;
    } else if (out_y_f < -32768.0f) {
        out_y_f = -32768.0f;
    }

    out_x = (int16_t)out_x_f; /* 本周期增�?du_x */
    out_y = (int16_t)out_y_f; /* 本周期增�?du_y */

    /* 舵机模式改为非累计输出，避免一有目标就积分打满到限�?*/
    cmd_x = (int32_t)out_x;
    cmd_y = (int32_t)out_y;
    if (cmd_x > ROLL_CMD_LIMIT) {
        cmd_x = ROLL_CMD_LIMIT;
    } else if (cmd_x < -ROLL_CMD_LIMIT) {
        cmd_x = -ROLL_CMD_LIMIT;
    }
    if (cmd_y > ROLL_CMD_LIMIT) {
        cmd_y = ROLL_CMD_LIMIT;
    } else if (cmd_y < -ROLL_CMD_LIMIT) {
        cmd_y = -ROLL_CMD_LIMIT;
    }

    g_abs_cmd_x = cmd_x;
    g_abs_cmd_y = cmd_y;

    /* �?500为舵机中位基准，PID增量在此基础上叠�?*/
    /* 反向X轴映射：提高PWM时，球朝x正方向运�?*/
    servo_cmd_x = 1350 - g_abs_cmd_x;
    servo_cmd_y = 1400 + g_abs_cmd_y;
    servo_cmd_x += ROLL_CH1_GLOBAL_BIAS;
    if (roll_target_matches_point(9U))
    {
        servo_cmd_x += 10;
    }
    if (roll_target_matches_point(7U))
    {
        servo_cmd_x -= 15;
    }
    // if (roll_target_matches_point(9U))
    // {
    //     servo_cmd_x += 150;
    //     servo_cmd_y += 150;
    // }
    // else if (roll_target_matches_point(1U))
    // {
    //     servo_cmd_x += 0;
    //     servo_cmd_y -= 0;
    // }
    // else if (roll_target_matches_point(7U))
    // {
    //     servo_cmd_x -= 10;
    //     servo_cmd_y += 10;
    // }
    // else if (roll_target_matches_point(3U))
    // {
    //     servo_cmd_x += 10;
    //     servo_cmd_y += 10;
    // }
    pos_x = (servo_cmd_x < 500) ? 500U : (servo_cmd_x > 2500 ? 2500U : (uint16_t)servo_cmd_x);
    pos_y = (servo_cmd_y < 500) ? 500U : (servo_cmd_y > 2500 ? 2500U : (uint16_t)servo_cmd_y);
    g_last_servo_pos_x = pos_x;
    g_last_servo_pos_y = pos_y;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pos_x);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pos_y);
}

void RollCtrl_Update(float dt_s)
{
    float measure_x = (float)g_shijue_x;
    float measure_y = (float)g_shijue_y;
    float measure_vx = g_shijue_vx;
    float measure_vy = g_shijue_vy;
    float measure_angle_x = 0.0f;
    float measure_angle_y = 0.0f;

    RollCtrl_UpdatePos(measure_x, measure_y, dt_s);
    RollCtrl_UpdateVel(measure_vx, measure_vy, dt_s);
    RollCtrl_UpdateAngleOutput(measure_angle_x, measure_angle_y, dt_s);
}

void RollCtrl_MovePair(uint8_t dir, uint32_t pulse)
{
    Emm_V5_MMCL_Pos_Control(3, dir, k_roll_pair_vel, k_roll_pair_acc, pulse, false, true);
    Emm_V5_MMCL_Pos_Control(4, dir, k_roll_pair_vel, k_roll_pair_acc, pulse, false, true);
    Emm_V5_Multi_Motor_Cmd_UART5(0);
}




