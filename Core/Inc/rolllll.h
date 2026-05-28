#ifndef __ROLLLLL_H
#define __ROLLLLL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float out_limit;
    float integral_limit;
} RollPid_t;

typedef struct {
    RollPid_t pos_pid;
    RollPid_t vel_pid;
    RollPid_t angle_pid;
} RollCascadePid_t;

extern float g_roll_target_x;
extern float g_roll_target_y;

void RollPid_Init(RollPid_t *pid, float kp, float ki, float kd, float out_limit, float integral_limit);
void RollPid_Reset(RollPid_t *pid);
float RollPid_Update(RollPid_t *pid, float target, float measure, float dt_s);

void RollCascade_Init(RollCascadePid_t *ctrl,
                      float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                      float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                      float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit);
void RollCascade_Reset(RollCascadePid_t *ctrl);
int16_t RollCascade_Update(RollCascadePid_t *ctrl,
                           float target_pos, float measure_pos,
                           float measure_vel, float measure_angle, float dt_s);

void RollCtrl_Init(float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                   float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                   float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit);
void RollCtrl_SetCornerPidProfile(float pos_kp, float pos_ki, float pos_kd, float pos_out_limit, float pos_integral_limit,
                                  float vel_kp, float vel_ki, float vel_kd, float vel_out_limit, float vel_integral_limit,
                                  float ang_kp, float ang_ki, float ang_kd, float ang_out_limit, float ang_integral_limit);
void RollCtrl_Reset(void);
int16_t RollCtrl_CalcX(float measure_x, float measure_vx, float measure_angle_x, float dt_s);
int16_t RollCtrl_CalcY(float measure_y, float measure_vy, float measure_angle_y, float dt_s);
void RollCtrl_UpdatePos(float measure_x, float measure_y, float dt_s);
void RollCtrl_UpdateVel(float measure_vx, float measure_vy, float dt_s);
void RollCtrl_UpdateAngleOutput(float measure_angle_x, float measure_angle_y, float dt_s);
void RollCtrl_Update(float dt_s);
void RollCtrl_UpdateAngleOutput_duoji(float measure_angle_x, float measure_angle_y, float dt_s);
void RollCtrl_MovePair(uint8_t dir, uint32_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __ROLLLLL_H */
