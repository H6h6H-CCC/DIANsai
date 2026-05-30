#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Balance_Init(void);
void Balance_Update10ms(void);
void Balance_SetTargetAngle(float angle_deg);
float Balance_GetTargetAngle(void);
float Balance_GetMinAngle(void);
float Balance_GetMaxAngle(void);
void Balance_SetPid(float kp, float ki, float kd);
void Balance_Enable(uint8_t enable);
void Balance_HoldPositionEnable(uint8_t enable);
void Balance_ResetHoldPosition(void);
uint8_t Balance_IsHoldPositionEnabled(void);
int16_t Balance_GetLastPwm(void);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_CONTROL_H */
