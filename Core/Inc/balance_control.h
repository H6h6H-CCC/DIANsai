#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Balance_Init(void);
void Balance_Update10ms(void);
void Balance_SetTargetAngle(float angle_deg);
void Balance_SetPid(float kp, float ki, float kd);
void Balance_Enable(uint8_t enable);
int16_t Balance_GetLastPwm(void);

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_CONTROL_H */
