#ifndef __BTN_H
#define __BTN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BTN_Init(void);

/*
 * pwm > 0: forward  -> PWM_A = duty, PWM_B = 0
 * pwm < 0: reverse  -> PWM_A = 0,    PWM_B = duty
 * pwm = 0: coasting -> PWM_A = 0,    PWM_B = 0
 */
void BTN_Set12(int16_t pwm);
void BTN_Set34(int16_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* __BTN_H */
