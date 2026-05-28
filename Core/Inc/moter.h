#ifndef __MOTER_H
#define __MOTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Moter_Init(void);

/* pwm: >0 = forward, <0 = reverse, 0 = stop */
void Moter_A(int16_t pwm);
void Moter_B(int16_t pwm);
void Moter_C(int16_t pwm);
void Moter_D(int16_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* __MOTER_H */
