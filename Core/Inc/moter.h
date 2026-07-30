#ifndef __MOTER_H
#define __MOTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Moter_Init(void);

/* 新车头方向：B为左轮、A为右轮；pwm<0前进，pwm>0后退。 */
void Moter_A(int16_t pwm);
void Moter_B(int16_t pwm);
void Moter_A_Brake(void);
void Moter_B_Brake(void);
void Moter_C(int16_t pwm);
void Moter_D(int16_t pwm);

#ifdef __cplusplus
}
#endif

#endif /* __MOTER_H */
