#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t echo_pin;
} HCSR04_HandleTypeDef;

void HCSR04_Init(HCSR04_HandleTypeDef *hcsr04,
                 GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 GPIO_TypeDef *echo_port, uint16_t echo_pin);
uint8_t HCSR04_ReadCm(HCSR04_HandleTypeDef *hcsr04, float *distance_cm);
uint8_t HCSR04_ReadUs(HCSR04_HandleTypeDef *hcsr04, uint32_t *echo_us);

#ifdef __cplusplus
}
#endif

#endif /* __HCSR04_H */
