#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Encoder3_Init(void);
int16_t Encoder3_GetCount(void);
int16_t Encoder3_GetDelta(void);
int32_t Encoder3_GetTotal(void);
void Encoder3_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
