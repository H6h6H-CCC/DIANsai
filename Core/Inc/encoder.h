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
void Encoder3_Update10ms(void);
int16_t Encoder3_GetLastDelta(void);
int32_t Encoder3_GetSpeedCps(void);
void Encoder3_ReportUart4(uint8_t count1);

void Encoder4_Init(void);
int16_t Encoder4_GetCount(void);
int16_t Encoder4_GetDelta(void);
int32_t Encoder4_GetTotal(void);
void Encoder4_Reset(void);
void Encoder4_Update10ms(void);
int16_t Encoder4_GetLastDelta(void);
int32_t Encoder4_GetSpeedCps(void);
void Encoder_ReportUart4(uint8_t count1);
void Encoder_GetAndClearReportDelta(int32_t *tim3_delta, int32_t *tim4_delta);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
