#ifndef __ANGLE_SENSOR_H
#define __ANGLE_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AngleSensor_Init(void);
float AngleSensor_GetAngle(void);
uint8_t AngleSensor_IsReady(void);
void AngleSensor_SetZeroOffset(float offset_deg);
void AngleSensor_ReportUart4(void);

extern volatile uint32_t g_angle_period_count;
extern volatile uint32_t g_angle_high_count;

#ifdef __cplusplus
}
#endif

#endif /* __ANGLE_SENSOR_H */
