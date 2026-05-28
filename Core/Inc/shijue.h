#ifndef __SHIJUE_H
#define __SHIJUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint16_t x;
    uint16_t y;
} ShijuePoint_t;

extern volatile uint16_t g_shijue_x;
extern volatile uint16_t g_shijue_y;
extern volatile float g_shijue_vx;
extern volatile float g_shijue_vy;
extern volatile ShijuePoint_t g_shijue_centers[9];
extern volatile uint8_t g_shijue_error_flag;

uint8_t Shijue_ParseFrame8(const uint8_t *buf, uint16_t len, uint16_t *x, uint16_t *y);
void Shijue_ProcessRxBuffer(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SHIJUE_H */
