#ifndef __TJC_SCREEN_H__
#define __TJC_SCREEN_H__

#include "bsp_uart.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bsp_status_t TJC_SetT0Text(const char *text);
bsp_status_t TJC_SetT0Number(int32_t value);
void TJC_OnRx(const uint8_t *data, uint16_t length);
uint16_t TJC_GetLastRx(uint8_t *data, uint16_t capacity);

#ifdef __cplusplus
}
#endif

#endif
