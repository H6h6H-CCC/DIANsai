#ifndef __TJC_SCREEN_H__
#define __TJC_SCREEN_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef TJC_SetT0Text(const char *text);
HAL_StatusTypeDef TJC_SetT0Number(int32_t value);

#ifdef __cplusplus
}
#endif

#endif
