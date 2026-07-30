#ifndef __STATE_H
#define __STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void State_Init(void);
void State_RunCurrent(void);
void State_DebugRx(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_H */
