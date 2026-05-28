#ifndef __STATE_H
#define __STATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint8_t place[20];
extern uint8_t place_len;
extern uint32_t g_state9_elapsed_ms;

void State_Init(void);
void State_Set(uint8_t state);
uint8_t State_Get(void);
void State_UpdateFromRxBuffer2(void);
void State_RunCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_H */
