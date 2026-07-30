#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1,
    KEY_EVENT_2,
    KEY_EVENT_3,
    KEY_EVENT_4
} KeyEvent_t;

void BSP_KeyInit(void);
KeyEvent_t BSP_KeyScan(uint32_t now_ms);

#endif /* BSP_KEY_H */
