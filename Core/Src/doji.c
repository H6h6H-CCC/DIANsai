#include "doji.h"
#include "app_config.h"
#include "bsp_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void doji_sendf(const char *fmt, ...)
{
    char buf[48];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    if ((size_t)n >= sizeof(buf)) {
        n = (int)sizeof(buf) - 1;
    }

#if APP_UART5_MODE == APP_UART5_MODE_DOJI
    (void)BSP_UartSend(BSP_UART_5, (uint8_t *)buf, (uint16_t)n, 100U);
#endif
}

void Doji_SendRaw(const char *cmd)
{
    if (cmd == NULL) {
        return;
    }
#if APP_UART5_MODE == APP_UART5_MODE_DOJI
    (void)BSP_UartSend(BSP_UART_5,
                       (const uint8_t *)cmd,
                       (uint16_t)strlen(cmd),
                       100U);
#endif
}

void Doji_ReadId(uint8_t id)
{
    doji_sendf("#%03uPID!", id);
}

void Doji_SetId(uint8_t current_id, uint8_t new_id)
{
    doji_sendf("#%03uPID%03u!", current_id, new_id);
}

void Doji_SetBaudPreset(uint8_t id, uint8_t preset)
{
    doji_sendf("#%03uPBD%u!", id, preset);
}

void Doji_MovePos(uint8_t id, uint16_t pos, uint16_t time_ms)
{
    if (pos < 500) pos = 500;
    if (pos > 2500) pos = 2500;
    doji_sendf("#%03uP%04uT%04u!", id, pos, time_ms);
}

void Doji_Stop(uint8_t id)
{
    doji_sendf("#%03uPDST!", id);
}

void Doji_Pause(uint8_t id)
{
    doji_sendf("#%03uPDPT!", id);
}

void Doji_Resume(uint8_t id)
{
    doji_sendf("#%03uPDCT!", id);
}

void Doji_ReadAngle(uint8_t id)
{
    doji_sendf("#%03uPRAD!", id);
}

void Doji_TorqueOff(uint8_t id)
{
    doji_sendf("#%03uPULK!", id);
}

void Doji_TorqueOn(uint8_t id)
{
    doji_sendf("#%03uPULR!", id);
}
