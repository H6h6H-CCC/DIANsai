#include "tjc_screen.h"

#include <stdio.h>
#include <string.h>

#define TJC_TX_BUFFER_SIZE 256U

static uint8_t s_tjc_tx_buffer[TJC_TX_BUFFER_SIZE];
static uint8_t s_tjc_rx_buffer[TJC_TX_BUFFER_SIZE];
static volatile uint16_t s_tjc_rx_length;

bsp_status_t TJC_SetT0Text(const char *text)
{
    static const uint8_t prefix[] = "t0.txt=\"";
    static const uint8_t suffix[] = {'\"', 0xFFU, 0xFFU, 0xFFU};
    size_t text_len;
    size_t command_len;

    if (text == NULL)
    {
        return BSP_STATUS_ERROR;
    }

    if (!BSP_UartTxReady(BSP_UART_3))
    {
        return BSP_STATUS_BUSY;
    }

    text_len = strlen(text);
    command_len = sizeof(prefix) - 1U + text_len + sizeof(suffix);
    if (command_len > sizeof(s_tjc_tx_buffer))
    {
        return BSP_STATUS_ERROR;
    }

    memcpy(s_tjc_tx_buffer, prefix, sizeof(prefix) - 1U);
    memcpy(&s_tjc_tx_buffer[sizeof(prefix) - 1U], text, text_len);
    /* The closing quote is followed by the three required command terminators. */
    memcpy(&s_tjc_tx_buffer[sizeof(prefix) - 1U + text_len], suffix, sizeof(suffix));

    return BSP_UartSendDma(BSP_UART_3,
                           s_tjc_tx_buffer,
                           (uint16_t)command_len);
}

bsp_status_t TJC_SetT0Number(int32_t value)
{
    char text[12];
    int length = snprintf(text, sizeof(text), "%ld", (long)value);

    if (length <= 0)
    {
        return BSP_STATUS_ERROR;
    }

    return TJC_SetT0Text(text);
}

void TJC_OnRx(const uint8_t *data, uint16_t length)
{
    if (length > sizeof(s_tjc_rx_buffer))
    {
        length = sizeof(s_tjc_rx_buffer);
    }

    memcpy(s_tjc_rx_buffer, data, length);
    s_tjc_rx_length = length;
}

uint16_t TJC_GetLastRx(uint8_t *data, uint16_t capacity)
{
    uint16_t length = s_tjc_rx_length;

    if (length > capacity)
    {
        length = capacity;
    }

    memcpy(data, s_tjc_rx_buffer, length);
    return length;
}
