#include "tjc_screen.h"

#include "usart.h"

#include <stdio.h>
#include <string.h>

#define TJC_TX_BUFFER_SIZE 256U

static uint8_t s_tjc_tx_buffer[TJC_TX_BUFFER_SIZE];

HAL_StatusTypeDef TJC_SetT0Text(const char *text)
{
    static const uint8_t prefix[] = "t0.txt=\"";
    static const uint8_t suffix[] = {'\"', 0xFFU, 0xFFU, 0xFFU};
    size_t text_len;
    size_t command_len;

    if (text == NULL)
    {
        return HAL_ERROR;
    }

    if (huart3.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    text_len = strlen(text);
    command_len = sizeof(prefix) - 1U + text_len + sizeof(suffix);
    if (command_len > sizeof(s_tjc_tx_buffer))
    {
        return HAL_ERROR;
    }

    memcpy(s_tjc_tx_buffer, prefix, sizeof(prefix) - 1U);
    memcpy(&s_tjc_tx_buffer[sizeof(prefix) - 1U], text, text_len);
    /* The closing quote is followed by the three required command terminators. */
    memcpy(&s_tjc_tx_buffer[sizeof(prefix) - 1U + text_len], suffix, sizeof(suffix));

    return HAL_UART_Transmit_DMA(&huart3, s_tjc_tx_buffer, (uint16_t)command_len);
}

HAL_StatusTypeDef TJC_SetT0Number(int32_t value)
{
    char text[12];
    int length = snprintf(text, sizeof(text), "%ld", (long)value);

    if (length <= 0)
    {
        return HAL_ERROR;
    }

    return TJC_SetT0Text(text);
}
