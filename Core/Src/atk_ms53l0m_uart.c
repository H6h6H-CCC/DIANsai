#include "atk_ms53l0m_uart.h"
#include "app_config.h"
#include "bsp_uart.h"
#include <string.h>

static struct
{
    uint8_t buf[ATK_MS53L0M_UART_RX_BUF_SIZE];
    struct
    {
        uint16_t len   : 15;
        uint16_t finsh : 1;
    } sta;
} g_uart_rx_frame = {0};

void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len)
{
#if APP_USART2_MODE == APP_USART2_MODE_ATK_TOF
    (void)BSP_UartSend(BSP_UART_2, dat, len, 0xFFFFFFFFU);
#else
    (void)dat;
    (void)len;
#endif
}

void atk_ms53l0m_uart_rx_restart(void)
{
    g_uart_rx_frame.sta.len = 0;
    g_uart_rx_frame.sta.finsh = 0;
}

uint8_t *atk_ms53l0m_uart_rx_get_frame(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        g_uart_rx_frame.buf[g_uart_rx_frame.sta.len] = '\0';
        return g_uart_rx_frame.buf;
    }

    return NULL;
}

uint16_t atk_ms53l0m_uart_rx_get_frame_len(void)
{
    if (g_uart_rx_frame.sta.finsh == 1)
    {
        return g_uart_rx_frame.sta.len;
    }

    return 0;
}

void atk_ms53l0m_uart_init(uint32_t baudrate)
{
    (void)baudrate;
    atk_ms53l0m_uart_rx_restart();
}

void atk_ms53l0m_uart_rx_event(uint8_t *dat, uint16_t len)
{
    uint16_t copy_len = len;

    if (copy_len >= ATK_MS53L0M_UART_RX_BUF_SIZE)
    {
        copy_len = ATK_MS53L0M_UART_RX_BUF_SIZE - 1;
    }

    memcpy(g_uart_rx_frame.buf, dat, copy_len);
    g_uart_rx_frame.sta.len = copy_len;
    g_uart_rx_frame.sta.finsh = 1;
}
