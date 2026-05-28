#ifndef __ATK_MS53L0M_UART_H
#define __ATK_MS53L0M_UART_H

#include "main.h"

#define ATK_MS53L0M_UART_INTERFACE              USART2
#define ATK_MS53L0M_UART_RX_BUF_SIZE            128

void atk_ms53l0m_uart_send(uint8_t *dat, uint8_t len);
void atk_ms53l0m_uart_rx_restart(void);
uint8_t *atk_ms53l0m_uart_rx_get_frame(void);
uint16_t atk_ms53l0m_uart_rx_get_frame_len(void);
void atk_ms53l0m_uart_init(uint32_t baudrate);
void atk_ms53l0m_uart_rx_event(uint8_t *dat, uint16_t len);

#endif
