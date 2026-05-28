#ifndef __ADC3_UART_REPORT_H__
#define __ADC3_UART_REPORT_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint16_t g_adc3_raw[2];
#define g_adc3_ch4_raw (g_adc3_raw[0])
#define g_adc3_ch5_raw (g_adc3_raw[1])

void Adc3UartReport_Init(void);
HAL_StatusTypeDef Adc3UartReport_Start(void);
HAL_StatusTypeDef Adc3UartReport_Stop(void);
HAL_StatusTypeDef Adc3UartReport_Send(void);
uint32_t Adc3UartReport_GetCh4Mv(void);
uint32_t Adc3UartReport_GetCh5Mv(void);

#ifdef __cplusplus
}
#endif

#endif
