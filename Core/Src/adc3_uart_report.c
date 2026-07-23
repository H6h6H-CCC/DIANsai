#include "adc3_uart_report.h"

#include "adc.h"
#include "bsp_uart.h"
#include "dma.h"

#include <stdio.h>

volatile uint16_t g_adc3_raw[2];
static char s_uart_msg[128];
static uint8_t s_adc_started;

void Adc3UartReport_Init(void)
{
    s_adc_started = 0U;
    g_adc3_ch4_raw = 0U;
    g_adc3_ch5_raw = 0U;
}

HAL_StatusTypeDef Adc3UartReport_Start(void)
{
    HAL_StatusTypeDef status;

    if (s_adc_started)
    {
        return HAL_OK;
    }

    status = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_adc3_raw, 2);
    if (status != HAL_OK)
    {
        char err_msg[64];
        int len = snprintf(err_msg, sizeof(err_msg),
                           "ADC DMA start err:%u adc:%lu dma:%lu\r\n",
                           (unsigned int)status,
                           HAL_ADC_GetError(&hadc3),
                           HAL_DMA_GetError(hadc3.DMA_Handle));
        if (len > 0)
        {
            (void)BSP_UartSend(BSP_UART_4,
                               (uint8_t *)err_msg,
                               (uint16_t)len,
                               100U);
        }
        return status;
    }

    __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle, DMA_IT_TC);
    s_adc_started = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Adc3UartReport_Stop(void)
{
    HAL_StatusTypeDef status;

    if (!s_adc_started)
    {
        return HAL_OK;
    }

    status = HAL_ADC_Stop_DMA(&hadc3);
    if (status == HAL_OK)
    {
        s_adc_started = 0U;
    }

    return status;
}

uint32_t Adc3UartReport_GetCh4Mv(void)
{
    return ((uint32_t)g_adc3_ch4_raw * 3300U + 2047U) / 4095U;
}

uint32_t Adc3UartReport_GetCh5Mv(void)
{
    return ((uint32_t)g_adc3_ch5_raw * 3300U + 2047U) / 4095U;
}

HAL_StatusTypeDef Adc3UartReport_Send(void)
{
    uint32_t ch4_mv = Adc3UartReport_GetCh4Mv();
    uint32_t ch5_mv = Adc3UartReport_GetCh5Mv();
    int len = snprintf(s_uart_msg, sizeof(s_uart_msg),
                       "ADC3 CH4=%lu.%03luV CH5=%lu.%03luV\r\n",
                       ch4_mv / 1000U, ch4_mv % 1000U,
                       ch5_mv / 1000U, ch5_mv % 1000U);

    if (!BSP_UartTxReady(BSP_UART_4))
    {
        return HAL_BUSY;
    }

    if (len <= 0)
    {
        return HAL_ERROR;
    }

    return (BSP_UartSendDma(BSP_UART_4,
                            (uint8_t *)s_uart_msg,
                            (uint16_t)len) == BSP_STATUS_OK)
               ? HAL_OK
               : HAL_BUSY;
}
