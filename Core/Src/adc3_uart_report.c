#include "adc3_uart_report.h"

#include "adc.h"
#include "dma.h"
#include "usart.h"

#include <stdio.h>

volatile uint16_t g_adc3_raw[2];
static uint8_t s_hello_msg[] = "hello\r\n";
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
            HAL_UART_Transmit(&huart4, (uint8_t *)err_msg, (uint16_t)len, 100);
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

    if (huart4.gState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }

    if (len <= 0)
    {
        return HAL_ERROR;
    }

    if (HAL_UART_Transmit_DMA(&huart4, s_hello_msg, sizeof(s_hello_msg) - 1U) != HAL_OK)
    {
        return HAL_BUSY;
    }

    uint32_t tx_wait = HAL_GetTick();
    while ((huart4.gState != HAL_UART_STATE_READY) && (HAL_GetTick() - tx_wait < 20U))
    {
    }

    if (huart4.gState == HAL_UART_STATE_READY)
    {
        return HAL_UART_Transmit_DMA(&huart4, (uint8_t *)s_uart_msg, (uint16_t)len);
    }

    return HAL_TIMEOUT;
}
