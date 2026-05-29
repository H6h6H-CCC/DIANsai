#include "encoder.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

#define ENCODER3_DIR (-1)
#define ENCODER4_DIR (1)

static int16_t s_last_cnt = 0;
static int16_t s_last_delta = 0;
static int32_t s_total = 0;
static volatile int32_t s_report_delta = 0;
static int16_t s_last_cnt4 = 0;
static int16_t s_last_delta4 = 0;
static int32_t s_total4 = 0;
static volatile int32_t s_report_delta4 = 0;

void Encoder3_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    s_last_cnt = 0;
    s_last_delta = 0;
    s_total = 0;
    s_report_delta = 0;
}

int16_t Encoder3_GetCount(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

int16_t Encoder3_GetDelta(void)
{
    int16_t now = Encoder3_GetCount();
    int16_t delta = (int16_t)((now - s_last_cnt) * ENCODER3_DIR);
    s_last_cnt = now;
    s_total += delta;
    return delta;
}

int32_t Encoder3_GetTotal(void)
{
    return s_total;
}

void Encoder3_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    s_last_cnt = 0;
    s_last_delta = 0;
    s_total = 0;
    s_report_delta = 0;
}

void Encoder3_Update10ms(void)
{
    s_last_delta = Encoder3_GetDelta();
    s_report_delta += s_last_delta;
}

int16_t Encoder3_GetLastDelta(void)
{
    return s_last_delta;
}

void Encoder3_ReportUart4(uint8_t count1)
{
    static char tx_buf[64];
    int len;

    if (huart4.gState != HAL_UART_STATE_READY) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "tim3_delta=%ld, count1=%u\r\n",
                   (long)s_report_delta,
                   (unsigned int)count1);

    if (HAL_UART_Transmit_DMA(&huart4, (uint8_t *)tx_buf, (uint16_t)len) == HAL_OK) {
        s_report_delta = 0;
    }
}

void Encoder4_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    s_last_cnt4 = 0;
    s_last_delta4 = 0;
    s_total4 = 0;
    s_report_delta4 = 0;
}

int16_t Encoder4_GetCount(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

int16_t Encoder4_GetDelta(void)
{
    int16_t now = Encoder4_GetCount();
    int16_t delta = (int16_t)((now - s_last_cnt4) * ENCODER4_DIR);
    s_last_cnt4 = now;
    s_total4 += delta;
    return delta;
}

int32_t Encoder4_GetTotal(void)
{
    return s_total4;
}

void Encoder4_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    s_last_cnt4 = 0;
    s_last_delta4 = 0;
    s_total4 = 0;
    s_report_delta4 = 0;
}

void Encoder4_Update10ms(void)
{
    s_last_delta4 = Encoder4_GetDelta();
    s_report_delta4 += s_last_delta4;
}

int16_t Encoder4_GetLastDelta(void)
{
    return s_last_delta4;
}

void Encoder_ReportUart4(uint8_t count1)
{
    static char tx_buf[96];
    int len;

    if (huart4.gState != HAL_UART_STATE_READY) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "tim3_delta=%ld, tim4_delta=%ld, count1=%u\r\n",
                   (long)s_report_delta,
                   (long)s_report_delta4,
                   (unsigned int)count1);

    if (HAL_UART_Transmit_DMA(&huart4, (uint8_t *)tx_buf, (uint16_t)len) == HAL_OK) {
        s_report_delta = 0;
        s_report_delta4 = 0;
    }
}

void Encoder_GetAndClearReportDelta(int32_t *tim3_delta, int32_t *tim4_delta)
{
    __disable_irq();
    *tim3_delta = s_report_delta;
    *tim4_delta = s_report_delta4;
    s_report_delta = 0;
    s_report_delta4 = 0;
    __enable_irq();
}
