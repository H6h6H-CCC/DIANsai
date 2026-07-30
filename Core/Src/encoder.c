#include "encoder.h"
#include "bsp_critical.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"
#include <stdio.h>

/* 新车头方向：左轮B使用TIM3，右轮A使用TIM4；车辆前进时均为正。 */
#define ENCODER3_DIR (1)
#define ENCODER4_DIR (-1)

static int16_t s_last_cnt = 0;
static int16_t s_last_delta = 0;
static int32_t s_total = 0;
static volatile int32_t s_report_delta = 0;
static volatile int32_t s_speed_cps = 0;
static int16_t s_last_cnt4 = 0;
static int16_t s_last_delta4 = 0;
static int32_t s_total4 = 0;
static volatile int32_t s_report_delta4 = 0;
static volatile int32_t s_speed_cps4 = 0;

void Encoder3_Init(void)
{
    BSP_EncoderInit(BSP_ENCODER_3);
    s_last_cnt = 0;
    s_last_delta = 0;
    s_total = 0;
    s_report_delta = 0;
    s_speed_cps = 0;
}

int16_t Encoder3_GetCount(void)
{
    return BSP_EncoderRead(BSP_ENCODER_3);
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
    BSP_EncoderReset(BSP_ENCODER_3);
    s_last_cnt = 0;
    s_last_delta = 0;
    s_total = 0;
    s_report_delta = 0;
}

void Encoder3_Update10ms(void)
{
    s_last_delta = Encoder3_GetDelta();
    s_report_delta += s_last_delta;
    s_speed_cps = (int32_t)s_last_delta * 100;
}

int16_t Encoder3_GetLastDelta(void)
{
    return s_last_delta;
}

int32_t Encoder3_GetSpeedCps(void)
{
    return s_speed_cps;
}

void Encoder3_ReportUart4(uint8_t count1)
{
    static char tx_buf[64];
    int len;

    if (!BSP_UartTxReady(BSP_UART_4)) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "tim3_delta=%ld, count1=%u\r\n",
                   (long)s_report_delta,
                   (unsigned int)count1);

    if (BSP_UartSendDma(BSP_UART_4,
                        (uint8_t *)tx_buf,
                        (uint16_t)len) == BSP_STATUS_OK) {
        s_report_delta = 0;
    }
}

void Encoder4_Init(void)
{
    BSP_EncoderInit(BSP_ENCODER_4);
    s_last_cnt4 = 0;
    s_last_delta4 = 0;
    s_total4 = 0;
    s_report_delta4 = 0;
    s_speed_cps4 = 0;
}

int16_t Encoder4_GetCount(void)
{
    return BSP_EncoderRead(BSP_ENCODER_4);
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
    BSP_EncoderReset(BSP_ENCODER_4);
    s_last_cnt4 = 0;
    s_last_delta4 = 0;
    s_total4 = 0;
    s_report_delta4 = 0;
}

void Encoder4_Update10ms(void)
{
    s_last_delta4 = Encoder4_GetDelta();
    s_report_delta4 += s_last_delta4;
    s_speed_cps4 = (int32_t)s_last_delta4 * 100;
}

int16_t Encoder4_GetLastDelta(void)
{
    return s_last_delta4;
}

int32_t Encoder4_GetSpeedCps(void)
{
    return s_speed_cps4;
}

void Encoder_ReportUart4(uint8_t count1)
{
    static char tx_buf[96];
    int len;

    if (!BSP_UartTxReady(BSP_UART_4)) {
        return;
    }

    len = snprintf(tx_buf, sizeof(tx_buf),
                   "tim3_delta=%ld, tim4_delta=%ld, count1=%u\r\n",
                   (long)s_report_delta,
                   (long)s_report_delta4,
                   (unsigned int)count1);

    if (BSP_UartSendDma(BSP_UART_4,
                        (uint8_t *)tx_buf,
                        (uint16_t)len) == BSP_STATUS_OK) {
        s_report_delta = 0;
        s_report_delta4 = 0;
    }
}

void Encoder_GetAndClearReportDelta(int32_t *tim3_delta, int32_t *tim4_delta)
{
    uint32_t primask = BSP_EnterCritical();

    *tim3_delta = s_report_delta;
    *tim4_delta = s_report_delta4;
    s_report_delta = 0;
    s_report_delta4 = 0;
    BSP_ExitCritical(primask);
}
