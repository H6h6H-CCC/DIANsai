#include "angle_sensor.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

#define ANGLE_SENSOR_RESOLUTION 4098.0f

volatile uint32_t g_angle_period_count = 0U;
volatile uint32_t g_angle_high_count = 0U;

static uint32_t s_last_rise = 0U;
static uint8_t s_wait_fall = 0U;
static uint8_t s_ready = 0U;
static float s_zero_offset = 138.0f;

static uint32_t AngleSensor_Delta(uint32_t now, uint32_t last)
{
    return now - last;
}

static float AngleSensor_Normalize(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

void AngleSensor_Init(void)
{
    g_angle_period_count = 0U;
    g_angle_high_count = 0U;
    s_last_rise = 0U;
    s_wait_fall = 0U;
    s_ready = 0U;

    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
}

float AngleSensor_GetAngle(void)
{
    float position;

    if ((g_angle_period_count == 0U) || (g_angle_high_count > g_angle_period_count)) {
        return 0.0f;
    }

    position = ((float)g_angle_high_count / (float)g_angle_period_count * ANGLE_SENSOR_RESOLUTION - 1.0f);
    position = position / ANGLE_SENSOR_RESOLUTION * 360.0f;

    return AngleSensor_Normalize(position - s_zero_offset);
}

uint8_t AngleSensor_IsReady(void)
{
    return s_ready;
}

void AngleSensor_SetZeroOffset(float offset_deg)
{
    s_zero_offset = AngleSensor_Normalize(offset_deg);
}

void AngleSensor_ReportUart4(void)
{
    static char tx_buf[80];
    int len;

    if (huart4.gState != HAL_UART_STATE_READY) {
        return;
    }

    if (AngleSensor_IsReady()) {
        float angle = AngleSensor_GetAngle();
        uint32_t angle_x100 = (uint32_t)(angle * 100.0f + 0.5f);

        len = snprintf(tx_buf, sizeof(tx_buf),
                       "angle=%lu.%02lu deg, high=%lu, period=%lu\r\n",
                       (unsigned long)(angle_x100 / 100U),
                       (unsigned long)(angle_x100 % 100U),
                       (unsigned long)g_angle_high_count,
                       (unsigned long)g_angle_period_count);
    } else {
        len = snprintf(tx_buf, sizeof(tx_buf), "angle sensor not ready\r\n");
    }

    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)tx_buf, (uint16_t)len);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t capture;

    if ((htim->Instance != TIM5) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (s_wait_fall) {
        g_angle_high_count = AngleSensor_Delta(capture, s_last_rise);
        s_wait_fall = 0U;
        s_ready = (g_angle_period_count != 0U);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    } else {
        if (s_last_rise != 0U) {
            g_angle_period_count = AngleSensor_Delta(capture, s_last_rise);
        }
        s_last_rise = capture;
        s_wait_fall = 1U;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
}
