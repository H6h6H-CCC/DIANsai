#include "encoder.h"
#include "tim.h"

static int16_t s_last_cnt = 0;
static int32_t s_total = 0;

void Encoder3_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    s_last_cnt = 0;
    s_total = 0;
}

int16_t Encoder3_GetCount(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

int16_t Encoder3_GetDelta(void)
{
    int16_t now = Encoder3_GetCount();
    int16_t delta = (int16_t)(now - s_last_cnt);
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
    s_total = 0;
}
