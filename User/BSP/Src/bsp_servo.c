#include "bsp_servo.h"

#include "tim.h"

/* 舵机逻辑编号与 TIM2 CH1~CH4 的映射。 */
static const uint32_t servo_channel[] =
{
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

void BSP_ServoInit(void)
{
    uint8_t i;

    /* 四路舵机共用 TIM2 的计数周期，但分别使用独立 CCR。 */
    for (i = 0U; i < 4U; i++)
    {
        HAL_TIM_PWM_Start(&htim2, servo_channel[i]);
    }
}

void BSP_ServoSetPulse(bsp_servo_t servo, uint16_t pulse)
{
    /* pulse 直接写入 CCR；角度到 pulse 的换算应由上层完成。 */
    if (servo <= BSP_SERVO_4)
    {
        __HAL_TIM_SET_COMPARE(&htim2, servo_channel[servo], pulse);
    }
}
