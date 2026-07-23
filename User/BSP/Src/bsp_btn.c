#include "bsp_btn.h"

#include "tim.h"

/* BTN 驱动允许的最大 CCR 值，超过此值时统一限幅。 */
#define BSP_BTN_PWM_MAX 1000U

/*
 * 一组电机由两个 PWM 通道组成：正转时 A 输出 PWM，反转时 B 输出 PWM。
 * 未工作的另一个通道始终置零，pwm=0 时两个通道都置零。
 */
static void BSP_BtnSetPair(uint32_t channel_a,
                           uint32_t channel_b,
                           int16_t pwm)
{
    uint16_t duty = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;

    if (duty > BSP_BTN_PWM_MAX)
    {
        duty = BSP_BTN_PWM_MAX;
    }

    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, channel_a, duty);
        __HAL_TIM_SET_COMPARE(&htim1, channel_b, 0U);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, channel_a, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, channel_b, duty);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, channel_a, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, channel_b, 0U);
    }
}

void BSP_BtnInit(void)
{
    /* 两组 BTN 电机共同占用 TIM1 CH1~CH4。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    BSP_BtnWritePair(0U, 0);
    BSP_BtnWritePair(1U, 0);
}

void BSP_BtnWritePair(uint8_t pair, int16_t pwm)
{
    /* pair=0 使用 CH1/CH2，pair=1 使用 CH3/CH4。 */
    if (pair == 0U)
    {
        BSP_BtnSetPair(TIM_CHANNEL_1, TIM_CHANNEL_2, pwm);
    }
    else if (pair == 1U)
    {
        BSP_BtnSetPair(TIM_CHANNEL_3, TIM_CHANNEL_4, pwm);
    }
}
