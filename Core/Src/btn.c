#include "btn.h"
#include "tim.h"

#define BTN_PWM_MAX 1000

static uint16_t clamp_abs_pwm(int16_t pwm)
{
    uint16_t duty = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;
    if (duty > BTN_PWM_MAX) {
        duty = BTN_PWM_MAX;
    }
    return duty;
}

static void btn_set_pair(uint32_t ch_a, uint32_t ch_b, int16_t pwm)
{
    uint16_t duty = clamp_abs_pwm(pwm);

    if (duty == 0) {
        __HAL_TIM_SET_COMPARE(&htim1, ch_a, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch_b, 0);
        return;
    }

    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, ch_a, duty);
        __HAL_TIM_SET_COMPARE(&htim1, ch_b, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, ch_a, 0);
        __HAL_TIM_SET_COMPARE(&htim1, ch_b, duty);
    }
}

void BTN_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    BTN_Set12(0);
    BTN_Set34(0);
}

void BTN_Set12(int16_t pwm)
{
    btn_set_pair(TIM_CHANNEL_1, TIM_CHANNEL_2, pwm);
}

void BTN_Set34(int16_t pwm)
{
    btn_set_pair(TIM_CHANNEL_3, TIM_CHANNEL_4, pwm);
}
