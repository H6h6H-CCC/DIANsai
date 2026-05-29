#include "moter.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"

#define PWM_MAX 1000

static uint16_t ClampPwmAbs(int16_t pwm)
{
    uint16_t pwm_abs = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;
    if (pwm_abs > PWM_MAX) return PWM_MAX;
    return pwm_abs;
}

void Moter_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void Moter_A(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
    } else if (pwm > 0) {
        HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_abs);
}

void Moter_B(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
    } else if (pwm > 0) {
        HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_abs);
}

void Moter_C(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
    } else if (pwm > 0) {
        HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_abs);
}

void Moter_D(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
    } else if (pwm > 0) {
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_abs);
}
