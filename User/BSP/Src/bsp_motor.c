#include "bsp_motor.h"

#include "main.h"
#include "tim.h"

typedef struct
{
    /* 每路电机的两个方向引脚和一个 TIM1 PWM 通道。 */
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t channel;
} bsp_motor_hw_t;

/* 逻辑电机 A~D 与开发板实际引脚、TIM1 通道的唯一映射表。 */
static const bsp_motor_hw_t motor_hw[] =
{
    {A1_GPIO_Port, A1_Pin, A2_GPIO_Port, A2_Pin, TIM_CHANNEL_1},
    {B1_GPIO_Port, B1_Pin, B2_GPIO_Port, B2_Pin, TIM_CHANNEL_2},
    {C1_GPIO_Port, C1_Pin, C2_GPIO_Port, C2_Pin, TIM_CHANNEL_3},
    {D1_GPIO_Port, D1_Pin, D2_GPIO_Port, D2_Pin, TIM_CHANNEL_4}
};

void BSP_MotorInit(void)
{
    uint8_t i;

    /* 启动四路 PWM，并确保初始化后电机输出为零。 */
    for (i = 0U; i < 4U; i++)
    {
        HAL_TIM_PWM_Start(&htim1, motor_hw[i].channel);
        __HAL_TIM_SET_COMPARE(&htim1, motor_hw[i].channel, 0U);
    }
}

void BSP_MotorWrite(bsp_motor_t motor,
                    uint8_t in1_level,
                    uint8_t in2_level,
                    uint16_t pwm)
{
    const bsp_motor_hw_t *hw;

    if (motor > BSP_MOTOR_D)
    {
        return;
    }

    /* 上层决定方向电平组合；BSP 只负责把逻辑值写入实际硬件。 */
    hw = &motor_hw[motor];
    HAL_GPIO_WritePin(hw->in1_port,
                      hw->in1_pin,
                      in1_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port,
                      hw->in2_pin,
                      in2_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, hw->channel, pwm);
}
