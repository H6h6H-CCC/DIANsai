#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

/*
 * 四路 PWM 舵机接口
 *
 * BSP_SERVO_1~4 依次对应 TIM2 CH1~CH4。
 * pulse 是定时器 CCR 原始值，其实际脉宽由 TIM2 的计数频率决定。
 */

/** 舵机逻辑编号。 */
typedef enum
{
    BSP_SERVO_1 = 0,  /**< TIM2 CH1。 */
    BSP_SERVO_2,      /**< TIM2 CH2。 */
    BSP_SERVO_3,      /**< TIM2 CH3。 */
    BSP_SERVO_4       /**< TIM2 CH4。 */
} bsp_servo_t;

/**
 * @brief 启动 TIM2 CH1~CH4 的 PWM 输出。
 * @note  调用前必须执行 MX_TIM2_Init()。
 */
void BSP_ServoInit(void);

/**
 * @brief 设置一路舵机的 PWM 比较值。
 * @param servo 舵机逻辑编号 BSP_SERVO_1~BSP_SERVO_4。
 * @param pulse TIM2 CCR 原始比较值，不是角度值。
 * @note  servo 超出范围时，本函数不改变硬件输出。
 */
void BSP_ServoSetPulse(bsp_servo_t servo, uint16_t pulse);

#endif /* BSP_SERVO_H */
