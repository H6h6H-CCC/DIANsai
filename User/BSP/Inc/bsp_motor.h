#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

/*
 * 四路直流电机底层接口
 *
 * 每路电机使用两个方向 GPIO 和 TIM1 的一个 PWM 通道：
 *   A/B/C/D -> TIM1 CH1/CH2/CH3/CH4
 * 方向的具体组合由上层 moter.c 决定，本层只负责写引脚和比较值。
 * 本模块不能与 bsp_btn 同时占用 TIM1。
 */

/** 四路电机的逻辑编号。 */
typedef enum
{
    BSP_MOTOR_A = 0,  /**< A1/A2，TIM1 CH1。 */
    BSP_MOTOR_B,      /**< B1/B2，TIM1 CH2。 */
    BSP_MOTOR_C,      /**< C1/C2，TIM1 CH3。 */
    BSP_MOTOR_D       /**< D1/D2，TIM1 CH4。 */
} bsp_motor_t;

/**
 * @brief 启动四路 TIM1 PWM，并把所有比较值清零。
 * @note  调用前必须执行 MX_TIM1_Init()。
 */
void BSP_MotorInit(void);

/**
 * @brief 写入一路电机的两个方向电平和 PWM 比较值。
 * @param motor     电机逻辑编号 BSP_MOTOR_A~BSP_MOTOR_D。
 * @param in1_level IN1 输出：0=低电平，非0=高电平。
 * @param in2_level IN2 输出：0=低电平，非0=高电平。
 * @param pwm       TIM1 CCR 原始比较值，合法范围由 TIM1 的 Period 决定。
 * @note  motor 超出范围时，本函数不改变硬件输出。
 */
void BSP_MotorWrite(bsp_motor_t motor,
                    uint8_t in1_level,
                    uint8_t in2_level,
                    uint16_t pwm);

#endif /* BSP_MOTOR_H */
