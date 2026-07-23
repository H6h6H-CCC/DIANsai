#ifndef BSP_BTN_H
#define BSP_BTN_H

#include <stdint.h>

/*
 * BTN 双 PWM 电机板接口
 *
 * 使用 TIM1 的四个 PWM 通道：
 *   pair=0 -> CH1、CH2
 *   pair=1 -> CH3、CH4
 * 每组使用两个 PWM 通道控制正反转，不能与 bsp_motor 同时占用 TIM1。
 */

/**
 * @brief 启动 TIM1 CH1~4 的 PWM，并把两组输出全部置零。
 * @note  调用前必须先执行 MX_TIM1_Init()。
 */
void BSP_BtnInit(void);

/**
 * @brief 设置一组 BTN 电机的方向和 PWM 占空比较值。
 * @param pair 电机组编号：0=CH1/CH2，1=CH3/CH4。
 * @param pwm  有符号输出值：正数正转，负数反转，0 停止；绝对值最大限制为 1000。
 * @note  pair 不是 0 或 1 时，本函数不改变任何输出。
 */
void BSP_BtnWritePair(uint8_t pair, int16_t pwm);

#endif /* BSP_BTN_H */
