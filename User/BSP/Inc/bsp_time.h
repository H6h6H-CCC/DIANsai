#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/*
 * 系统时间接口
 *
 * 毫秒时间和延时来自 HAL SysTick；应用周期节拍当前由 TIM9 中断产生。
 * 上层代码只表达“获取时间/延时/启动周期节拍”，不直接依赖 HAL 和 TIM9。
 */

/**
 * @brief 获取系统启动后的毫秒计数。
 * @return HAL SysTick 的 32 位毫秒计数，约 49.7 天后自然回绕。
 */
uint32_t BSP_TimeMs(void);

/**
 * @brief 阻塞当前代码指定的毫秒数。
 * @param ms 延时时间，单位为毫秒。
 * @note  这是阻塞延时，不应在中断服务函数中调用。
 */
void BSP_DelayMs(uint32_t ms);

/**
 * @brief 启动 TIM9 基本定时器中断，作为应用层周期节拍。
 * @note  调用前必须执行 MX_TIM9_Init()。
 */
void BSP_TimeStartPeriodic(void);

#endif /* BSP_TIME_H */
