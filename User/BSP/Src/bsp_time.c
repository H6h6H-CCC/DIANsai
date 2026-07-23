#include "bsp_time.h"

#include "stm32f4xx_hal.h"
#include "tim.h"

uint32_t BSP_TimeMs(void)
{
    /* HAL SysTick 默认每 1 ms 递增一次。 */
    return HAL_GetTick();
}

void BSP_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

void BSP_TimeStartPeriodic(void)
{
    /* TIM9 的周期和中断优先级由 CubeMX 配置决定。 */
    HAL_TIM_Base_Start_IT(&htim9);
}
