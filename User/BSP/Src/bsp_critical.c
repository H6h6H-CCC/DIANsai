#include "bsp_critical.h"

#include "stm32f4xx.h"

uint32_t BSP_EnterCritical(void)
{
    /* 先保存原状态，保证退出时不会错误地打开原本已经关闭的中断。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void BSP_ExitCritical(uint32_t primask)
{
    /* PRIMASK=0 表示进入临界区前中断处于开启状态。 */
    if (primask == 0U)
    {
        __enable_irq();
    }
}
