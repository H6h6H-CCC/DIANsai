#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H

#include <stdint.h>

/*
 * 临界区接口
 *
 * 用于保护会同时被主循环和中断访问的短小代码段。
 * 进入临界区时保存原中断状态，退出时按原状态恢复，支持在已关中断时调用。
 */

/**
 * @brief 保存当前 PRIMASK 并关闭可屏蔽中断。
 * @return 进入前的 PRIMASK，必须原样传给 BSP_ExitCritical()。
 */
uint32_t BSP_EnterCritical(void);

/**
 * @brief 按进入临界区前的状态恢复中断。
 * @param primask BSP_EnterCritical() 返回的值。
 * @note  临界区应尽量短，不能在其中执行延时或阻塞通信。
 */
void BSP_ExitCritical(uint32_t primask);

#endif /* BSP_CRITICAL_H */
