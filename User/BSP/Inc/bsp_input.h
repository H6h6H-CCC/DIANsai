#ifndef BSP_INPUT_H
#define BSP_INPUT_H

#include <stdint.h>

/* 板载按键和拨码开关输入接口。GPIO 端口和引脚只在 BSP 实现中使用。 */

/**
 * @brief 检查 KAIGUAN1~KAIGUAN3 中是否有任意按键按下。
 * @return 1=至少一个按键按下，0=全部松开。
 * @note  按键为低电平有效，本函数不包含软件消抖。
 */
uint8_t BSP_InputAnyKeyPressed(void);

/**
 * @brief 读取 bo1~bo4 四位拨码开关并组合为一个数值。
 * @return bit0=bo1，bit1=bo2，bit2=bo3，bit3=bo4；高电平对应位为 1。
 */
uint8_t BSP_InputReadDipSwitch(void);

#endif /* BSP_INPUT_H */
