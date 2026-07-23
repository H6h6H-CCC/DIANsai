#ifndef BSP_GRAY_H
#define BSP_GRAY_H

#include <stdint.h>

/*
 * 八路灰度传感器 GPIO 接口
 *
 * 返回值的 bit0~bit7 依次对应 HUI1~HUI8。
 * 当前硬件为低电平有效：检测到黑线时，对应位为 1。
 */

/**
 * @brief 一次读取八路灰度输入并组合成位图。
 * @return bit0=HUI1，...，bit7=HUI8；1 表示对应输入为低电平。
 */
uint8_t BSP_GrayReadRaw(void);

#endif /* BSP_GRAY_H */
