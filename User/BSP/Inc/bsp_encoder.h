#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

/*
 * 正交编码器定时器接口
 *
 * 把业务层使用的编码器编号与 STM32 定时器隔离：
 *   BSP_ENCODER_3 -> TIM3
 *   BSP_ENCODER_4 -> TIM4
 */

/** 编码器逻辑编号。 */
typedef enum
{
    BSP_ENCODER_3 = 0,  /**< 使用 TIM3。 */
    BSP_ENCODER_4       /**< 使用 TIM4。 */
} bsp_encoder_t;

/**
 * @brief 启动指定定时器的编码器模式，并把硬件计数器清零。
 * @param encoder 编码器逻辑编号。
 * @note  调用前必须完成对应的 MX_TIM3_Init() 或 MX_TIM4_Init()。
 */
void BSP_EncoderInit(bsp_encoder_t encoder);

/**
 * @brief 读取指定编码器当前的 16 位硬件计数值。
 * @param encoder 编码器逻辑编号。
 * @return 转换为 int16_t 的当前计数；可通过正负号判断本次转动方向。
 * @note  这里只返回硬件计数器值，圈数累计和溢出处理由 encoder.c 完成。
 */
int16_t BSP_EncoderRead(bsp_encoder_t encoder);

/**
 * @brief 把指定编码器的硬件计数器清零。
 * @param encoder 编码器逻辑编号。
 */
void BSP_EncoderReset(bsp_encoder_t encoder);

#endif /* BSP_ENCODER_H */
