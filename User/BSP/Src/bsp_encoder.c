#include "bsp_encoder.h"

#include "tim.h"

/* 将 BSP 编码器编号映射到 CubeMX 生成的定时器句柄。 */
static TIM_HandleTypeDef *BSP_EncoderGetHandle(bsp_encoder_t encoder)
{
    return (encoder == BSP_ENCODER_3) ? &htim3 : &htim4;
}

void BSP_EncoderInit(bsp_encoder_t encoder)
{
    TIM_HandleTypeDef *handle = BSP_EncoderGetHandle(encoder);

    /* 同时启动编码器模式的两个输入通道，并从零开始计数。 */
    HAL_TIM_Encoder_Start(handle, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(handle, 0U);
}

int16_t BSP_EncoderRead(bsp_encoder_t encoder)
{
    /* 以 int16_t 解释 16 位计数器，便于上层计算正负增量和处理回绕。 */
    return (int16_t)__HAL_TIM_GET_COUNTER(BSP_EncoderGetHandle(encoder));
}

void BSP_EncoderReset(bsp_encoder_t encoder)
{
    __HAL_TIM_SET_COUNTER(BSP_EncoderGetHandle(encoder), 0U);
}
