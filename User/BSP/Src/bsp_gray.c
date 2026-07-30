#include "bsp_gray.h"

#include "main.h"

uint8_t BSP_GrayReadRaw(void)
{
    uint8_t value = 0x01U;

    /* 模块在白色上输出低电平；转换后统一使用0黑1白。 */
    /* HUI1已损坏并持续为0，暂时强制按白色处理。 */
    // if (HAL_GPIO_ReadPin(HUI1_GPIO_Port, HUI1_Pin) == GPIO_PIN_RESET) value |= 0x01U;
    if (HAL_GPIO_ReadPin(HUI2_GPIO_Port, HUI2_Pin) == GPIO_PIN_RESET) value |= 0x02U;
    if (HAL_GPIO_ReadPin(HUI3_GPIO_Port, HUI3_Pin) == GPIO_PIN_RESET) value |= 0x04U;
    if (HAL_GPIO_ReadPin(HUI4_GPIO_Port, HUI4_Pin) == GPIO_PIN_RESET) value |= 0x08U;
    if (HAL_GPIO_ReadPin(HUI5_GPIO_Port, HUI5_Pin) == GPIO_PIN_RESET) value |= 0x10U;
    if (HAL_GPIO_ReadPin(HUI6_GPIO_Port, HUI6_Pin) == GPIO_PIN_RESET) value |= 0x20U;
    if (HAL_GPIO_ReadPin(HUI7_GPIO_Port, HUI7_Pin) == GPIO_PIN_RESET) value |= 0x40U;
    if (HAL_GPIO_ReadPin(HUI8_GPIO_Port, HUI8_Pin) == GPIO_PIN_RESET) value |= 0x80U;

    return value;
}
