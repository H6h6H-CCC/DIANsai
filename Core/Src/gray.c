#include "gray.h"
#include "main.h"

uint8_t Gray_Read(void)
{
    uint8_t value = 0U;

    if (HAL_GPIO_ReadPin(HUI1_GPIO_Port, HUI1_Pin) == GPIO_PIN_RESET) value |= 0x01U;
    if (HAL_GPIO_ReadPin(HUI2_GPIO_Port, HUI2_Pin) == GPIO_PIN_RESET) value |= 0x02U;
    if (HAL_GPIO_ReadPin(HUI3_GPIO_Port, HUI3_Pin) == GPIO_PIN_RESET) value |= 0x04U;
    if (HAL_GPIO_ReadPin(HUI4_GPIO_Port, HUI4_Pin) == GPIO_PIN_RESET) value |= 0x08U;
    if (HAL_GPIO_ReadPin(HUI5_GPIO_Port, HUI5_Pin) == GPIO_PIN_RESET) value |= 0x10U;
    if (HAL_GPIO_ReadPin(HUI6_GPIO_Port, HUI6_Pin) == GPIO_PIN_RESET) value |= 0x20U;
    if (HAL_GPIO_ReadPin(HUI7_GPIO_Port, HUI7_Pin) == GPIO_PIN_RESET) value |= 0x40U;
    if (HAL_GPIO_ReadPin(HUI8_GPIO_Port, HUI8_Pin) == GPIO_PIN_RESET) value |= 0x80U;

    return value;
}

int16_t Gray_GetError(void)
{
    static const int16_t weight[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    uint8_t gray = Gray_Read();
    int16_t sum = 0;
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if ((gray & (uint8_t)(1U << i)) != 0U) {
            sum += weight[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0;
    }

    return sum / (int16_t)count;
}

uint8_t Gray_AllBlack(void)
{
    return (Gray_Read() == 0xFFU) ? 1U : 0U;
}

uint8_t Gray_AllWhite(void)
{
    return (Gray_Read() == 0x00U) ? 1U : 0U;
}
