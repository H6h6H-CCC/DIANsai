#include "bsp_input.h"

#include "main.h"

uint8_t BSP_InputAnyKeyPressed(void)
{
    /* 三个按键均为低电平有效，任意一个按下就返回 1。 */
    return (HAL_GPIO_ReadPin(KAIGUAN1_GPIO_Port, KAIGUAN1_Pin) == GPIO_PIN_RESET) ||
           (HAL_GPIO_ReadPin(KAIGUAN2_GPIO_Port, KAIGUAN2_Pin) == GPIO_PIN_RESET) ||
           (HAL_GPIO_ReadPin(KAIGUAN3_GPIO_Port, KAIGUAN3_Pin) == GPIO_PIN_RESET);
}

uint8_t BSP_InputReadDipSwitch(void)
{
    uint8_t value = 0U;

    /* bo1~bo4 依次编码到返回值的 bit0~bit3。 */
    if (HAL_GPIO_ReadPin(bo1_GPIO_Port, bo1_Pin) == GPIO_PIN_SET) value |= 0x01U;
    if (HAL_GPIO_ReadPin(bo2_GPIO_Port, bo2_Pin) == GPIO_PIN_SET) value |= 0x02U;
    if (HAL_GPIO_ReadPin(bo3_GPIO_Port, bo3_Pin) == GPIO_PIN_SET) value |= 0x04U;
    if (HAL_GPIO_ReadPin(bo4_GPIO_Port, bo4_Pin) == GPIO_PIN_SET) value |= 0x08U;

    return value;
}
