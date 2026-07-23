#include "gray.h"
#include "bsp_gray.h"

uint8_t Gray_Read(void)
{
    return BSP_GrayReadRaw();
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
