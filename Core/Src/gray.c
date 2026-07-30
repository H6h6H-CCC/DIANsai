#include "gray.h"
#include "bsp_gray.h"

uint8_t Gray_Read(void)
{
    return BSP_GrayReadRaw();
}

int16_t Gray_GetError(void)
{
    /* HUI1~HUI8 从车体左向右排列；正误差表示黑线位于左侧。 */
    static const int16_t weight[8] = {22, 17, 9,0, -0, -9, -17, -22};
    uint8_t gray = Gray_Read();
    int16_t sum = 0;
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < 8U; i++) {
        if ((gray & (uint8_t)(1U << i)) == 0U) {
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
    return (Gray_Read() == 0x00U) ? 1U : 0U;
}

uint8_t Gray_AllWhite(void)
{
    return (Gray_Read() == 0xFFU) ? 1U : 0U;
}
