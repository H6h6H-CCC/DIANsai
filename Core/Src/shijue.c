#include "shijue.h"
#include "stm32f4xx_hal.h"

volatile uint16_t g_shijue_x = 0;
volatile uint16_t g_shijue_y = 0;
volatile float g_shijue_vx = 0.0f;
volatile float g_shijue_vy = 0.0f;
volatile ShijuePoint_t g_shijue_centers[9] = {
    {22U, 22U},   /* 1 */
    {160U, 22U},  /* 2 */
    {296U, 22U},  /* 3 */
    {22U, 160U},  /* 4 */
    {160U, 160U}, /* 5 */
    {296U, 160U}, /* 6 */
    {22U, 296U},  /* 7 */
    {160U, 296U}, /* 8 */
    {290U, 290U}  /* 9 */
};
volatile uint8_t g_shijue_error_flag = 0;

static uint8_t shijue_ascii_to_digit(uint8_t c, uint8_t *digit)
{
    if ((c >= 0x30U) && (c <= 0x39U))
    {
        *digit = (uint8_t)(c - 0x30U);
        return 1U;
    }
    return 0U;
}

static uint8_t shijue_parse_frame_at(const uint8_t *buf, uint16_t pos, uint8_t *tail, uint16_t *x, uint16_t *y)
{
    uint8_t d1, d2, d3, d4, d5, d6;

    if ((buf[pos] != 0xAAU) || (x == 0) || (y == 0) || (tail == 0))
    {
        return 0U;
    }

    if (!shijue_ascii_to_digit(buf[pos + 1U], &d1)) { return 0U; }
    if (!shijue_ascii_to_digit(buf[pos + 2U], &d2)) { return 0U; }
    if (!shijue_ascii_to_digit(buf[pos + 3U], &d3)) { return 0U; }
    if (!shijue_ascii_to_digit(buf[pos + 4U], &d4)) { return 0U; }
    if (!shijue_ascii_to_digit(buf[pos + 5U], &d5)) { return 0U; }
    if (!shijue_ascii_to_digit(buf[pos + 6U], &d6)) { return 0U; }

    *x = (uint16_t)(d1 * 100U + d2 * 10U + d3);
    *y = (uint16_t)(d4 * 100U + d5 * 10U + d6);
    *tail = buf[pos + 7U];
    return 1U;
}

static void shijue_update_speed(uint16_t x, uint16_t y)
{
    static uint8_t has_last = 0U;
    static uint16_t last_x = 0U;
    static uint16_t last_y = 0U;
    static uint32_t last_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if (!has_last)
    {
        last_x = x;
        last_y = y;
        last_tick = now_tick;
        g_shijue_vx = 0.0f;
        g_shijue_vy = 0.0f;
        has_last = 1U;
        return;
    }

    if (now_tick > last_tick)
    {
        float dt_s = (float)(now_tick - last_tick) / 1000.0f;
        g_shijue_vx = ((float)x - (float)last_x) / dt_s;
        g_shijue_vy = ((float)y - (float)last_y) / dt_s;
    }

    last_x = x;
    last_y = y;
    last_tick = now_tick;
}

uint8_t Shijue_ParseFrame8(const uint8_t *buf, uint16_t len, uint16_t *x, uint16_t *y)
{
    uint16_t i;

    if ((buf == 0) || (x == 0) || (y == 0) || (len < 8U))
    {
        return 0U;
    }

    for (i = 0; i <= (uint16_t)(len - 8U); i++)
    {
        if ((buf[i] == 0xAAU) && (buf[i + 7U] == 0xFFU))
        {
            uint8_t tail;
            if (shijue_parse_frame_at(buf, i, &tail, x, y))
            {
                return 1U;
            }
        }
    }

    return 0U;
}

void Shijue_ProcessRxBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t x;
    uint16_t y;
    uint8_t tail;

    if ((buf == 0) || (len < 8U))
    {
        return;
    }

    for (i = 0; i <= (uint16_t)(len - 8U); i++)
    {
        if (!shijue_parse_frame_at(buf, i, &tail, &x, &y))
        {
            continue;
        }

        if (tail == 0xFFU)
        {
            if ((x == 0U) && (y == 0U))
            {
                g_shijue_error_flag = 1U;
                g_shijue_vx = 0.0f;
                g_shijue_vy = 0.0f;
                continue;
            }

            g_shijue_x = x;
            g_shijue_y = y;
            g_shijue_error_flag = 0U;
            shijue_update_speed(g_shijue_x, g_shijue_y);
        }
        else if ((tail >= 0xF1U) && (tail <= 0xF9U))
        {
            /* 中心点坐标固定写死，不再动态更新 */
            (void)x;
            (void)y;
        }
    }
}
