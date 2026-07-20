#include "shijue.h"
#include "stm32f4xx_hal.h"
#include <string.h>

volatile uint16_t g_shijue_count = 0;
volatile uint16_t g_shijue_class_id = 0;
volatile uint16_t g_shijue_x = 0;
volatile uint16_t g_shijue_y = 0;
volatile uint16_t g_shijue_w = 0;
volatile uint16_t g_shijue_h = 0;
volatile float g_shijue_score = 0.0f;
char g_shijue_label[SHIJUE_LABEL_SIZE + 1U] = {0};
volatile float g_shijue_vx = 0.0f;
volatile float g_shijue_vy = 0.0f;
volatile ShijuePoint_t g_shijue_centers[9] = {
    {22U, 22U}, {160U, 22U}, {296U, 22U},
    {22U, 160U}, {160U, 160U}, {296U, 160U},
    {22U, 296U}, {160U, 296U}, {290U, 290U}
};
volatile uint8_t g_shijue_error_flag = 0;

static uint16_t shijue_read_u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint8_t shijue_parse_frame_at(const uint8_t *buf, uint16_t pos)
{
    float score;

    /* 视觉帧：AA + <6Hf32s> + 77，小端序。 */
    if ((buf[pos] != 0xAAU) || (buf[pos + SHIJUE_FRAME_SIZE - 1U] != 0x77U))
    {
        return 0U;
    }

    g_shijue_count = shijue_read_u16_le(&buf[pos + 1U]);
    g_shijue_class_id = shijue_read_u16_le(&buf[pos + 3U]);
    g_shijue_x = shijue_read_u16_le(&buf[pos + 5U]);
    g_shijue_y = shijue_read_u16_le(&buf[pos + 7U]);
    g_shijue_w = shijue_read_u16_le(&buf[pos + 9U]);
    g_shijue_h = shijue_read_u16_le(&buf[pos + 11U]);
    memcpy(&score, &buf[pos + 13U], sizeof(score));
    g_shijue_score = score;
    memcpy(g_shijue_label, &buf[pos + 17U], SHIJUE_LABEL_SIZE);
    g_shijue_label[SHIJUE_LABEL_SIZE] = '\0';
    return 1U;
}

static void shijue_update_speed(uint16_t x, uint16_t y)
{
    static uint8_t has_last = 0U;
    static uint16_t last_x = 0U;
    static uint16_t last_y = 0U;
    static uint32_t last_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if (!has_last) {
        last_x = x;
        last_y = y;
        last_tick = now_tick;
        has_last = 1U;
        return;
    }

    if (now_tick > last_tick) {
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

    if ((buf == 0) || (x == 0) || (y == 0) || (len < SHIJUE_FRAME_SIZE)) {
        return 0U;
    }

    for (i = 0; i <= (uint16_t)(len - SHIJUE_FRAME_SIZE); i++) {
        if (shijue_parse_frame_at(buf, i)) {
            *x = g_shijue_x;
            *y = g_shijue_y;
            return 1U;
        }
    }

    return 0U;
}

void Shijue_ProcessRxBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len < SHIJUE_FRAME_SIZE)) {
        return;
    }

    for (i = 0; i <= (uint16_t)(len - SHIJUE_FRAME_SIZE); i++) {
        if (shijue_parse_frame_at(buf, i)) {
            g_shijue_error_flag = 0U;
            shijue_update_speed(g_shijue_x, g_shijue_y);
        }
    }
}
