#include "state.h"
#include "main.h"
#include "tim.h"
#include "shijue.h"
#include "oled.h"

#define HOLD_RADIUS_UNIT    12.0f
#define STATE5_HOLD_MS      3000U
#define STATE6_HOLD_MS      4000U
#define STATE8_HOLD_MS      1000U
#define ROLL_VEL_DT_S       0.033f
#define ROLL_POS_DT_S       0.066f
#define STATE_TARGET9_INSET 8.0f
#define STATE_TARGET137_INSET 6.0f
#define STATE_PRE_TARGET_RADIUS 16.0f
#define STATE_PRE_TARGET_HOLD_MS 1000U
#define STATE_PRE_TARGET_1_X 72.912f
#define STATE_PRE_TARGET_1_Y 72.912f
#define STATE_PRE_TARGET_3_X 245.461f
#define STATE_PRE_TARGET_3_Y 73.282f
#define STATE_PRE_TARGET_7_X 73.282f
#define STATE_PRE_TARGET_7_Y 245.461f
#define STATE_PRE_TARGET_9_X 239.088f
#define STATE_PRE_TARGET_9_Y 239.088f
float g_roll_target_x = 0.0f;
float g_roll_target_y = 0.0f;

static void RollCtrl_UpdatePos(float measure_x, float measure_y, float dt_s)
{
    (void)measure_x;
    (void)measure_y;
    (void)dt_s;
}

static void RollCtrl_UpdateVel(float measure_vx, float measure_vy, float dt_s)
{
    (void)measure_vx;
    (void)measure_vy;
    (void)dt_s;
}

static void RollCtrl_UpdateAngleOutput_duoji(float measure_angle_x, float measure_angle_y, float dt_s)
{
    (void)measure_angle_x;
    (void)measure_angle_y;
    (void)dt_s;
}
extern uint8_t rxBuffer2[256];
extern volatile uint16_t g_rx2_size;
extern volatile uint8_t g_roll_flag_vel;
extern volatile uint8_t g_roll_flag_pos;
uint8_t place[20] = {0};
uint8_t place_len = 0U;
uint32_t g_state9_elapsed_ms = 0U;
static uint8_t g_pre_target_valid = 0U;
static uint8_t g_pre_target_point = 0U;
static uint8_t g_pre_target_hold_started = 0U;
static uint32_t g_pre_target_hold_tick = 0U;
static uint8_t g_place_prev[20] = {0xFFU};
static uint8_t g_place_prev_len = 0xFFU;
uint8_t g_state = 0x00;
static void State_RunPidUpdate(void);
static uint8_t State_TryGetAsciiTargetPoint(uint8_t *point);
static uint8_t State_IsPlaceChanged(void);
static void State_SavePlaceSnapshot(void);
static void State_PreTargetReset(void);
static void State_SetTargetDirect(uint8_t target_point);
static uint8_t State_GetDirectTargetCoord(uint8_t target_point, float *x, float *y);
static uint8_t State_IsPreTargetPoint(uint8_t point);
static uint8_t State_GetPreTargetCoord(uint8_t point, float *x, float *y);
static uint8_t State_GetCurrentCornerPoint(uint8_t *point);

static uint8_t State_Normalize(uint8_t state)
{
    if ((state >= 0x01U) && (state <= 0x0BU))
    {
        return state;
    }
    return 0x00U;
}

static uint8_t State_DecodePlaceDigit(uint8_t c)
{
    if ((c >= 0x31U) && (c <= 0x39U))
    {
        return (uint8_t)(c - 0x30U);
    }
    return 0U;
}

static uint8_t State_IsPlaceFrame(void)
{
    uint8_t i;
    uint8_t n;

    if (g_rx2_size == 0U)
    {
        return 0U;
    }
    n = (g_rx2_size > 20U) ? 20U : (uint8_t)g_rx2_size;
    for (i = 0U; i < n; i++)
    {
        if ((rxBuffer2[i] < 0x31U) || (rxBuffer2[i] > 0x39U))
        {
            return 0U;
        }
    }
    return 1U;
}

static void State_UpdatePlaceFromRxBuffer2(void)
{
    uint8_t i;
    uint8_t n = (g_rx2_size > 20U) ? 20U : (uint8_t)g_rx2_size;

    for (i = 0U; i < 20U; i++)
    {
        place[i] = 0U;
    }

    for (i = 0U; i < n; i++)
    {
        place[i] = State_DecodePlaceDigit(rxBuffer2[i]);
    }
    place_len = n;
}

static void State_PreTargetReset(void)
{
    g_pre_target_valid = 0U;
    g_pre_target_point = 0U;
    g_pre_target_hold_started = 0U;
    g_pre_target_hold_tick = 0U;
}

static uint8_t State_GetDirectTargetCoord(uint8_t target_point, float *x, float *y)
{
    if ((x == 0) || (y == 0))
    {
        return 0U;
    }
    if ((target_point < 1U) || (target_point > 9U))
    {
        return 0U;
    }

    *x = (float)g_shijue_centers[target_point - 1U].x;
    *y = (float)g_shijue_centers[target_point - 1U].y;
    if (target_point == 1U)
    {
        *x += STATE_TARGET137_INSET;
        *y += STATE_TARGET137_INSET;
    }
    else if (target_point == 3U)
    {
        *x -= STATE_TARGET137_INSET;
        *y += STATE_TARGET137_INSET;
    }
    else if (target_point == 7U)
    {
        *x += STATE_TARGET137_INSET;
        *y -= STATE_TARGET137_INSET;
    }
    else if (target_point == 9U)
    {
        *x += STATE_TARGET9_INSET;
        *y += STATE_TARGET9_INSET;
    }
    return 1U;
}

static uint8_t State_IsPreTargetPoint(uint8_t point)
{
    if ((point == 1U) || (point == 3U) || (point == 7U) || (point == 9U))
    {
        return 1U;
    }
    return 0U;
}

static uint8_t State_GetPreTargetCoord(uint8_t point, float *x, float *y)
{
    if ((x == 0) || (y == 0))
    {
        return 0U;
    }

    switch (point)
    {
    case 1U:
        *x = STATE_PRE_TARGET_1_X;
        *y = STATE_PRE_TARGET_1_Y;
        return 1U;
    case 3U:
        *x = STATE_PRE_TARGET_3_X;
        *y = STATE_PRE_TARGET_3_Y;
        return 1U;
    case 7U:
        *x = STATE_PRE_TARGET_7_X;
        *y = STATE_PRE_TARGET_7_Y;
        return 1U;
    case 9U:
        *x = STATE_PRE_TARGET_9_X;
        *y = STATE_PRE_TARGET_9_Y;
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t State_GetCurrentCornerPoint(uint8_t *point)
{
    const uint8_t corners[4] = {1U, 3U, 7U, 9U};
    const float detect_r2 = HOLD_RADIUS_UNIT * HOLD_RADIUS_UNIT;
    uint8_t i;

    if (point == 0)
    {
        return 0U;
    }

    for (i = 0U; i < 4U; i++)
    {
        uint8_t p = corners[i];
        float cx = (float)g_shijue_centers[p - 1U].x;
        float cy = (float)g_shijue_centers[p - 1U].y;
        float dx = (float)g_shijue_x - cx;
        float dy = (float)g_shijue_y - cy;
        float d2 = dx * dx + dy * dy;
        if (d2 <= detect_r2)
        {
            *point = p;
            return 1U;
        }
    }
    return 0U;
}

static void State_SetTargetDirect(uint8_t target_point)
{
    float final_x;
    float final_y;
    float pre_x;
    float pre_y;
    float dx;
    float dy;
    float dist2;
    uint32_t now;
    const float pre_r2 = STATE_PRE_TARGET_RADIUS * STATE_PRE_TARGET_RADIUS;
    uint8_t start_corner = 0U;

    if (!State_GetDirectTargetCoord(target_point, &final_x, &final_y))
    {
        return;
    }

    if (!g_pre_target_valid)
    {
        if (State_GetCurrentCornerPoint(&start_corner) &&
            State_IsPreTargetPoint(start_corner) &&
            (start_corner != target_point))
        {
            g_pre_target_valid = 1U;
            g_pre_target_point = start_corner;
            g_pre_target_hold_started = 0U;
            g_pre_target_hold_tick = 0U;
        }
    }

    if (g_pre_target_valid && State_GetPreTargetCoord(g_pre_target_point, &pre_x, &pre_y))
    {
        dx = (float)g_shijue_x - pre_x;
        dy = (float)g_shijue_y - pre_y;
        dist2 = dx * dx + dy * dy;

        if (dist2 <= pre_r2)
        {
            now = HAL_GetTick();
            if (!g_pre_target_hold_started)
            {
                g_pre_target_hold_started = 1U;
                g_pre_target_hold_tick = now;
            }
            else if ((uint32_t)(now - g_pre_target_hold_tick) >= STATE_PRE_TARGET_HOLD_MS)
            {
                g_pre_target_valid = 0U;
                g_pre_target_hold_started = 0U;
                g_pre_target_hold_tick = 0U;
            }
        }
        else
        {
            g_pre_target_hold_started = 0U;
            g_pre_target_hold_tick = 0U;
        }
    }

    if (g_pre_target_valid && State_GetPreTargetCoord(g_pre_target_point, &pre_x, &pre_y))
    {
        g_roll_target_x = pre_x;
        g_roll_target_y = pre_y;
    }
    else
    {
        g_roll_target_x = final_x;
        g_roll_target_y = final_y;
    }
}

void State_Init(void)
{
    {
        uint8_t i;
        for (i = 0U; i < 20U; i++)
        {
            place[i] = 0U;
            g_place_prev[i] = 0xFFU;
        }
    }
    place_len = 0U;
    State_PreTargetReset();
    g_place_prev_len = 0xFFU;

    g_state = 0x00U;
    g_state9_elapsed_ms = 0U;
}

void State_Set(uint8_t state)
{
    g_state = State_Normalize(state);
}

uint8_t State_Get(void)
{
    return g_state;
}

void State_UpdateFromRxBuffer2(void)
{
    uint8_t raw = rxBuffer2[0];
    uint8_t current = State_Get();

    if (((current == 0x09U) || (current == 0x08U) || (current == 0x06U) || (current == 0x05U)) && State_IsPlaceFrame())
    {
        return;
    }
    if (((current == 0x01U) || (current == 0x04U)) &&
        (g_rx2_size > 0U) &&
        (rxBuffer2[0] >= 0x31U) && (rxBuffer2[0] <= 0x39U))
    {
        /* state1/state4 下，单字节 ASCII 数字作为目标点输入，不当成状态切换 */
        return;
    }

    if ((raw >= 0x01U) && (raw <= 0x0BU))
    {
        State_Set(raw);
    }
    else
    {
        State_Set(0x00U);
    }
}

void State_RunCurrent(void)
{
    State_UpdateFromRxBuffer2();

    switch (g_state)
    {
    case 0x01:
    {
        uint8_t target_point = 9U;
        uint8_t has_target = 0U;
        State_PreTargetReset();
        while (g_state == 0x01U)
        {
            uint8_t new_point;
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(1, 1, g_shijue_x  , 3);
            OLED_ShowNum(2, 1, g_shijue_y    , 3);

            if (State_TryGetAsciiTargetPoint(&new_point))
            {
                if (target_point != new_point)
                {
                    target_point = new_point;
                    State_PreTargetReset();
                }
                has_target = 1U;
            }

            if (!has_target)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x01U) break;
                HAL_Delay(1);
                continue;
            }
            State_SetTargetDirect(target_point);

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x01U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x02:
        State_PreTargetReset();
        while (g_state == 0x02U)
        {
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(1, 1, g_shijue_x  , 3);
            OLED_ShowNum(2, 1, g_shijue_y    , 3);
            State_SetTargetDirect(5U);

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x02U) break;
            HAL_Delay(1);
        }
        break;

    case 0x03:
        while (g_state == 0x03U)
        {
            State_UpdateFromRxBuffer2();
            if (g_state != 0x03U) break;
            HAL_Delay(1);
        }
        break;

    case 0x04:
    {
        uint8_t target_point = 9U;
        uint8_t has_target = 1U;
        State_PreTargetReset();

        while (g_state == 0x04U)
        {
            uint8_t new_point;
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);

            if (State_TryGetAsciiTargetPoint(&new_point))
            {
                if (target_point != new_point)
                {
                    target_point = new_point;
                    State_PreTargetReset();
                }
                has_target = 1U;
            }

            if (!has_target)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x04U) break;
                HAL_Delay(1);
                continue;
            }
            State_SetTargetDirect(target_point);

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x04U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x05:
    {
        uint8_t seq_ready = 0U;
        uint8_t seq_idx = 0U;
        uint8_t hold_started = 0U;
        uint32_t hold_tick = 0U;
        const float r2 = HOLD_RADIUS_UNIT * HOLD_RADIUS_UNIT;

        State_PreTargetReset();
        while (g_state == 0x05U)
        {
            float target_x = (float)g_shijue_centers[4].x;
            float target_y = (float)g_shijue_centers[4].y;
            float dx;
            float dy;
            float dist2;
            uint32_t now;
            uint32_t hold_ms_show;
            uint8_t p;

            hold_ms_show = 0U;
            if (hold_started)
            {
                hold_ms_show = (uint32_t)(HAL_GetTick() - hold_tick);
                if (hold_ms_show > STATE5_HOLD_MS)
                {
                    hold_ms_show = STATE5_HOLD_MS;
                }
            }
            OLED_ShowNum(4,11, hold_ms_show,4);

            if (State_IsPlaceFrame())
            {
                State_UpdatePlaceFromRxBuffer2();
                if ((place_len >= 3U) && State_IsPlaceChanged())
                {
                    seq_ready = 1U;
                    seq_idx = 0U;
                    hold_started = 0U;
                    State_PreTargetReset();
                    State_SavePlaceSnapshot();
                }
            }

            if (!seq_ready)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x05U) break;
                HAL_Delay(1);
                continue;
            }

            if (seq_ready)
            {
                p = place[seq_idx];
                if ((p >= 1U) && (p <= 9U))
                {
                    State_GetDirectTargetCoord(p, &target_x, &target_y);
                }
                else
                {
                    seq_ready = 0U;
                    hold_started = 0U;
                    State_PreTargetReset();
                }
            }

            if (seq_ready)
            {
                State_SetTargetDirect(p);
            }

            if (seq_ready)
            {
                dx = (float)g_shijue_x - target_x;
                dy = (float)g_shijue_y - target_y;
                dist2 = dx * dx + dy * dy;

                if (dist2 <= r2)
                {
                    now = HAL_GetTick();
                    if (!hold_started)
                    {
                        hold_started = 1U;
                        hold_tick = now;
                    }
                    else if ((uint32_t)(now - hold_tick) >= STATE5_HOLD_MS)
                    {
                        if ((seq_idx + 1U) < place_len)
                        {
                            seq_idx++;
                            State_PreTargetReset();
                        }
                        hold_started = 0U;
                    }
                }
                else
                {
                    hold_started = 0U;
                }
            }

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x05U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x06:
    {
        uint8_t seq_ready = 0U;
        uint8_t point_a = 1U;
        uint8_t point_b = 9U;
        uint8_t target_is_a = 1U;
        uint8_t switches_done = 0U; /* 两次循环共4次切换 */
        uint8_t hold_started = 0U;
        uint32_t hold_tick = 0U;
        const float r2 = HOLD_RADIUS_UNIT * HOLD_RADIUS_UNIT;

        State_PreTargetReset();
        while (g_state == 0x06U)
        {
            uint8_t target_point = point_a;
            float target_x;
            float target_y;
            float dx;
            float dy;
            float dist2;
            uint32_t now;

            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);

            if (State_IsPlaceFrame())
            {
                State_UpdatePlaceFromRxBuffer2();
                if ((place_len >= 2U) && State_IsPlaceChanged())
                {
                    if ((place[0] >= 1U) && (place[0] <= 9U) &&
                        (place[1] >= 1U) && (place[1] <= 9U))
                    {
                        point_a = place[0];
                        point_b = place[1];
                        seq_ready = 1U;
                        target_is_a = 1U;
                        switches_done = 0U;
                        hold_started = 0U;
                        State_PreTargetReset();
                    }
                    State_SavePlaceSnapshot();
                }
            }

            if (!seq_ready)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x06U) break;
                HAL_Delay(1);
                continue;
            }

            if (seq_ready)
            {
                target_point = target_is_a ? point_a : point_b;
            }
            State_GetDirectTargetCoord(target_point, &target_x, &target_y);
            State_SetTargetDirect(target_point);

            if (seq_ready)
            {
                dx = (float)g_shijue_x - target_x;
                dy = (float)g_shijue_y - target_y;
                dist2 = dx * dx + dy * dy;

                if (dist2 <= r2)
                {
                    now = HAL_GetTick();
                    if (!hold_started)
                    {
                        hold_started = 1U;
                        hold_tick = now;
                    }
                    else if ((uint32_t)(now - hold_tick) >= STATE6_HOLD_MS)
                    {
                        if ((point_a != point_b) && (switches_done < 4U))
                        {
                            target_is_a = target_is_a ? 0U : 1U;
                            switches_done++;
                            State_PreTargetReset();
                        }
                        hold_started = 0U;
                    }
                }
                else
                {
                    hold_started = 0U;
                }
            }

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x06U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x07:
        while (g_state == 0x07U)
        {
            State_UpdateFromRxBuffer2();
            if (g_state != 0x07U) break;
            HAL_Delay(1);
        }
        break;

    case 0x08:
    {
        uint8_t seq_ready = 0U;
        uint8_t seq_idx = 0U;
        uint8_t hold_started = 0U;
        uint32_t hold_tick = 0U;
        const float r2 = HOLD_RADIUS_UNIT * HOLD_RADIUS_UNIT;

        State_PreTargetReset();
        while (g_state == 0x08U)
        {
            float target_x = (float)g_shijue_centers[4].x;
            float target_y = (float)g_shijue_centers[4].y;
            float dx;
            float dy;
            float dist2;
            uint32_t now;
            uint8_t p;

            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);

            if (State_IsPlaceFrame())
            {
                State_UpdatePlaceFromRxBuffer2();
                if ((place_len >= 1U) && State_IsPlaceChanged())
                {
                    seq_ready = 1U;
                    seq_idx = 0U;
                    hold_started = 0U;
                    State_PreTargetReset();
                    State_SavePlaceSnapshot();
                }
            }

            if (!seq_ready)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x08U) break;
                HAL_Delay(1);
                continue;
            }

            if (seq_ready)
            {
                p = place[seq_idx];
                if ((p >= 1U) && (p <= 9U))
                {
                    State_GetDirectTargetCoord(p, &target_x, &target_y);
                }
                else
                {
                    seq_ready = 0U;
                    hold_started = 0U;
                    State_PreTargetReset();
                }
            }

            if (seq_ready)
            {
                State_SetTargetDirect(p);
            }

            if (seq_ready)
            {
                dx = (float)g_shijue_x - target_x;
                dy = (float)g_shijue_y - target_y;
                dist2 = dx * dx + dy * dy;

                if (dist2 <= r2)
                {
                    now = HAL_GetTick();
                    if (!hold_started)
                    {
                        hold_started = 1U;
                        hold_tick = now;
                    }
                    else if ((uint32_t)(now - hold_tick) >= STATE8_HOLD_MS)
                    {
                        if ((seq_idx + 1U) < place_len)
                        {
                            seq_idx++;
                            State_PreTargetReset();
                        }
                        hold_started = 0U;
                    }
                }
                else
                {
                    hold_started = 0U;
                }
            }

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x08U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x09:
    {
        uint8_t seq_ready = 0U;
        uint8_t seq_idx = 0U;
        uint8_t hold_started = 0U;
        uint8_t elapsed_latched = 0U;
        uint32_t hold_tick = 0U;
        uint32_t start_tick = 0U;
        const float r2 = HOLD_RADIUS_UNIT * HOLD_RADIUS_UNIT;

        State_PreTargetReset();
        while (g_state == 0x09U)
        {
            float target_x = (float)g_shijue_centers[4].x;
            float target_y = (float)g_shijue_centers[4].y;
            float dx;
            float dy;
            float dist2;
            uint32_t now;
            uint8_t p;

            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);
            OLED_ShowNum(4,11, g_state,2);

            if (State_IsPlaceFrame())
            {
                State_UpdatePlaceFromRxBuffer2();
                if ((place_len >= 1U) && State_IsPlaceChanged())
                {
                    seq_ready = 1U;
                    seq_idx = 0U;
                    hold_started = 0U;
                    elapsed_latched = 0U;
                    start_tick = HAL_GetTick();
                    g_state9_elapsed_ms = 0U;
                    State_PreTargetReset();
                    State_SavePlaceSnapshot();
                }
            }

            if (!seq_ready)
            {
                (void)0;
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
                State_UpdateFromRxBuffer2();
                if (g_state != 0x09U) break;
                HAL_Delay(1);
                continue;
            }

            if (seq_ready)
            {
                p = place[seq_idx];
                if ((p >= 1U) && (p <= 9U))
                {
                    State_GetDirectTargetCoord(p, &target_x, &target_y);
                }
                else
                {
                    seq_ready = 0U;
                    hold_started = 0U;
                    State_PreTargetReset();
                }
            }

            if (seq_ready)
            {
                State_SetTargetDirect(p);
            }

            if (seq_ready)
            {
                dx = (float)g_shijue_x - target_x;
                dy = (float)g_shijue_y - target_y;
                dist2 = dx * dx + dy * dy;

                if (dist2 <= r2)
                {
                    now = HAL_GetTick();
                    if (!hold_started)
                    {
                        hold_started = 1U;
                        hold_tick = now;
                    }
                    else if ((uint32_t)(now - hold_tick) >= STATE8_HOLD_MS)
                    {
                        if ((seq_idx + 1U) < place_len)
                        {
                            seq_idx++;
                            State_PreTargetReset();
                        }
                        else if (!elapsed_latched)
                        {
                            g_state9_elapsed_ms = (uint32_t)(now - start_tick);
                            elapsed_latched = 1U;
                        }
                        hold_started = 0U;
                    }
                }
                else
                {
                    hold_started = 0U;
                }
            }

            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x09U) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x0A:
    {
        uint8_t step = 0U;
        uint32_t step_tick = HAL_GetTick();
        const int32_t amp = 50;
        const uint32_t step_ms = 50U;

        while (g_state == 0x0AU)
        {
            uint32_t now = HAL_GetTick();
            int32_t out_x = 1350;
            int32_t out_y = 1400;

            if ((uint32_t)(now - step_tick) >= step_ms)
            {
                step = (uint8_t)((step + 1U) & 0x03U);
                step_tick = now;
            }

            if (step == 0U)
            {
                out_y += amp; /* y抬升 */
            }
            else if (step == 1U)
            {
                out_x += amp; /* x抬升 */
            }
            else if (step == 2U)
            {
                out_y -= amp; /* y下降 */
            }
            else
            {
                out_x -= amp; /* x下降 */
            }

            (void)0;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)out_y);
            OLED_ShowHexNum(4,11, g_state,2);
            State_UpdateFromRxBuffer2();
            if (g_state != 0x0AU) break;
            HAL_Delay(1);
        }
        break;
    }

    case 0x0B:
        while (g_state == 0x0BU)
        {
            OLED_ShowHexNum(4,11, g_state,2);
            State_UpdateFromRxBuffer2();
            if (g_state != 0x0BU) break;
            (void)0;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
            HAL_Delay(1);
        }
        break;

    default:
        while (g_state == 0x00U)
        {
            OLED_ShowHexNum(4,11, g_state,2);
            State_RunPidUpdate();
            State_UpdateFromRxBuffer2();
            if (g_state != 0x00U) break;
            (void)0;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1400);
            HAL_Delay(1);
        }
        break;
    }
}

static void State_RunPidUpdate(void)
{
    if (g_roll_flag_pos)
    {
        g_roll_flag_pos = 0U;
        RollCtrl_UpdatePos((float)g_shijue_x, (float)g_shijue_y, ROLL_POS_DT_S);
    }

    if (g_roll_flag_vel)
    {
        g_roll_flag_vel = 0U;
        RollCtrl_UpdateVel(g_shijue_vx, g_shijue_vy, ROLL_VEL_DT_S);
        RollCtrl_UpdateAngleOutput_duoji(0.0f, 0.0f, ROLL_VEL_DT_S);
    }
}

static uint8_t State_TryGetAsciiTargetPoint(uint8_t *point)
{
    uint8_t raw;
    if (point == 0)
    {
        return 0U;
    }
    if (g_rx2_size == 0U)
    {
        return 0U;
    }
    raw = rxBuffer2[0];
    if ((raw >= 0x31U) && (raw <= 0x39U))
    {
        *point = (uint8_t)(raw - 0x30U);
        return 1U;
    }
    return 0U;
}

static uint8_t State_IsPlaceChanged(void)
{
    uint8_t i;
    if (place_len != g_place_prev_len)
    {
        return 1U;
    }
    for (i = 0U; i < place_len; i++)
    {
        if (place[i] != g_place_prev[i])
        {
            return 1U;
        }
    }
    return 0U;
}

static void State_SavePlaceSnapshot(void)
{
    uint8_t i;
    g_place_prev_len = place_len;
    for (i = 0U; i < 20U; i++)
    {
        g_place_prev[i] = place[i];
    }
}
