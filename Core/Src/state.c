#include "state.h"

#include "OLED.h"
#include "bsp_key.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "gray.h"
#include "moter.h"
#include "shijue.h"

#include <stdio.h>
#include <string.h>

#define H2_BASE_PWM           350
#define H2_TRACK_KP           8
#define H2_REVERSE_LEFT_PWM   850
#define H2_REVERSE_RIGHT_PWM  1000
#define H2_REVERSE_MS         250U
#define H2_FIRST_STRAIGHT_MS  100U
#define H2_MARKER_BLACK_COUNT 4U

typedef enum
{
    STATE_MODE_NONE = 0,
    STATE_MODE_H2_CAR_LOOP = 2,
    STATE_MODE_H3_BALL_MOVE = 3,
    STATE_MODE_H4_AB_BALANCE = 4,
    STATE_MODE_H5_LOOP_CENTER = 5,
    STATE_MODE_H6_LOOP_TARGET = 6
} StateMode_t;

typedef enum
{
    STATE_PAGE_SELECT = 0,
    STATE_PAGE_TARGET_SET,
    STATE_PAGE_READY,
    STATE_PAGE_RUNNING,
    STATE_PAGE_FINISHED,
    STATE_PAGE_STOPPED,
    STATE_PAGE_TIMEOUT
} StatePage_t;

static StateMode_t current_mode;
static StatePage_t current_page;
static uint8_t selected_item;
static int16_t target_tenth_cm;
static uint32_t start_time_ms;
static uint32_t stopped_elapsed_ms;

static uint8_t h2_marker_count;
static uint8_t h2_marker_active;
static uint8_t h2_straight_active;
static uint8_t h2_reverse_active;
static uint8_t h2_stop_locked;
static uint32_t h2_straight_start_ms;
static uint32_t h2_reverse_start_ms;

static uint8_t display_dirty;
static StatePage_t last_display_page;
static uint32_t last_display_half_second;
static char display_cache[4][16];

static uint8_t vision_debug_buffer[128];
static uint32_t vision_debug_time_ms;

static const char *State_GetModeName(StateMode_t mode)
{
    switch (mode)
    {
    case STATE_MODE_H2_CAR_LOOP:    return "H2 CAR LOOP";
    case STATE_MODE_H3_BALL_MOVE:   return "H3 BALL MOVE";
    case STATE_MODE_H4_AB_BALANCE:  return "H4 AB BALANCE";
    case STATE_MODE_H5_LOOP_CENTER: return "H5 LOOP CENTER";
    case STATE_MODE_H6_LOOP_TARGET: return "H6 LOOP TARGET";
    default:                        return "SELECT H ITEM";
    }
}

static uint32_t State_GetElapsedHalfSeconds(uint32_t now_ms)
{
    uint32_t elapsed_ms = stopped_elapsed_ms;

    if (current_page == STATE_PAGE_RUNNING)
    {
        elapsed_ms = now_ms - start_time_ms;
    }
    return elapsed_ms / 500U;
}

static void State_ShowLine(uint8_t line, const char *text)
{
    char output[17];
    size_t length = strlen(text);
    uint8_t column;

    if (length > 16U) length = 16U;
    memset(output, ' ', 16U);
    memcpy(output, text, length);
    output[16] = '\0';

    for (column = 0U; column < 16U; column++)
    {
        if (display_cache[line - 1U][column] != output[column])
        {
            OLED_ShowChar(line, column + 1U, output[column]);
            display_cache[line - 1U][column] = output[column];
        }
    }
}

static void State_FormatTime(char *output, size_t size, uint32_t half_seconds)
{
    (void)snprintf(output,
                   size,
                   "TIME:%03lu.%cS",
                   (unsigned long)(half_seconds / 2U),
                   ((half_seconds & 1U) != 0U) ? '5' : '0');
}

static void State_FormatTarget(char *output, size_t size)
{
    uint16_t magnitude = (target_tenth_cm < 0)
                       ? (uint16_t)(-target_tenth_cm)
                       : (uint16_t)target_tenth_cm;

    (void)snprintf(output,
                   size,
                   "TARGET:%c%02u.%uCM",
                   (target_tenth_cm < 0) ? '-' : '+',
                   magnitude / 10U,
                   magnitude % 10U);
}

static void State_RenderSelect(void)
{
    uint8_t first_item = selected_item - 1U;
    uint8_t row;
    char line[17];

    if (first_item < 2U) first_item = 2U;
    if (first_item > 4U) first_item = 4U;

    State_ShowLine(1U, "SELECT H ITEM");
    for (row = 0U; row < 3U; row++)
    {
        uint8_t item = first_item + row;
        (void)snprintf(line,
                       sizeof(line),
                       "%c%s",
                       (item == selected_item) ? '>' : ' ',
                       State_GetModeName((StateMode_t)item));
        State_ShowLine(row + 2U, line);
    }
}

static void State_RenderReady(void)
{
    char target[17];

    State_ShowLine(1U, State_GetModeName(current_mode));
    if (current_mode == STATE_MODE_H3_BALL_MOVE)
    {
        State_ShowLine(2U, "0>+5>-5CM");
    }
    else if ((current_mode == STATE_MODE_H4_AB_BALANCE) ||
             (current_mode == STATE_MODE_H5_LOOP_CENTER))
    {
        State_ShowLine(2U, "TARGET:CENTER");
    }
    else if (current_mode == STATE_MODE_H6_LOOP_TARGET)
    {
        State_FormatTarget(target, sizeof(target));
        State_ShowLine(2U, target);
    }
    else
    {
        State_ShowLine(2U, "STATE:READY");
    }
    State_ShowLine(3U, "K3 START");
    State_ShowLine(4U,
                   (current_mode == STATE_MODE_H6_LOOP_TARGET)
                   ? "K4 BACK" : "K4 MENU");
}

static void State_RenderStatus(uint32_t now_ms)
{
    char time_text[17];
    const char *state_text;

    switch (current_page)
    {
    case STATE_PAGE_RUNNING:  state_text = "STATE:RUNNING";  break;
    case STATE_PAGE_FINISHED: state_text = "STATE:FINISHED"; break;
    case STATE_PAGE_TIMEOUT:  state_text = "STATE:TIMEOUT";  break;
    default:                  state_text = "STATE:STOPPED";  break;
    }

    State_FormatTime(time_text,
                     sizeof(time_text),
                     State_GetElapsedHalfSeconds(now_ms));
    State_ShowLine(1U, State_GetModeName(current_mode));
    State_ShowLine(2U, state_text);
    State_ShowLine(3U, time_text);
    State_ShowLine(4U,
                   (current_page == STATE_PAGE_RUNNING)
                   ? "K4 STOP" : "K4 BACK");
}

static void State_Render(uint32_t now_ms)
{
    switch (current_page)
    {
    case STATE_PAGE_SELECT:
        State_RenderSelect();
        break;

    case STATE_PAGE_TARGET_SET:
    {
        char target[17];
        State_FormatTarget(target, sizeof(target));
        State_ShowLine(1U, "H6 LOOP TARGET");
        State_ShowLine(2U, target);
        State_ShowLine(3U, "K1+ K2-");
        State_ShowLine(4U, "K3 OK K4 MENU");
        break;
    }

    case STATE_PAGE_READY:
        State_RenderReady();
        break;

    default:
        State_RenderStatus(now_ms);
        break;
    }
    display_dirty = 0U;
}

static void State_H2Reset(void)
{
    h2_marker_count = 0U;
    h2_marker_active = 0U;
    h2_straight_active = 0U;
    h2_reverse_active = 0U;
    h2_stop_locked = 0U;
    h2_straight_start_ms = 0U;
    h2_reverse_start_ms = 0U;
}

static void State_H2Brake(void)
{
    Moter_A_Brake();
    Moter_B_Brake();
}

static uint8_t State_H2Run(uint32_t now_ms)
{
    uint8_t gray = Gray_Read();
    uint8_t black_count = 0U;
    int16_t turn = Gray_GetError() * H2_TRACK_KP;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((gray & (uint8_t)(1U << i)) == 0U)
        {
            black_count++;
        }
    }

    if (h2_stop_locked != 0U)
    {
        State_H2Brake();
        return 1U;
    }

    if (h2_reverse_active != 0U)
    {
        if ((uint32_t)(now_ms - h2_reverse_start_ms) < H2_REVERSE_MS)
        {
            Moter_A(H2_REVERSE_RIGHT_PWM);
            Moter_B(H2_REVERSE_LEFT_PWM);
            return 0U;
        }

        h2_reverse_active = 0U;
        h2_stop_locked = 1U;
        State_H2Brake();
        return 1U;
    }

    if ((black_count >= H2_MARKER_BLACK_COUNT) &&
        (h2_marker_active == 0U))
    {
        h2_marker_active = 1U;
        h2_marker_count++;

        if (h2_marker_count == 1U)
        {
            h2_straight_active = 1U;
            h2_straight_start_ms = now_ms;
        }
        else
        {
            h2_reverse_active = 1U;
            h2_reverse_start_ms = now_ms;
            Moter_A(H2_REVERSE_RIGHT_PWM);
            Moter_B(H2_REVERSE_LEFT_PWM);
            return 0U;
        }
    }
    else if (black_count < H2_MARKER_BLACK_COUNT)
    {
        h2_marker_active = 0U;
    }

    if (h2_straight_active != 0U)
    {
        if ((uint32_t)(now_ms - h2_straight_start_ms) < H2_FIRST_STRAIGHT_MS)
        {
            Moter_A(-H2_BASE_PWM);
            Moter_B(-H2_BASE_PWM);
            return 0U;
        }
        h2_straight_active = 0U;
    }

    /* 左轮B/TIM3，右轮A/TIM4；负参数前进，正参数后退。 */
    Moter_A(-H2_BASE_PWM - turn);
    Moter_B(-H2_BASE_PWM + turn);
    return 0U;
}

static void State_End(StatePage_t page, uint32_t now_ms)
{
    if (current_page == STATE_PAGE_RUNNING)
    {
        stopped_elapsed_ms = now_ms - start_time_ms;
        current_page = page;
        display_dirty = 1U;
    }
}

static void State_Start(uint32_t now_ms)
{
    if (current_page != STATE_PAGE_READY)
    {
        return;
    }

    start_time_ms = now_ms;
    stopped_elapsed_ms = 0U;
    current_page = STATE_PAGE_RUNNING;
    last_display_half_second = 0xFFFFFFFFU;

    if (current_mode == STATE_MODE_H2_CAR_LOOP)
    {
        State_H2Reset();
    }
    display_dirty = 1U;
}

static void State_SetTarget(int16_t new_target_tenth_cm)
{
    if (new_target_tenth_cm > 125) new_target_tenth_cm = 125;
    if (new_target_tenth_cm < -125) new_target_tenth_cm = -125;
    target_tenth_cm = new_target_tenth_cm;
}

static void State_HandleKey(KeyEvent_t key, uint32_t now_ms)
{
    switch (current_page)
    {
    case STATE_PAGE_SELECT:
        if (key == KEY_EVENT_1)
        {
            selected_item = (selected_item <= 2U) ? 6U : selected_item - 1U;
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_2)
        {
            selected_item = (selected_item >= 6U) ? 2U : selected_item + 1U;
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_3)
        {
            current_mode = (StateMode_t)selected_item;
            stopped_elapsed_ms = 0U;
            current_page = (current_mode == STATE_MODE_H6_LOOP_TARGET)
                         ? STATE_PAGE_TARGET_SET
                         : STATE_PAGE_READY;
            display_dirty = 1U;
        }
        break;

    case STATE_PAGE_TARGET_SET:
        if (key == KEY_EVENT_1)
        {
            State_SetTarget(target_tenth_cm + 1);
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_2)
        {
            State_SetTarget(target_tenth_cm - 1);
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_3)
        {
            current_page = STATE_PAGE_READY;
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_4)
        {
            current_mode = STATE_MODE_NONE;
            current_page = STATE_PAGE_SELECT;
            display_dirty = 1U;
        }
        break;

    case STATE_PAGE_READY:
        if (key == KEY_EVENT_3)
        {
            State_Start(now_ms);
        }
        else if (key == KEY_EVENT_4)
        {
            if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                current_page = STATE_PAGE_TARGET_SET;
            }
            else
            {
                current_mode = STATE_MODE_NONE;
                current_page = STATE_PAGE_SELECT;
            }
            display_dirty = 1U;
        }
        break;

    case STATE_PAGE_RUNNING:
        if (key == KEY_EVENT_4)
        {
            State_End(STATE_PAGE_STOPPED, now_ms);
            if (current_mode == STATE_MODE_H2_CAR_LOOP)
            {
                State_H2Brake();
            }
        }
        break;

    case STATE_PAGE_FINISHED:
    case STATE_PAGE_STOPPED:
    case STATE_PAGE_TIMEOUT:
        if (key == KEY_EVENT_4)
        {
            stopped_elapsed_ms = 0U;
            current_page = STATE_PAGE_READY;
            display_dirty = 1U;
        }
        break;

    default:
        break;
    }
}

static void State_RunMode(uint32_t now_ms)
{
    switch (current_mode)
    {
    case STATE_MODE_H2_CAR_LOOP:
        if (State_H2Run(now_ms) != 0U)
        {
            State_End(STATE_PAGE_FINISHED, now_ms);
        }
        break;

    case STATE_MODE_H3_BALL_MOVE:
    case STATE_MODE_H4_AB_BALANCE:
    case STATE_MODE_H5_LOOP_CENTER:
    case STATE_MODE_H6_LOOP_TARGET:
        /* H3-H6 先保留空实现。 */
        break;

    default:
        break;
    }
}

static int32_t State_FloatToCenti(float value)
{
    return (int32_t)(value * 100.0f);
}

static void State_SendVisionDebug(uint32_t now_ms)
{
    int32_t origin = State_FloatToCenti(g_shijue_origin_cm);
    int32_t position = State_FloatToCenti(g_shijue_position_cm);
    int32_t velocity = State_FloatToCenti(g_shijue_velocity_cm_s);
    uint32_t origin_abs = (uint32_t)((origin < 0) ? -origin : origin);
    uint32_t position_abs = (uint32_t)((position < 0) ? -position : position);
    uint32_t velocity_abs = (uint32_t)((velocity < 0) ? -velocity : velocity);
    int length;

    if (((uint32_t)(now_ms - vision_debug_time_ms) < 1000U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    /* USART2 每秒输出一次 USART1 视觉协议的解析结果。 */
    length = snprintf((char *)vision_debug_buffer,
                      sizeof(vision_debug_buffer),
                      "VISION O=%c%lu.%02lu(%u) P=%c%lu.%02lu(%u) V=%c%lu.%02lu(%u) TYPE=0x%02X SEQ=%u ERR=%u CNT=%u\r\n",
                      (origin < 0) ? '-' : '+',
                      (unsigned long)(origin_abs / 100U),
                      (unsigned long)(origin_abs % 100U),
                      (unsigned int)g_shijue_origin_valid,
                      (position < 0) ? '-' : '+',
                      (unsigned long)(position_abs / 100U),
                      (unsigned long)(position_abs % 100U),
                      (unsigned int)g_shijue_position_valid,
                      (velocity < 0) ? '-' : '+',
                      (unsigned long)(velocity_abs / 100U),
                      (unsigned long)(velocity_abs % 100U),
                      (unsigned int)g_shijue_velocity_valid,
                      (unsigned int)g_shijue_last_type,
                      (unsigned int)g_shijue_last_seq,
                      (unsigned int)g_shijue_error_flag,
                      (unsigned int)g_shijue_count);

    if ((length > 0) && ((size_t)length < sizeof(vision_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         vision_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        vision_debug_time_ms = now_ms;
    }
}

static void State_UpdateDisplay(uint32_t now_ms)
{
    if (current_page == STATE_PAGE_RUNNING)
    {
        uint32_t half_seconds = State_GetElapsedHalfSeconds(now_ms);
        if (half_seconds != last_display_half_second)
        {
            last_display_half_second = half_seconds;
            display_dirty = 1U;
        }
    }

    if (current_page != last_display_page)
    {
        last_display_page = current_page;
        display_dirty = 1U;
    }

    if (display_dirty != 0U)
    {
        State_Render(now_ms);
    }
}

void State_Init(void)
{
    uint32_t now_ms = BSP_TimeMs();

    BSP_KeyInit();
    OLED_Init();

    current_mode = STATE_MODE_NONE;
    current_page = STATE_PAGE_SELECT;
    selected_item = 2U;
    target_tenth_cm = 0;
    start_time_ms = 0U;
    stopped_elapsed_ms = 0U;
    vision_debug_time_ms = now_ms;
    last_display_half_second = 0xFFFFFFFFU;
    last_display_page = STATE_PAGE_SELECT;
    memset(display_cache, 0, sizeof(display_cache));
    State_H2Reset();

    display_dirty = 1U;
    State_Render(now_ms);
}

void State_RunCurrent(void)
{
    uint32_t now_ms = BSP_TimeMs();
    KeyEvent_t key = BSP_KeyScan(now_ms);

    State_HandleKey(key, now_ms);

    if (current_page == STATE_PAGE_RUNNING)
    {
        State_RunMode(now_ms);
    }
    else if ((current_mode == STATE_MODE_H2_CAR_LOOP) &&
             ((current_page == STATE_PAGE_STOPPED) ||
              (current_page == STATE_PAGE_FINISHED)))
    {
        State_H2Brake();
    }

    State_SendVisionDebug(now_ms);
    State_UpdateDisplay(now_ms);
}
