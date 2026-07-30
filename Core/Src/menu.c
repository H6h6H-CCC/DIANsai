#include "menu.h"

#include "app_mode.h"
#include "OLED.h"

#include <stdio.h>
#include <string.h>

static uint8_t selected_item = 2U;
static uint32_t last_display_half_second = 0xFFFFFFFFU;
static uint8_t display_dirty;
static AppState_t last_display_state;
static char display_cache[4][16];

static const char *Menu_GetModeName(uint8_t mode)
{
    switch (mode)
    {
    case APP_MODE_H2_CAR_LOOP:    return "H2 CAR LOOP";
    case APP_MODE_H3_BALL_MOVE:   return "H3 BALL MOVE";
    case APP_MODE_H4_AB_BALANCE:  return "H4 AB BALANCE";
    case APP_MODE_H5_LOOP_CENTER: return "H5 LOOP CENTER";
    case APP_MODE_H6_LOOP_TARGET: return "H6 LOOP TARGET";
    default:                      return "SELECT H ITEM";
    }
}

static void Menu_ShowLine(uint8_t line, const char *text)
{
    char output[17];
    size_t length = strlen(text);
    uint8_t column;

    if (length > 16U) length = 16U;
    memset(output, ' ', 16U);
    memcpy(output, text, length);
    output[16] = '\0';

    /* 仅写入变化字符，减少运行期间阻塞式 I2C 对控制周期的影响。 */
    for (column = 0U; column < 16U; column++)
    {
        if (display_cache[line - 1U][column] != output[column])
        {
            OLED_ShowChar(line, column + 1U, output[column]);
            display_cache[line - 1U][column] = output[column];
        }
    }
}

static void Menu_FormatTime(char *output, size_t size, uint32_t half_seconds)
{
    (void)snprintf(output,
                   size,
                   "TIME:%03lu.%cS",
                   (unsigned long)(half_seconds / 2U),
                   ((half_seconds & 1U) != 0U) ? '5' : '0');
}

static void Menu_FormatTarget(char *output, size_t size)
{
    int16_t target = AppMode_GetTargetTenthCm();
    uint16_t magnitude = (target < 0) ? (uint16_t)(-target) : (uint16_t)target;

    (void)snprintf(output,
                   size,
                   "TARGET:%c%02u.%uCM",
                   (target < 0) ? '-' : '+',
                   magnitude / 10U,
                   magnitude % 10U);
}

static void Menu_RenderSelect(void)
{
    uint8_t first_item = selected_item - 1U;
    uint8_t row;
    char line[17];

    if (first_item < 2U) first_item = 2U;
    if (first_item > 4U) first_item = 4U;

    Menu_ShowLine(1U, "SELECT H ITEM");
    for (row = 0U; row < 3U; row++)
    {
        uint8_t item = first_item + row;
        (void)snprintf(line,
                       sizeof(line),
                       "%c%s",
                       (item == selected_item) ? '>' : ' ',
                       Menu_GetModeName(item));
        Menu_ShowLine(row + 2U, line);
    }
}

static void Menu_RenderReady(AppMode_t mode)
{
    char target[17];

    Menu_ShowLine(1U, Menu_GetModeName((uint8_t)mode));
    if (mode == APP_MODE_H3_BALL_MOVE)
    {
        Menu_ShowLine(2U, "0>+5>-5CM");
    }
    else if ((mode == APP_MODE_H4_AB_BALANCE) ||
             (mode == APP_MODE_H5_LOOP_CENTER))
    {
        Menu_ShowLine(2U, "TARGET:CENTER");
    }
    else if (mode == APP_MODE_H6_LOOP_TARGET)
    {
        Menu_FormatTarget(target, sizeof(target));
        Menu_ShowLine(2U, target);
    }
    else
    {
        Menu_ShowLine(2U, "STATE:READY");
    }
    Menu_ShowLine(3U, "K3 START");
    Menu_ShowLine(4U, (mode == APP_MODE_H6_LOOP_TARGET) ? "K4 BACK" : "K4 MENU");
}

static void Menu_RenderStatus(AppState_t state, uint32_t now_ms)
{
    char time_text[17];
    const char *state_text;

    switch (state)
    {
    case APP_STATE_RUNNING:  state_text = "STATE:RUNNING";  break;
    case APP_STATE_FINISHED: state_text = "STATE:FINISHED"; break;
    case APP_STATE_TIMEOUT:  state_text = "STATE:TIMEOUT";  break;
    default:                 state_text = "STATE:STOPPED";  break;
    }

    Menu_FormatTime(time_text,
                    sizeof(time_text),
                    AppMode_GetElapsedHalfSeconds(now_ms));
    Menu_ShowLine(1U, Menu_GetModeName((uint8_t)AppMode_GetMode()));
    Menu_ShowLine(2U, state_text);
    Menu_ShowLine(3U, time_text);
    Menu_ShowLine(4U, (state == APP_STATE_RUNNING) ? "K4 STOP" : "K4 BACK");
}

static void Menu_Render(uint32_t now_ms)
{
    AppState_t state = AppMode_GetState();

    switch (state)
    {
    case APP_STATE_SELECT:
        Menu_RenderSelect();
        break;

    case APP_STATE_TARGET_SET:
    {
        char target[17];
        Menu_FormatTarget(target, sizeof(target));
        Menu_ShowLine(1U, "H6 LOOP TARGET");
        Menu_ShowLine(2U, target);
        Menu_ShowLine(3U, "K1+ K2-");
        Menu_ShowLine(4U, "K3 OK K4 MENU");
        break;
    }

    case APP_STATE_READY:
        Menu_RenderReady(AppMode_GetMode());
        break;

    default:
        Menu_RenderStatus(state, now_ms);
        break;
    }

    display_dirty = 0U;
}

static void Menu_SelectMode(uint8_t mode)
{
    selected_item = mode;
    AppMode_Select((AppMode_t)mode);
    display_dirty = 1U;
}

void Menu_Init(uint32_t now_ms)
{
    AppMode_Init();
    memset(display_cache, 0, sizeof(display_cache));
    display_dirty = 1U;
    Menu_Render(now_ms);
    last_display_state = AppMode_GetState();
}

void Menu_Process(KeyEvent_t key, uint32_t now_ms)
{
    AppState_t state = AppMode_GetState();

    switch (state)
    {
    case APP_STATE_SELECT:
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
            Menu_SelectMode(selected_item);
        }
        break;

    case APP_STATE_TARGET_SET:
        if (key == KEY_EVENT_1)
        {
            AppMode_SetTargetTenthCm(AppMode_GetTargetTenthCm() + 1);
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_2)
        {
            AppMode_SetTargetTenthCm(AppMode_GetTargetTenthCm() - 1);
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_3)
        {
            AppMode_ConfirmTarget();
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_4)
        {
            AppMode_BackToSelect();
            display_dirty = 1U;
        }
        break;

    case APP_STATE_READY:
        if (key == KEY_EVENT_3)
        {
            AppMode_Start(now_ms);
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_4)
        {
            if (AppMode_GetMode() == APP_MODE_H6_LOOP_TARGET)
            {
                AppMode_BackToTarget();
            }
            else
            {
                AppMode_BackToSelect();
            }
            display_dirty = 1U;
        }
        break;

    case APP_STATE_RUNNING:
        if (key == KEY_EVENT_4)
        {
            AppMode_Stop(now_ms);
            display_dirty = 1U;
        }
        break;

    case APP_STATE_FINISHED:
    case APP_STATE_STOPPED:
    case APP_STATE_TIMEOUT:
        if (key == KEY_EVENT_4)
        {
            AppMode_BackToReady();
            display_dirty = 1U;
        }
        break;

    default:
        break;
    }

    if (AppMode_GetState() == APP_STATE_RUNNING)
    {
        uint32_t half_seconds = AppMode_GetElapsedHalfSeconds(now_ms);
        if (half_seconds != last_display_half_second)
        {
            last_display_half_second = half_seconds;
            display_dirty = 1U;
        }
    }

    if (AppMode_GetState() != last_display_state)
    {
        last_display_state = AppMode_GetState();
        display_dirty = 1U;
    }

    if (display_dirty != 0U)
    {
        Menu_Render(now_ms);
    }
}
