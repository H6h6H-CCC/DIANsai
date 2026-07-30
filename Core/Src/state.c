#include "state.h"

#include "OLED.h"
#include "ball_control.h"
#include "bsp_key.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "gray.h"
#include "jy61p.h"
#include "moter.h"
#include "shijue.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define H2_BASE_PWM           350
#define H2_TRACK_KP           8
#define H2_REVERSE_LEFT_PWM   850
#define H2_REVERSE_RIGHT_PWM  1000
#define H2_REVERSE_MS         250U
#define H2_FIRST_STRAIGHT_MS  100U
#define H2_MARKER_BLACK_COUNT 4U
#define BALANCE_SERVO_CENTER_US 1730U
#define BALANCE_SETTLE_MS       1000U
#define BALANCE_SAMPLE_MS       1000U
#define BALANCE_SAMPLE_PERIOD_MS 10U
#define BALANCE_SAMPLE_MIN_COUNT  50U
#define BALANCE_MAX_RAW_ROLL_DEG  30.0f
#define BALANCE_MAX_GYRO_DPS      5.0f
#define BALANCE_MAX_ROLL_SPAN_DEG 1.0f
#define H3_POSITIVE_TARGET_CM       5.0f
#define H3_NEGATIVE_TARGET_CM      -5.0f
#define H3_TARGET_TOLERANCE_CM      1.0f
#define H3_STABLE_MS              500U
#define H3_TIMEOUT_MS            5000U

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

typedef enum
{
    BALANCE_CAL_IDLE = 0,
    BALANCE_CAL_SETTLING,
    BALANCE_CAL_SAMPLING,
    BALANCE_CAL_DONE
} BalanceCalibration_t;

static StateMode_t current_mode;
static StatePage_t current_page;
static uint8_t selected_item;
static int16_t target_tenth_cm;
static uint8_t h6_target_send_pending;
static float balance_roll_zero_deg;
static float balance_wx_zero_dps;
static uint8_t balance_roll_zero_valid;
static BalanceCalibration_t balance_calibration;
static uint8_t balance_calibration_auto_start;
static uint32_t balance_calibration_time_ms;
static uint32_t balance_last_sample_time_ms;
static float balance_roll_sum;
static float balance_wx_sum;
static float balance_roll_min;
static float balance_roll_max;
static uint16_t balance_sample_count;
static uint32_t start_time_ms;
static uint32_t stopped_elapsed_ms;
static uint8_t h3_returning;
static uint32_t h3_stable_start_ms;

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

static uint8_t balance_debug_buffer[220];
static uint32_t balance_debug_time_ms;
static uint8_t vision_debug_buffer[128];
static uint32_t vision_debug_time_ms;
static uint8_t imu_debug_buffer[160];
static uint32_t imu_debug_time_ms;

#define DEBUG_COMMAND_SIZE  64U
#define DEBUG_REPLY_SIZE    160U
static char debug_rx_build[DEBUG_COMMAND_SIZE];
static uint8_t debug_rx_build_length;
static char debug_command[DEBUG_COMMAND_SIZE];
static volatile uint8_t debug_command_ready;
static char debug_reply[DEBUG_REPLY_SIZE];
static uint16_t debug_reply_length;
static uint8_t debug_reply_ready;
static volatile uint32_t debug_rx_event_count;
static volatile uint32_t debug_rx_byte_count;
extern volatile uint32_t g_jy61_rx_event_count;
extern volatile uint32_t g_jy61_rx_byte_count;

static void State_Start(uint32_t now_ms);
static void State_ProcessDebugCommand(void);
static void State_SendDebugReply(void);

static const char *State_SkipSpaces(const char *text)
{
    while ((*text == ' ') || (*text == '\t')) text++;
    return text;
}

static uint8_t State_ParseFloat(const char **text, float *value)
{
    char *end;
    const char *start = State_SkipSpaces(*text);

    *value = strtof(start, &end);
    if (end == start) return 0U;
    *text = end;
    return 1U;
}

static uint8_t State_ParsePid(const char *text, float *kp, float *ki, float *kd)
{
    if ((State_ParseFloat(&text, kp) == 0U) ||
        (State_ParseFloat(&text, ki) == 0U) ||
        (State_ParseFloat(&text, kd) == 0U))
    {
        return 0U;
    }
    text = State_SkipSpaces(text);
    return (*text == '\0') && (*kp >= 0.0f) && (*ki >= 0.0f) && (*kd >= 0.0f);
}

void State_DebugRx(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint8_t line_finished = 0U;

    debug_rx_event_count++;
    debug_rx_byte_count += length;

    for (i = 0U; i < length; i++)
    {
        char value = (char)data[i];

        if ((value == '\r') || (value == '\n'))
        {
            line_finished = 1U;
            continue;
        }

        if ((value >= 32) && (value <= 126) &&
            (debug_rx_build_length < (DEBUG_COMMAND_SIZE - 1U)))
        {
            debug_rx_build[debug_rx_build_length++] = value;
        }
    }

    /* DMA 空闲事件本身也表示一次手动发送结束，因此换行可省略。 */
    if (((line_finished != 0U) || (length != 0U)) &&
        (debug_rx_build_length != 0U) &&
        (debug_command_ready == 0U))
    {
        memcpy(debug_command, debug_rx_build, debug_rx_build_length);
        debug_command[debug_rx_build_length] = '\0';
        debug_command_ready = 1U;
        debug_rx_build_length = 0U;
    }
}

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
    if (balance_calibration == BALANCE_CAL_SETTLING)
    {
        State_ShowLine(2U, "CAL:SETTLING");
        State_ShowLine(3U, "PLEASE WAIT 1S");
        State_ShowLine(4U, "SERVO:LEVEL");
        return;
    }
    if (balance_calibration == BALANCE_CAL_SAMPLING)
    {
        State_ShowLine(2U, "CAL:SAMPLING");
        State_ShowLine(3U, "ANALYZE 1S");
        State_ShowLine(4U, "KEEP STILL");
        return;
    }

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

static void State_BeginBalanceCalibration(uint32_t now_ms, uint8_t auto_start)
{
    BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
    BallControl_SetEnabled(0U);

    balance_roll_zero_deg = 0.0f;
    balance_wx_zero_dps = 0.0f;
    balance_roll_zero_valid = 0U;
    balance_roll_sum = 0.0f;
    balance_wx_sum = 0.0f;
    balance_roll_min = 0.0f;
    balance_roll_max = 0.0f;
    balance_sample_count = 0U;
    balance_calibration_time_ms = now_ms;
    balance_last_sample_time_ms = now_ms;
    balance_calibration = BALANCE_CAL_SETTLING;
    balance_calibration_auto_start = auto_start;
    current_page = STATE_PAGE_READY;
    display_dirty = 1U;
}

static void State_ProcessBalanceCalibration(uint32_t now_ms)
{
    if ((current_mode == STATE_MODE_NONE) ||
        (current_page != STATE_PAGE_READY))
    {
        return;
    }

    if (balance_calibration == BALANCE_CAL_SETTLING)
    {
        if ((uint32_t)(now_ms - balance_calibration_time_ms) >= BALANCE_SETTLE_MS)
        {
            balance_roll_sum = 0.0f;
            balance_wx_sum = 0.0f;
            balance_roll_min = 0.0f;
            balance_roll_max = 0.0f;
            balance_sample_count = 0U;
            balance_calibration_time_ms = now_ms;
            balance_last_sample_time_ms = now_ms;
            balance_calibration = BALANCE_CAL_SAMPLING;
            display_dirty = 1U;
        }
        return;
    }

    if (balance_calibration != BALANCE_CAL_SAMPLING)
    {
        return;
    }

    if (((uint32_t)(now_ms - balance_last_sample_time_ms) >= BALANCE_SAMPLE_PERIOD_MS) &&
        (angleValid != 0U) &&
        (gyroValid != 0U) &&
        ((uint32_t)(now_ms - lastPacketTime) <= 50U))
    {
        balance_last_sample_time_ms = now_ms;

        /* JY61P 上电初期姿态可能从错误角度收敛，必须连续稳定后才标零。 */
        if ((sensorData.roll > BALANCE_MAX_RAW_ROLL_DEG) ||
            (sensorData.roll < -BALANCE_MAX_RAW_ROLL_DEG) ||
            (sensorData.wx > BALANCE_MAX_GYRO_DPS) ||
            (sensorData.wx < -BALANCE_MAX_GYRO_DPS))
        {
            balance_roll_sum = 0.0f;
            balance_wx_sum = 0.0f;
            balance_roll_min = 0.0f;
            balance_roll_max = 0.0f;
            balance_sample_count = 0U;
            balance_calibration_time_ms = now_ms;
            return;
        }

        if (balance_sample_count == 0U)
        {
            balance_roll_min = sensorData.roll;
            balance_roll_max = sensorData.roll;
        }
        else
        {
            if (sensorData.roll < balance_roll_min) balance_roll_min = sensorData.roll;
            if (sensorData.roll > balance_roll_max) balance_roll_max = sensorData.roll;
        }
        balance_roll_sum += sensorData.roll;
        balance_wx_sum += sensorData.wx;
        balance_sample_count++;
    }

    if ((uint32_t)(now_ms - balance_calibration_time_ms) >= BALANCE_SAMPLE_MS)
    {
        if ((balance_sample_count < BALANCE_SAMPLE_MIN_COUNT) ||
            ((balance_roll_max - balance_roll_min) > BALANCE_MAX_ROLL_SPAN_DEG))
        {
            balance_roll_sum = 0.0f;
            balance_wx_sum = 0.0f;
            balance_roll_min = 0.0f;
            balance_roll_max = 0.0f;
            balance_sample_count = 0U;
            balance_calibration_time_ms = now_ms;
            return;
        }

        balance_roll_zero_deg = balance_roll_sum / (float)balance_sample_count;
        balance_wx_zero_dps = balance_wx_sum / (float)balance_sample_count;
        balance_roll_zero_valid = 1U;
        balance_calibration = BALANCE_CAL_DONE;

        BallControl_SetEnabled(0U);
        current_page = STATE_PAGE_READY;
        display_dirty = 1U;

        if (balance_calibration_auto_start != 0U)
        {
            balance_calibration_auto_start = 0U;
            State_Start(now_ms);
        }
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
    else if (current_mode == STATE_MODE_H3_BALL_MOVE)
    {
        h3_returning = 0U;
        h3_stable_start_ms = 0U;
        BallControl_SetTargetPosition(H3_POSITIVE_TARGET_CM);
        BallControl_SetEnabled(1U);
    }
    else if ((current_mode == STATE_MODE_H5_LOOP_CENTER) ||
             (current_mode == STATE_MODE_H6_LOOP_TARGET))
    {
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(1U);
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
            if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                current_page = STATE_PAGE_TARGET_SET;
            }
            else
            {
                State_BeginBalanceCalibration(now_ms, 0U);
            }
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
            /* H6目标为相对中点坐标，进入准备界面后发送给视觉设置绝对原点。 */
            h6_target_send_pending = 1U;
            State_BeginBalanceCalibration(now_ms, 0U);
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
        if ((key == KEY_EVENT_3) &&
            (balance_calibration == BALANCE_CAL_DONE))
        {
            State_Start(now_ms);
        }
        else if (key == KEY_EVENT_4)
        {
            if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                balance_calibration = BALANCE_CAL_IDLE;
                current_page = STATE_PAGE_TARGET_SET;
            }
            else
            {
                balance_calibration = BALANCE_CAL_IDLE;
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
            else if ((current_mode == STATE_MODE_H5_LOOP_CENTER) ||
                     (current_mode == STATE_MODE_H6_LOOP_TARGET) ||
                     (current_mode == STATE_MODE_H3_BALL_MOVE))
            {
                BallControl_SetEnabled(0U);
            }
        }
        break;

    case STATE_PAGE_FINISHED:
    case STATE_PAGE_STOPPED:
    case STATE_PAGE_TIMEOUT:
        if (key == KEY_EVENT_4)
        {
            stopped_elapsed_ms = 0U;
            State_BeginBalanceCalibration(now_ms, 0U);
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

    case STATE_MODE_H5_LOOP_CENTER:
    case STATE_MODE_H6_LOOP_TARGET:
    case STATE_MODE_H3_BALL_MOVE:
        if ((balance_roll_zero_valid != 0U) &&
            (angleValid != 0U) &&
            (gyroValid != 0U) &&
            ((uint32_t)(now_ms - lastPacketTime) <= 50U))
        {
            BallControl_SetPipeAngle(sensorData.roll - balance_roll_zero_deg,
                                     sensorData.wx - balance_wx_zero_dps,
                                     now_ms);
        }
        BallControl_Process(now_ms);

        if (current_mode != STATE_MODE_H3_BALL_MOVE)
        {
            break;
        }

        if ((uint32_t)(now_ms - start_time_ms) > H3_TIMEOUT_MS)
        {
            State_End(STATE_PAGE_TIMEOUT, now_ms);
            break;
        }

        if ((g_shijue_position_valid == 0U) ||
            (h3_returning == 0U &&
             ((g_shijue_position_cm < (H3_POSITIVE_TARGET_CM - H3_TARGET_TOLERANCE_CM)) ||
              (g_shijue_position_cm > (H3_POSITIVE_TARGET_CM + H3_TARGET_TOLERANCE_CM)))))
        {
            break;
        }

        if (h3_returning == 0U)
        {
            h3_returning = 1U;
            h3_stable_start_ms = 0U;
            BallControl_SetTargetPosition(H3_NEGATIVE_TARGET_CM);
            break;
        }

        if ((g_shijue_position_cm >= (H3_NEGATIVE_TARGET_CM - H3_TARGET_TOLERANCE_CM)) &&
            (g_shijue_position_cm <= (H3_NEGATIVE_TARGET_CM + H3_TARGET_TOLERANCE_CM)))
        {
            if (h3_stable_start_ms == 0U)
            {
                h3_stable_start_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - h3_stable_start_ms) >= H3_STABLE_MS)
            {
                State_End(STATE_PAGE_FINISHED, now_ms);
            }
        }
        else
        {
            h3_stable_start_ms = 0U;
        }
        break;

    case STATE_MODE_H4_AB_BALANCE:
        /* H3-H5 先保留空实现。 */
        break;

    default:
        break;
    }
}

static void State_SendH6Target(void)
{
    uint16_t origin_tenth_cm;

    if ((h6_target_send_pending == 0U) ||
        (current_mode != STATE_MODE_H6_LOOP_TARGET) ||
        (current_page != STATE_PAGE_READY))
    {
        return;
    }

    origin_tenth_cm = (uint16_t)(125 + target_tenth_cm);
    if (Shijue_SetOriginTenthCm(origin_tenth_cm) != 0U)
    {
        h6_target_send_pending = 0U;
    }
}

static int32_t State_FloatToCenti(float value)
{
    return (int32_t)(value * 100.0f);
}

static void State_FormatSignedCenti(char *output, size_t size, float value)
{
    int32_t centi = State_FloatToCenti(value);
    uint32_t magnitude = (uint32_t)((centi < 0) ? -centi : centi);

    (void)snprintf(output,
                   size,
                   "%c%lu.%02lu",
                   (centi < 0) ? '-' : '+',
                   (unsigned long)(magnitude / 100U),
                   (unsigned long)(magnitude % 100U));
}

static void State_ProcessDebugCommand(void)
{
    float kp, ki, kd;
    float target;
    float position_kp, position_ki, position_kd;
    float angle_kp, angle_ki, angle_kd;
    char a_kp[12], a_ki[12], a_kd[12];
    char p_kp[12], p_ki[12], p_kd[12];
    char target_text[12];
    char position_target_text[12];
    const char *text;
    int length;

    if ((debug_command_ready == 0U) || (debug_reply_ready != 0U)) return;

    if ((strncmp(debug_command, "APID", 4U) == 0) &&
        (State_ParsePid(&debug_command[4], &kp, &ki, &kd) != 0U))
    {
        BallControl_SetAnglePid(kp, ki, kd);
        State_FormatSignedCenti(a_kp, sizeof(a_kp), kp);
        State_FormatSignedCenti(a_ki, sizeof(a_ki), ki);
        State_FormatSignedCenti(a_kd, sizeof(a_kd), kd);
        length = snprintf(debug_reply, sizeof(debug_reply),
                          "OK APID KP=%s KI=%s KD=%s\r\n", a_kp, a_ki, a_kd);
    }
    else if ((strncmp(debug_command, "PPID", 4U) == 0) &&
             (State_ParsePid(&debug_command[4], &kp, &ki, &kd) != 0U))
    {
        BallControl_SetPositionPid(kp, ki, kd);
        State_FormatSignedCenti(p_kp, sizeof(p_kp), kp);
        State_FormatSignedCenti(p_ki, sizeof(p_ki), ki);
        State_FormatSignedCenti(p_kd, sizeof(p_kd), kd);
        length = snprintf(debug_reply, sizeof(debug_reply),
                          "OK PPID KP=%s KI=%s KD=%s\r\n", p_kp, p_ki, p_kd);
    }
    else if (strncmp(debug_command, "ATG", 3U) == 0)
    {
        text = &debug_command[3];
        if ((State_ParseFloat(&text, &target) != 0U) &&
            (*State_SkipSpaces(text) == '\0'))
        {
            BallControl_SetManualTargetAngle(target);
            State_FormatSignedCenti(target_text, sizeof(target_text),
                                    BallControl_GetTargetAngle());
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK ATG=%s\r\n", target_text);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR ATG\r\n");
        }
    }
    else if (strncmp(debug_command, "PTG", 3U) == 0)
    {
        text = &debug_command[3];
        if ((State_ParseFloat(&text, &target) != 0U) &&
            (*State_SkipSpaces(text) == '\0') &&
            (target >= -12.5f) && (target <= 12.5f))
        {
            BallControl_SetAutoTargetAngle();
            BallControl_SetTargetPosition(target);
            State_FormatSignedCenti(position_target_text,
                                    sizeof(position_target_text), target);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK PTG=%s\r\n", position_target_text);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR PTG\r\n");
        }
    }
    else if (strcmp(debug_command, "AUTO") == 0)
    {
        BallControl_SetAutoTargetAngle();
        length = snprintf(debug_reply, sizeof(debug_reply), "OK AUTO\r\n");
    }
    else if (strcmp(debug_command, "ZERO") == 0)
    {
        State_BeginBalanceCalibration(BSP_TimeMs(), 1U);
        length = snprintf(debug_reply, sizeof(debug_reply), "OK ZERO\r\n");
    }
    else if (strcmp(debug_command, "GET") == 0)
    {
        BallControl_GetAnglePid(&angle_kp, &angle_ki, &angle_kd);
        BallControl_GetPositionPid(&position_kp, &position_ki, &position_kd);
        State_FormatSignedCenti(a_kp, sizeof(a_kp), angle_kp);
        State_FormatSignedCenti(a_ki, sizeof(a_ki), angle_ki);
        State_FormatSignedCenti(a_kd, sizeof(a_kd), angle_kd);
        State_FormatSignedCenti(p_kp, sizeof(p_kp), position_kp);
        State_FormatSignedCenti(p_ki, sizeof(p_ki), position_ki);
        State_FormatSignedCenti(p_kd, sizeof(p_kd), position_kd);
        State_FormatSignedCenti(target_text, sizeof(target_text),
                                BallControl_GetTargetAngle());
        State_FormatSignedCenti(position_target_text,
                                sizeof(position_target_text),
                                BallControl_GetTargetPosition());
        length = snprintf(debug_reply, sizeof(debug_reply),
                          "GET A=%s,%s,%s P=%s,%s,%s MODE=%s ATG=%s PTG=%s\r\n",
                          a_kp, a_ki, a_kd,
                          p_kp, p_ki, p_kd,
                          (BallControl_IsManualTargetAngle() != 0U) ? "MAN" : "AUTO",
                          target_text, position_target_text);
    }
    else
    {
        length = snprintf(debug_reply, sizeof(debug_reply), "ERR CMD\r\n");
    }

    debug_command_ready = 0U;
    if ((length > 0) && ((size_t)length < sizeof(debug_reply)))
    {
        debug_reply_length = (uint16_t)length;
        debug_reply_ready = 1U;
    }
}

static void State_SendDebugReply(void)
{
    if ((debug_reply_ready != 0U) &&
        (BSP_UartTxReady(BSP_UART_2) != 0U) &&
        (BSP_UartSendDma(BSP_UART_2,
                         (uint8_t *)debug_reply,
                         debug_reply_length) == BSP_STATUS_OK))
    {
        debug_reply_ready = 0U;
    }
}

static void State_SendBalanceDebug(uint32_t now_ms)
{
    char target_angle[12];
    char pipe_angle[12];
    char pipe_gyro[12];
    char raw_roll[12];
    char roll_zero[12];
    char position[12];
    char velocity[12];
    char position_error[12];
    int length;

    if ((debug_reply_ready != 0U) ||
        ((uint32_t)(now_ms - balance_debug_time_ms) < 10U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    State_FormatSignedCenti(target_angle, sizeof(target_angle), BallControl_GetTargetAngle());
    State_FormatSignedCenti(pipe_angle, sizeof(pipe_angle), BallControl_GetPipeAngle());
    State_FormatSignedCenti(pipe_gyro, sizeof(pipe_gyro), BallControl_GetPipeGyro());
    State_FormatSignedCenti(raw_roll, sizeof(raw_roll), sensorData.roll);
    State_FormatSignedCenti(roll_zero, sizeof(roll_zero), balance_roll_zero_deg);
    State_FormatSignedCenti(position, sizeof(position), g_shijue_position_cm);
    State_FormatSignedCenti(velocity, sizeof(velocity), g_shijue_velocity_cm_s);
    State_FormatSignedCenti(position_error, sizeof(position_error),
                            BallControl_GetTargetPosition() - BallControl_GetPosition());

    /* 100 Hz 输出角度内环关键量，便于直接观察阶跃响应。 */
    length = snprintf((char *)balance_debug_buffer,
                      sizeof(balance_debug_buffer),
                      "CTRL T=%lu P=%s PV=%u VEL=%s VV=%u PE=%s VC=%u TG=%s ANG=%s W=%s PWM=%u RAW=%s Z=%s IV=%u%u AGE=%lu JRX=%lu/%lu RX=%lu/%lu\r\n",
                      (unsigned long)now_ms,
                      position,
                      (unsigned int)g_shijue_position_valid,
                      velocity,
                      (unsigned int)g_shijue_velocity_valid,
                      position_error,
                      (unsigned int)g_shijue_count,
                      target_angle,
                      pipe_angle,
                      pipe_gyro,
                      (unsigned int)BallControl_GetServoPulse(),
                      raw_roll,
                      roll_zero,
                      (unsigned int)angleValid,
                      (unsigned int)gyroValid,
                      (unsigned long)(now_ms - lastPacketTime),
                      (unsigned long)g_jy61_rx_event_count,
                      (unsigned long)g_jy61_rx_byte_count,
                      (unsigned long)debug_rx_event_count,
                      (unsigned long)debug_rx_byte_count);

    if ((length > 0) && ((size_t)length < sizeof(balance_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         balance_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        balance_debug_time_ms = now_ms;
    }
}

static void State_SendVisionDebug(uint32_t now_ms)
{
    int32_t position = State_FloatToCenti(g_shijue_position_cm);
    int32_t velocity = State_FloatToCenti(g_shijue_velocity_cm_s);
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
                      "VISION P=%c%lu.%02lu(%u) V=%c%lu.%02lu(%u) TYPE=0x%02X SEQ=%u ERR=%u CNT=%u\r\n",
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

static void State_SendImuDebug(uint32_t now_ms)
{
    char ax[12], ay[12], az[12];
    char wx[12], wy[12], wz[12];
    char roll[12], pitch[12], yaw[12];
    int length;

    if (((uint32_t)(now_ms - imu_debug_time_ms) < 1000U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    /* nano printf默认不支持%f，先转为定点字符串再发送。 */
    State_FormatSignedCenti(ax, sizeof(ax), sensorData.ax);
    State_FormatSignedCenti(ay, sizeof(ay), sensorData.ay);
    State_FormatSignedCenti(az, sizeof(az), sensorData.az);
    State_FormatSignedCenti(wx, sizeof(wx), sensorData.wx);
    State_FormatSignedCenti(wy, sizeof(wy), sensorData.wy);
    State_FormatSignedCenti(wz, sizeof(wz), sensorData.wz);
    State_FormatSignedCenti(roll, sizeof(roll), sensorData.roll);
    State_FormatSignedCenti(pitch, sizeof(pitch), sensorData.pitch);
    State_FormatSignedCenti(yaw, sizeof(yaw), sensorData.yaw);

    length = snprintf((char *)imu_debug_buffer,
                      sizeof(imu_debug_buffer),
                      "JY61P A=%s,%s,%s G=%s,%s,%s ANG=%s,%s,%s VALID=%u%u%u AGE=%lu\r\n",
                      ax, ay, az,
                      wx, wy, wz,
                      roll, pitch, yaw,
                      (unsigned int)accValid,
                      (unsigned int)gyroValid,
                      (unsigned int)angleValid,
                      (unsigned long)(now_ms - lastPacketTime));

    if ((length > 0) && ((size_t)length < sizeof(imu_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         imu_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        imu_debug_time_ms = now_ms;
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

    /* 调试阶段上电先校准，再自动进入H5中心稳球。 */
    current_mode = STATE_MODE_H3_BALL_MOVE;
    current_page = STATE_PAGE_READY;
    selected_item = 3U;
    target_tenth_cm = 0;
    h6_target_send_pending = 0U;
    balance_roll_zero_deg = 0.0f;
    balance_wx_zero_dps = 0.0f;
    balance_roll_zero_valid = 0U;
    balance_calibration = BALANCE_CAL_IDLE;
    balance_calibration_auto_start = 0U;
    start_time_ms = 0U;
    stopped_elapsed_ms = 0U;
    h3_returning = 0U;
    h3_stable_start_ms = 0U;
    vision_debug_time_ms = now_ms;
    imu_debug_time_ms = now_ms;
    balance_debug_time_ms = now_ms;
    last_display_half_second = 0xFFFFFFFFU;
    last_display_page = STATE_PAGE_SELECT;
    memset(display_cache, 0, sizeof(display_cache));
    State_H2Reset();
    State_BeginBalanceCalibration(now_ms, 1U);

    display_dirty = 1U;
    State_Render(now_ms);
}

void State_RunCurrent(void)
{
    uint32_t now_ms = BSP_TimeMs();
    KeyEvent_t key = BSP_KeyScan(now_ms);

    State_HandleKey(key, now_ms);
    State_ProcessDebugCommand();
    State_SendDebugReply();
    State_SendH6Target();
    State_ProcessBalanceCalibration(now_ms);

    if (current_page == STATE_PAGE_RUNNING)
    {
        State_RunMode(now_ms);
    }
    else if ((current_mode == STATE_MODE_H3_BALL_MOVE) &&
             ((current_page == STATE_PAGE_FINISHED) ||
              (current_page == STATE_PAGE_TIMEOUT)))
    {
        /* H3结束后继续闭环，使小球保持在-5 cm，而不是冻结最后一次舵机输出。 */
        if ((balance_roll_zero_valid != 0U) &&
            (angleValid != 0U) &&
            (gyroValid != 0U) &&
            ((uint32_t)(now_ms - lastPacketTime) <= 50U))
        {
            BallControl_SetPipeAngle(sensorData.roll - balance_roll_zero_deg,
                                     sensorData.wx - balance_wx_zero_dps,
                                     now_ms);
        }
        BallControl_Process(now_ms);
    }
    else if ((current_mode == STATE_MODE_H2_CAR_LOOP) &&
             ((current_page == STATE_PAGE_STOPPED) ||
              (current_page == STATE_PAGE_FINISHED)))
    {
        State_H2Brake();
    }

    State_SendBalanceDebug(now_ms);
    /* 陀螺仪解析回传先保留，后续需要时取消注释即可。 */
    // State_SendImuDebug(now_ms);
    State_UpdateDisplay(now_ms);
}
