#include "state.h"

#include "OLED.h"
#include "ball_control.h"
#include "bsp_key.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "encoder.h"
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
#define H4_BASE_PWM           250
#define H4_TRACK_KP           8
#define H4_RAMP_UP_MS         2000U
#define H4_LIFT_START_PWM     80
#define H4_RAMP_FF_DEG        -0.60f
#define H4_RUN_FF_DEG           0.40f
#define H4_DOWN_FF_DEG          0.60f
#define H4_SLOWDOWN_COUNT     340000L
#define H4_SLOW_PWM           80
#define H4_ENCODER_STOP_COUNT 430000L
#define H4_COAST_MS           150U
#define H5_BASE_PWM           220
#define H5_TRACK_KP           8
#define H5_RAMP_UP_MS         2000U
#define H5_LIFT_START_PWM     80
#define H5_STOP_RAMP_MS       3000U
#define H5_MARKER_BLACK_COUNT 4U
#define H5_RAMP_FF_DEG        -0.80f
#define H5_RUN_BIAS_FF_DEG     0.10f
#define H5_RAMP_FF_IN_MS       300U
#define H5_RAMP_FF_OUT_MS      500U
#define H5_STOP_FF_DEG         0.30f
#define H5_STOP_FF_IN_MS       200U
#define H5_STOP_FF_OUT_MS      600U
#define START_DELAY_MS        500U
#define BALANCE_START_ENCODER_COUNT 10L
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
#define H3_STABLE_RANGE_CM           0.5f
#define H3_FINISH_TOLERANCE_CM        0.4f
#define H3_CENTER_TOLERANCE_CM       1.0f
#define H3_CENTER_STABLE_MS         500U
#define H3_TIMEOUT_MS            5000U
#define H3_BRAKE_START_CM           -1.0f
#define H3_POSITION_KP                 0.50f
#define H3_POSITION_KI                 0.20f
#define H3_POSITION_KD                 0.45f
#define H3_BRAKE_KD                    0.65f
#define H3_ANGLE_KP                  100.0f
#define H3_ANGLE_KI                  100.0f
#define H3_ANGLE_KD                    0.5f

/* H4/H5/H6分别保留独立参数，调试一个模式不会影响其他模式。 */
#define H4_POSITION_KP                 0.50f
#define H4_POSITION_KI                 0.20f
#define H4_POSITION_KD                 0.45f
#define H4_ANGLE_KP                  100.0f
#define H4_ANGLE_KI                  100.0f
#define H4_ANGLE_KD                    0.5f

#define H5_POSITION_KP                 0.60f  /* 实测回退值，兼顾纠偏速度和超调。 */
#define H5_POSITION_KI                 0.30f
#define H5_POSITION_KD                 0.70f  /* 实测综合最优，兼顾峰值和回正速度。 */
#define H5_ANGLE_KP                  100.0f
#define H5_ANGLE_KI                  100.0f
#define H5_ANGLE_KD                    0.5f

#define H6_POSITION_KP                 0.50f
#define H6_POSITION_KI                 0.20f
#define H6_POSITION_KD                 0.45f
#define H6_ANGLE_KP                  100.0f
#define H6_ANGLE_KI                  100.0f
#define H6_ANGLE_KD                    0.5f

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
static uint8_t start_delay_active;
static uint32_t start_delay_time_ms;
static uint8_t h3_centering;
static uint8_t h3_returning;
static uint8_t h3_braking;
static uint32_t h3_stable_start_ms;
static float h3_turn_position_cm;
static float h3_stable_min_cm;
static float h3_stable_max_cm;

static uint8_t h2_marker_count;
static uint8_t h2_marker_active;
static uint8_t h2_straight_active;
static uint8_t h2_reverse_active;
static uint8_t h2_stop_locked;
static uint32_t h2_straight_start_ms;
static uint32_t h2_reverse_start_ms;
static int16_t h4_drive_pwm;
static const char *h4_drive_phase;
static uint8_t h4_coast_active;
static uint32_t h4_coast_start_ms;
static uint32_t h4_stable_start_ms;
static float h4_stable_min_cm;
static float h4_stable_max_cm;
static uint8_t h4_flat_done;
static uint8_t h4_balance_started;
static int32_t h4_start_encoder3;
static int32_t h4_start_encoder4;
static int16_t h5_drive_pwm;
static int16_t h5_stop_start_pwm;
static const char *h5_drive_phase;
static uint8_t h5_marker_detected;
static uint8_t h5_stop_locked;
static uint8_t h5_black_count;
static uint8_t h5_gray;
static const char *h5_bend_state;
static uint32_t h5_stop_start_ms;
static uint32_t h5_stable_start_ms;
static float h5_stable_min_cm;
static float h5_stable_max_cm;
static uint8_t h5_flat_done;
static uint8_t h5_balance_started;
static uint32_t h5_balance_start_ms;
static int32_t h5_start_encoder3;
static int32_t h5_start_encoder4;

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
static uint8_t encoder_total_buffer[80];
static uint32_t encoder_total_time_ms;
static uint8_t h4_debug_buffer[256];
static uint32_t h4_debug_time_ms;
static uint8_t h5_debug_buffer[320];
static uint32_t h5_debug_time_ms;

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
extern volatile uint32_t g_vision_rx_event_count;
extern volatile uint32_t g_vision_rx_byte_count;
extern volatile uint32_t g_vision_rx_restart_fail_count;
extern volatile uint32_t g_vision_uart_error_count;
extern volatile uint32_t g_vision_uart_last_error;

static void State_Start(uint32_t now_ms);
static void State_RunBallControl(uint32_t now_ms);
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

    if ((current_page == STATE_PAGE_RUNNING) &&
        (start_delay_active == 0U))
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

static void State_FormatTimeTenths(char *output, size_t size, uint32_t elapsed_ms)
{
    (void)snprintf(output,
                   size,
                   "TIME:%03lu.%luS",
                   (unsigned long)(elapsed_ms / 1000U),
                   (unsigned long)((elapsed_ms % 1000U) / 100U));
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
    case STATE_PAGE_RUNNING:
        state_text = (start_delay_active != 0U) ? "STATE:WAIT 0.5S" : "STATE:RUNNING";
        break;
    case STATE_PAGE_FINISHED: state_text = "STATE:FINISHED"; break;
    case STATE_PAGE_TIMEOUT:  state_text = "STATE:TIMEOUT";  break;
    default:                  state_text = "STATE:STOPPED";  break;
    }

    if (((current_mode == STATE_MODE_H4_AB_BALANCE) ||
         (current_mode == STATE_MODE_H5_LOOP_CENTER)) &&
        (current_page == STATE_PAGE_FINISHED))
    {
        /* H4/H5完成页显示冻结时间，精确到0.1秒。 */
        State_FormatTimeTenths(time_text, sizeof(time_text), stopped_elapsed_ms);
    }
    else
    {
        State_FormatTime(time_text,
                         sizeof(time_text),
                         State_GetElapsedHalfSeconds(now_ms));
    }
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

static uint8_t State_H4Run(uint32_t now_ms)
{
    int32_t encoder_max;
    int32_t slowdown_progress;
    uint32_t ramp_elapsed_ms;
    int16_t turn;

    /* H4不检测黑线停车；任一路编码器达到目标后立即制动。 */
    if ((Encoder3_GetTotal() >= H4_ENCODER_STOP_COUNT) ||
        (Encoder4_GetTotal() >= H4_ENCODER_STOP_COUNT))
    {
        h4_drive_pwm = 0;
        h4_drive_phase = "COAST";
        h4_coast_active = 1U;
        h4_coast_start_ms = now_ms;
        Moter_A(0);
        Moter_B(0);
        return 1U;
    }

    encoder_max = Encoder3_GetTotal();
    if (Encoder4_GetTotal() > encoder_max)
    {
        encoder_max = Encoder4_GetTotal();
    }

    ramp_elapsed_ms = (uint32_t)(now_ms - start_time_ms);
    if (ramp_elapsed_ms < H4_RAMP_UP_MS)
    {
        h4_drive_phase = "UP";
        h4_drive_pwm = (int16_t)(((uint32_t)H4_BASE_PWM * ramp_elapsed_ms) /
                                 H4_RAMP_UP_MS);
    }
    else if (encoder_max >= H4_SLOWDOWN_COUNT)
    {
        h4_drive_phase = "DOWN";
        slowdown_progress = encoder_max - H4_SLOWDOWN_COUNT;
        h4_drive_pwm = (int16_t)(H4_BASE_PWM -
            ((int32_t)(H4_BASE_PWM - H4_SLOW_PWM) * slowdown_progress) /
            (H4_ENCODER_STOP_COUNT - H4_SLOWDOWN_COUNT));
        if (h4_drive_pwm < H4_SLOW_PWM)
        {
            h4_drive_pwm = H4_SLOW_PWM;
        }
    }
    else
    {
        h4_drive_phase = "RUN";
        h4_drive_pwm = H4_BASE_PWM;
    }

    /* 缓启动时保留完整循迹力度，但不允许修正量大于当前基础PWM。 */
    turn = (int16_t)(Gray_GetError() * H4_TRACK_KP);
    if (turn > h4_drive_pwm)
    {
        turn = h4_drive_pwm;
    }
    else if (turn < -h4_drive_pwm)
    {
        turn = -h4_drive_pwm;
    }
    Moter_A(-h4_drive_pwm - turn);
    Moter_B(-h4_drive_pwm + turn);
    return 0U;
}

static uint8_t State_H5Run(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int16_t turn;
    uint8_t first_marker = 0U;

    h5_gray = Gray_Read();
    h5_black_count = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((h5_gray & (uint8_t)(1U << i)) == 0U)
        {
            h5_black_count++;
        }
    }

    /* 实测右弯先压到HUI6；HUI6、HUI7同时黑记为深弯R2。 */
    if ((h5_gray & 0x20U) == 0U)
    {
        h5_bend_state = ((h5_gray & 0x40U) == 0U) ? "R2" : "R1";
    }
    else
    {
        h5_bend_state = "NONE";
    }

    if (h5_stop_locked != 0U)
    {
        State_H2Brake();
        return 0U;
    }

    if (h5_marker_detected == 0U)
    {
        elapsed_ms = (uint32_t)(now_ms - start_time_ms);
        if (elapsed_ms < H5_RAMP_UP_MS)
        {
            h5_drive_phase = "UP";
            h5_drive_pwm = (int16_t)(((uint32_t)H5_BASE_PWM * elapsed_ms) /
                                     H5_RAMP_UP_MS);
        }
        else
        {
            h5_drive_phase = "RUN";
            h5_drive_pwm = H5_BASE_PWM;
        }

        if (h5_black_count >= H5_MARKER_BLACK_COUNT)
        {
            /* 第一次4黑立即完成并冻结题目时间，内部继续执行3秒缓停。 */
            h5_marker_detected = 1U;
            h5_stop_start_ms = now_ms;
            h5_stop_start_pwm = h5_drive_pwm;
            h5_drive_phase = "STOP";
            first_marker = 1U;
        }
    }
    else
    {
        elapsed_ms = (uint32_t)(now_ms - h5_stop_start_ms);
        if (elapsed_ms >= H5_STOP_RAMP_MS)
        {
            h5_drive_pwm = 0;
            h5_drive_phase = "DONE";
            h5_stop_locked = 1U;
            /* 缓停前馈撤销时清空位置环积分，避免旧补偿继续推球。 */
            BallControl_SetAngleFeedforward(0.0f);
            BallControl_SetTargetPosition(0.0f);
            State_H2Brake();
            return 0U;
        }

        h5_drive_phase = "STOP";
        h5_drive_pwm = (int16_t)(((uint32_t)h5_stop_start_pwm *
                                  (H5_STOP_RAMP_MS - elapsed_ms)) /
                                 H5_STOP_RAMP_MS);
    }

    turn = (int16_t)(Gray_GetError() * H5_TRACK_KP);
    if (turn > h5_drive_pwm) turn = h5_drive_pwm;
    if (turn < -h5_drive_pwm) turn = -h5_drive_pwm;

    /* 左轮B、右轮A，负参数前进；缓停期间继续循迹。 */
    Moter_A(-h5_drive_pwm - turn);
    Moter_B(-h5_drive_pwm + turn);
    return first_marker;
}

static void State_H5FinishBalance(uint32_t now_ms)
{
    float position;

    if (h5_stop_locked == 0U)
    {
        return;
    }

    State_H2Brake();
    if (h5_flat_done != 0U)
    {
        return;
    }

    if (g_shijue_position_valid == 0U)
    {
        h5_stable_start_ms = 0U;
        return;
    }

    position = g_shijue_position_cm;
    if ((position < -H3_FINISH_TOLERANCE_CM) ||
        (position > H3_FINISH_TOLERANCE_CM))
    {
        h5_stable_start_ms = 0U;
        return;
    }

    if (h5_stable_start_ms == 0U)
    {
        h5_stable_start_ms = now_ms;
        h5_stable_min_cm = position;
        h5_stable_max_cm = position;
        return;
    }

    if (position < h5_stable_min_cm) h5_stable_min_cm = position;
    if (position > h5_stable_max_cm) h5_stable_max_cm = position;
    if ((uint32_t)(now_ms - h5_stable_start_ms) < H3_STABLE_MS)
    {
        return;
    }

    if ((h5_stable_max_cm - h5_stable_min_cm) <= H3_STABLE_RANGE_CM)
    {
        /* 停车且小球稳定后关闭闭环，舵机自动回到1730us水平位。 */
        h5_flat_done = 1U;
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetEnabled(0U);
    }
    else
    {
        h5_stable_start_ms = now_ms;
        h5_stable_min_cm = position;
        h5_stable_max_cm = position;
    }
}

static void State_H4FinishStop(uint32_t now_ms)
{
    if ((h4_coast_active != 0U) &&
        ((uint32_t)(now_ms - h4_coast_start_ms) < H4_COAST_MS))
    {
        Moter_A(0);
        Moter_B(0);
        return;
    }

    h4_coast_active = 0U;
    h4_drive_phase = "DONE";
    State_H2Brake();
}

static void State_H4FinishBalance(uint32_t now_ms)
{
    float position;

    State_H4FinishStop(now_ms);
    if (h4_flat_done != 0U)
    {
        return;
    }

    State_RunBallControl(now_ms);
    if (g_shijue_position_valid == 0U)
    {
        h4_stable_start_ms = 0U;
        return;
    }

    position = g_shijue_position_cm;
    if ((position < -H3_FINISH_TOLERANCE_CM) ||
        (position > H3_FINISH_TOLERANCE_CM))
    {
        h4_stable_start_ms = 0U;
        return;
    }

    if (h4_stable_start_ms == 0U)
    {
        h4_stable_start_ms = now_ms;
        h4_stable_min_cm = position;
        h4_stable_max_cm = position;
        return;
    }

    if (position < h4_stable_min_cm) h4_stable_min_cm = position;
    if (position > h4_stable_max_cm) h4_stable_max_cm = position;
    if ((uint32_t)(now_ms - h4_stable_start_ms) < H3_STABLE_MS)
    {
        return;
    }

    if ((h4_stable_max_cm - h4_stable_min_cm) <= H3_STABLE_RANGE_CM)
    {
        /* 与H3相同的判稳条件满足后，关闭闭环并回到1730us机械水平位。 */
        h4_flat_done = 1U;
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetEnabled(0U);
    }
    else
    {
        h4_stable_start_ms = now_ms;
        h4_stable_min_cm = position;
        h4_stable_max_cm = position;
    }
}

static void State_RunBallControl(uint32_t now_ms)
{
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

static void State_End(StatePage_t page, uint32_t now_ms)
{
    if (current_page == STATE_PAGE_RUNNING)
    {
        stopped_elapsed_ms = (start_delay_active != 0U) ? 0U :
                             (now_ms - start_time_ms);
        start_delay_active = 0U;
        current_page = page;
        if ((current_mode == STATE_MODE_H3_BALL_MOVE) &&
            ((page == STATE_PAGE_FINISHED) || (page == STATE_PAGE_TIMEOUT)))
        {
            /* H3结束后退出闭环并回机械中位，避免位置环继续驱动造成震荡。 */
            BallControl_SetEnabled(0U);
        }
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

static void State_LoadBallPid(StateMode_t mode)
{
    switch (mode)
    {
        case STATE_MODE_H3_BALL_MOVE:
            BallControl_SetPositionPid(H3_POSITION_KP, H3_POSITION_KI, H3_POSITION_KD);
            BallControl_SetAnglePid(H3_ANGLE_KP, H3_ANGLE_KI, H3_ANGLE_KD);
            break;

        case STATE_MODE_H4_AB_BALANCE:
            BallControl_SetPositionPid(H4_POSITION_KP, H4_POSITION_KI, H4_POSITION_KD);
            BallControl_SetAnglePid(H4_ANGLE_KP, H4_ANGLE_KI, H4_ANGLE_KD);
            break;

        case STATE_MODE_H5_LOOP_CENTER:
            BallControl_SetPositionPid(H5_POSITION_KP, H5_POSITION_KI, H5_POSITION_KD);
            BallControl_SetAnglePid(H5_ANGLE_KP, H5_ANGLE_KI, H5_ANGLE_KD);
            break;

        case STATE_MODE_H6_LOOP_TARGET:
            BallControl_SetPositionPid(H6_POSITION_KP, H6_POSITION_KI, H6_POSITION_KD);
            BallControl_SetAnglePid(H6_ANGLE_KP, H6_ANGLE_KI, H6_ANGLE_KD);
            break;

        default:
            break;
    }
}

static uint8_t State_WheelHasMoved(int32_t encoder3_start,
                                   int32_t encoder4_start)
{
    int32_t encoder3_delta = Encoder3_GetTotal() - encoder3_start;
    int32_t encoder4_delta = Encoder4_GetTotal() - encoder4_start;

    if (encoder3_delta < 0) encoder3_delta = -encoder3_delta;
    if (encoder4_delta < 0) encoder4_delta = -encoder4_delta;
    return ((encoder3_delta >= BALANCE_START_ENCODER_COUNT) ||
            (encoder4_delta >= BALANCE_START_ENCODER_COUNT));
}

static float State_GetH5RunFeedforward(uint32_t now_ms)
{
    uint32_t balance_elapsed_ms = now_ms - h5_balance_start_ms;
    uint32_t ramp_elapsed_ms = now_ms - start_time_ms;

    /* 稳球刚启动时渐入，避免舵机从机械中位突然跳到前馈角度。 */
    if (balance_elapsed_ms < H5_RAMP_FF_IN_MS)
    {
        return H5_RAMP_FF_DEG * (float)balance_elapsed_ms /
               (float)H5_RAMP_FF_IN_MS;
    }

    /* 缓启动结束前平滑过渡到恒速偏置，补偿实测负向固定偏差。 */
    if (ramp_elapsed_ms < (H5_RAMP_UP_MS - H5_RAMP_FF_OUT_MS))
    {
        return H5_RAMP_FF_DEG;
    }
    if (ramp_elapsed_ms < H5_RAMP_UP_MS)
    {
        float transition = (float)(ramp_elapsed_ms -
                           (H5_RAMP_UP_MS - H5_RAMP_FF_OUT_MS)) /
                           (float)H5_RAMP_FF_OUT_MS;
        return H5_RAMP_FF_DEG +
               (H5_RUN_BIAS_FF_DEG - H5_RAMP_FF_DEG) * transition;
    }
    return H5_RUN_BIAS_FF_DEG;
}

static float State_GetH5StopFeedforward(uint32_t now_ms)
{
    uint32_t stop_elapsed_ms = now_ms - h5_stop_start_ms;

    /* 缓停前馈先渐入，停车前再渐出，避免两端角度突变。 */
    if (stop_elapsed_ms < H5_STOP_FF_IN_MS)
    {
        return H5_STOP_FF_DEG * (float)stop_elapsed_ms /
               (float)H5_STOP_FF_IN_MS;
    }
    if (stop_elapsed_ms < (H5_STOP_RAMP_MS - H5_STOP_FF_OUT_MS))
    {
        return H5_STOP_FF_DEG;
    }
    if (stop_elapsed_ms < H5_STOP_RAMP_MS)
    {
        return H5_STOP_FF_DEG *
               (float)(H5_STOP_RAMP_MS - stop_elapsed_ms) /
               (float)H5_STOP_FF_OUT_MS;
    }
    return 0.0f;
}

static void State_ActivateStart(uint32_t now_ms)
{
    start_delay_active = 0U;
    start_time_ms = now_ms;
    last_display_half_second = 0xFFFFFFFFU;

    if (current_mode == STATE_MODE_H2_CAR_LOOP)
    {
        State_H2Reset();
    }
    else if (current_mode == STATE_MODE_H4_AB_BALANCE)
    {
        /* H4按下开始时两路编码器清零，从0累计到目标距离。 */
        Encoder3_Reset();
        Encoder4_Reset();
        h4_start_encoder3 = Encoder3_GetTotal();
        h4_start_encoder4 = Encoder4_GetTotal();
        h4_drive_pwm = 0;
        h4_drive_phase = "UP";
        h4_coast_active = 0U;
        h4_coast_start_ms = 0U;
        h4_stable_start_ms = 0U;
        h4_stable_min_cm = 0.0f;
        h4_stable_max_cm = 0.0f;
        h4_flat_done = 0U;
        h4_balance_started = 0U;
        State_LoadBallPid(current_mode);
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(0U);
    }
    else if (current_mode == STATE_MODE_H5_LOOP_CENTER)
    {
        h5_drive_pwm = 0;
        h5_stop_start_pwm = 0;
        h5_drive_phase = "UP";
        h5_marker_detected = 0U;
        h5_stop_locked = 0U;
        h5_black_count = 0U;
        h5_gray = Gray_Read();
        h5_bend_state = "NONE";
        h5_stop_start_ms = 0U;
        h5_stable_start_ms = 0U;
        h5_stable_min_cm = 0.0f;
        h5_stable_max_cm = 0.0f;
        h5_flat_done = 0U;
        h5_balance_started = 0U;
        h5_balance_start_ms = 0U;
        h5_start_encoder3 = Encoder3_GetTotal();
        h5_start_encoder4 = Encoder4_GetTotal();
        State_LoadBallPid(current_mode);
        BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(0U);
    }
    else if (current_mode == STATE_MODE_H3_BALL_MOVE)
    {
        h3_centering = 1U;
        h3_returning = 0U;
        h3_braking = 0U;
        h3_stable_start_ms = 0U;
        h3_turn_position_cm = 0.0f;
        h3_stable_min_cm = 0.0f;
        h3_stable_max_cm = 0.0f;
        /* 先回到中心并稳定，再从0开始计入题目要求的5秒。 */
        State_LoadBallPid(current_mode);
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(1U);
    }
    else if (current_mode == STATE_MODE_H6_LOOP_TARGET)
    {
        State_LoadBallPid(current_mode);
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(1U);
    }
    display_dirty = 1U;
}

static void State_Start(uint32_t now_ms)
{
    if (current_page != STATE_PAGE_READY)
    {
        return;
    }

    stopped_elapsed_ms = 0U;
    current_page = STATE_PAGE_RUNNING;
    start_delay_active = 1U;
    start_delay_time_ms = now_ms;
    start_time_ms = now_ms;
    last_display_half_second = 0xFFFFFFFFU;

    /* 所有模式的0.5秒等待期间，车轮、球控和舵机都保持静止。 */
    State_H2Brake();
    BallControl_SetEnabled(0U);
    BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
    display_dirty = 1U;
}

static void State_ProcessStartDelay(uint32_t now_ms)
{
    if (start_delay_active == 0U)
    {
        return;
    }

    /* 等待期间每轮强制钳制，禁止任何残留控制输出改变车轮或舵机。 */
    State_H2Brake();
    BallControl_SetEnabled(0U);
    BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);

    if ((uint32_t)(now_ms - start_delay_time_ms) >= START_DELAY_MS)
    {
        State_ActivateStart(now_ms);
    }
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
            else if (current_mode == STATE_MODE_H4_AB_BALANCE)
            {
                State_H2Brake();
                BallControl_SetEnabled(0U);
            }
            else if (current_mode == STATE_MODE_H5_LOOP_CENTER)
            {
                State_H2Brake();
                BallControl_SetEnabled(0U);
            }
            else if ((current_mode == STATE_MODE_H6_LOOP_TARGET) ||
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
            if (current_mode == STATE_MODE_H5_LOOP_CENTER)
            {
                /* 提前退出完成页时先锁止，再重新执行稳球零点标定。 */
                State_H2Brake();
                h5_stop_locked = 1U;
                State_BeginBalanceCalibration(now_ms, 0U);
            }
            else
            {
                State_BeginBalanceCalibration(now_ms, 0U);
            }
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

    case STATE_MODE_H6_LOOP_TARGET:
    case STATE_MODE_H3_BALL_MOVE:
        State_RunBallControl(now_ms);

        if (current_mode != STATE_MODE_H3_BALL_MOVE)
        {
            break;
        }

        if (h3_centering != 0U)
        {
            if ((g_shijue_position_valid != 0U) &&
                (g_shijue_position_cm >= -H3_CENTER_TOLERANCE_CM) &&
                (g_shijue_position_cm <= H3_CENTER_TOLERANCE_CM))
            {
                if (h3_stable_start_ms == 0U)
                {
                    h3_stable_start_ms = now_ms;
                }
                else if ((uint32_t)(now_ms - h3_stable_start_ms) >= H3_CENTER_STABLE_MS)
                {
                    h3_centering = 0U;
                    h3_stable_start_ms = 0U;
                    start_time_ms = now_ms;
                    BallControl_SetTargetPosition(H3_POSITIVE_TARGET_CM);
                }
            }
            else
            {
                h3_stable_start_ms = 0U;
            }
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
            h3_turn_position_cm = g_shijue_position_cm;
            BallControl_SetTargetPosition(H3_NEGATIVE_TARGET_CM);
            break;
        }

        if ((h3_braking == 0U) &&
            (g_shijue_position_cm <= H3_BRAKE_START_CM))
        {
            /* 折返后接近-5 cm再增强速度反馈，兼顾运行时间和制动。 */
            h3_braking = 1U;
            BallControl_SetPositionPid(H3_POSITION_KP,
                                       H3_POSITION_KI,
                                       H3_BRAKE_KD);
        }

        if ((g_shijue_position_cm >= (H3_NEGATIVE_TARGET_CM - H3_FINISH_TOLERANCE_CM)) &&
            (g_shijue_position_cm <= (H3_NEGATIVE_TARGET_CM + H3_FINISH_TOLERANCE_CM)))
        {
            if (h3_stable_start_ms == 0U)
            {
                h3_stable_start_ms = now_ms;
                h3_stable_min_cm = g_shijue_position_cm;
                h3_stable_max_cm = g_shijue_position_cm;
            }
            else
            {
                if (g_shijue_position_cm < h3_stable_min_cm)
                {
                    h3_stable_min_cm = g_shijue_position_cm;
                }
                if (g_shijue_position_cm > h3_stable_max_cm)
                {
                    h3_stable_max_cm = g_shijue_position_cm;
                }
                if ((uint32_t)(now_ms - h3_stable_start_ms) >= H3_STABLE_MS)
                {
                    if ((h3_stable_max_cm - h3_stable_min_cm) <= H3_STABLE_RANGE_CM)
                    {
                        State_End(STATE_PAGE_FINISHED, now_ms);
                    }
                    else
                    {
                        /* 穿越目标区不算稳定，从当前位置重新统计500 ms。 */
                        h3_stable_start_ms = now_ms;
                        h3_stable_min_cm = g_shijue_position_cm;
                        h3_stable_max_cm = g_shijue_position_cm;
                    }
                }
            }
        }
        else
        {
            h3_stable_start_ms = 0U;
        }
        break;

    case STATE_MODE_H5_LOOP_CENTER:
    {
        uint8_t finished = State_H5Run(now_ms);

        if (h5_balance_started == 0U)
        {
            if (State_WheelHasMoved(h5_start_encoder3,
                                    h5_start_encoder4) == 0U)
            {
                BallControl_SetEnabled(0U);
                BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
            }
            else
            {
                /* 编码器确认车轮实际转动后，才同步启动稳球。 */
                h5_balance_started = 1U;
                h5_balance_start_ms = now_ms;
                BallControl_SetEnabled(1U);
            }
        }

        if (h5_balance_started != 0U)
        {
            BallControl_SetAngleFeedforward(State_GetH5RunFeedforward(now_ms));
            State_RunBallControl(now_ms);
        }

        if (finished != 0U)
        {
            /* State_End只冻结题目时间，H5完成页仍会继续执行内部缓停。 */
            State_End(STATE_PAGE_FINISHED, now_ms);
        }
        break;
    }

    case STATE_MODE_H4_AB_BALANCE:
    {
        uint8_t finished = State_H4Run(now_ms);

        if (h4_balance_started == 0U)
        {
            if (State_WheelHasMoved(h4_start_encoder3,
                                    h4_start_encoder4) == 0U)
            {
                BallControl_SetEnabled(0U);
                BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
            }
            else
            {
                h4_balance_started = 1U;
                BallControl_SetEnabled(1U);
            }
        }

        /* H4分阶段补偿：加速抑制惯性前冲，恒速补偿运行中的固定偏移。 */
        if (h4_balance_started == 0U)
        {
            BallControl_SetAngleFeedforward(0.0f);
        }
        else if ((uint32_t)(now_ms - start_time_ms) < H4_RAMP_UP_MS)
        {
            BallControl_SetAngleFeedforward(H4_RAMP_FF_DEG);
        }
        else if ((Encoder3_GetTotal() < H4_SLOWDOWN_COUNT) &&
                 (Encoder4_GetTotal() < H4_SLOWDOWN_COUNT))
        {
            BallControl_SetAngleFeedforward(H4_RUN_FF_DEG);
        }
        else if ((Encoder3_GetTotal() < H4_ENCODER_STOP_COUNT) &&
                 (Encoder4_GetTotal() < H4_ENCODER_STOP_COUNT))
        {
            BallControl_SetAngleFeedforward(H4_DOWN_FF_DEG);
        }
        else
        {
            BallControl_SetAngleFeedforward(0.0f);
        }
        if (h4_balance_started != 0U)
        {
            State_RunBallControl(now_ms);
        }
        if (finished != 0U)
        {
            State_End(STATE_PAGE_FINISHED, now_ms);
        }
        break;
    }

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
    char target_position[12];
    char velocity[12];
    char position_error[12];
    char h3_turn_position[12];
    char h3_stable_min[12];
    char h3_stable_max[12];
    int length;

    if ((debug_reply_ready != 0U) ||
        ((uint32_t)(now_ms - balance_debug_time_ms) < 50U) ||
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
    State_FormatSignedCenti(target_position, sizeof(target_position),
                            BallControl_GetTargetPosition());
    State_FormatSignedCenti(velocity, sizeof(velocity), g_shijue_velocity_cm_s);
    State_FormatSignedCenti(position_error, sizeof(position_error),
                            BallControl_GetTargetPosition() - BallControl_GetPosition());

    if (current_mode == STATE_MODE_H3_BALL_MOVE)
    {
        const char *phase;

        if (current_page == STATE_PAGE_FINISHED) phase = "DONE";
        else if (current_page == STATE_PAGE_TIMEOUT) phase = "TIMEOUT";
        else if (current_page != STATE_PAGE_RUNNING) phase = "READY";
        else if (h3_centering != 0U) phase = "CENTER";
        else if (h3_returning == 0U) phase = "PLUS";
        else phase = "MINUS";

        State_FormatSignedCenti(h3_turn_position, sizeof(h3_turn_position),
                                h3_turn_position_cm);
        State_FormatSignedCenti(h3_stable_min, sizeof(h3_stable_min),
                                h3_stable_min_cm);
        State_FormatSignedCenti(h3_stable_max, sizeof(h3_stable_max),
                                h3_stable_max_cm);
        length = snprintf((char *)balance_debug_buffer,
                          sizeof(balance_debug_buffer),
                          "H3 T=%lu E=%lu S=%s TP=%s P=%s V=%s TURN=%s RANGE=%s,%s\r\n",
                          (unsigned long)now_ms,
                          (unsigned long)((h3_centering != 0U) ? 0U :
                                          ((current_page == STATE_PAGE_RUNNING) ?
                                           (now_ms - start_time_ms) :
                                           stopped_elapsed_ms)),
                          phase,
                          target_position,
                          position,
                          velocity,
                          h3_turn_position,
                          h3_stable_min,
                          h3_stable_max);
    }
    else
    {
    /* 20 Hz遥测匹配115200串口带宽；控制内环仍保持100 Hz。 */
    length = snprintf((char *)balance_debug_buffer,
                      sizeof(balance_debug_buffer),
                      "CTRL T=%lu P=%s TP=%s PV=%u VEL=%s VV=%u PE=%s VC=%u TG=%s ANG=%s W=%s PWM=%u RAW=%s Z=%s IV=%u%u AGE=%lu JRX=%lu/%lu RX=%lu/%lu\r\n",
                      (unsigned long)now_ms,
                      position,
                      target_position,
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
    }

    if ((length > 0) && ((size_t)length < sizeof(balance_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         balance_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        balance_debug_time_ms = now_ms;
    }
}

static void State_SendEncoderTotal(uint32_t now_ms)
{
    int length;

    if (((uint32_t)(now_ms - encoder_total_time_ms) < 1000U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    /* TIM3为左轮B，TIM4为右轮A；累计值从上电初始化后持续保留。 */
    length = snprintf((char *)encoder_total_buffer,
                      sizeof(encoder_total_buffer),
                      "ENC_TOTAL TIM3_LEFT=%ld TIM4_RIGHT=%ld\r\n",
                      (long)Encoder3_GetTotal(),
                      (long)Encoder4_GetTotal());

    if ((length > 0) && ((size_t)length < sizeof(encoder_total_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         encoder_total_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        encoder_total_time_ms = now_ms;
    }
}

static void State_SendH4Debug(uint32_t now_ms)
{
    char position[12];
    char velocity[12];
    char target_angle[12];
    char pipe_angle[12];
    char pipe_gyro[12];
    const char *phase;
    const char *balance_phase;
    int length;

    if ((current_mode != STATE_MODE_H4_AB_BALANCE) ||
        ((uint32_t)(now_ms - h4_debug_time_ms) < 50U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    if (current_page == STATE_PAGE_FINISHED) phase = "DONE";
    else if (current_page == STATE_PAGE_STOPPED) phase = "STOP";
    else if (current_page == STATE_PAGE_READY) phase = "READY";
    else if (start_delay_active != 0U) phase = "WAIT";
    else phase = "RUN";

    if (h4_balance_started == 0U) balance_phase = "HOLD";
    else if (h4_flat_done != 0U) balance_phase = "FLAT";
    else if ((current_page == STATE_PAGE_FINISHED) &&
             (h4_stable_start_ms != 0U)) balance_phase = "CHECK";
    else if (current_page == STATE_PAGE_FINISHED) balance_phase = "BAL";
    else balance_phase = "RUN";

    State_FormatSignedCenti(position, sizeof(position), BallControl_GetPosition());
    State_FormatSignedCenti(velocity, sizeof(velocity), g_shijue_velocity_cm_s);
    State_FormatSignedCenti(target_angle, sizeof(target_angle),
                            BallControl_GetTargetAngle());
    State_FormatSignedCenti(pipe_angle, sizeof(pipe_angle),
                            BallControl_GetPipeAngle());
    State_FormatSignedCenti(pipe_gyro, sizeof(pipe_gyro),
                            BallControl_GetPipeGyro());

    /* H4调参遥测：同时带视觉状态，便于区分真实零位和未收到数据。 */
    length = snprintf((char *)h4_debug_buffer,
                      sizeof(h4_debug_buffer),
                      "H4 E=%lu S=%s D=%s BS=%s BP=%d L=%ld R=%ld LS=%ld RS=%ld G=%02X GE=%d P=%s PV=%u V=%s VV=%u VC=%u VT=%02X VS=%u VE=%u VRX=%lu/%lu RF=%lu UE=%lu/%lX TG=%s A=%s W=%s PWM=%u\r\n",
                      (unsigned long)((current_page == STATE_PAGE_RUNNING &&
                                      start_delay_active == 0U) ?
                                      (now_ms - start_time_ms) :
                      stopped_elapsed_ms),
                      phase,
                      h4_drive_phase,
                      balance_phase,
                      (int)h4_drive_pwm,
                      (long)Encoder3_GetTotal(),
                      (long)Encoder4_GetTotal(),
                      (long)Encoder3_GetSpeedCps(),
                      (long)Encoder4_GetSpeedCps(),
                      (unsigned int)Gray_Read(),
                      (int)Gray_GetError(),
                      position,
                      (unsigned int)g_shijue_position_valid,
                      velocity,
                      (unsigned int)g_shijue_velocity_valid,
                      (unsigned int)g_shijue_count,
                      (unsigned int)g_shijue_last_type,
                      (unsigned int)g_shijue_last_seq,
                      (unsigned int)g_shijue_error_flag,
                      (unsigned long)g_vision_rx_event_count,
                      (unsigned long)g_vision_rx_byte_count,
                      (unsigned long)g_vision_rx_restart_fail_count,
                      (unsigned long)g_vision_uart_error_count,
                      (unsigned long)g_vision_uart_last_error,
                      target_angle,
                      pipe_angle,
                      pipe_gyro,
                      (unsigned int)BallControl_GetServoPulse());

    if ((length > 0) && ((size_t)length < sizeof(h4_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         h4_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        h4_debug_time_ms = now_ms;
    }
}

static void State_SendH5Debug(uint32_t now_ms)
{
    char position[12];
    char velocity[12];
    char target_angle[12];
    char pipe_angle[12];
    char pipe_gyro[12];
    const char *state;
    const char *balance_phase;
    uint32_t elapsed_ms;
    int length;

    if ((current_mode != STATE_MODE_H5_LOOP_CENTER) ||
        ((uint32_t)(now_ms - h5_debug_time_ms) < 50U) ||
        (BSP_UartTxReady(BSP_UART_2) == 0U))
    {
        return;
    }

    if (current_page == STATE_PAGE_FINISHED) state = "FIN";
    else if (current_page == STATE_PAGE_STOPPED) state = "STOP";
    else if (current_page == STATE_PAGE_READY) state = "READY";
    else if (start_delay_active != 0U) state = "WAIT";
    else state = "RUN";

    if (h5_balance_started == 0U) balance_phase = "HOLD";
    else if (h5_flat_done != 0U) balance_phase = "FLAT";
    else if ((current_page == STATE_PAGE_FINISHED) &&
             (h5_stop_locked == 0U)) balance_phase = "SLOW";
    else if ((current_page == STATE_PAGE_FINISHED) &&
             (h5_stable_start_ms != 0U)) balance_phase = "CHECK";
    else if (current_page == STATE_PAGE_FINISHED) balance_phase = "BAL";
    else balance_phase = "RUN";

    elapsed_ms = ((current_page == STATE_PAGE_RUNNING) &&
                  (start_delay_active == 0U))
                 ? (now_ms - start_time_ms)
                 : stopped_elapsed_ms;

    State_FormatSignedCenti(position, sizeof(position), BallControl_GetPosition());
    State_FormatSignedCenti(velocity, sizeof(velocity), g_shijue_velocity_cm_s);
    State_FormatSignedCenti(target_angle, sizeof(target_angle),
                            BallControl_GetTargetAngle());
    State_FormatSignedCenti(pipe_angle, sizeof(pipe_angle),
                            BallControl_GetPipeAngle());
    State_FormatSignedCenti(pipe_gyro, sizeof(pipe_gyro),
                            BallControl_GetPipeGyro());

    /* H5遥测同时记录车辆、视觉、稳球闭环和补偿阶段。 */
    length = snprintf((char *)h5_debug_buffer,
                      sizeof(h5_debug_buffer),
                      "H5 E=%lu S=%s D=%s BS=%s BP=%d G=%02X GE=%d BLACK=%u MARK=%u BEND=%s L=%ld R=%ld LS=%ld RS=%ld P=%s PV=%u V=%s VV=%u TG=%s A=%s W=%s PWM=%u IMU=%u AGE=%lu\r\n",
                      (unsigned long)elapsed_ms,
                      state,
                      h5_drive_phase,
                      balance_phase,
                      (int)h5_drive_pwm,
                      (unsigned int)h5_gray,
                      (int)Gray_GetError(),
                      (unsigned int)h5_black_count,
                      (unsigned int)h5_marker_detected,
                      h5_bend_state,
                      (long)Encoder3_GetTotal(),
                      (long)Encoder4_GetTotal(),
                      (long)Encoder3_GetSpeedCps(),
                      (long)Encoder4_GetSpeedCps(),
                      position,
                      (unsigned int)g_shijue_position_valid,
                      velocity,
                      (unsigned int)g_shijue_velocity_valid,
                      target_angle,
                      pipe_angle,
                      pipe_gyro,
                      (unsigned int)BallControl_GetServoPulse(),
                      (unsigned int)(angleValid && gyroValid),
                      (unsigned long)(now_ms - lastPacketTime));

    if ((length > 0) && ((size_t)length < sizeof(h5_debug_buffer)) &&
        (BSP_UartSendDma(BSP_UART_2,
                         h5_debug_buffer,
                         (uint16_t)length) == BSP_STATUS_OK))
    {
        h5_debug_time_ms = now_ms;
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

    /* 上电进入OLED题目菜单，由按键选择并启动对应状态。 */
    current_mode = STATE_MODE_NONE;
    current_page = STATE_PAGE_SELECT;
    selected_item = 2U;
    target_tenth_cm = 0;
    h6_target_send_pending = 0U;
    balance_roll_zero_deg = 0.0f;
    balance_wx_zero_dps = 0.0f;
    balance_roll_zero_valid = 0U;
    balance_calibration = BALANCE_CAL_IDLE;
    balance_calibration_auto_start = 0U;
    start_time_ms = 0U;
    stopped_elapsed_ms = 0U;
    start_delay_active = 0U;
    start_delay_time_ms = 0U;
    h3_centering = 0U;
    h3_returning = 0U;
    h3_braking = 0U;
    h3_stable_start_ms = 0U;
    h3_turn_position_cm = 0.0f;
    h3_stable_min_cm = 0.0f;
    h3_stable_max_cm = 0.0f;
    h4_drive_pwm = 0;
    h4_drive_phase = "IDLE";
    h4_coast_active = 0U;
    h4_coast_start_ms = 0U;
    h4_stable_start_ms = 0U;
    h4_stable_min_cm = 0.0f;
    h4_stable_max_cm = 0.0f;
    h4_flat_done = 0U;
    h4_balance_started = 0U;
    h4_start_encoder3 = 0;
    h4_start_encoder4 = 0;
    h5_drive_pwm = 0;
    h5_stop_start_pwm = 0;
    h5_drive_phase = "IDLE";
    h5_marker_detected = 0U;
    h5_stop_locked = 0U;
    h5_black_count = 0U;
    h5_gray = Gray_Read();
    h5_bend_state = "NONE";
    h5_stop_start_ms = 0U;
    h5_stable_start_ms = 0U;
    h5_stable_min_cm = 0.0f;
    h5_stable_max_cm = 0.0f;
    h5_flat_done = 0U;
    h5_balance_started = 0U;
    h5_balance_start_ms = 0U;
    h5_start_encoder3 = 0;
    h5_start_encoder4 = 0;
    vision_debug_time_ms = now_ms;
    imu_debug_time_ms = now_ms;
    balance_debug_time_ms = now_ms;
    encoder_total_time_ms = now_ms;
    h4_debug_time_ms = now_ms;
    h5_debug_time_ms = now_ms;
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
    /* USART2调参协议暂时停用，代码保留供后续恢复。 */
    // State_ProcessDebugCommand();
    // State_SendDebugReply();
    State_SendH6Target();
    State_ProcessBalanceCalibration(now_ms);

    if (current_page == STATE_PAGE_RUNNING)
    {
        State_ProcessStartDelay(now_ms);
        if (start_delay_active == 0U)
        {
            State_RunMode(now_ms);
        }
    }
    else if (((current_mode == STATE_MODE_H2_CAR_LOOP) ||
              (current_mode == STATE_MODE_H4_AB_BALANCE) ||
              (current_mode == STATE_MODE_H5_LOOP_CENTER)) &&
             ((current_page == STATE_PAGE_STOPPED) ||
              (current_page == STATE_PAGE_FINISHED)))
    {
        if ((current_mode == STATE_MODE_H4_AB_BALANCE) &&
            (current_page == STATE_PAGE_FINISHED))
        {
            /* H4停车后按H3条件判稳，随后关闭闭环并回机械水平位。 */
            State_H4FinishBalance(now_ms);
        }
        else if ((current_mode == STATE_MODE_H5_LOOP_CENTER) &&
                 (current_page == STATE_PAGE_FINISHED))
        {
            /* OLED时间已冻结，缓停结束且球稳定后再回机械水平位。 */
            BallControl_SetAngleFeedforward(State_GetH5StopFeedforward(now_ms));
            State_RunBallControl(now_ms);
            (void)State_H5Run(now_ms);
            State_H5FinishBalance(now_ms);
        }
        else
        {
            State_H2Brake();
        }
    }

    // State_SendBalanceDebug(now_ms);
    // State_SendEncoderTotal(now_ms);
    State_SendH4Debug(now_ms);
    State_SendH5Debug(now_ms);
    /* 陀螺仪解析回传先保留，后续需要时取消注释即可。 */
    // State_SendImuDebug(now_ms);
    State_UpdateDisplay(now_ms);
}
