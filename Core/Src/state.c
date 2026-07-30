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
static uint16_t balance_sample_count;
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
static uint8_t imu_debug_buffer[160];
static uint32_t imu_debug_time_ms;

static void State_Start(uint32_t now_ms);

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
        balance_roll_sum += sensorData.roll;
        balance_wx_sum += sensorData.wx;
        balance_sample_count++;
    }

    if (((uint32_t)(now_ms - balance_calibration_time_ms) >= BALANCE_SAMPLE_MS) &&
        (balance_sample_count > 0U))
    {
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
                     (current_mode == STATE_MODE_H6_LOOP_TARGET))
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
        break;

    case STATE_MODE_H3_BALL_MOVE:
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
    current_mode = STATE_MODE_H5_LOOP_CENTER;
    current_page = STATE_PAGE_READY;
    selected_item = 5U;
    target_tenth_cm = 0;
    h6_target_send_pending = 0U;
    balance_roll_zero_deg = 0.0f;
    balance_wx_zero_dps = 0.0f;
    balance_roll_zero_valid = 0U;
    balance_calibration = BALANCE_CAL_IDLE;
    balance_calibration_auto_start = 0U;
    start_time_ms = 0U;
    stopped_elapsed_ms = 0U;
    vision_debug_time_ms = now_ms;
    imu_debug_time_ms = now_ms;
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
    State_SendH6Target();
    State_ProcessBalanceCalibration(now_ms);

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
    /* 陀螺仪解析回传先保留，后续需要时取消注释即可。 */
    // State_SendImuDebug(now_ms);
    State_UpdateDisplay(now_ms);
}
