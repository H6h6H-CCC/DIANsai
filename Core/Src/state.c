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

#define H2_BASE_PWM           300
#define H2_TRACK_KP           8
#define H2_REVERSE_LEFT_PWM   850
#define H2_REVERSE_RIGHT_PWM  1000
#define H2_REVERSE_MS         80
#define H2_MARKER_BLACK_COUNT 4U
#define H2_MARKER_CONFIRM_MS  20
#define H2_MARKER_ARM_MS      12500U
#define H4_BASE_PWM           250
#define H4_TRACK_KP           8
#define H4_RAMP_UP_MS         2000U
#define H4_LIFT_START_PWM     80
#define H4_RAMP_FF_DEG        -0.95f  /* 修改：两轮起步均在1.3~1.5s向正方向超调，加强H4独立起步补偿。 */
#define H4_RUN_FF_DEG           0.40f
#define H4_DOWN_FF_DEG          0.60f
#define H4_SLOWDOWN_COUNT     360000L
#define H4_SLOW_PWM           80
#define H4_ENCODER_STOP_COUNT 450000L
#define H4_COAST_MS           150U
#define H5_BASE_PWM           220
#define H5_TRACK_KP           8
#define H5_RAMP_UP_MS         2000U
#define H5_LIFT_START_PWM     80
#define H5_STOP_RAMP_MS       3000U
#define H5_MARKER_BLACK_COUNT 4U
#define H5_MARKER_ARM_MS      24500U
#define H5_MARKER_CONFIRM_MS  30U
#define H5_RAMP_FF_DEG        -0.80f
#define H5_RUN_BIAS_FF_DEG     0.10f
#define H5_RAMP_FF_IN_MS       300U
#define H5_RAMP_FF_OUT_MS      500U
#define H5_STOP_FF_DEG         0.30f
#define H5_STOP_FF_IN_MS       200U
#define H5_STOP_FF_OUT_MS      600U
#define START_DELAY_MS        500U
#define BALANCE_START_ENCODER_COUNT 10L
#define BALANCE_SERVO_CENTER_US 1750U
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
#define H3_BRAKE_START_CM            4.5f  /* 修改：折返后立即进入低增益减速，降低到达-5cm的速度。 */
#define H3_PLUS_POSITION_KP            0.50f
#define H3_PLUS_POSITION_KI            0.20f
#define H3_PLUS_POSITION_KD            0.60f  /* 修改：降低到达+5cm时的速度与惯性超调。 */
#define H3_MINUS_POSITION_KP           0.50f
#define H3_MINUS_POSITION_KI           0.20f
#define H3_MINUS_POSITION_KD           0.45f
#define H3_BRAKE_POSITION_KP           0.35f  /* 修改：提前减速后略增推进，保证5秒内到达-5cm。 */
#define H3_BRAKE_POSITION_KI           0.05f
#define H3_BRAKE_POSITION_KD           0.65f  /* 修改：进一步减小目标附近速度反向抖动。 */
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
#define H6_BASE_PWM                    220
#define H6_TRACK_KP                      8
#define H6_RAMP_UP_MS                 2000U
#define H6_MINUS9_RAMP_UP_MS          2500U
#define H6_MINUS11_RAMP_UP_MS         2500U
#define H6_STOP_RAMP_MS               3000U
#define H6_MARKER_BLACK_COUNT            4U
#define H6_MARKER_CONFIRM_MS             30U
#define H6_RAMP_FF_DEG                -0.80f
#define H6_RUN_BIAS_FF_DEG             0.10f
#define H6_RAMP_FF_IN_MS               300U
#define H6_RAMP_FF_OUT_MS              500U
#define H6_STOP_FF_DEG                  0.32f  /* 两圈缓停轻微偏负，仅增加0.02度补偿。 */
#define H6_STOP_FF_IN_MS               200U
#define H6_STOP_FF_OUT_MS              600U
#define H6_PLUS3_BASE_TRIM_DEG           0.05f  /* +3cm运行全程保留，拉平后随闭环关闭。 */
#define H6_PLUS3_START_TRIM_DEG          0.105f
#define H6_PLUS3_STOP_TRIM_DEG           0.07f
#define H6_PLUS5_BASE_TRIM_DEG           0.05f  /* +5cm独立参数，初值复制+3cm实测值。 */
#define H6_PLUS5_START_TRIM_DEG          0.105f
#define H6_PLUS5_STOP_TRIM_DEG           0.07f
#define H6_PLUS7_BASE_TRIM_DEG           0.05f  /* +7cm独立参数，初值复制+5cm实测值。 */
#define H6_PLUS7_START_TRIM_DEG          0.08f
#define H6_PLUS7_STOP_TRIM_DEG           0.07f
#define H6_PLUS9_BASE_TRIM_DEG           0.05f  /* +9cm独立参数，初值复制+7cm实测值。 */
#define H6_PLUS9_START_TRIM_DEG          0.08f
#define H6_PLUS9_STOP_TRIM_DEG           0.07f
#define H6_PLUS11_BASE_TRIM_DEG          0.06f  /* +11cm恒速仍轻微偏负，基础补偿增加0.01度。 */
#define H6_PLUS11_START_TRIM_DEG         0.105f  /* 抑制起步后3~4秒的负向惯性过冲。 */
#define H6_PLUS11_STOP_TRIM_DEG          0.07f
#define H6_PLUS11_BRAKE_TRIM_DEG         0.03f  /* 新PID下仅补偿起步后3~5秒的小幅残余过冲。 */
#define H6_PLUS11_BRAKE_START_MS       3000U
#define H6_PLUS11_BRAKE_END_MS         5000U
#define H6_PLUS11_BRAKE_RAMP_MS         500U
#define H6_MINUS1_BASE_TRIM_DEG          0.04f  /* -1cm稳定段偏正，基础补偿减少0.01度。 */
#define H6_MINUS1_START_TRIM_DEG         0.055f  /* 小幅增加起步反向补偿，压低正向超调。 */
#define H6_MINUS1_STOP_TRIM_DEG          0.07f
#define H6_MINUS1_STOP_KICK_ERROR_CM     0.35f
#define H6_MINUS1_STOP_KICK_ANGLE_DEG    0.4f
#define H6_MINUS1_STOP_KICK_MOVE_CM       0.10f
#define H6_MINUS1_STOP_KICK_SPEED_CM_S    0.80f
#define H6_MINUS1_STOP_KICK_MAX_COUNT      2U
#define H6_MINUS3_BASE_TRIM_DEG          0.02f  /* -3cm巡航仍偏正约0.45cm，再独立小降0.01度。 */
#define H6_MINUS3_START_TRIM_DEG         0.080f /* -3cm两圈实测后再小幅增加，继续压低起步峰值。 */
#define H6_MINUS3_STOP_TRIM_DEG          0.07f  /* 恢复上版停车补偿，抑制负向超调。 */
#define H6_MINUS5_BASE_TRIM_DEG          0.02f  /* 三圈巡航平均偏车头0.17cm，再小幅向车尾修正。 */
#define H6_MINUS5_START_TRIM_DEG         0.10f  /* 减弱起步阶段的负向抬升，压低越过目标后的超调。 */
#define H6_MINUS5_STOP_TRIM_DEG          0.00f
#define H6_MINUS5_TRANSITION_TRIM_DEG    0.06f  /* 加强起步后3~5秒的负向过渡补偿。 */
#define H6_MINUS5_TRANSITION_START_MS  3000U
#define H6_MINUS5_TRANSITION_END_MS    5000U
#define H6_MINUS5_TRANSITION_RAMP_MS    500U
#define H6_MINUS7_BASE_TRIM_DEG         -0.195f  /* 两圈巡航仍平均偏正约0.47cm，再向车头补偿0.01度。 */
#define H6_MINUS7_START_TRIM_DEG         -0.8f  /* 两圈峰值均在起步后1.55秒，继续压低起步超调。 */
#define H6_MINUS7_START_FF_OUT_START_MS  1200U  /* -7cm独立折中渐出，平衡正负双向超调。 */
#define H6_MINUS7_START_FF_OUT_END_MS    1700U
#define H6_MINUS7_STOP_TRIM_DEG          -0.005f
#define H6_MINUS7_PRE_BEND_TRIM_DEG      -0.17f /* 6s后至第一弯前段的专用补偿。 */
#define H6_MINUS7_PRE_BEND_START_MS      6000U
#define H6_MINUS7_PRE_BEND_RAMP_MS        500U
#define H6_MINUS7_BEND_LATE_DELAY_MS     4000U  /* 第一弯后段开始减弱，抑制出口回摆。 */
#define H6_MINUS7_EXIT_TRIM_DEG           -0.05f
#define H6_MINUS7_EXIT_RAMP_MS             500U
#define H6_MINUS7_BETWEEN_DELAY_MS        1000U /* 第一弯退出后等待1s再加强弯间补偿。 */
#define H6_MINUS7_BETWEEN_TRIM_DEG        -0.25f
#define H6_MINUS7_BETWEEN_RAMP_MS          500U
#define H6_MINUS7_SECOND_EXIT_RAMP_MS      500U
#define H6_MINUS9_HOLD_ENTER_ERROR_CM       0.30f
#define H6_MINUS9_HOLD_ENTER_SPEED_CM_S     0.80f
#define H6_MINUS9_HOLD_EXIT_ERROR_CM        0.80f
#define H6_MINUS9_HOLD_WAKE_ERROR_CM        0.50f
#define H6_MINUS9_HOLD_WAKE_SPEED_CM_S      1.20f
#define H6_MINUS9_HOLD_ENTER_CONFIRM_MS      300U
#define H6_MINUS9_HOLD_WAKE_CONFIRM_MS       100U
#define H6_MINUS9_HOLD_ARM_MS               5000U
#define H6_MINUS9_START_TRIM_DEG              -0.70f
#define H6_MINUS9_START_FULL_ERROR_CM           0.50f
#define H6_MINUS9_START_RELEASE_SPEED_CM_S      0.20f
#define H6_MINUS9_START_FF_OUT_START_MS       1800U
#define H6_MINUS9_START_FF_OUT_END_MS         2500U
#define H6_MINUS9_HOLD_TRIM_DEG               0.03f
#define H6_MINUS9_START_KP                     0.45f
#define H6_MINUS9_START_KI                     0.06f
#define H6_MINUS9_START_KD                     0.30f
#define H6_MINUS9_RECOVERY_ENTER_CM            -0.30f
#define H6_MINUS9_RECOVERY_ENTER_SPEED_CM_S     0.50f
#define H6_MINUS9_RECOVERY_KP                    0.25f
#define H6_MINUS9_RECOVERY_KI                    0.00f
#define H6_MINUS9_RECOVERY_KD                    0.20f
#define H6_MINUS9_RECOVERY_CONFIRM_MS             300U
#define H6_MINUS11_HOLD_ENTER_ERROR_CM       0.30f
#define H6_MINUS11_HOLD_ENTER_SPEED_CM_S     0.80f
#define H6_MINUS11_HOLD_EXIT_ERROR_CM        0.80f
#define H6_MINUS11_HOLD_WAKE_ERROR_CM        0.50f
#define H6_MINUS11_HOLD_WAKE_SPEED_CM_S      1.20f
#define H6_MINUS11_HOLD_ENTER_CONFIRM_MS      300U
#define H6_MINUS11_HOLD_WAKE_CONFIRM_MS       100U
#define H6_MINUS11_HOLD_ARM_MS               5000U
#define H6_MINUS11_START_TRIM_DEG             -0.70f
#define H6_MINUS11_START_FULL_ERROR_CM          0.50f
#define H6_MINUS11_START_RELEASE_SPEED_CM_S     0.20f
#define H6_MINUS11_START_FF_OUT_START_MS      1800U
#define H6_MINUS11_START_FF_OUT_END_MS        2500U
#define H6_MINUS11_HOLD_TRIM_DEG               0.03f
#define H6_MINUS11_START_KP                     0.45f
#define H6_MINUS11_START_KI                     0.06f
#define H6_MINUS11_START_KD                     0.30f
#define H6_MINUS11_RECOVERY_ENTER_CM            0.80f
#define H6_MINUS11_RECOVERY_ENTER_SPEED_CM_S    1.00f
#define H6_MINUS11_RECOVERY_KP                   0.20f
#define H6_MINUS11_RECOVERY_KI                   0.00f
#define H6_MINUS11_RECOVERY_KD                   0.15f
#define H6_MINUS11_RECOVERY_CONFIRM_MS            300U
#define H6_BEND_ARM_MS                    6000U  /* 过滤起步阶段的右侧黑线毛刺。 */
#define H6_BEND_EXIT_CONFIRM_MS            300U
#define H6_SECOND_BEND_MIN_GAP_MS        1000U
#define H6_FLAT_TOLERANCE_CM              0.6f  /* H6结束仍留出题目+/-1cm要求的余量。 */
#define H6_FLAT_MAX_SPEED_CM_S            1.2f
#define H6_FLAT_STABLE_RANGE_CM           0.8f
#define H6_END_TARGET_TENTH_CM          120
#define H6_MARKER_ARM_MS              24500U
#define H6_PID_NODE_COUNT             23U
#define H6_TRIM_NODE_COUNT            23U
#define H6_KICK_ERROR_CM               1.0f
#define H6_KICK_MAX_SPEED_CM_S          0.2f
#define H6_KICK_WAIT_MS                300U
#define H6_KICK_MIN_MS                 150U
#define H6_KICK_RAMP_IN_MS            1000U
#define H6_KICK_MAX_MS                1300U
#define H6_KICK_RAMP_OUT_MS            150U
#define H6_KICK_RETRY_MS              1200U
#define H6_KICK_MOVE_CM                  0.15f
#define H6_KICK_START_SPEED_CM_S         0.5f
#define H6_KICK_ANGLE_DEG                2.0f
#define H6_KICK_MAX_COUNT                3U
#define H6_NEAR_TARGET_TENTH_CM          30
#define H6_NEAR_KICK_MOVE_CM              0.05f
#define H6_NEAR_KICK_START_SPEED_CM_S     0.2f
#define H6_NEAR_KICK_ANGLE_DEG             0.8f
#define H6_NEAR_KICK_MAX_COUNT             0U

/* H7独立参数：0cm起步，两个弯道分别平滑切换到+5cm和-5cm。 */
#define H7_BASE_PWM                       220
#define H7_TRACK_KP                         8
#define H7_RAMP_UP_MS                    2000U
#define H7_STOP_RAMP_MS                  3000U
#define H7_MARKER_BLACK_COUNT               4U
#define H7_MARKER_CONFIRM_MS               30U
#define H7_MARKER_ARM_MS                 24500U
#define H7_BEND_ARM_MS                    6000U
#define H7_BEND_EXIT_CONFIRM_MS            300U
#define H7_SECOND_BEND_MIN_GAP_MS         1000U
#define H7_TARGET_TRANSITION_MS           3000U  /* 修改：按约4s整段弯道尺度过渡，出弯前留约1s稳定。 */
#define H7_CENTER_TARGET_CM                0.0f
#define H7_PLUS_TARGET_TENTH_CM             50
#define H7_MINUS_TARGET_TENTH_CM           -50
#define H7_RAMP_FF_DEG                    -0.80f
#define H7_RUN_FF_DEG                      0.10f
#define H7_RAMP_FF_IN_MS                    300U
#define H7_RAMP_FF_OUT_MS                   500U
#define H7_STOP_FF_DEG                     0.32f
#define H7_STOP_FF_IN_MS                    200U
#define H7_STOP_FF_OUT_MS                   600U
#define H7_CENTER_TRIM_DEG                 0.00f
#define H7_PLUS5_TRIM_DEG                  0.05f
#define H7_MINUS5_TRIM_DEG                 0.02f
#define H7_CENTER_POSITION_KP              0.60f
#define H7_CENTER_POSITION_KI              0.30f
#define H7_CENTER_POSITION_KD              0.70f
#define H7_PLUS5_POSITION_KP               0.50f
#define H7_PLUS5_POSITION_KI               0.20f
#define H7_PLUS5_POSITION_KD               0.45f
#define H7_MINUS5_POSITION_KP              0.45f
#define H7_MINUS5_POSITION_KI              0.06f
#define H7_MINUS5_POSITION_KD              0.30f
#define H7_ANGLE_KP                      100.0f
#define H7_ANGLE_KI                      100.0f
#define H7_ANGLE_KD                        0.5f
#define H7_FLAT_TOLERANCE_CM               0.6f
#define H7_FLAT_MAX_SPEED_CM_S             1.2f
#define H7_FLAT_STABLE_RANGE_CM            0.8f

typedef enum
{
    STATE_MODE_NONE = 0,
    STATE_MODE_H2_CAR_LOOP = 2,
    STATE_MODE_H3_BALL_MOVE = 3,
    STATE_MODE_H4_AB_BALANCE = 4,
    STATE_MODE_H5_LOOP_CENTER = 5,
    STATE_MODE_H6_LOOP_TARGET = 6,
    STATE_MODE_H7_BEND_TARGET = 7
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

typedef struct
{
    int16_t target_tenth_cm;
    float kp;
    float ki;
    float kd;
} H6PositionPidNode_t;

typedef struct
{
    int16_t target_tenth_cm;
    float base_trim_deg;
    float start_trim_deg;
    float stop_trim_deg;
} H6TrimNode_t;

typedef struct
{
    int16_t target_tenth_cm;
    uint32_t ramp_up_ms;
    float hold_enter_error_cm;
    float hold_enter_speed_cm_s;
    float hold_exit_error_cm;
    float hold_wake_error_cm;
    float hold_wake_speed_cm_s;
    uint32_t hold_enter_confirm_ms;
    uint32_t hold_wake_confirm_ms;
    uint32_t hold_arm_ms;
    float start_trim_deg;
    float start_full_error_cm;
    float start_release_speed_cm_s;
    uint32_t start_ff_out_start_ms;
    uint32_t start_ff_out_end_ms;
    float hold_trim_deg;
    float start_kp;
    float start_ki;
    float start_kd;
    float recovery_enter_cm;
    float recovery_enter_speed_cm_s;
    float recovery_kp;
    float recovery_ki;
    float recovery_kd;
    uint32_t recovery_confirm_ms;
} H6MinusEndConfig_t;

typedef struct
{
    uint8_t horizontal_hold;
    uint8_t recovery_pid_active;
    uint8_t start_ff_done;
    uint8_t soft_recovery_active;
    uint32_t recovery_candidate_ms;
    uint8_t hold_entered_once;
    uint32_t hold_candidate_ms;
    uint32_t wake_candidate_ms;
} H6MinusEndRuntime_t;

static StateMode_t current_mode;
static StatePage_t current_page;
static uint8_t selected_item;
static int16_t target_tenth_cm;
static uint8_t vision_origin_send_pending;
static uint8_t h6_kick_state;
static uint32_t h6_still_start_ms;
static uint32_t h6_kick_start_ms;
static float h6_kick_angle_deg;
static float h6_kick_applied_deg;
static float h6_kick_start_position_cm;
static uint8_t h6_kick_count;
static int16_t h6_drive_pwm;
static int16_t h6_stop_start_pwm;
static const char *h6_drive_phase;
static uint8_t h6_marker_detected;
static uint32_t h6_marker_candidate_ms;
static uint8_t h6_stop_locked;
static uint8_t h6_black_count;
static uint8_t h6_gray;
static const char *h6_bend_state;
static uint8_t h6_bend_active;
static uint8_t h6_bend_count;
static uint32_t h6_bend_last_seen_ms;
static uint32_t h6_first_bend_enter_ms;
static uint32_t h6_first_bend_exit_ms;
static uint32_t h6_second_bend_enter_ms;
static uint32_t h6_stop_start_ms;
static uint32_t h6_stable_start_ms;
static float h6_stable_min_cm;
static float h6_stable_max_cm;
static uint8_t h6_flat_done;
static uint8_t h6_balance_started;
static H6MinusEndRuntime_t h6_minus9_runtime;
static H6MinusEndRuntime_t h6_minus11_runtime;
static const H6MinusEndConfig_t h6_minus9_config =
{
    -90, H6_MINUS9_RAMP_UP_MS,
    H6_MINUS9_HOLD_ENTER_ERROR_CM, H6_MINUS9_HOLD_ENTER_SPEED_CM_S,
    H6_MINUS9_HOLD_EXIT_ERROR_CM, H6_MINUS9_HOLD_WAKE_ERROR_CM,
    H6_MINUS9_HOLD_WAKE_SPEED_CM_S, H6_MINUS9_HOLD_ENTER_CONFIRM_MS,
    H6_MINUS9_HOLD_WAKE_CONFIRM_MS, H6_MINUS9_HOLD_ARM_MS,
    H6_MINUS9_START_TRIM_DEG, H6_MINUS9_START_FULL_ERROR_CM,
    H6_MINUS9_START_RELEASE_SPEED_CM_S,
    H6_MINUS9_START_FF_OUT_START_MS, H6_MINUS9_START_FF_OUT_END_MS,
    H6_MINUS9_HOLD_TRIM_DEG,
    H6_MINUS9_START_KP, H6_MINUS9_START_KI, H6_MINUS9_START_KD,
    H6_MINUS9_RECOVERY_ENTER_CM, H6_MINUS9_RECOVERY_ENTER_SPEED_CM_S,
    H6_MINUS9_RECOVERY_KP, H6_MINUS9_RECOVERY_KI, H6_MINUS9_RECOVERY_KD,
    H6_MINUS9_RECOVERY_CONFIRM_MS
};
static const H6MinusEndConfig_t h6_minus11_config =
{
    -110, H6_MINUS11_RAMP_UP_MS,
    H6_MINUS11_HOLD_ENTER_ERROR_CM, H6_MINUS11_HOLD_ENTER_SPEED_CM_S,
    H6_MINUS11_HOLD_EXIT_ERROR_CM, H6_MINUS11_HOLD_WAKE_ERROR_CM,
    H6_MINUS11_HOLD_WAKE_SPEED_CM_S, H6_MINUS11_HOLD_ENTER_CONFIRM_MS,
    H6_MINUS11_HOLD_WAKE_CONFIRM_MS, H6_MINUS11_HOLD_ARM_MS,
    H6_MINUS11_START_TRIM_DEG, H6_MINUS11_START_FULL_ERROR_CM,
    H6_MINUS11_START_RELEASE_SPEED_CM_S,
    H6_MINUS11_START_FF_OUT_START_MS, H6_MINUS11_START_FF_OUT_END_MS,
    H6_MINUS11_HOLD_TRIM_DEG,
    H6_MINUS11_START_KP, H6_MINUS11_START_KI, H6_MINUS11_START_KD,
    H6_MINUS11_RECOVERY_ENTER_CM, H6_MINUS11_RECOVERY_ENTER_SPEED_CM_S,
    H6_MINUS11_RECOVERY_KP, H6_MINUS11_RECOVERY_KI, H6_MINUS11_RECOVERY_KD,
    H6_MINUS11_RECOVERY_CONFIRM_MS
};
static uint32_t h6_balance_start_ms;
static int32_t h6_start_encoder3;
static int32_t h6_start_encoder4;
static H6PositionPidNode_t h6_position_pid_nodes[H6_PID_NODE_COUNT] =
{
    /* 修改：偶数厘米初值由相邻实测点平均后独立保存，后续可单点调参。 */
    {-110, 0.3000f, 0.0000f, 0.1800f},
    {-100, 0.4500f, 0.0500f, 0.1900f},
    { -90, 0.6000f, 0.1000f, 0.2000f},
    { -80, 0.5250f, 0.0800f, 0.2500f},
    { -70, 0.4500f, 0.0600f, 0.3000f},
    { -60, 0.4500f, 0.0600f, 0.3000f},
    { -50, 0.4500f, 0.0600f, 0.3000f},
    { -40, 0.5250f, 0.1800f, 0.5000f},
    { -30, 0.6000f, 0.3000f, 0.7000f},
    { -20, 0.5500f, 0.2250f, 0.6500f},
    { -10, 0.5000f, 0.1500f, 0.6000f},
    {   0, 0.5000f, 0.1500f, 0.4250f},
    {  10, 0.5000f, 0.1500f, 0.2500f},
    {  20, 0.5000f, 0.1500f, 0.3500f},
    {  30, 0.5000f, 0.1500f, 0.4500f},
    {  40, 0.5000f, 0.1750f, 0.4500f},
    {  50, 0.5000f, 0.2000f, 0.4500f},
    {  60, 0.5000f, 0.2000f, 0.4500f},
    {  70, 0.5000f, 0.2000f, 0.4500f},
    {  80, 0.5000f, 0.2000f, 0.4500f},
    {  90, 0.5000f, 0.2000f, 0.4500f},
    { 100, 0.5500f, 0.2000f, 0.6750f},
    { 110, 0.6000f, 0.2000f, 0.9000f}
};

static const H6TrimNode_t h6_trim_nodes[H6_TRIM_NODE_COUNT] =
{
    /* 修改：基础、起步、停车补偿均为独立整数厘米节点。 */
    {-110,  0.0000f, H6_MINUS11_START_TRIM_DEG, 0.0000f},
    {-100,  0.0000f, -0.7000f,  0.0000f},
    { -90,  0.0000f, H6_MINUS9_START_TRIM_DEG,  0.0000f},
    { -80, -0.0925f, -0.6750f, -0.0025f},
    { -70, H6_MINUS7_BASE_TRIM_DEG,
           H6_MINUS7_START_TRIM_DEG, H6_MINUS7_STOP_TRIM_DEG},
    { -60, -0.0825f, -0.2750f, -0.0025f},
    { -50, H6_MINUS5_BASE_TRIM_DEG,
           H6_MINUS5_START_TRIM_DEG, H6_MINUS5_STOP_TRIM_DEG},
    { -40,  0.0200f,  0.0900f,  0.0350f},
    { -30, H6_MINUS3_BASE_TRIM_DEG,
           H6_MINUS3_START_TRIM_DEG, H6_MINUS3_STOP_TRIM_DEG},
    { -20,  0.0300f,  0.0675f,  0.0700f},
    { -10, H6_MINUS1_BASE_TRIM_DEG,
           H6_MINUS1_START_TRIM_DEG, H6_MINUS1_STOP_TRIM_DEG},
    {   0,  0.0200f,  0.0275f,  0.0350f},
    {  10,  0.0000f,  0.0000f,  0.0000f},
    {  20,  0.0250f,  0.0525f,  0.0350f},
    {  30, H6_PLUS3_BASE_TRIM_DEG,
           H6_PLUS3_START_TRIM_DEG, H6_PLUS3_STOP_TRIM_DEG},
    {  40,  0.0500f,  0.1050f,  0.0700f},
    {  50, H6_PLUS5_BASE_TRIM_DEG,
           H6_PLUS5_START_TRIM_DEG, H6_PLUS5_STOP_TRIM_DEG},
    {  60,  0.0500f,  0.0925f,  0.0700f},
    {  70, H6_PLUS7_BASE_TRIM_DEG,
           H6_PLUS7_START_TRIM_DEG, H6_PLUS7_STOP_TRIM_DEG},
    {  80,  0.0500f,  0.0800f,  0.0700f},
    {  90, H6_PLUS9_BASE_TRIM_DEG,
           H6_PLUS9_START_TRIM_DEG, H6_PLUS9_STOP_TRIM_DEG},
    { 100,  0.0550f,  0.0925f,  0.0700f},
    { 110, H6_PLUS11_BASE_TRIM_DEG,
           H6_PLUS11_START_TRIM_DEG, H6_PLUS11_STOP_TRIM_DEG}
};
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
/* 修改：H3正向、反向和反向末段制动分别使用独立RAM参数。 */
static float h3_plus_kp = H3_PLUS_POSITION_KP;
static float h3_plus_ki = H3_PLUS_POSITION_KI;
static float h3_plus_kd = H3_PLUS_POSITION_KD;
static float h3_minus_kp = H3_MINUS_POSITION_KP;
static float h3_minus_ki = H3_MINUS_POSITION_KI;
static float h3_minus_kd = H3_MINUS_POSITION_KD;
static float h3_brake_kp = H3_BRAKE_POSITION_KP;
static float h3_brake_ki = H3_BRAKE_POSITION_KI;
static float h3_brake_kd = H3_BRAKE_POSITION_KD;
static uint32_t h3_stable_start_ms;
static float h3_turn_position_cm;
static float h3_stable_min_cm;
static float h3_stable_max_cm;

static uint8_t h2_marker_active;
static uint32_t h2_marker_candidate_ms;
static uint8_t h2_reverse_active;
static uint8_t h2_stop_locked;
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
static uint32_t h5_marker_candidate_ms;
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

static int16_t h7_drive_pwm;
static int16_t h7_stop_start_pwm;
static const char *h7_drive_phase;
static uint8_t h7_marker_detected;
static uint32_t h7_marker_candidate_ms;
static uint8_t h7_stop_locked;
static uint8_t h7_black_count;
static uint8_t h7_gray;
static const char *h7_bend_state;
static uint8_t h7_bend_active;
static uint8_t h7_bend_count;
static uint32_t h7_bend_last_seen_ms;
static uint32_t h7_first_bend_exit_ms;
static uint32_t h7_stop_start_ms;
static uint8_t h7_target_phase;
static uint32_t h7_transition_start_ms;
static float h7_transition_from_cm;
static float h7_transition_to_cm;
static float h7_active_target_cm;
static uint8_t h7_balance_started;
static uint32_t h7_balance_start_ms;
static int32_t h7_start_encoder3;
static int32_t h7_start_encoder4;
static uint32_t h7_stable_start_ms;
static float h7_stable_min_cm;
static float h7_stable_max_cm;
static uint8_t h7_flat_done;

static uint8_t display_dirty;
static StatePage_t last_display_page;
static uint32_t last_display_half_second;
static char display_cache[4][16];

static uint8_t balance_debug_buffer[320];
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
extern volatile uint32_t g_debug_rx_restart_fail_count;
extern volatile uint32_t g_debug_uart_error_count;
extern volatile uint32_t g_debug_uart_last_error;

static void State_Start(uint32_t now_ms);
static void State_RunBallControl(uint32_t now_ms);
static void State_H3ApplyPositionPid(void);
static void State_H6UpdateKick(uint32_t now_ms);
static void State_H6UpdateHorizontalHold(uint32_t now_ms);
static const H6MinusEndConfig_t *State_H6GetMinusEndConfig(void);
static H6MinusEndRuntime_t *State_H6GetMinusEndRuntime(void);
static int16_t State_CorrectVisionTarget(int16_t target_tenth_cm);
static void State_ProcessDebugCommand(void);
static void State_SendDebugReply(void);

static int16_t State_CmToTenth(float target_cm)
{
    float scaled = target_cm * 10.0f;
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

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

    debug_rx_event_count++;
    debug_rx_byte_count += length;

    for (i = 0U; i < length; i++)
    {
        char value = (char)data[i];

        if ((value == '\r') || (value == '\n'))
        {
            /* 只按行结束符提交，DMA 空闲事件可能把一条命令拆成多段。 */
            if ((debug_rx_build_length != 0U) &&
                (debug_command_ready == 0U))
            {
                memcpy(debug_command, debug_rx_build, debug_rx_build_length);
                debug_command[debug_rx_build_length] = '\0';
                debug_command_ready = 1U;
                debug_rx_build_length = 0U;
            }
            continue;
        }

        if ((debug_command_ready == 0U) &&
            (value >= 32) && (value <= 126) &&
            (debug_rx_build_length < (DEBUG_COMMAND_SIZE - 1U)))
        {
            debug_rx_build[debug_rx_build_length++] = value;
        }
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
    case STATE_MODE_H7_BEND_TARGET: return "H7 BEND TARGET";
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
    if (first_item > 5U) first_item = 5U;

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
    else if (current_mode == STATE_MODE_H7_BEND_TARGET)
    {
        State_ShowLine(2U, "0>+5>-5CM");
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
         (current_mode == STATE_MODE_H5_LOOP_CENTER) ||
         (current_mode == STATE_MODE_H7_BEND_TARGET)) &&
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
    h2_marker_active = 0U;
    h2_marker_candidate_ms = 0U;
    h2_reverse_active = 0U;
    h2_stop_locked = 0U;
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

    if (((uint32_t)(now_ms - start_time_ms) >= H2_MARKER_ARM_MS) &&
        (black_count >= H2_MARKER_BLACK_COUNT) &&
        (h2_marker_active == 0U))
    {
        if (h2_marker_candidate_ms == 0U)
        {
            h2_marker_candidate_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - h2_marker_candidate_ms) >=
                 H2_MARKER_CONFIRM_MS)
        {
            /* 修改：12.5秒后首次连续4黑即为终点，反向制动后锁死。 */
            h2_marker_active = 1U;
            h2_marker_candidate_ms = 0U;
            h2_reverse_active = 1U;
            h2_reverse_start_ms = now_ms;
            Moter_A(H2_REVERSE_RIGHT_PWM);
            Moter_B(H2_REVERSE_LEFT_PWM);
            return 0U;
        }
    }
    else if (((uint32_t)(now_ms - start_time_ms) < H2_MARKER_ARM_MS) ||
             (black_count < H2_MARKER_BLACK_COUNT))
    {
        h2_marker_candidate_ms = 0U;
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

        if ((elapsed_ms >= H5_MARKER_ARM_MS) &&
            (h5_black_count >= H5_MARKER_BLACK_COUNT))
        {
            if (h5_marker_candidate_ms == 0U)
            {
                h5_marker_candidate_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - h5_marker_candidate_ms) >=
                     H5_MARKER_CONFIRM_MS)
            {
                /* 连续4黑才完成，过滤弯道瞬时扫过多路黑线。 */
                h5_marker_detected = 1U;
                h5_marker_candidate_ms = 0U;
                h5_stop_start_ms = now_ms;
                h5_stop_start_pwm = h5_drive_pwm;
                h5_drive_phase = "STOP";
                first_marker = 1U;
            }
        }
        else
        {
            h5_marker_candidate_ms = 0U;
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

static uint8_t State_H6Run(uint32_t now_ms)
{
    const H6MinusEndConfig_t *minus_end_config;
    uint32_t elapsed_ms;
    uint32_t ramp_up_ms;
    int16_t turn;
    uint8_t first_marker = 0U;

    h6_gray = Gray_Read();
    h6_black_count = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((h6_gray & (uint8_t)(1U << i)) == 0U)
        {
            h6_black_count++;
        }
    }

    if (((uint32_t)(now_ms - start_time_ms) >= H6_BEND_ARM_MS) &&
        ((h6_gray & 0x20U) == 0U))
    {
        h6_bend_state = ((h6_gray & 0x40U) == 0U) ? "R2" : "R1";
        h6_bend_last_seen_ms = now_ms;
        if ((h6_bend_active == 0U) &&
            ((h6_bend_count == 0U) ||
             ((uint32_t)(now_ms - h6_first_bend_exit_ms) >=
              H6_SECOND_BEND_MIN_GAP_MS)))
        {
            h6_bend_active = 1U;
            h6_bend_count++;
            if (h6_bend_count == 1U)
            {
                h6_first_bend_enter_ms = now_ms;
            }
            else if (h6_bend_count == 2U)
            {
                h6_second_bend_enter_ms = now_ms;
            }
        }
    }
    else
    {
        h6_bend_state = "NONE";
        if ((h6_bend_active != 0U) &&
            ((uint32_t)(now_ms - h6_bend_last_seen_ms) >=
             H6_BEND_EXIT_CONFIRM_MS))
        {
            h6_bend_active = 0U;
            if (h6_bend_count == 1U)
            {
                h6_first_bend_exit_ms = now_ms;
            }
        }
    }

    if (h6_stop_locked != 0U)
    {
        State_H2Brake();
        return 0U;
    }

    if (h6_marker_detected == 0U)
    {
        elapsed_ms = (uint32_t)(now_ms - start_time_ms);
        /* 修改：-9cm独立延长缓发车，降低2秒附近的车尾惯性峰值。 */
        minus_end_config = State_H6GetMinusEndConfig();
        ramp_up_ms = (minus_end_config != 0) ?
                     minus_end_config->ramp_up_ms : H6_RAMP_UP_MS;
        if (elapsed_ms < ramp_up_ms)
        {
            h6_drive_phase = "UP";
            h6_drive_pwm = (int16_t)(((uint32_t)H6_BASE_PWM * elapsed_ms) /
                                     ramp_up_ms);
        }
        else
        {
            h6_drive_phase = "RUN";
            h6_drive_pwm = H6_BASE_PWM;
        }

        if (((uint32_t)(now_ms - start_time_ms) >= H6_MARKER_ARM_MS) &&
            (h6_black_count >= H6_MARKER_BLACK_COUNT))
        {
            if (h6_marker_candidate_ms == 0U)
            {
                h6_marker_candidate_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - h6_marker_candidate_ms) >=
                     H6_MARKER_CONFIRM_MS)
            {
                /* 终点需连续4黑，过滤弯道横切黑线产生的瞬时毛刺。 */
                h6_marker_detected = 1U;
                h6_marker_candidate_ms = 0U;
                h6_stop_start_ms = now_ms;
                h6_stop_start_pwm = h6_drive_pwm;
                h6_drive_phase = "STOP";
                first_marker = 1U;
            }
        }
        else
        {
            h6_marker_candidate_ms = 0U;
        }
    }
    else
    {
        elapsed_ms = (uint32_t)(now_ms - h6_stop_start_ms);
        if (elapsed_ms >= H6_STOP_RAMP_MS)
        {
            h6_drive_pwm = 0;
            h6_drive_phase = "DONE";
            h6_stop_locked = 1U;
            /* 保留调用方设置的基础补偿，直到判稳后拉平。 */
            State_H2Brake();
            return 0U;
        }

        h6_drive_phase = "STOP";
        h6_drive_pwm = (int16_t)(((uint32_t)h6_stop_start_pwm *
                                  (H6_STOP_RAMP_MS - elapsed_ms)) /
                                 H6_STOP_RAMP_MS);
    }

    turn = (int16_t)(Gray_GetError() * H6_TRACK_KP);
    if (turn > h6_drive_pwm) turn = h6_drive_pwm;
    if (turn < -h6_drive_pwm) turn = -h6_drive_pwm;
    Moter_A(-h6_drive_pwm - turn);
    Moter_B(-h6_drive_pwm + turn);
    return first_marker;
}

static void State_H7SetPositionPid(uint8_t target_phase)
{
    if (target_phase <= 1U)
    {
        BallControl_SetPositionPid(H7_CENTER_POSITION_KP,
                                   H7_CENTER_POSITION_KI,
                                   H7_CENTER_POSITION_KD);
    }
    else if (target_phase <= 3U)
    {
        BallControl_SetPositionPid(H7_PLUS5_POSITION_KP,
                                   H7_PLUS5_POSITION_KI,
                                   H7_PLUS5_POSITION_KD);
    }
    else
    {
        BallControl_SetPositionPid(H7_MINUS5_POSITION_KP,
                                   H7_MINUS5_POSITION_KI,
                                   H7_MINUS5_POSITION_KD);
    }
}

static void State_H7ResetRun(void)
{
    h7_drive_pwm = 0;
    h7_stop_start_pwm = 0;
    h7_drive_phase = "UP";
    h7_marker_detected = 0U;
    h7_marker_candidate_ms = 0U;
    h7_stop_locked = 0U;
    h7_black_count = 0U;
    h7_gray = Gray_Read();
    h7_bend_state = "NONE";
    h7_bend_active = 0U;
    h7_bend_count = 0U;
    h7_bend_last_seen_ms = 0U;
    h7_first_bend_exit_ms = 0U;
    h7_stop_start_ms = 0U;
    h7_target_phase = 1U;
    h7_transition_start_ms = 0U;
    h7_transition_from_cm = H7_CENTER_TARGET_CM;
    h7_transition_to_cm = H7_CENTER_TARGET_CM;
    h7_active_target_cm = H7_CENTER_TARGET_CM;
    h7_balance_started = 0U;
    h7_balance_start_ms = 0U;
    h7_start_encoder3 = Encoder3_GetTotal();
    h7_start_encoder4 = Encoder4_GetTotal();
    h7_stable_start_ms = 0U;
    h7_stable_min_cm = 0.0f;
    h7_stable_max_cm = 0.0f;
    h7_flat_done = 0U;
}

static void State_H7StartTransition(uint32_t now_ms, int16_t target_tenth_cm)
{
    h7_transition_start_ms = now_ms;
    h7_transition_from_cm = h7_active_target_cm;
    h7_transition_to_cm = (float)State_CorrectVisionTarget(target_tenth_cm) / 10.0f;
    h7_target_phase = (target_tenth_cm > 0) ? 2U : 4U;
    State_H7SetPositionPid(h7_target_phase);
}

static void State_H7UpdateTarget(uint32_t now_ms)
{
    uint32_t transition_elapsed_ms;

    if ((h7_target_phase != 2U) && (h7_target_phase != 4U))
    {
        return;
    }

    transition_elapsed_ms = (uint32_t)(now_ms - h7_transition_start_ms);
    if (transition_elapsed_ms >= H7_TARGET_TRANSITION_MS)
    {
        h7_active_target_cm = h7_transition_to_cm;
        h7_target_phase = (h7_target_phase == 2U) ? 3U : 5U;
    }
    else
    {
        h7_active_target_cm = h7_transition_from_cm +
            (h7_transition_to_cm - h7_transition_from_cm) *
            (float)transition_elapsed_ms / (float)H7_TARGET_TRANSITION_MS;
    }
    BallControl_UpdateTargetPosition(h7_active_target_cm);
}

static uint8_t State_H7Run(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    int16_t turn;
    uint8_t first_marker = 0U;

    h7_gray = Gray_Read();
    h7_black_count = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        if ((h7_gray & (uint8_t)(1U << i)) == 0U)
        {
            h7_black_count++;
        }
    }

    if (((uint32_t)(now_ms - start_time_ms) >= H7_BEND_ARM_MS) &&
        ((h7_gray & 0x20U) == 0U))
    {
        h7_bend_state = ((h7_gray & 0x40U) == 0U) ? "R2" : "R1";
        h7_bend_last_seen_ms = now_ms;
        if ((h7_bend_active == 0U) && (h7_bend_count < 2U) &&
            ((h7_bend_count == 0U) ||
             ((uint32_t)(now_ms - h7_first_bend_exit_ms) >=
              H7_SECOND_BEND_MIN_GAP_MS)))
        {
            h7_bend_active = 1U;
            h7_bend_count++;
            if (h7_bend_count == 1U)
            {
                State_H7StartTransition(now_ms, H7_PLUS_TARGET_TENTH_CM);
            }
            else if (h7_bend_count == 2U)
            {
                State_H7StartTransition(now_ms, H7_MINUS_TARGET_TENTH_CM);
            }
        }
    }
    else
    {
        h7_bend_state = "NONE";
        if ((h7_bend_active != 0U) &&
            ((uint32_t)(now_ms - h7_bend_last_seen_ms) >=
             H7_BEND_EXIT_CONFIRM_MS))
        {
            h7_bend_active = 0U;
            if (h7_bend_count == 1U)
            {
                h7_first_bend_exit_ms = now_ms;
            }
        }
    }

    State_H7UpdateTarget(now_ms);
    if (h7_stop_locked != 0U)
    {
        State_H2Brake();
        return 0U;
    }

    if (h7_marker_detected == 0U)
    {
        elapsed_ms = (uint32_t)(now_ms - start_time_ms);
        if (elapsed_ms < H7_RAMP_UP_MS)
        {
            h7_drive_phase = "UP";
            h7_drive_pwm = (int16_t)(((uint32_t)H7_BASE_PWM * elapsed_ms) /
                                     H7_RAMP_UP_MS);
        }
        else
        {
            h7_drive_phase = "RUN";
            h7_drive_pwm = H7_BASE_PWM;
        }

        /* 已识别第二弯且进入终点时间窗后才允许4黑，终点全黑本身仍会呈现弯道特征。 */
        if ((h7_bend_count >= 2U) &&
            (elapsed_ms >= H7_MARKER_ARM_MS) &&
            (h7_black_count >= H7_MARKER_BLACK_COUNT))
        {
            if (h7_marker_candidate_ms == 0U)
            {
                h7_marker_candidate_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - h7_marker_candidate_ms) >=
                     H7_MARKER_CONFIRM_MS)
            {
                h7_marker_detected = 1U;
                h7_marker_candidate_ms = 0U;
                h7_stop_start_ms = now_ms;
                h7_stop_start_pwm = h7_drive_pwm;
                h7_drive_phase = "STOP";
                first_marker = 1U;
            }
        }
        else
        {
            h7_marker_candidate_ms = 0U;
        }
    }
    else
    {
        elapsed_ms = (uint32_t)(now_ms - h7_stop_start_ms);
        if (elapsed_ms >= H7_STOP_RAMP_MS)
        {
            h7_drive_pwm = 0;
            h7_drive_phase = "DONE";
            h7_stop_locked = 1U;
            State_H2Brake();
            return 0U;
        }
        h7_drive_phase = "STOP";
        h7_drive_pwm = (int16_t)(((uint32_t)h7_stop_start_pwm *
                                  (H7_STOP_RAMP_MS - elapsed_ms)) /
                                 H7_STOP_RAMP_MS);
    }

    turn = (int16_t)(Gray_GetError() * H7_TRACK_KP);
    if (turn > h7_drive_pwm) turn = h7_drive_pwm;
    if (turn < -h7_drive_pwm) turn = -h7_drive_pwm;
    Moter_A(-h7_drive_pwm - turn);
    Moter_B(-h7_drive_pwm + turn);
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
        /* 停车且小球稳定后关闭闭环，舵机自动回到1750us水平位。 */
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

static void State_H6FinishBalance(uint32_t now_ms)
{
    float position;

    /* 四黑后的3秒缓停期间也必须持续运行双环，不能保持旧舵机输出。 */
    State_H6UpdateHorizontalHold(now_ms);
    State_RunBallControl(now_ms);
    if (h6_stop_locked == 0U)
    {
        return;
    }

    State_H2Brake();
    /* H6完成后仍保持目标闭环，不能拉平后让球从目标点滚走。 */
    State_H6UpdateKick(now_ms);
    if (h6_flat_done != 0U)
    {
        return;
    }
    if ((g_shijue_position_valid == 0U) ||
        (g_shijue_velocity_valid == 0U))
    {
        h6_stable_start_ms = 0U;
        return;
    }

    position = g_shijue_position_cm;
    if ((position < -H6_FLAT_TOLERANCE_CM) ||
        (position > H6_FLAT_TOLERANCE_CM) ||
        (g_shijue_velocity_cm_s < -H6_FLAT_MAX_SPEED_CM_S) ||
        (g_shijue_velocity_cm_s > H6_FLAT_MAX_SPEED_CM_S))
    {
        h6_stable_start_ms = 0U;
        return;
    }

    if (h6_stable_start_ms == 0U)
    {
        h6_stable_start_ms = now_ms;
        h6_stable_min_cm = position;
        h6_stable_max_cm = position;
        return;
    }

    if (position < h6_stable_min_cm) h6_stable_min_cm = position;
    if (position > h6_stable_max_cm) h6_stable_max_cm = position;
    if ((uint32_t)(now_ms - h6_stable_start_ms) < H3_STABLE_MS)
    {
        return;
    }

    if ((h6_stable_max_cm - h6_stable_min_cm) <= H6_FLAT_STABLE_RANGE_CM)
    {
        /* 与H4/H5一致：停车且球稳定后关闭闭环，舵机回机械水平位。 */
        h6_flat_done = 1U;
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetEnabled(0U);
    }
    else
    {
        h6_stable_start_ms = now_ms;
        h6_stable_min_cm = position;
        h6_stable_max_cm = position;
    }
}

static void State_H7FinishBalance(uint32_t now_ms)
{
    float position_error;

    State_RunBallControl(now_ms);
    if (h7_stop_locked == 0U)
    {
        return;
    }

    State_H2Brake();
    if (h7_flat_done != 0U)
    {
        return;
    }
    if ((g_shijue_position_valid == 0U) ||
        (g_shijue_velocity_valid == 0U))
    {
        h7_stable_start_ms = 0U;
        return;
    }

    position_error = g_shijue_position_cm - h7_active_target_cm;
    if ((position_error < -H7_FLAT_TOLERANCE_CM) ||
        (position_error > H7_FLAT_TOLERANCE_CM) ||
        (g_shijue_velocity_cm_s < -H7_FLAT_MAX_SPEED_CM_S) ||
        (g_shijue_velocity_cm_s > H7_FLAT_MAX_SPEED_CM_S))
    {
        h7_stable_start_ms = 0U;
        return;
    }

    if (h7_stable_start_ms == 0U)
    {
        h7_stable_start_ms = now_ms;
        h7_stable_min_cm = g_shijue_position_cm;
        h7_stable_max_cm = g_shijue_position_cm;
        return;
    }
    if (g_shijue_position_cm < h7_stable_min_cm)
        h7_stable_min_cm = g_shijue_position_cm;
    if (g_shijue_position_cm > h7_stable_max_cm)
        h7_stable_max_cm = g_shijue_position_cm;
    if ((uint32_t)(now_ms - h7_stable_start_ms) < H3_STABLE_MS)
    {
        return;
    }

    if ((h7_stable_max_cm - h7_stable_min_cm) <= H7_FLAT_STABLE_RANGE_CM)
    {
        h7_flat_done = 1U;
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetEnabled(0U);
    }
    else
    {
        h7_stable_start_ms = now_ms;
        h7_stable_min_cm = g_shijue_position_cm;
        h7_stable_max_cm = g_shijue_position_cm;
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
        /* 与H3相同的判稳条件满足后，关闭闭环并回到1750us机械水平位。 */
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

    /* 进入准备页时同步视觉零点：H7和H3/H4/H5固定使用管中点。 */
    vision_origin_send_pending =
        ((current_mode == STATE_MODE_H3_BALL_MOVE) ||
         (current_mode == STATE_MODE_H4_AB_BALANCE) ||
         (current_mode == STATE_MODE_H5_LOOP_CENTER) ||
         (current_mode == STATE_MODE_H6_LOOP_TARGET) ||
         (current_mode == STATE_MODE_H7_BEND_TARGET)) ? 1U : 0U;

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

static int8_t State_H6FindPidNode(int16_t target_tenth)
{
    uint8_t i;

    for (i = 0U; i < H6_PID_NODE_COUNT; i++)
    {
        if (h6_position_pid_nodes[i].target_tenth_cm == target_tenth)
        {
            return (int8_t)i;
        }
    }
    return -1;
}

static void State_H6GetPositionPid(int16_t target_tenth,
                                   float *kp,
                                   float *ki,
                                   float *kd)
{
    uint8_t i;
    H6PositionPidNode_t *left;
    H6PositionPidNode_t *right;
    float ratio;

    if (target_tenth <= h6_position_pid_nodes[0].target_tenth_cm)
    {
        *kp = h6_position_pid_nodes[0].kp;
        *ki = h6_position_pid_nodes[0].ki;
        *kd = h6_position_pid_nodes[0].kd;
        return;
    }
    if (target_tenth >= h6_position_pid_nodes[H6_PID_NODE_COUNT - 1U].target_tenth_cm)
    {
        *kp = h6_position_pid_nodes[H6_PID_NODE_COUNT - 1U].kp;
        *ki = h6_position_pid_nodes[H6_PID_NODE_COUNT - 1U].ki;
        *kd = h6_position_pid_nodes[H6_PID_NODE_COUNT - 1U].kd;
        return;
    }

    for (i = 1U; i < H6_PID_NODE_COUNT; i++)
    {
        right = &h6_position_pid_nodes[i];
        if (target_tenth <= right->target_tenth_cm)
        {
            left = &h6_position_pid_nodes[i - 1U];
            ratio = (float)(target_tenth - left->target_tenth_cm) /
                    (float)(right->target_tenth_cm - left->target_tenth_cm);
            *kp = left->kp + (right->kp - left->kp) * ratio;
            *ki = left->ki + (right->ki - left->ki) * ratio;
            *kd = left->kd + (right->kd - left->kd) * ratio;
            return;
        }
    }
}

static void State_H6ApplyPositionPid(void)
{
    float kp, ki, kd;

    State_H6GetPositionPid(target_tenth_cm, &kp, &ki, &kd);
    BallControl_SetPositionPid(kp, ki, kd);
}

static void State_H6GetTrim(int16_t target_tenth,
                            float *base_trim_deg,
                            float *start_trim_deg,
                            float *stop_trim_deg)
{
    uint8_t i;
    const H6TrimNode_t *left;
    const H6TrimNode_t *right;
    float ratio;

    *base_trim_deg = 0.0f;
    *start_trim_deg = 0.0f;
    *stop_trim_deg = 0.0f;

    /* +/-12.5cm物理管端保持原逻辑，不把11cm节点补偿继续外推。 */
    if ((target_tenth < h6_trim_nodes[0].target_tenth_cm) ||
        (target_tenth > h6_trim_nodes[H6_TRIM_NODE_COUNT - 1U].target_tenth_cm))
    {
        return;
    }

    for (i = 1U; i < H6_TRIM_NODE_COUNT; i++)
    {
        right = &h6_trim_nodes[i];
        if (target_tenth <= right->target_tenth_cm)
        {
            left = &h6_trim_nodes[i - 1U];
            ratio = (float)(target_tenth - left->target_tenth_cm) /
                    (float)(right->target_tenth_cm - left->target_tenth_cm);
            *base_trim_deg = left->base_trim_deg +
                (right->base_trim_deg - left->base_trim_deg) * ratio;
            *start_trim_deg = left->start_trim_deg +
                (right->start_trim_deg - left->start_trim_deg) * ratio;
            *stop_trim_deg = left->stop_trim_deg +
                (right->stop_trim_deg - left->stop_trim_deg) * ratio;
            return;
        }
    }
}

static const H6MinusEndConfig_t *State_H6GetMinusEndConfig(void)
{
    if (target_tenth_cm == h6_minus9_config.target_tenth_cm)
    {
        return &h6_minus9_config;
    }
    if (target_tenth_cm == h6_minus11_config.target_tenth_cm)
    {
        return &h6_minus11_config;
    }
    return 0;
}

static H6MinusEndRuntime_t *State_H6GetMinusEndRuntime(void)
{
    if (target_tenth_cm == h6_minus9_config.target_tenth_cm)
    {
        return &h6_minus9_runtime;
    }
    if (target_tenth_cm == h6_minus11_config.target_tenth_cm)
    {
        return &h6_minus11_runtime;
    }
    return 0;
}

static void State_H6ResetMinusEndRuntime(H6MinusEndRuntime_t *runtime)
{
    if (runtime == 0) return;
    memset(runtime, 0, sizeof(*runtime));
}

static void State_H6UpdateHorizontalHold(uint32_t now_ms)
{
    const H6MinusEndConfig_t *config = State_H6GetMinusEndConfig();
    H6MinusEndRuntime_t *runtime = State_H6GetMinusEndRuntime();
    float error_cm;
    float speed_cm_s;
    uint8_t moving_away;

    if ((config == 0) || (runtime == 0))
    {
        return;
    }

    if (((uint32_t)(now_ms - start_time_ms) >= config->hold_arm_ms) &&
        (runtime->recovery_pid_active == 0U) &&
        (runtime->soft_recovery_active == 0U))
    {
        runtime->recovery_pid_active = 1U;
        State_H6ApplyPositionPid();
    }

    error_cm = BallControl_GetTargetPosition() - g_shijue_position_cm;
    speed_cm_s = g_shijue_velocity_cm_s;

    /* 修改：越过车头后切换为低增益无积分回收，避免位置环反复强力换向。 */
    if ((runtime->soft_recovery_active == 0U) &&
        (runtime->horizontal_hold == 0U) &&
        (g_shijue_position_valid != 0U) &&
        (g_shijue_velocity_valid != 0U) &&
        (g_shijue_position_cm <= config->recovery_enter_cm) &&
        (speed_cm_s >= config->recovery_enter_speed_cm_s))
    {
        runtime->soft_recovery_active = 1U;
        runtime->recovery_candidate_ms = 0U;
        runtime->start_ff_done = 1U;
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetPositionPid(config->recovery_kp,
                                   config->recovery_ki,
                                   config->recovery_kd);
    }

    if (runtime->soft_recovery_active != 0U)
    {
        runtime->hold_candidate_ms = 0U;
        /* 修改：-11cm仅按位置硬阈值退出，避免视觉速度毛刺引起PID反复切换。 */
        if ((h6_bend_active != 0U) ||
            ((config == &h6_minus11_config) &&
             (error_cm <= -config->hold_exit_error_cm)))
        {
            runtime->soft_recovery_active = 0U;
            runtime->recovery_candidate_ms = 0U;
            runtime->recovery_pid_active = 1U;
            State_H6ApplyPositionPid();
            return;
        }

        if ((error_cm > -config->hold_enter_error_cm) &&
            (error_cm < config->hold_enter_error_cm) &&
            (speed_cm_s > -config->hold_enter_speed_cm_s) &&
            (speed_cm_s < config->hold_enter_speed_cm_s))
        {
            if (runtime->recovery_candidate_ms == 0U)
            {
                runtime->recovery_candidate_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - runtime->recovery_candidate_ms) >=
                     config->recovery_confirm_ms)
            {
                runtime->soft_recovery_active = 0U;
                runtime->recovery_candidate_ms = 0U;
                runtime->recovery_pid_active = 1U;
                runtime->hold_entered_once = 1U;
                runtime->horizontal_hold = 1U;
                State_H6ApplyPositionPid();
                BallControl_SetAngleFeedforward(config->hold_trim_deg);
                BallControl_SetManualTargetAngle(config->hold_trim_deg);
            }
        }
        else
        {
            runtime->recovery_candidate_ms = 0U;
        }
        return;
    }

    if (runtime->horizontal_hold != 0U)
    {
        runtime->hold_candidate_ms = 0U;
        moving_away = (((error_cm > 0.0f) && (speed_cm_s > 0.0f)) ||
                       ((error_cm < 0.0f) && (speed_cm_s < 0.0f)));

        /* 修改：弯道立即恢复位置环；直线偏出趋势持续100ms也提前恢复。 */
        if ((error_cm <= -config->hold_exit_error_cm) ||
            (error_cm >= config->hold_exit_error_cm) ||
            (h6_bend_active != 0U))
        {
            runtime->horizontal_hold = 0U;
            runtime->wake_candidate_ms = 0U;
            BallControl_SetAngleFeedforward(0.0f);
            BallControl_SetAutoTargetAngle();
        }
        else if ((moving_away != 0U) &&
                 (((error_cm <= -config->hold_wake_error_cm) ||
                   (error_cm >= config->hold_wake_error_cm)) ||
                  (speed_cm_s <= -config->hold_wake_speed_cm_s) ||
                  (speed_cm_s >= config->hold_wake_speed_cm_s)))
        {
            if (runtime->wake_candidate_ms == 0U)
            {
                runtime->wake_candidate_ms = now_ms;
            }
            else if ((uint32_t)(now_ms - runtime->wake_candidate_ms) >=
                     config->hold_wake_confirm_ms)
            {
                runtime->horizontal_hold = 0U;
                runtime->wake_candidate_ms = 0U;
                BallControl_SetAngleFeedforward(0.0f);
                BallControl_SetAutoTargetAngle();
            }
        }
        else
        {
            runtime->wake_candidate_ms = 0U;
            BallControl_SetAngleFeedforward(config->hold_trim_deg);
        }
        return;
    }

    /* 发车扰动尚未结束时保持位置环工作，避免过早水平导致钢球滚到管端。 */
    if ((uint32_t)(now_ms - start_time_ms) < config->hold_arm_ms)
    {
        runtime->hold_candidate_ms = 0U;
        return;
    }

    if ((g_shijue_position_valid == 0U) ||
        (g_shijue_velocity_valid == 0U) ||
        (h6_bend_active != 0U))
    {
        runtime->hold_candidate_ms = 0U;
        return;
    }

    if ((error_cm > -config->hold_enter_error_cm) &&
        (error_cm < config->hold_enter_error_cm) &&
        (speed_cm_s > -config->hold_enter_speed_cm_s) &&
        (speed_cm_s < config->hold_enter_speed_cm_s))
    {
        /* 修改：首次到位立即锁住；后续重进需稳定300ms，避免过程反复切换。 */
        if (runtime->hold_entered_once == 0U)
        {
            runtime->hold_entered_once = 1U;
            runtime->hold_candidate_ms = 0U;
            runtime->wake_candidate_ms = 0U;
            runtime->horizontal_hold = 1U;
            BallControl_SetAngleFeedforward(config->hold_trim_deg);
            BallControl_SetManualTargetAngle(config->hold_trim_deg);
        }
        else if (runtime->hold_candidate_ms == 0U)
        {
            runtime->hold_candidate_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - runtime->hold_candidate_ms) >=
                 config->hold_enter_confirm_ms)
        {
            runtime->hold_candidate_ms = 0U;
            runtime->wake_candidate_ms = 0U;
            runtime->horizontal_hold = 1U;
            BallControl_SetAngleFeedforward(config->hold_trim_deg);
            BallControl_SetManualTargetAngle(config->hold_trim_deg);
        }
    }
    else
    {
        runtime->hold_candidate_ms = 0U;
    }
}

static void State_H6ResetKick(void)
{
    h6_kick_state = 0U;
    h6_still_start_ms = 0U;
    h6_kick_start_ms = 0U;
    h6_kick_angle_deg = 0.0f;
    h6_kick_applied_deg = 0.0f;
    h6_kick_start_position_cm = 0.0f;
    h6_kick_count = 0U;
    BallControl_SetAngleFeedforward(0.0f);
}

static void State_H6UpdateKick(uint32_t now_ms)
{
    float error_cm;
    float speed_cm_s;
    float moved_cm;
    float toward_speed_cm_s;
    float kick_move_cm;
    float kick_start_speed_cm_s;
    float kick_error_cm = H6_KICK_ERROR_CM;
    float kick_angle_deg;
    uint8_t kick_max_count;
    uint32_t kick_elapsed_ms;

    /* -9/-11cm共用端部专用控制，运行和停车阶段均不叠加静摩擦补偿。 */
    if (State_H6GetMinusEndConfig() != 0)
    {
        h6_kick_state = 0U;
        h6_still_start_ms = 0U;
        h6_kick_start_ms = 0U;
        h6_kick_angle_deg = 0.0f;
        h6_kick_applied_deg = 0.0f;
        h6_kick_count = 0U;
        return;
    }

    /* -7cm行驶期间由专用前馈控制，避免静摩擦踢球覆盖它；停车后恢复。 */
    if ((target_tenth_cm == -70) && (h6_stop_locked == 0U))
    {
        h6_kick_state = 0U;
        h6_still_start_ms = 0U;
        h6_kick_start_ms = 0U;
        h6_kick_angle_deg = 0.0f;
        h6_kick_applied_deg = 0.0f;
        h6_kick_count = 0U;
        return;
    }

    if ((target_tenth_cm >= -H6_NEAR_TARGET_TENTH_CM) &&
        (target_tenth_cm <= H6_NEAR_TARGET_TENTH_CM))
    {
        kick_move_cm = H6_NEAR_KICK_MOVE_CM;
        kick_start_speed_cm_s = H6_NEAR_KICK_START_SPEED_CM_S;
        kick_max_count = H6_NEAR_KICK_MAX_COUNT;
    }
    else
    {
        kick_move_cm = H6_KICK_MOVE_CM;
        kick_start_speed_cm_s = H6_KICK_START_SPEED_CM_S;
        kick_max_count = H6_KICK_MAX_COUNT;
    }

    /* -1cm仅在四黑停车后使用专用轻推，避免速度毛刺过早结束。 */
    if ((target_tenth_cm == -10) && (h6_stop_locked != 0U))
    {
        kick_error_cm = H6_MINUS1_STOP_KICK_ERROR_CM;
        kick_move_cm = H6_MINUS1_STOP_KICK_MOVE_CM;
        kick_start_speed_cm_s = H6_MINUS1_STOP_KICK_SPEED_CM_S;
        kick_max_count = H6_MINUS1_STOP_KICK_MAX_COUNT;
    }

    if (h6_kick_state == 1U)
    {
        kick_elapsed_ms = (uint32_t)(now_ms - h6_kick_start_ms);
        moved_cm = g_shijue_position_cm - h6_kick_start_position_cm;
        if (h6_kick_angle_deg < 0.0f) moved_cm = -moved_cm;
        /* 视觉速度符号与位置误差变化一致，朝目标运动时符号与误差相反。 */
        toward_speed_cm_s = (h6_kick_angle_deg < 0.0f) ?
                            g_shijue_velocity_cm_s :
                            -g_shijue_velocity_cm_s;

        if (kick_elapsed_ms < H6_KICK_RAMP_IN_MS)
        {
            h6_kick_applied_deg = h6_kick_angle_deg *
                                  (float)kick_elapsed_ms /
                                  (float)H6_KICK_RAMP_IN_MS;
        }
        else
        {
            h6_kick_applied_deg = h6_kick_angle_deg;
        }

        if ((kick_elapsed_ms < H6_KICK_MAX_MS) &&
            ((kick_elapsed_ms < H6_KICK_MIN_MS) ||
             ((moved_cm < kick_move_cm) &&
              (toward_speed_cm_s < kick_start_speed_cm_s))))
        {
            BallControl_SetAngleFeedforward(h6_kick_applied_deg);
        }
        else
        {
            /* 检测到小球真正启动后渐退前馈，避免目标角瞬间跳变。 */
            h6_kick_state = 2U;
            h6_kick_start_ms = now_ms;
        }
        return;
    }

    if (h6_kick_state == 2U)
    {
        kick_elapsed_ms = (uint32_t)(now_ms - h6_kick_start_ms);
        if (kick_elapsed_ms < H6_KICK_RAMP_OUT_MS)
        {
            BallControl_SetAngleFeedforward(
                h6_kick_applied_deg *
                (float)(H6_KICK_RAMP_OUT_MS - kick_elapsed_ms) /
                (float)H6_KICK_RAMP_OUT_MS);
        }
        else
        {
            h6_kick_state = 3U;
            h6_kick_start_ms = now_ms;
            h6_kick_angle_deg = 0.0f;
            h6_kick_applied_deg = 0.0f;
            BallControl_SetAngleFeedforward(0.0f);
        }
        return;
    }

    if (h6_kick_state == 3U)
    {
        BallControl_SetAngleFeedforward(0.0f);
        if ((uint32_t)(now_ms - h6_kick_start_ms) >= H6_KICK_RETRY_MS)
        {
            h6_kick_state = 0U;
            h6_still_start_ms = 0U;
        }
        return;
    }

    if ((g_shijue_position_valid == 0U) ||
        (g_shijue_velocity_valid == 0U))
    {
        h6_still_start_ms = 0U;
        return;
    }

    error_cm = BallControl_GetTargetPosition() - g_shijue_position_cm;
    speed_cm_s = g_shijue_velocity_cm_s;
    if (((error_cm > kick_error_cm) || (error_cm < -kick_error_cm)) &&
        (h6_kick_count < kick_max_count) &&
        (speed_cm_s <= H6_KICK_MAX_SPEED_CM_S) &&
        (speed_cm_s >= -H6_KICK_MAX_SPEED_CM_S))
    {
        if (h6_still_start_ms == 0U)
        {
            h6_still_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - h6_still_start_ms) >= H6_KICK_WAIT_MS)
        {
            /* 实测静摩擦方向：误差为负时叠加负角度，反向同理。 */
            if ((target_tenth_cm >= -H6_NEAR_TARGET_TENTH_CM) &&
                (target_tenth_cm <= H6_NEAR_TARGET_TENTH_CM))
            {
                kick_angle_deg = ((target_tenth_cm == -10) &&
                                  (h6_stop_locked != 0U)) ?
                                 H6_MINUS1_STOP_KICK_ANGLE_DEG :
                                 H6_NEAR_KICK_ANGLE_DEG;
                h6_kick_angle_deg = (error_cm < 0.0f) ?
                                    -kick_angle_deg : kick_angle_deg;
            }
            else
            {
                h6_kick_angle_deg = (error_cm < 0.0f) ?
                                    -H6_KICK_ANGLE_DEG : H6_KICK_ANGLE_DEG;
            }
            h6_kick_start_position_cm = g_shijue_position_cm;
            h6_kick_start_ms = now_ms;
            h6_kick_count++;
            h6_kick_state = 1U;
            h6_kick_applied_deg = 0.0f;
            BallControl_SetAngleFeedforward(0.0f);
        }
    }
    else
    {
        h6_still_start_ms = 0U;
    }
}

static void State_H3ApplyPositionPid(void)
{
    if (h3_braking != 0U)
    {
        BallControl_SetPositionPid(h3_brake_kp, h3_brake_ki, h3_brake_kd);
    }
    else if (h3_returning != 0U)
    {
        BallControl_SetPositionPid(h3_minus_kp, h3_minus_ki, h3_minus_kd);
    }
    else
    {
        BallControl_SetPositionPid(h3_plus_kp, h3_plus_ki, h3_plus_kd);
    }
}

static void State_LoadBallPid(StateMode_t mode)
{
    switch (mode)
    {
        case STATE_MODE_H3_BALL_MOVE:
            State_H3ApplyPositionPid();
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
            State_H6ApplyPositionPid();
            BallControl_SetAnglePid(H6_ANGLE_KP, H6_ANGLE_KI, H6_ANGLE_KD);
            break;

        case STATE_MODE_H7_BEND_TARGET:
            State_H7SetPositionPid(h7_target_phase);
            BallControl_SetAnglePid(H7_ANGLE_KP, H7_ANGLE_KI, H7_ANGLE_KD);
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

static float State_GetH7TargetTrim(void)
{
    if (h7_target_phase <= 1U) return H7_CENTER_TRIM_DEG;
    if (h7_target_phase <= 3U) return H7_PLUS5_TRIM_DEG;
    return H7_MINUS5_TRIM_DEG;
}

static float State_GetH7RunFeedforward(uint32_t now_ms)
{
    uint32_t balance_elapsed_ms = now_ms - h7_balance_start_ms;
    uint32_t ramp_elapsed_ms = now_ms - start_time_ms;
    float trim_deg = State_GetH7TargetTrim();

    if (balance_elapsed_ms < H7_RAMP_FF_IN_MS)
    {
        return (H7_RAMP_FF_DEG + trim_deg) *
               (float)balance_elapsed_ms / (float)H7_RAMP_FF_IN_MS;
    }
    if (ramp_elapsed_ms < (H7_RAMP_UP_MS - H7_RAMP_FF_OUT_MS))
    {
        return H7_RAMP_FF_DEG + trim_deg;
    }
    if (ramp_elapsed_ms < H7_RAMP_UP_MS)
    {
        float transition = (float)(ramp_elapsed_ms -
                           (H7_RAMP_UP_MS - H7_RAMP_FF_OUT_MS)) /
                           (float)H7_RAMP_FF_OUT_MS;
        return H7_RAMP_FF_DEG +
               (H7_RUN_FF_DEG - H7_RAMP_FF_DEG) * transition + trim_deg;
    }
    return H7_RUN_FF_DEG + trim_deg;
}

static float State_GetH7StopFeedforward(uint32_t now_ms)
{
    uint32_t stop_elapsed_ms = now_ms - h7_stop_start_ms;
    float trim_deg = State_GetH7TargetTrim();

    if (stop_elapsed_ms < H7_STOP_FF_IN_MS)
    {
        return trim_deg + H7_STOP_FF_DEG *
               (float)stop_elapsed_ms / (float)H7_STOP_FF_IN_MS;
    }
    if (stop_elapsed_ms < (H7_STOP_RAMP_MS - H7_STOP_FF_OUT_MS))
    {
        return trim_deg + H7_STOP_FF_DEG;
    }
    if (stop_elapsed_ms < H7_STOP_RAMP_MS)
    {
        return trim_deg + H7_STOP_FF_DEG *
               (float)(H7_STOP_RAMP_MS - stop_elapsed_ms) /
               (float)H7_STOP_FF_OUT_MS;
    }
    return trim_deg;
}

static float State_GetH6RunFeedforward(uint32_t now_ms)
{
    const H6MinusEndConfig_t *minus_end_config = State_H6GetMinusEndConfig();
    H6MinusEndRuntime_t *minus_end_runtime = State_H6GetMinusEndRuntime();
    uint32_t balance_elapsed_ms = now_ms - h6_balance_start_ms;
    uint32_t ramp_elapsed_ms = now_ms - start_time_ms;
    uint32_t bend_elapsed_ms;
    float base_trim_deg = 0.0f;
    float start_trim_deg = 0.0f;
    float unused_stop_trim_deg;
    float brake_trim_deg = 0.0f;
    float minus_end_start_trim_deg;

    /* 修改：-9/-11cm独立参数共用端部起步流程，越过中心后立即停止反推。 */
    if ((minus_end_config != 0) && (minus_end_runtime != 0))
    {
        /* 修改：球到过车尾侧且已掉头后，锁死本次起步补偿，避免再次向车头加速。 */
        if (minus_end_runtime->start_ff_done != 0U)
        {
            return 0.0f;
        }
        if ((g_shijue_position_valid != 0U) &&
            (g_shijue_velocity_valid != 0U) &&
            (g_shijue_position_cm >= minus_end_config->start_full_error_cm) &&
            (g_shijue_velocity_cm_s >= minus_end_config->start_release_speed_cm_s))
        {
            minus_end_runtime->start_ff_done = 1U;
            return 0.0f;
        }

        if (balance_elapsed_ms < H6_RAMP_FF_IN_MS)
        {
            minus_end_start_trim_deg = minus_end_config->start_trim_deg *
                                       (float)balance_elapsed_ms /
                                       (float)H6_RAMP_FF_IN_MS;
        }
        else if (ramp_elapsed_ms < minus_end_config->start_ff_out_start_ms)
        {
            minus_end_start_trim_deg = minus_end_config->start_trim_deg;
        }
        else if (ramp_elapsed_ms < minus_end_config->start_ff_out_end_ms)
        {
            minus_end_start_trim_deg = minus_end_config->start_trim_deg *
                (float)(minus_end_config->start_ff_out_end_ms - ramp_elapsed_ms) /
                (float)(minus_end_config->start_ff_out_end_ms -
                        minus_end_config->start_ff_out_start_ms);
        }
        else
        {
            return 0.0f;
        }

        if (g_shijue_position_valid != 0U)
        {
            if (g_shijue_position_cm <= 0.0f)
            {
                return 0.0f;
            }
            if (g_shijue_position_cm < minus_end_config->start_full_error_cm)
            {
                minus_end_start_trim_deg *= g_shijue_position_cm /
                                            minus_end_config->start_full_error_cm;
            }
        }
        return minus_end_start_trim_deg;
    }

    State_H6GetTrim(target_tenth_cm, &base_trim_deg,
                    &start_trim_deg, &unused_stop_trim_deg);

    if (balance_elapsed_ms < H6_RAMP_FF_IN_MS)
    {
        return (base_trim_deg + H6_RAMP_FF_DEG + start_trim_deg) *
               (float)balance_elapsed_ms / (float)H6_RAMP_FF_IN_MS;
    }
    if ((target_tenth_cm == -70) &&
        (ramp_elapsed_ms < H6_MINUS7_START_FF_OUT_START_MS))
    {
        return base_trim_deg + H6_RAMP_FF_DEG + start_trim_deg;
    }
    if ((target_tenth_cm == -70) &&
        (ramp_elapsed_ms < H6_MINUS7_START_FF_OUT_END_MS))
    {
        float transition = (float)(ramp_elapsed_ms -
                           H6_MINUS7_START_FF_OUT_START_MS) /
                           (float)(H6_MINUS7_START_FF_OUT_END_MS -
                                   H6_MINUS7_START_FF_OUT_START_MS);
        return base_trim_deg + H6_RAMP_FF_DEG +
               (H6_RUN_BIAS_FF_DEG - H6_RAMP_FF_DEG) * transition +
               start_trim_deg * (1.0f - transition);
    }
    if ((target_tenth_cm != -70) &&
        (ramp_elapsed_ms < (H6_RAMP_UP_MS - H6_RAMP_FF_OUT_MS)))
    {
        return base_trim_deg + H6_RAMP_FF_DEG + start_trim_deg;
    }
    if ((target_tenth_cm != -70) && (ramp_elapsed_ms < H6_RAMP_UP_MS))
    {
        float transition = (float)(ramp_elapsed_ms -
                           (H6_RAMP_UP_MS - H6_RAMP_FF_OUT_MS)) /
                           (float)H6_RAMP_FF_OUT_MS;
        return base_trim_deg + H6_RAMP_FF_DEG +
               (H6_RUN_BIAS_FF_DEG - H6_RAMP_FF_DEG) * transition +
               start_trim_deg * (1.0f - transition);
    }
    if ((target_tenth_cm == -70) &&
        (h6_second_bend_enter_ms != 0U))
    {
        bend_elapsed_ms = (uint32_t)(now_ms - h6_second_bend_enter_ms);
        if (bend_elapsed_ms < H6_MINUS7_SECOND_EXIT_RAMP_MS)
        {
            brake_trim_deg = H6_MINUS7_BETWEEN_TRIM_DEG *
                             (float)(H6_MINUS7_SECOND_EXIT_RAMP_MS -
                                     bend_elapsed_ms) /
                             (float)H6_MINUS7_SECOND_EXIT_RAMP_MS;
        }
    }
    else if ((target_tenth_cm == -70) &&
             (h6_first_bend_exit_ms != 0U))
    {
        bend_elapsed_ms = (uint32_t)(now_ms - h6_first_bend_exit_ms);
        if (bend_elapsed_ms < H6_MINUS7_BETWEEN_DELAY_MS)
        {
            brake_trim_deg = H6_MINUS7_EXIT_TRIM_DEG;
        }
        else if (bend_elapsed_ms < (H6_MINUS7_BETWEEN_DELAY_MS +
                                    H6_MINUS7_BETWEEN_RAMP_MS))
        {
            float transition = (float)(bend_elapsed_ms -
                               H6_MINUS7_BETWEEN_DELAY_MS) /
                               (float)H6_MINUS7_BETWEEN_RAMP_MS;
            brake_trim_deg = H6_MINUS7_EXIT_TRIM_DEG +
                             (H6_MINUS7_BETWEEN_TRIM_DEG -
                              H6_MINUS7_EXIT_TRIM_DEG) * transition;
        }
        else
        {
            brake_trim_deg = H6_MINUS7_BETWEEN_TRIM_DEG;
        }
    }
    else if ((target_tenth_cm == -70) &&
             (h6_first_bend_enter_ms != 0U))
    {
        bend_elapsed_ms = (uint32_t)(now_ms - h6_first_bend_enter_ms);
        if (bend_elapsed_ms < H6_MINUS7_BEND_LATE_DELAY_MS)
        {
            brake_trim_deg = H6_MINUS7_PRE_BEND_TRIM_DEG;
        }
        else if (bend_elapsed_ms < (H6_MINUS7_BEND_LATE_DELAY_MS +
                                    H6_MINUS7_EXIT_RAMP_MS))
        {
            float transition = (float)(bend_elapsed_ms -
                               H6_MINUS7_BEND_LATE_DELAY_MS) /
                               (float)H6_MINUS7_EXIT_RAMP_MS;
            brake_trim_deg = H6_MINUS7_PRE_BEND_TRIM_DEG +
                             (H6_MINUS7_EXIT_TRIM_DEG -
                              H6_MINUS7_PRE_BEND_TRIM_DEG) * transition;
        }
        else
        {
            brake_trim_deg = H6_MINUS7_EXIT_TRIM_DEG;
        }
    }
    else if ((target_tenth_cm == -70) &&
             (ramp_elapsed_ms >= H6_MINUS7_PRE_BEND_START_MS))
    {
        bend_elapsed_ms = ramp_elapsed_ms - H6_MINUS7_PRE_BEND_START_MS;
        if (bend_elapsed_ms < H6_MINUS7_PRE_BEND_RAMP_MS)
        {
            brake_trim_deg = H6_MINUS7_PRE_BEND_TRIM_DEG *
                             (float)bend_elapsed_ms /
                             (float)H6_MINUS7_PRE_BEND_RAMP_MS;
        }
        else
        {
            brake_trim_deg = H6_MINUS7_PRE_BEND_TRIM_DEG;
        }
    }
    else if ((target_tenth_cm == -50) &&
        (ramp_elapsed_ms >= H6_MINUS5_TRANSITION_START_MS) &&
        (ramp_elapsed_ms < H6_MINUS5_TRANSITION_END_MS))
    {
        if (ramp_elapsed_ms < (H6_MINUS5_TRANSITION_START_MS +
                               H6_MINUS5_TRANSITION_RAMP_MS))
        {
            brake_trim_deg = H6_MINUS5_TRANSITION_TRIM_DEG *
                             (float)(ramp_elapsed_ms -
                                     H6_MINUS5_TRANSITION_START_MS) /
                             (float)H6_MINUS5_TRANSITION_RAMP_MS;
        }
        else if (ramp_elapsed_ms < (H6_MINUS5_TRANSITION_END_MS -
                                    H6_MINUS5_TRANSITION_RAMP_MS))
        {
            brake_trim_deg = H6_MINUS5_TRANSITION_TRIM_DEG;
        }
        else
        {
            brake_trim_deg = H6_MINUS5_TRANSITION_TRIM_DEG *
                             (float)(H6_MINUS5_TRANSITION_END_MS -
                                     ramp_elapsed_ms) /
                             (float)H6_MINUS5_TRANSITION_RAMP_MS;
        }
    }
    else if ((target_tenth_cm == 110) &&
        (ramp_elapsed_ms >= H6_PLUS11_BRAKE_START_MS) &&
        (ramp_elapsed_ms < H6_PLUS11_BRAKE_END_MS))
    {
        if (ramp_elapsed_ms < (H6_PLUS11_BRAKE_START_MS +
                               H6_PLUS11_BRAKE_RAMP_MS))
        {
            brake_trim_deg = H6_PLUS11_BRAKE_TRIM_DEG *
                             (float)(ramp_elapsed_ms -
                                     H6_PLUS11_BRAKE_START_MS) /
                             (float)H6_PLUS11_BRAKE_RAMP_MS;
        }
        else if (ramp_elapsed_ms < (H6_PLUS11_BRAKE_END_MS -
                                    H6_PLUS11_BRAKE_RAMP_MS))
        {
            brake_trim_deg = H6_PLUS11_BRAKE_TRIM_DEG;
        }
        else
        {
            brake_trim_deg = H6_PLUS11_BRAKE_TRIM_DEG *
                             (float)(H6_PLUS11_BRAKE_END_MS -
                                     ramp_elapsed_ms) /
                             (float)H6_PLUS11_BRAKE_RAMP_MS;
        }
    }
    return base_trim_deg + H6_RUN_BIAS_FF_DEG + brake_trim_deg;
}

static float State_GetH6StopFeedforward(uint32_t now_ms)
{
    uint32_t stop_elapsed_ms = now_ms - h6_stop_start_ms;
    float base_trim_deg = 0.0f;
    float unused_start_trim_deg;
    float stop_trim_deg = 0.0f;

    /* -9/-11cm端部独立基线：停车阶段同样只保留双环闭环。 */
    if (State_H6GetMinusEndConfig() != 0)
    {
        return 0.0f;
    }

    State_H6GetTrim(target_tenth_cm, &base_trim_deg,
                    &unused_start_trim_deg, &stop_trim_deg);

    if (stop_elapsed_ms < H6_STOP_FF_IN_MS)
    {
        return base_trim_deg + (H6_STOP_FF_DEG + stop_trim_deg) *
               (float)stop_elapsed_ms / (float)H6_STOP_FF_IN_MS;
    }
    if (stop_elapsed_ms < (H6_STOP_RAMP_MS - H6_STOP_FF_OUT_MS))
    {
        return base_trim_deg + H6_STOP_FF_DEG + stop_trim_deg;
    }
    if (stop_elapsed_ms < H6_STOP_RAMP_MS)
    {
        return base_trim_deg + (H6_STOP_FF_DEG *
                (float)(H6_STOP_RAMP_MS - stop_elapsed_ms) /
                (float)H6_STOP_FF_OUT_MS) +
               stop_trim_deg * (float)(H6_STOP_RAMP_MS - stop_elapsed_ms) /
               (float)H6_STOP_FF_OUT_MS;
    }
    return base_trim_deg;
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
        h5_marker_candidate_ms = 0U;
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
        const H6MinusEndConfig_t *minus_end_config = State_H6GetMinusEndConfig();

        State_H6ResetKick();
        State_H6ResetMinusEndRuntime(&h6_minus9_runtime);
        State_H6ResetMinusEndRuntime(&h6_minus11_runtime);
        h6_drive_pwm = 0;
        h6_stop_start_pwm = 0;
        h6_drive_phase = "UP";
        h6_marker_detected = 0U;
        h6_marker_candidate_ms = 0U;
        h6_stop_locked = 0U;
        h6_black_count = 0U;
        h6_gray = Gray_Read();
        h6_bend_state = "NONE";
        h6_bend_active = 0U;
        h6_bend_count = 0U;
        h6_bend_last_seen_ms = 0U;
        h6_first_bend_enter_ms = 0U;
        h6_first_bend_exit_ms = 0U;
        h6_second_bend_enter_ms = 0U;
        h6_stop_start_ms = 0U;
        h6_stable_start_ms = 0U;
        h6_stable_min_cm = 0.0f;
        h6_stable_max_cm = 0.0f;
        h6_flat_done = 0U;
        h6_balance_started = 0U;
        h6_balance_start_ms = 0U;
        h6_start_encoder3 = Encoder3_GetTotal();
        h6_start_encoder4 = Encoder4_GetTotal();
        State_LoadBallPid(current_mode);
        if (minus_end_config != 0)
        {
            BallControl_SetPositionPid(minus_end_config->start_kp,
                                       minus_end_config->start_ki,
                                       minus_end_config->start_kd);
        }
        BallControl_SetAutoTargetAngle();
        BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetTargetPosition(0.0f);
        BallControl_SetEnabled(0U);
    }
    else if (current_mode == STATE_MODE_H7_BEND_TARGET)
    {
        State_H7ResetRun();
        State_LoadBallPid(current_mode);
        BallControl_SetAutoTargetAngle();
        BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetTargetPosition(H7_CENTER_TARGET_CM);
        BallControl_SetEnabled(0U);
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

    if (current_mode == STATE_MODE_H6_LOOP_TARGET)
    {
        /* 修改：按下开始就清除上一圈HOLD，避免0.5秒等待阶段显示旧状态。 */
        State_H6ResetMinusEndRuntime(State_H6GetMinusEndRuntime());
    }
    else if (current_mode == STATE_MODE_H7_BEND_TARGET)
    {
        /* 修改：H7每次按开始都清空上一圈弯道、目标过渡和停车状态。 */
        State_H7ResetRun();
    }

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
    /* 端部不保留0.5 cm死区：达到+/-12 cm就把视觉原点写到物理管端。 */
    if (new_target_tenth_cm >= H6_END_TARGET_TENTH_CM) new_target_tenth_cm = 125;
    if (new_target_tenth_cm <= -H6_END_TARGET_TENTH_CM) new_target_tenth_cm = -125;
    target_tenth_cm = new_target_tenth_cm;
    State_H6ResetKick();
}

static void State_HandleKey(KeyEvent_t key, uint32_t now_ms)
{
    switch (current_page)
    {
    case STATE_PAGE_SELECT:
        if (key == KEY_EVENT_1)
        {
            selected_item = (selected_item <= 2U) ? 7U : selected_item - 1U;
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_2)
        {
            selected_item = (selected_item >= 7U) ? 2U : selected_item + 1U;
            display_dirty = 1U;
        }
        else if (key == KEY_EVENT_3)
        {
            current_mode = (StateMode_t)selected_item;
            stopped_elapsed_ms = 0U;
            if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                /* 修改：每次从菜单进入H6都从0.0cm开始，不继承上一次设置。 */
                State_SetTarget(0);
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
            else if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                State_H2Brake();
                BallControl_SetEnabled(0U);
            }
            else if (current_mode == STATE_MODE_H7_BEND_TARGET)
            {
                State_H2Brake();
                BallControl_SetEnabled(0U);
            }
            else if (current_mode == STATE_MODE_H3_BALL_MOVE)
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

    case STATE_MODE_H7_BEND_TARGET:
    {
        uint8_t finished = State_H7Run(now_ms);

        if (h7_balance_started == 0U)
        {
            if (State_WheelHasMoved(h7_start_encoder3,
                                    h7_start_encoder4) == 0U)
            {
                BallControl_SetEnabled(0U);
                BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
            }
            else
            {
                h7_balance_started = 1U;
                h7_balance_start_ms = now_ms;
                BallControl_SetEnabled(1U);
            }
        }

        if (h7_balance_started != 0U)
        {
            BallControl_SetAngleFeedforward(State_GetH7RunFeedforward(now_ms));
            State_RunBallControl(now_ms);
        }
        if (finished != 0U)
        {
            /* H7第一次有效4黑冻结显示时间，完成页继续3秒缓停和-5cm稳球。 */
            State_End(STATE_PAGE_FINISHED, now_ms);
        }
        break;
    }

    case STATE_MODE_H6_LOOP_TARGET:
    {
        uint8_t finished = State_H6Run(now_ms);

        if (h6_balance_started == 0U)
        {
            if (State_WheelHasMoved(h6_start_encoder3,
                                    h6_start_encoder4) == 0U)
            {
                BallControl_SetEnabled(0U);
                BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
            }
            else
            {
                h6_balance_started = 1U;
                h6_balance_start_ms = now_ms;
                BallControl_SetEnabled(1U);
            }
        }

        if (h6_balance_started != 0U)
        {
            BallControl_SetAngleFeedforward(State_GetH6RunFeedforward(now_ms));
            State_H6UpdateKick(now_ms);
            State_H6UpdateHorizontalHold(now_ms);
            State_RunBallControl(now_ms);
        }

        if (finished != 0U)
        {
            /* 第一次4黑冻结本圈时间，完成页继续内部缓停和稳球。 */
            State_End(STATE_PAGE_FINISHED, now_ms);
        }
        break;
    }

    case STATE_MODE_H3_BALL_MOVE:
        State_RunBallControl(now_ms);

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
            /* 修改：折返瞬间切到反向独立PID，不沿用0到+5cm参数。 */
            State_H3ApplyPositionPid();
            BallControl_SetTargetPosition(H3_NEGATIVE_TARGET_CM);
            break;
        }

        if ((h3_braking == 0U) &&
            (g_shijue_position_cm <= H3_BRAKE_START_CM))
        {
            /* 折返后接近-5 cm再增强速度反馈，兼顾运行时间和制动。 */
            h3_braking = 1U;
            State_H3ApplyPositionPid();
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

typedef struct
{
    int16_t target_tenth_cm;
    int16_t correction_tenth_cm;
} VisionTargetCorrection_t;

static int16_t State_CorrectVisionTarget(int16_t target_tenth_cm)
{
    static const VisionTargetCorrection_t corrections[] =
    {
        {-110,  1}, {-100,  2}, {-90,  3}, {-80,  2}, {-70,  3},
        { -60,  2}, { -50,  2}, {-30,  1}, {-20,  0}, { 20,  0},
        {  30,  1}, {  40,  1}, { 50,  0}, { 60, -1}, { 70, -1},
        {  80,  0}, {  90,  1}, {100,  2}, {110,  4}
    };
    const uint8_t count = (uint8_t)(sizeof(corrections) /
                                    sizeof(corrections[0]));
    uint8_t i;
    float correction;
    int16_t rounded_correction;

    /* +/-12 cm以上已映射到物理管端，端点不能继续叠加刻度补偿。 */
    if ((target_tenth_cm <= -125) || (target_tenth_cm >= 125))
    {
        return target_tenth_cm;
    }

    if (target_tenth_cm <= corrections[0].target_tenth_cm)
    {
        return target_tenth_cm + corrections[0].correction_tenth_cm;
    }
    if (target_tenth_cm >= corrections[count - 1U].target_tenth_cm)
    {
        return target_tenth_cm + corrections[count - 1U].correction_tenth_cm;
    }

    for (i = 1U; i < count; i++)
    {
        const VisionTargetCorrection_t *left = &corrections[i - 1U];
        const VisionTargetCorrection_t *right = &corrections[i];

        if (target_tenth_cm <= right->target_tenth_cm)
        {
            correction = (float)left->correction_tenth_cm +
                         (float)(right->correction_tenth_cm -
                                 left->correction_tenth_cm) *
                         (float)(target_tenth_cm - left->target_tenth_cm) /
                         (float)(right->target_tenth_cm -
                                 left->target_tenth_cm);
            rounded_correction = (int16_t)(correction +
                                 ((correction >= 0.0f) ? 0.5f : -0.5f));
            return target_tenth_cm + rounded_correction;
        }
    }

    return target_tenth_cm;
}

static void State_SendVisionOrigin(void)
{
    uint16_t origin_tenth_cm;
    int16_t vision_target_tenth_cm;

    if (vision_origin_send_pending == 0U)
    {
        return;
    }

    vision_target_tenth_cm = (current_mode == STATE_MODE_H6_LOOP_TARGET) ?
                             State_CorrectVisionTarget(target_tenth_cm) : 0;

    origin_tenth_cm = (current_mode == STATE_MODE_H6_LOOP_TARGET) ?
                      (uint16_t)(125 + vision_target_tenth_cm) : 125U;
    if (Shijue_SetOriginTenthCm(origin_tenth_cm) != 0U)
    {
        vision_origin_send_pending = 0U;
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

/* 修改：调参回显使用定点数，避免精简版snprintf不支持浮点格式。 */
static void State_FormatSignedMilli(char *output, size_t size, float value)
{
    int32_t milli = (int32_t)(value * 1000.0f +
                              ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (uint32_t)((milli < 0) ? -milli : milli);

    (void)snprintf(output,
                   size,
                   "%c%lu.%03lu",
                   (milli < 0) ? '-' : '+',
                   (unsigned long)(magnitude / 1000U),
                   (unsigned long)(magnitude % 1000U));
}

static void State_ProcessDebugCommand(void)
{
    float kp, ki, kd;
    float brake_kd;
    float target;
    int16_t command_target_tenth;
    int8_t node_index;
    float position_kp, position_ki, position_kd;
    float angle_kp, angle_ki, angle_kd;
    char a_kp[12], a_ki[12], a_kd[12];
    char p_kp[12], p_ki[12], p_kd[12];
    char h3_p_kp[12], h3_p_ki[12], h3_p_kd[12];
    char h3_m_kp[12], h3_m_ki[12], h3_m_kd[12];
    char h3_b_kp[12], h3_b_ki[12], h3_b_kd[12];
    char target_text[12];
    char position_target_text[12];
    const char *text;
    int length;

    if ((debug_command_ready == 0U) || (debug_reply_ready != 0U)) return;

    if (strncmp(debug_command, "H3P ", 4U) == 0)
    {
        if (State_ParsePid(&debug_command[4], &kp, &ki, &kd) != 0U)
        {
            h3_plus_kp = kp;
            h3_plus_ki = ki;
            h3_plus_kd = kd;
            if (current_mode == STATE_MODE_H3_BALL_MOVE) State_H3ApplyPositionPid();
            State_FormatSignedMilli(h3_p_kp, sizeof(h3_p_kp), h3_plus_kp);
            State_FormatSignedMilli(h3_p_ki, sizeof(h3_p_ki), h3_plus_ki);
            State_FormatSignedMilli(h3_p_kd, sizeof(h3_p_kd), h3_plus_kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H3P=%s,%s,%s\r\n",
                              h3_p_kp, h3_p_ki, h3_p_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H3P\r\n");
        }
    }
    else if (strncmp(debug_command, "H3M ", 4U) == 0)
    {
        if (State_ParsePid(&debug_command[4], &kp, &ki, &kd) != 0U)
        {
            h3_minus_kp = kp;
            h3_minus_ki = ki;
            h3_minus_kd = kd;
            if (current_mode == STATE_MODE_H3_BALL_MOVE) State_H3ApplyPositionPid();
            State_FormatSignedMilli(h3_m_kp, sizeof(h3_m_kp), h3_minus_kp);
            State_FormatSignedMilli(h3_m_ki, sizeof(h3_m_ki), h3_minus_ki);
            State_FormatSignedMilli(h3_m_kd, sizeof(h3_m_kd), h3_minus_kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H3M=%s,%s,%s\r\n",
                              h3_m_kp, h3_m_ki, h3_m_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H3M\r\n");
        }
    }
    else if (strncmp(debug_command, "H3B ", 4U) == 0)
    {
        if (State_ParsePid(&debug_command[4], &kp, &ki, &kd) != 0U)
        {
            h3_brake_kp = kp;
            h3_brake_ki = ki;
            h3_brake_kd = kd;
            if (current_mode == STATE_MODE_H3_BALL_MOVE) State_H3ApplyPositionPid();
            State_FormatSignedMilli(h3_b_kp, sizeof(h3_b_kp), h3_brake_kp);
            State_FormatSignedMilli(h3_b_ki, sizeof(h3_b_ki), h3_brake_ki);
            State_FormatSignedMilli(h3_b_kd, sizeof(h3_b_kd), h3_brake_kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H3B=%s,%s,%s\r\n",
                              h3_b_kp, h3_b_ki, h3_b_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H3B\r\n");
        }
    }
    else if (strncmp(debug_command, "H3PID", 5U) == 0)
    {
        text = &debug_command[5];
        if ((State_ParseFloat(&text, &kp) != 0U) &&
            (State_ParseFloat(&text, &ki) != 0U) &&
            (State_ParseFloat(&text, &kd) != 0U) &&
            (State_ParseFloat(&text, &brake_kd) != 0U) &&
            (*State_SkipSpaces(text) == '\0') &&
            (kp >= 0.0f) && (ki >= 0.0f) &&
            (kd >= 0.0f) && (brake_kd >= 0.0f))
        {
            /* 兼容旧命令：前三项同时写入正反向，第四项仅作为制动Kd。 */
            h3_plus_kp = kp;
            h3_plus_ki = ki;
            h3_plus_kd = kd;
            h3_minus_kp = kp;
            h3_minus_ki = ki;
            h3_minus_kd = kd;
            h3_brake_kp = kp;
            h3_brake_ki = ki;
            h3_brake_kd = brake_kd;
            if (current_mode == STATE_MODE_H3_BALL_MOVE) State_H3ApplyPositionPid();
            State_FormatSignedMilli(h3_p_kp, sizeof(h3_p_kp), h3_plus_kp);
            State_FormatSignedMilli(h3_p_ki, sizeof(h3_p_ki), h3_plus_ki);
            State_FormatSignedMilli(h3_p_kd, sizeof(h3_p_kd), h3_plus_kd);
            State_FormatSignedMilli(h3_b_kd, sizeof(h3_b_kd), h3_brake_kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H3PID=%s,%s,%s,%s\r\n",
                              h3_p_kp, h3_p_ki, h3_p_kd, h3_b_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "ERR H3PID\r\n");
        }
    }
    else if (strcmp(debug_command, "H3GET") == 0)
    {
        State_FormatSignedMilli(h3_p_kp, sizeof(h3_p_kp), h3_plus_kp);
        State_FormatSignedMilli(h3_p_ki, sizeof(h3_p_ki), h3_plus_ki);
        State_FormatSignedMilli(h3_p_kd, sizeof(h3_p_kd), h3_plus_kd);
        State_FormatSignedMilli(h3_m_kp, sizeof(h3_m_kp), h3_minus_kp);
        State_FormatSignedMilli(h3_m_ki, sizeof(h3_m_ki), h3_minus_ki);
        State_FormatSignedMilli(h3_m_kd, sizeof(h3_m_kd), h3_minus_kd);
        State_FormatSignedMilli(h3_b_kp, sizeof(h3_b_kp), h3_brake_kp);
        State_FormatSignedMilli(h3_b_ki, sizeof(h3_b_ki), h3_brake_ki);
        State_FormatSignedMilli(h3_b_kd, sizeof(h3_b_kd), h3_brake_kd);
        length = snprintf(debug_reply, sizeof(debug_reply),
                          "H3GET P=%s,%s,%s M=%s,%s,%s B=%s,%s,%s\r\n",
                          h3_p_kp, h3_p_ki, h3_p_kd,
                          h3_m_kp, h3_m_ki, h3_m_kd,
                          h3_b_kp, h3_b_ki, h3_b_kd);
    }
    else if (strncmp(debug_command, "H6TG", 4U) == 0)
    {
        text = &debug_command[4];
        if ((current_mode == STATE_MODE_H6_LOOP_TARGET) &&
            (State_ParseFloat(&text, &target) != 0U) &&
            (*State_SkipSpaces(text) == '\0') &&
            (target >= -12.5f) && (target <= 12.5f))
        {
            State_SetTarget(State_CmToTenth(target));
            State_H6ApplyPositionPid();
            BallControl_SetAutoTargetAngle();
            BallControl_SetTargetPosition(0.0f);
            vision_origin_send_pending = 1U;
            BallControl_GetPositionPid(&kp, &ki, &kd);
            State_FormatSignedCenti(target_text, sizeof(target_text),
                                    (float)target_tenth_cm / 10.0f);
            State_FormatSignedCenti(p_kp, sizeof(p_kp), kp);
            State_FormatSignedCenti(p_ki, sizeof(p_ki), ki);
            State_FormatSignedCenti(p_kd, sizeof(p_kd), kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H6TG=%s P=%s,%s,%s\r\n",
                              target_text, p_kp, p_ki, p_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H6TG\r\n");
        }
    }
    else if (strncmp(debug_command, "H6PID", 5U) == 0)
    {
        text = &debug_command[5];
        if ((State_ParseFloat(&text, &target) != 0U) &&
            (State_ParseFloat(&text, &kp) != 0U) &&
            (State_ParseFloat(&text, &ki) != 0U) &&
            (State_ParseFloat(&text, &kd) != 0U) &&
            (*State_SkipSpaces(text) == '\0') &&
            (kp >= 0.0f) && (ki >= 0.0f) && (kd >= 0.0f))
        {
            command_target_tenth = State_CmToTenth(target);
            node_index = State_H6FindPidNode(command_target_tenth);
        }
        else
        {
            node_index = -1;
        }

        if (node_index >= 0)
        {
            h6_position_pid_nodes[(uint8_t)node_index].kp = kp;
            h6_position_pid_nodes[(uint8_t)node_index].ki = ki;
            h6_position_pid_nodes[(uint8_t)node_index].kd = kd;
            if (current_mode == STATE_MODE_H6_LOOP_TARGET)
            {
                State_H6ApplyPositionPid();
            }
            State_FormatSignedCenti(target_text, sizeof(target_text), target);
            State_FormatSignedCenti(p_kp, sizeof(p_kp), kp);
            State_FormatSignedCenti(p_ki, sizeof(p_ki), ki);
            State_FormatSignedCenti(p_kd, sizeof(p_kd), kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "OK H6PID=%s P=%s,%s,%s\r\n",
                              target_text, p_kp, p_ki, p_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H6PID NODE\r\n");
        }
    }
    else if (strncmp(debug_command, "H6GET", 5U) == 0)
    {
        text = &debug_command[5];
        if ((State_ParseFloat(&text, &target) != 0U) &&
            (*State_SkipSpaces(text) == '\0') &&
            (target >= -12.5f) && (target <= 12.5f))
        {
            command_target_tenth = State_CmToTenth(target);
            node_index = State_H6FindPidNode(command_target_tenth);
            State_H6GetPositionPid(command_target_tenth, &kp, &ki, &kd);
            State_FormatSignedCenti(target_text, sizeof(target_text),
                                    (float)command_target_tenth / 10.0f);
            State_FormatSignedCenti(p_kp, sizeof(p_kp), kp);
            State_FormatSignedCenti(p_ki, sizeof(p_ki), ki);
            State_FormatSignedCenti(p_kd, sizeof(p_kd), kd);
            length = snprintf(debug_reply, sizeof(debug_reply),
                              "H6GET T=%s NODE=%u P=%s,%s,%s\r\n",
                              target_text, (unsigned int)(node_index >= 0),
                              p_kp, p_ki, p_kd);
        }
        else
        {
            length = snprintf(debug_reply, sizeof(debug_reply), "ERR H6GET\r\n");
        }
    }
    else if ((strncmp(debug_command, "APID", 4U) == 0) &&
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
    else if (strcmp(debug_command, "STOP") == 0)
    {
        uint32_t now_ms = BSP_TimeMs();

        /* 调试急停：结束当前运行，锁止车轮并让舵机立即回机械中位。 */
        State_H2Brake();
        BallControl_SetAngleFeedforward(0.0f);
        BallControl_SetEnabled(0U);
        BallControl_SetServoCenter(BALANCE_SERVO_CENTER_US);
        if (current_page == STATE_PAGE_RUNNING)
        {
            State_End(STATE_PAGE_STOPPED, now_ms);
        }
        else if ((current_mode != STATE_MODE_NONE) &&
                 (current_page != STATE_PAGE_SELECT) &&
                 (current_page != STATE_PAGE_TARGET_SET))
        {
            start_delay_active = 0U;
            current_page = STATE_PAGE_STOPPED;
            display_dirty = 1U;
        }
        length = snprintf(debug_reply, sizeof(debug_reply), "OK STOP\r\n");
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
    char h3_active_kp[12], h3_active_ki[12], h3_active_kd[12];
    char h6_physical_target[12];
    char position_kp[12], position_ki[12], position_kd[12];
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
        const char *pid_phase;
        float kp, ki, kd;

        if (current_page == STATE_PAGE_FINISHED) phase = "DONE";
        else if (current_page == STATE_PAGE_TIMEOUT) phase = "TIMEOUT";
        else if (current_page != STATE_PAGE_RUNNING) phase = "READY";
        else if (h3_centering != 0U) phase = "CENTER";
        else if (h3_returning == 0U) phase = "PLUS";
        else phase = "MINUS";

        if (h3_braking != 0U) pid_phase = "B";
        else if (h3_returning != 0U) pid_phase = "M";
        else pid_phase = "P";
        BallControl_GetPositionPid(&kp, &ki, &kd);
        State_FormatSignedMilli(h3_active_kp, sizeof(h3_active_kp), kp);
        State_FormatSignedMilli(h3_active_ki, sizeof(h3_active_ki), ki);
        State_FormatSignedMilli(h3_active_kd, sizeof(h3_active_kd), kd);

        State_FormatSignedCenti(h3_turn_position, sizeof(h3_turn_position),
                                h3_turn_position_cm);
        State_FormatSignedCenti(h3_stable_min, sizeof(h3_stable_min),
                                h3_stable_min_cm);
        State_FormatSignedCenti(h3_stable_max, sizeof(h3_stable_max),
                                h3_stable_max_cm);
        length = snprintf((char *)balance_debug_buffer,
                          sizeof(balance_debug_buffer),
                          "H3 T=%lu E=%lu S=%s PID=%s:%s,%s,%s TP=%s P=%s V=%s TURN=%s RANGE=%s,%s\r\n",
                          (unsigned long)now_ms,
                          (unsigned long)((h3_centering != 0U) ? 0U :
                                          ((current_page == STATE_PAGE_RUNNING) ?
                                           (now_ms - start_time_ms) :
                                           stopped_elapsed_ms)),
                          phase,
                          pid_phase,
                          h3_active_kp,
                          h3_active_ki,
                          h3_active_kd,
                          target_position,
                          position,
                          velocity,
                          h3_turn_position,
                          h3_stable_min,
                          h3_stable_max);
    }
    else if (current_mode == STATE_MODE_H6_LOOP_TARGET)
    {
        float kp, ki, kd;
        const char *state;
        H6MinusEndRuntime_t *minus_end_runtime = State_H6GetMinusEndRuntime();

        BallControl_GetPositionPid(&kp, &ki, &kd);
        State_FormatSignedCenti(h6_physical_target, sizeof(h6_physical_target),
                                (float)target_tenth_cm / 10.0f);
        State_FormatSignedCenti(position_kp, sizeof(position_kp), kp);
        State_FormatSignedCenti(position_ki, sizeof(position_ki), ki);
        State_FormatSignedCenti(position_kd, sizeof(position_kd), kd);
        if (current_page == STATE_PAGE_RUNNING) state = "RUN";
        else if (current_page == STATE_PAGE_FINISHED) state = "FINISH";
        else if (current_page == STATE_PAGE_STOPPED) state = "STOPPED";
        else state = "READY";
        length = snprintf((char *)balance_debug_buffer,
                          sizeof(balance_debug_buffer),
                          "H6 S=%s D=%s BP=%d G=%02X GE=%d BLACK=%u MARK=%u FLAT=%u BEND=%s T=%s P=%s V=%s E=%s KP=%s KI=%s KD=%s TG=%s A=%s W=%s PWM=%u K=%u PV=%u VV=%u VR=%lu/%lu IMU=%u AGE=%lu RX=%lu/%lu UE=%lu/%lu/%lu HOLD=%u\r\n",
                          state, h6_drive_phase, (int)h6_drive_pwm,
                          (unsigned int)h6_gray, (int)Gray_GetError(),
                          (unsigned int)h6_black_count,
                          (unsigned int)h6_marker_detected,
                          (unsigned int)h6_flat_done,
                          h6_bend_state,
                          h6_physical_target, position, velocity, position_error,
                          position_kp, position_ki, position_kd,
                          target_angle, pipe_angle, pipe_gyro,
                          (unsigned int)BallControl_GetServoPulse(),
                          (unsigned int)h6_kick_state,
                          (unsigned int)g_shijue_position_valid,
                          (unsigned int)g_shijue_velocity_valid,
                          (unsigned long)g_shijue_position_reject_count,
                          (unsigned long)g_shijue_velocity_reject_count,
                          (unsigned int)(angleValid && gyroValid),
                          (unsigned long)(now_ms - lastPacketTime),
                          (unsigned long)debug_rx_event_count,
                          (unsigned long)debug_rx_byte_count,
                          (unsigned long)g_debug_uart_error_count,
                          (unsigned long)g_debug_rx_restart_fail_count,
                          (unsigned long)g_debug_uart_last_error,
                          (unsigned int)((minus_end_runtime != 0) ?
                                         minus_end_runtime->horizontal_hold : 0U));
    }
    else if (current_mode == STATE_MODE_H7_BEND_TARGET)
    {
        const char *state;
        const char *target_phase;

        if (current_page == STATE_PAGE_RUNNING) state = "RUN";
        else if (current_page == STATE_PAGE_FINISHED) state = "FINISH";
        else if (current_page == STATE_PAGE_STOPPED) state = "STOPPED";
        else state = "READY";

        if (h7_target_phase == 1U) target_phase = "CENTER";
        else if (h7_target_phase == 2U) target_phase = "TO+5";
        else if (h7_target_phase == 3U) target_phase = "+5";
        else if (h7_target_phase == 4U) target_phase = "TO-5";
        else target_phase = "-5";

        length = snprintf((char *)balance_debug_buffer,
                          sizeof(balance_debug_buffer),
                          "H7 S=%s D=%s BP=%d G=%02X GE=%d BLACK=%u MARK=%u BEND=%s BC=%u PH=%s TP=%s P=%s V=%s E=%s TG=%s A=%s W=%s PWM=%u FLAT=%u PV=%u VV=%u\r\n",
                          state, h7_drive_phase, (int)h7_drive_pwm,
                          (unsigned int)h7_gray, (int)Gray_GetError(),
                          (unsigned int)h7_black_count,
                          (unsigned int)h7_marker_detected,
                          h7_bend_state, (unsigned int)h7_bend_count,
                          target_phase, target_position, position, velocity,
                          position_error, target_angle, pipe_angle, pipe_gyro,
                          (unsigned int)BallControl_GetServoPulse(),
                          (unsigned int)h7_flat_done,
                          (unsigned int)g_shijue_position_valid,
                          (unsigned int)g_shijue_velocity_valid);
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
    selected_item = 7U;  /* 修改：当前下地调试默认选中H7，仍由F5进入和启动。 */
    target_tenth_cm = 0;
    vision_origin_send_pending = 0U;
    h6_kick_state = 0U;
    h6_still_start_ms = 0U;
    h6_kick_start_ms = 0U;
    h6_kick_angle_deg = 0.0f;
    h6_kick_applied_deg = 0.0f;
    h6_kick_start_position_cm = 0.0f;
    h6_kick_count = 0U;
    h6_drive_pwm = 0;
    h6_stop_start_pwm = 0;
    h6_drive_phase = "IDLE";
    h6_marker_detected = 0U;
    h6_marker_candidate_ms = 0U;
    h6_stop_locked = 0U;
    h6_black_count = 0U;
    h6_gray = Gray_Read();
    h6_bend_state = "NONE";
    h6_bend_active = 0U;
    h6_bend_count = 0U;
    h6_bend_last_seen_ms = 0U;
    h6_first_bend_enter_ms = 0U;
    h6_first_bend_exit_ms = 0U;
    h6_second_bend_enter_ms = 0U;
    h6_stop_start_ms = 0U;
    h6_stable_start_ms = 0U;
    h6_stable_min_cm = 0.0f;
    h6_stable_max_cm = 0.0f;
    h6_flat_done = 0U;
    h6_balance_started = 0U;
    State_H6ResetMinusEndRuntime(&h6_minus9_runtime);
    State_H6ResetMinusEndRuntime(&h6_minus11_runtime);
    h6_balance_start_ms = 0U;
    h6_start_encoder3 = 0;
    h6_start_encoder4 = 0;
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
    h5_marker_candidate_ms = 0U;
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
    State_H7ResetRun();
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
    State_ProcessDebugCommand();
    State_SendDebugReply();
    State_SendVisionOrigin();
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
              (current_mode == STATE_MODE_H5_LOOP_CENTER) ||
              (current_mode == STATE_MODE_H6_LOOP_TARGET) ||
              (current_mode == STATE_MODE_H7_BEND_TARGET)) &&
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
        else if ((current_mode == STATE_MODE_H6_LOOP_TARGET) &&
                 (current_page == STATE_PAGE_FINISHED))
        {
            BallControl_SetAngleFeedforward(State_GetH6StopFeedforward(now_ms));
            (void)State_H6Run(now_ms);
            State_H6FinishBalance(now_ms);
        }
        else if ((current_mode == STATE_MODE_H7_BEND_TARGET) &&
                 (current_page == STATE_PAGE_FINISHED))
        {
            BallControl_SetAngleFeedforward(State_GetH7StopFeedforward(now_ms));
            (void)State_H7Run(now_ms);
            State_H7FinishBalance(now_ms);
        }
        else
        {
            State_H2Brake();
        }
    }

    /* 修改：H3/H6/H7共用20Hz控制遥测。 */
    if ((current_mode == STATE_MODE_H3_BALL_MOVE) ||
        (current_mode == STATE_MODE_H6_LOOP_TARGET) ||
        (current_mode == STATE_MODE_H7_BEND_TARGET))
    {
        State_SendBalanceDebug(now_ms);
    }
    // State_SendEncoderTotal(now_ms);
    State_SendH4Debug(now_ms);
    State_SendH5Debug(now_ms);
    /* 陀螺仪解析回传先保留，后续需要时取消注释即可。 */
    // State_SendImuDebug(now_ms);
    State_UpdateDisplay(now_ms);
}
