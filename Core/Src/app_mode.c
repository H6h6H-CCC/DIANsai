#include "app_mode.h"

static AppMode_t current_mode;
static AppState_t current_state;
static uint32_t start_time_ms;
static uint32_t stopped_elapsed_ms;
static int16_t target_tenth_cm;

static void AppMode_End(AppState_t state, uint32_t now_ms)
{
    if (current_state == APP_STATE_RUNNING)
    {
        stopped_elapsed_ms = now_ms - start_time_ms;
        current_state = state;
    }
}

void AppMode_Init(void)
{
    current_mode = APP_MODE_NONE;
    current_state = APP_STATE_SELECT;
    start_time_ms = 0U;
    stopped_elapsed_ms = 0U;
    target_tenth_cm = 0;
}

void AppMode_Select(AppMode_t mode)
{
    current_mode = mode;
    stopped_elapsed_ms = 0U;
    current_state = (mode == APP_MODE_H6_LOOP_TARGET)
                  ? APP_STATE_TARGET_SET
                  : APP_STATE_READY;
}

void AppMode_BackToSelect(void)
{
    current_mode = APP_MODE_NONE;
    current_state = APP_STATE_SELECT;
}

void AppMode_ConfirmTarget(void)
{
    if (current_state == APP_STATE_TARGET_SET)
    {
        current_state = APP_STATE_READY;
    }
}

void AppMode_BackToTarget(void)
{
    if (current_mode == APP_MODE_H6_LOOP_TARGET)
    {
        current_state = APP_STATE_TARGET_SET;
    }
}

void AppMode_Start(uint32_t now_ms)
{
    if (current_state == APP_STATE_READY)
    {
        start_time_ms = now_ms;
        stopped_elapsed_ms = 0U;
        current_state = APP_STATE_RUNNING;
    }
}

void AppMode_Stop(uint32_t now_ms)
{
    AppMode_End(APP_STATE_STOPPED, now_ms);
}

void AppMode_Finish(uint32_t now_ms)
{
    AppMode_End(APP_STATE_FINISHED, now_ms);
}

void AppMode_Timeout(uint32_t now_ms)
{
    AppMode_End(APP_STATE_TIMEOUT, now_ms);
}

void AppMode_BackToReady(void)
{
    if ((current_state == APP_STATE_STOPPED) ||
        (current_state == APP_STATE_FINISHED) ||
        (current_state == APP_STATE_TIMEOUT))
    {
        stopped_elapsed_ms = 0U;
        current_state = APP_STATE_READY;
    }
}

void AppMode_Process(uint32_t now_ms)
{
    (void)now_ms;

    if (current_state != APP_STATE_RUNNING)
    {
        return;
    }

    switch (current_mode)
    {
    case APP_MODE_H2_CAR_LOOP:
    case APP_MODE_H3_BALL_MOVE:
    case APP_MODE_H4_AB_BALANCE:
    case APP_MODE_H5_LOOP_CENTER:
    case APP_MODE_H6_LOOP_TARGET:
        /* 各题控制与完成条件后续在这里接入。 */
        break;

    default:
        break;
    }
}

void AppMode_SetTargetTenthCm(int16_t new_target_tenth_cm)
{
    /* 25 cm 摆杆以中点 O 为 0，允许设置到两端 ±12.5 cm。 */
    if (new_target_tenth_cm > 125) new_target_tenth_cm = 125;
    if (new_target_tenth_cm < -125) new_target_tenth_cm = -125;
    target_tenth_cm = new_target_tenth_cm;
}

int16_t AppMode_GetTargetTenthCm(void)
{
    return target_tenth_cm;
}

AppMode_t AppMode_GetMode(void)
{
    return current_mode;
}

AppState_t AppMode_GetState(void)
{
    return current_state;
}

uint32_t AppMode_GetElapsedHalfSeconds(uint32_t now_ms)
{
    uint32_t elapsed_ms = stopped_elapsed_ms;

    if (current_state == APP_STATE_RUNNING)
    {
        elapsed_ms = now_ms - start_time_ms;
    }

    return elapsed_ms / 500U;
}
