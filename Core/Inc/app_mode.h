#ifndef APP_MODE_H
#define APP_MODE_H

#include <stdint.h>

typedef enum
{
    APP_MODE_NONE = 0,
    APP_MODE_H2_CAR_LOOP = 2,
    APP_MODE_H3_BALL_MOVE = 3,
    APP_MODE_H4_AB_BALANCE = 4,
    APP_MODE_H5_LOOP_CENTER = 5,
    APP_MODE_H6_LOOP_TARGET = 6
} AppMode_t;

typedef enum
{
    APP_STATE_SELECT = 0,
    APP_STATE_TARGET_SET,
    APP_STATE_READY,
    APP_STATE_RUNNING,
    APP_STATE_FINISHED,
    APP_STATE_STOPPED,
    APP_STATE_TIMEOUT
} AppState_t;

void AppMode_Init(void);
void AppMode_Select(AppMode_t mode);
void AppMode_BackToSelect(void);
void AppMode_ConfirmTarget(void);
void AppMode_BackToTarget(void);
void AppMode_Start(uint32_t now_ms);
void AppMode_Stop(uint32_t now_ms);
void AppMode_Finish(uint32_t now_ms);
void AppMode_Timeout(uint32_t now_ms);
void AppMode_BackToReady(void);
void AppMode_Process(uint32_t now_ms);

void AppMode_SetTargetTenthCm(int16_t target_tenth_cm);
int16_t AppMode_GetTargetTenthCm(void);
AppMode_t AppMode_GetMode(void);
AppState_t AppMode_GetState(void);
uint32_t AppMode_GetElapsedHalfSeconds(uint32_t now_ms);

#endif /* APP_MODE_H */
