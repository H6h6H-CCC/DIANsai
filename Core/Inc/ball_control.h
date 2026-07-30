#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H

#include <stdint.h>

/* 初始化后默认增益为 0，完成机械和传感器标定后再设置实际参数。 */
void BallControl_Init(void);
void BallControl_SetEnabled(uint8_t enabled);
void BallControl_SetTargetPosition(float target_cm);
void BallControl_SetServoCenter(uint16_t center_us);
void BallControl_SetPositionPid(float kp, float ki, float kd);
void BallControl_SetAnglePid(float kp, float ki, float kd);

/* 视觉完成像素到厘米换算后，通过此接口送入钢球位置。 */
void BallControl_SetPosition(float position_cm, uint32_t timestamp_ms);
/* 视觉上报 STATUS=0 时立即停用上一帧位置，避免继续使用失效测量。 */
void BallControl_InvalidatePosition(void);

/* JY61P 轴向和零点标定完成后，通过此接口送入摆杆角度和角速度。 */
void BallControl_SetPipeAngle(float angle_deg,
                              float gyro_dps,
                              uint32_t timestamp_ms);

/* 主循环持续调用；内环在函数内部按 10 ms 周期运行。 */
void BallControl_Process(uint32_t now_ms);

float BallControl_GetTargetAngle(void);
uint16_t BallControl_GetServoPulse(void);

#endif /* BALL_CONTROL_H */
