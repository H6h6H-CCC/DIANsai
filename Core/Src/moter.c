#include "moter.h"
#include "bsp_motor.h"

#define PWM_MAX 1000
static uint16_t ClampPwmAbs(int16_t pwm)
{
    uint16_t pwm_abs = (pwm < 0) ? (uint16_t)(-pwm) : (uint16_t)pwm;
    if (pwm_abs > PWM_MAX) return PWM_MAX;
    return pwm_abs;
}

void Moter_Init(void)
{
    BSP_MotorInit();
}

void Moter_A(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        BSP_MotorWrite(BSP_MOTOR_A, 1U, 1U, 0U);
    } else if (pwm > 0) {
        BSP_MotorWrite(BSP_MOTOR_A, 0U, 1U, pwm_abs);
    } else {
        BSP_MotorWrite(BSP_MOTOR_A, 1U, 0U, pwm_abs);
    }
}

void Moter_B(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        BSP_MotorWrite(BSP_MOTOR_B, 1U, 1U, 0U);
    } else if (pwm > 0) {
        BSP_MotorWrite(BSP_MOTOR_B, 0U, 1U, pwm_abs);
    } else {
        BSP_MotorWrite(BSP_MOTOR_B, 1U, 0U, pwm_abs);
    }
}

void Moter_C(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        BSP_MotorWrite(BSP_MOTOR_C, 0U, 0U, 0U);
    } else if (pwm > 0) {
        BSP_MotorWrite(BSP_MOTOR_C, 1U, 0U, pwm_abs);
    } else {
        BSP_MotorWrite(BSP_MOTOR_C, 0U, 1U, pwm_abs);
    }
}

void Moter_D(int16_t pwm)
{
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        BSP_MotorWrite(BSP_MOTOR_D, 0U, 0U, 0U);
    } else if (pwm > 0) {
        BSP_MotorWrite(BSP_MOTOR_D, 1U, 0U, pwm_abs);
    } else {
        BSP_MotorWrite(BSP_MOTOR_D, 0U, 1U, pwm_abs);
    }
}
