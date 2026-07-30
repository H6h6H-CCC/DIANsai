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
    /* 新车头方向的右轮：负值前进，正值后退。 */
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
    /* 新车头方向的左轮：负值前进，正值后退。 */
    uint16_t pwm_abs = ClampPwmAbs(pwm);
    if (pwm_abs == 0) {
        BSP_MotorWrite(BSP_MOTOR_B, 1U, 1U, 0U);
    } else if (pwm > 0) {
        BSP_MotorWrite(BSP_MOTOR_B, 0U, 1U, pwm_abs);
    } else {
        BSP_MotorWrite(BSP_MOTOR_B, 1U, 0U, pwm_abs);
    }
}

void Moter_A_Brake(void)
{
    /* IN1/IN2同时为高并保持满PWM，使用H桥短路制动。 */
    BSP_MotorWrite(BSP_MOTOR_A, 1U, 1U, PWM_MAX);
}

void Moter_B_Brake(void)
{
    /* IN1/IN2同时为高并保持满PWM，使用H桥短路制动。 */
    BSP_MotorWrite(BSP_MOTOR_B, 1U, 1U, PWM_MAX);
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
