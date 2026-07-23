#include "btn.h"
#include "bsp_btn.h"

void BTN_Init(void)
{
    BSP_BtnInit();
}

void BTN_Set12(int16_t pwm)
{
    BSP_BtnWritePair(0U, pwm);
}

void BTN_Set34(int16_t pwm)
{
    BSP_BtnWritePair(1U, pwm);
}
