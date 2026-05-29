#include "hcsr04.h"

#define HCSR04_TIMEOUT_US 30000U
#define HCSR04_TRIG_US    10U

static uint8_t dwt_ready = 0U;

static void HCSR04_EnableGpioClock(GPIO_TypeDef *port)
{
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
#ifdef GPIOH
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#ifdef GPIOI
    else if (port == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
}

static void HCSR04_DwtInit(void)
{
    if (dwt_ready) return;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = 1U;
}

static void HCSR04_DelayUs(uint32_t us)
{
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    }
}

static uint8_t HCSR04_WaitPin(HCSR04_HandleTypeDef *hcsr04, GPIO_PinState state)
{
    uint32_t ticks = HCSR04_TIMEOUT_US * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;

    while (HAL_GPIO_ReadPin(hcsr04->echo_port, hcsr04->echo_pin) != state) {
        if ((uint32_t)(DWT->CYCCNT - start) > ticks) return 0U;
    }

    return 1U;
}

void HCSR04_Init(HCSR04_HandleTypeDef *hcsr04,
                 GPIO_TypeDef *trig_port, uint16_t trig_pin,
                 GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
    GPIO_InitTypeDef gpio = {0};

    hcsr04->trig_port = trig_port;
    hcsr04->trig_pin = trig_pin;
    hcsr04->echo_port = echo_port;
    hcsr04->echo_pin = echo_pin;

    HCSR04_DwtInit();
    HCSR04_EnableGpioClock(trig_port);
    HCSR04_EnableGpioClock(echo_port);

    gpio.Pin = trig_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(trig_port, &gpio);
    HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

    gpio.Pin = echo_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(echo_port, &gpio);
}

uint8_t HCSR04_ReadUs(HCSR04_HandleTypeDef *hcsr04, uint32_t *echo_us)
{
    uint32_t start;
    uint32_t end;
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    HAL_GPIO_WritePin(hcsr04->trig_port, hcsr04->trig_pin, GPIO_PIN_RESET);
    HCSR04_DelayUs(2U);
    HAL_GPIO_WritePin(hcsr04->trig_port, hcsr04->trig_pin, GPIO_PIN_SET);
    HCSR04_DelayUs(HCSR04_TRIG_US);
    HAL_GPIO_WritePin(hcsr04->trig_port, hcsr04->trig_pin, GPIO_PIN_RESET);

    if (!HCSR04_WaitPin(hcsr04, GPIO_PIN_SET)) return 0U;
    start = DWT->CYCCNT;

    if (!HCSR04_WaitPin(hcsr04, GPIO_PIN_RESET)) return 0U;
    end = DWT->CYCCNT;

    *echo_us = (uint32_t)(end - start) / cycles_per_us;
    return 1U;
}

uint8_t HCSR04_ReadCm(HCSR04_HandleTypeDef *hcsr04, float *distance_cm)
{
    uint32_t echo_us = 0U;

    if (!HCSR04_ReadUs(hcsr04, &echo_us)) return 0U;

    *distance_cm = (float)echo_us * 0.01715f;
    return 1U;
}
