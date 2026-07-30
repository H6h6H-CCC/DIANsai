#include "bsp_key.h"

#include "main.h"

#define KEY_COUNT        4U
#define KEY_DEBOUNCE_MS 20U
#define KEY_REPEAT_DELAY_MS    500U
#define KEY_REPEAT_INTERVAL_MS 100U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} KeyHardware_t;

static const KeyHardware_t key_hardware[KEY_COUNT] =
{
    {GPIOF, GPIO_PIN_5},  /* KEY1 */
    {GPIOF, GPIO_PIN_3},  /* KEY2 */
    {GPIOC, GPIO_PIN_13}, /* KEY3 */
    {GPIOE, GPIO_PIN_4}   /* KEY4 */
};

static uint8_t raw_pressed[KEY_COUNT];
static uint8_t stable_pressed[KEY_COUNT];
static uint32_t changed_time_ms[KEY_COUNT];
static uint32_t pressed_time_ms[KEY_COUNT];
static uint32_t repeat_time_ms[KEY_COUNT];

static uint8_t BSP_KeyRead(uint8_t index)
{
    return HAL_GPIO_ReadPin(key_hardware[index].port,
                            key_hardware[index].pin) == GPIO_PIN_RESET;
}

void BSP_KeyInit(void)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        raw_pressed[i] = BSP_KeyRead(i);
        stable_pressed[i] = raw_pressed[i];
        changed_time_ms[i] = 0U;
        pressed_time_ms[i] = 0U;
        repeat_time_ms[i] = 0U;
    }
}

KeyEvent_t BSP_KeyScan(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        uint8_t pressed = BSP_KeyRead(i);

        if (pressed != raw_pressed[i])
        {
            raw_pressed[i] = pressed;
            changed_time_ms[i] = now_ms;
        }

        if ((stable_pressed[i] != raw_pressed[i]) &&
            ((uint32_t)(now_ms - changed_time_ms[i]) >= KEY_DEBOUNCE_MS))
        {
            stable_pressed[i] = raw_pressed[i];
            if (stable_pressed[i] != 0U)
            {
                pressed_time_ms[i] = now_ms;
                repeat_time_ms[i] = now_ms;
                return (KeyEvent_t)(KEY_EVENT_1 + i);
            }
        }
    }

    /* H6 调值时，KEY1/KEY2 长按 500 ms 后每 100 ms 连发一次。 */
    for (i = 0U; i < 2U; i++)
    {
        if ((stable_pressed[i] != 0U) &&
            ((uint32_t)(now_ms - pressed_time_ms[i]) >= KEY_REPEAT_DELAY_MS) &&
            ((uint32_t)(now_ms - repeat_time_ms[i]) >= KEY_REPEAT_INTERVAL_MS))
        {
            repeat_time_ms[i] = now_ms;
            return (KeyEvent_t)(KEY_EVENT_1 + i);
        }
    }

    return KEY_EVENT_NONE;
}
