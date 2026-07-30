#include "shijue.h"

#include "ball_control.h"
#include "bsp_time.h"
#include "bsp_uart.h"

#include <string.h>

volatile float g_shijue_origin_cm = 12.50f;
volatile float g_shijue_position_cm = 0.0f;
volatile float g_shijue_velocity_cm_s = 0.0f;
volatile uint8_t g_shijue_origin_valid = 0U;
volatile uint8_t g_shijue_position_valid = 0U;
volatile uint8_t g_shijue_velocity_valid = 0U;
volatile uint8_t g_shijue_last_type = 0U;
volatile uint8_t g_shijue_last_seq = 0U;
volatile uint8_t g_shijue_error_flag = 0U;

/* Legacy symbols kept so the disabled old state code still compiles. */
volatile uint16_t g_shijue_count = 0U;
volatile uint16_t g_shijue_class_id = 0U;
volatile uint16_t g_shijue_x = 0U;
volatile uint16_t g_shijue_y = 0U;
volatile uint16_t g_shijue_w = 0U;
volatile uint16_t g_shijue_h = 0U;
volatile float g_shijue_score = 0.0f;
char g_shijue_label[SHIJUE_LABEL_SIZE + 1U] = {0};
volatile float g_shijue_vx = 0.0f;
volatile float g_shijue_vy = 0.0f;
volatile ShijuePoint_t g_shijue_centers[9] = {
    {22U, 22U}, {160U, 22U}, {296U, 22U},
    {22U, 160U}, {160U, 160U}, {296U, 160U},
    {22U, 296U}, {160U, 296U}, {290U, 290U}
};

static uint8_t rx_frame[SHIJUE_FRAME_SIZE];
static uint8_t rx_frame_len;
static uint8_t tx_frame[SHIJUE_FRAME_SIZE];
static uint8_t tx_seq;

static uint8_t Shijue_CheckFrame(const uint8_t *frame)
{
    uint8_t checksum;

    if ((frame[0] != SHIJUE_FRAME_HEAD) ||
        (frame[SHIJUE_FRAME_SIZE - 1U] != SHIJUE_FRAME_TAIL))
    {
        return 0U;
    }

    checksum = (uint8_t)(frame[0] + frame[1] + frame[2] +
                         frame[3] + frame[4] + frame[5]);
    return checksum == frame[6];
}

static int16_t Shijue_ReadI16Le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

uint8_t Shijue_ParseFrame8(const uint8_t *frame, uint16_t len)
{
    float value;
    uint8_t valid;

    if ((frame == NULL) || (len != SHIJUE_FRAME_SIZE) ||
        (Shijue_CheckFrame(frame) == 0U))
    {
        g_shijue_error_flag = 1U;
        return 0U;
    }

    value = (float)Shijue_ReadI16Le(&frame[2]) / 100.0f;
    valid = ((frame[4] & 0x01U) != 0U) ? 1U : 0U;
    g_shijue_last_type = frame[1];
    g_shijue_last_seq = frame[5];

    switch (frame[1])
    {
    case SHIJUE_TYPE_ORIGIN:
        g_shijue_origin_valid = valid;
        if (valid != 0U)
        {
            g_shijue_origin_cm = value;
        }
        break;

    case SHIJUE_TYPE_POSITION:
        g_shijue_position_valid = valid;
        if (valid != 0U)
        {
            g_shijue_position_cm = value;
            BallControl_SetPosition(value, BSP_TimeMs());
        }
        else
        {
            BallControl_InvalidatePosition();
        }
        break;

    case SHIJUE_TYPE_VELOCITY:
        g_shijue_velocity_valid = valid;
        if (valid != 0U)
        {
            g_shijue_velocity_cm_s = value;
        }
        break;

    default:
        g_shijue_error_flag = 1U;
        return 0U;
    }

    g_shijue_count++;
    g_shijue_error_flag = 0U;
    return 1U;
}

static void Shijue_ResyncFrame(void)
{
    uint8_t next_head;

    for (next_head = 1U; next_head < SHIJUE_FRAME_SIZE; next_head++)
    {
        if (rx_frame[next_head] == SHIJUE_FRAME_HEAD)
        {
            rx_frame_len = (uint8_t)(SHIJUE_FRAME_SIZE - next_head);
            memmove(rx_frame, &rx_frame[next_head], rx_frame_len);
            return;
        }
    }

    rx_frame_len = 0U;
}

void Shijue_ProcessRxBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == NULL)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        if (rx_frame_len == 0U)
        {
            if (buf[i] != SHIJUE_FRAME_HEAD)
            {
                continue;
            }
            rx_frame[rx_frame_len++] = buf[i];
            continue;
        }

        rx_frame[rx_frame_len++] = buf[i];
        if (rx_frame_len == SHIJUE_FRAME_SIZE)
        {
            if (Shijue_ParseFrame8(rx_frame, SHIJUE_FRAME_SIZE) != 0U)
            {
                rx_frame_len = 0U;
            }
            else
            {
                Shijue_ResyncFrame();
            }
        }
    }
}

void Shijue_ProcessUsbRxBuffer(const uint8_t *buf, uint16_t len)
{
    Shijue_ProcessRxBuffer(buf, len);
}

uint8_t Shijue_SetOrigin(uint8_t origin_cm)
{
    if ((origin_cm < 1U) || (origin_cm > 25U) ||
        (BSP_UartTxReady(BSP_UART_1) == 0U))
    {
        return 0U;
    }

    tx_frame[0] = SHIJUE_FRAME_HEAD;
    tx_frame[1] = SHIJUE_TYPE_SET_ORIGIN;
    tx_frame[2] = origin_cm;
    tx_frame[3] = 0U;
    tx_frame[4] = 1U;
    tx_frame[5] = tx_seq;
    tx_frame[6] = (uint8_t)(tx_frame[0] + tx_frame[1] + tx_frame[2] +
                            tx_frame[3] + tx_frame[4] + tx_frame[5]);
    tx_frame[7] = SHIJUE_FRAME_TAIL;

    if (BSP_UartSendDma(BSP_UART_1, tx_frame, SHIJUE_FRAME_SIZE) != BSP_STATUS_OK)
    {
        return 0U;
    }

    tx_seq++;
    return 1U;
}

uint8_t Shijue_Send(const uint8_t *data, uint16_t len)
{
    return BSP_UartSendDma(BSP_UART_1, data, len) == BSP_STATUS_OK;
}
