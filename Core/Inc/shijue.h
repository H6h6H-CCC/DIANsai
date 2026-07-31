#ifndef __SHIJUE_H
#define __SHIJUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SHIJUE_FRAME_SIZE       8U
#define SHIJUE_FRAME_HEAD       0xAAU
#define SHIJUE_FRAME_TAIL       0x77U

#define SHIJUE_TYPE_POSITION    0x02U
#define SHIJUE_TYPE_VELOCITY    0x03U
#define SHIJUE_TYPE_SET_ORIGIN  0x10U

/* MaixCAM2 reports physical values in cm or cm/s. */
extern volatile float g_shijue_position_cm;
extern volatile float g_shijue_velocity_cm_s;
extern volatile uint8_t g_shijue_position_valid;
extern volatile uint8_t g_shijue_velocity_valid;
extern volatile uint8_t g_shijue_last_type;
extern volatile uint8_t g_shijue_last_seq;
extern volatile uint8_t g_shijue_error_flag;
extern volatile uint32_t g_shijue_position_reject_count;
extern volatile uint32_t g_shijue_velocity_reject_count;

/* Retained for old, currently disabled state code. */
typedef struct
{
    uint16_t x;
    uint16_t y;
} ShijuePoint_t;

#define SHIJUE_LABEL_SIZE 32U

extern volatile uint16_t g_shijue_count;
extern volatile uint16_t g_shijue_class_id;
extern volatile uint16_t g_shijue_x;
extern volatile uint16_t g_shijue_y;
extern volatile uint16_t g_shijue_w;
extern volatile uint16_t g_shijue_h;
extern volatile float g_shijue_score;
extern char g_shijue_label[SHIJUE_LABEL_SIZE + 1U];
extern volatile float g_shijue_vx;
extern volatile float g_shijue_vy;
extern volatile ShijuePoint_t g_shijue_centers[9];

/* Parse one complete frame. Returns 1 only when framing and checksum pass. */
uint8_t Shijue_ParseFrame8(const uint8_t *frame, uint16_t len);

/* Accepts arbitrary UART DMA chunks and preserves incomplete frames. */
void Shijue_ProcessRxBuffer(const uint8_t *buf, uint16_t len);
void Shijue_ProcessUsbRxBuffer(const uint8_t *buf, uint16_t len);

/* 设置0.0~25.0 cm绝对原点，参数单位为0.1 cm。 */
uint8_t Shijue_SetOriginTenthCm(uint16_t origin_tenth_cm);
uint8_t Shijue_Send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SHIJUE_H */
