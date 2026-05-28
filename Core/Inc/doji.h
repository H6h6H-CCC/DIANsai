#ifndef __DOJI_H
#define __DOJI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Doji_SendRaw(const char *cmd);

void Doji_ReadId(uint8_t id);
void Doji_SetId(uint8_t current_id, uint8_t new_id);
void Doji_SetBaudPreset(uint8_t id, uint8_t preset);

void Doji_MovePos(uint8_t id, uint16_t pos, uint16_t time_ms);
void Doji_Stop(uint8_t id);
void Doji_Pause(uint8_t id);
void Doji_Resume(uint8_t id);
void Doji_ReadAngle(uint8_t id);
void Doji_TorqueOff(uint8_t id);
void Doji_TorqueOn(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __DOJI_H */
