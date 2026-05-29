#ifndef __GRAY_H
#define __GRAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Gray_Read(void);
int16_t Gray_GetError(void);
uint8_t Gray_AllBlack(void);
uint8_t Gray_AllWhite(void);

#ifdef __cplusplus
}
#endif

#endif /* __GRAY_H */
