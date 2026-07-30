#ifndef MENU_H
#define MENU_H

#include <stdint.h>

#include "bsp_key.h"

void Menu_Init(uint32_t now_ms);
void Menu_Process(KeyEvent_t key, uint32_t now_ms);

#endif /* MENU_H */
