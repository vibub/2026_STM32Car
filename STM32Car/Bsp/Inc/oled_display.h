#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>

void OledDisplay_Init(void);
void OledDisplay_ShowHello(void);
void OledDisplay_ShowTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
uint8_t OledDisplay_IsReady(void);

#endif /* OLED_DISPLAY_H */