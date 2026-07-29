#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#define APP_MODE_STOP 0x00U
#define APP_MODE_GRAY_LINE 0x0DU

void App_Init(void);
void App_Loop(void);
uint8_t App_RequestMode(uint8_t mode);
uint8_t App_GetCurrentMode(void);

#endif /* APP_MAIN_H */
