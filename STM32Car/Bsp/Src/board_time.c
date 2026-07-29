#include "board_time.h"
#include "stm32h7xx_hal.h"

uint32_t Board_TimeMs(void)
{
    return HAL_GetTick();
}
