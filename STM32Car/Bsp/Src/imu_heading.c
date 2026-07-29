#include "imu_heading.h"

void ImuHeading_Init(void)
{
    /* 等待用户确认 STM32 侧 I2C/SPI 接线后接入 LSM6DSV16X。 */
}

void ImuHeading_Poll(void)
{
}

uint8_t ImuHeading_IsFresh(uint32_t now_ms)
{
    (void) now_ms;
    return 0U;
}

float ImuHeading_GetDegrees(void)
{
    return 0.0f;
}
