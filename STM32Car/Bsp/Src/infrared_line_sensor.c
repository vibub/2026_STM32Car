#include "infrared_line_sensor.h"
#include "main.h"

#if (INFRARED_LINE_REVERSE_ORDER != 0U)
static uint8_t InfraredLineSensor_ReverseMask(uint8_t mask)
{
    uint8_t reversed = 0U;

    if ((mask & INFRARED_LINE_X1_MASK) != 0U) {
        reversed |= INFRARED_LINE_X4_MASK;
    }
    if ((mask & INFRARED_LINE_X2_MASK) != 0U) {
        reversed |= INFRARED_LINE_X3_MASK;
    }
    if ((mask & INFRARED_LINE_X3_MASK) != 0U) {
        reversed |= INFRARED_LINE_X2_MASK;
    }
    if ((mask & INFRARED_LINE_X4_MASK) != 0U) {
        reversed |= INFRARED_LINE_X1_MASK;
    }
    return reversed;
}
#endif

void InfraredLineSensor_Init(void)
{
    /* GPIO 时钟和输入模式由 CM7 的 MX_GPIO_Init() 统一配置。 */
}

uint8_t InfraredLineSensor_ReadMask(void)
{
    uint32_t input = IR_LINE_PORT->IDR;
    uint8_t mask = 0U;

    /* 一次读取同一端口，避免四个通道跨时刻采样。 */
    if ((input & IR_LINE_X1_PIN) != 0U) {
        mask |= INFRARED_LINE_X1_MASK;
    }
    if ((input & IR_LINE_X2_PIN) != 0U) {
        mask |= INFRARED_LINE_X2_MASK;
    }
    if ((input & IR_LINE_X3_PIN) != 0U) {
        mask |= INFRARED_LINE_X3_MASK;
    }
    if ((input & IR_LINE_X4_PIN) != 0U) {
        mask |= INFRARED_LINE_X4_MASK;
    }

#if (INFRARED_LINE_ACTIVE_HIGH == 0U)
    mask = (uint8_t) ((~mask) & INFRARED_LINE_ALL_MASK);
#endif
#if (INFRARED_LINE_REVERSE_ORDER != 0U)
    mask = InfraredLineSensor_ReverseMask(mask);
#endif

    return mask;
}
