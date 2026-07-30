#include "infrared_line_sensor.h"
#include "main.h"

#define INFRARED_LINE_I2C_ADDRESS       (0x12U << 1U) /* 八路红外模块的 HAL 传输地址。 */
#define INFRARED_LINE_STATUS_REGISTER   0x30U         /* 八路探头状态寄存器地址。 */
#define INFRARED_LINE_I2C_TIMEOUT_MS    2U            /* 单次 I2C 读取超时时间，单位为毫秒。 */

static uint8_t s_raw_value;
static uint32_t s_read_count;
static uint32_t s_error_count;

static uint8_t InfraredLineSensor_ReverseBits(uint8_t value)
{
    uint8_t index;
    uint8_t reversed = 0U;

    for (index = 0U; index < 8U; index++) {
        if ((value & (uint8_t) (1U << index)) != 0U) {
            reversed |= (uint8_t) (1U << (7U - index));
        }
    }
    return reversed;
}

void InfraredLineSensor_Init(void)
{
    s_raw_value = 0xFFU;
    s_read_count = 0U;
    s_error_count = 0U;
}

uint8_t InfraredLineSensor_ReadMask(uint8_t *sensor_mask)
{
    uint8_t raw_value;

    if (sensor_mask == 0) {
        s_error_count++;
        return 0U;
    }

    if (HAL_I2C_Mem_Read(&hi2c4,
                         INFRARED_LINE_I2C_ADDRESS,
                         INFRARED_LINE_STATUS_REGISTER,
                         I2C_MEMADD_SIZE_8BIT,
                         &raw_value,
                         1U,
                         INFRARED_LINE_I2C_TIMEOUT_MS) != HAL_OK) {
        s_error_count++;
        return 0U;
    }

    s_raw_value = raw_value;
    s_read_count++;
    /* 黑线位为 0；实测原始位序左右相反，再反转位序归一化。 */
    *sensor_mask = InfraredLineSensor_ReverseBits(
        (uint8_t) (~raw_value));
    return 1U;
}

uint8_t InfraredLineSensor_GetRawValue(void)
{
    return s_raw_value;
}

uint32_t InfraredLineSensor_GetReadCount(void)
{
    return s_read_count;
}

uint32_t InfraredLineSensor_GetErrorCount(void)
{
    return s_error_count;
}
