#include "imu_heading.h"
#include "board_time.h"
#include "main.h"

#define LSM6DSV16X_I2C_ADDRESS_LOW      (0x6AU << 1U) /* SA0 为低电平时的 HAL 左移地址。 */
#define LSM6DSV16X_I2C_ADDRESS_HIGH     (0x6BU << 1U) /* SA0 为高电平时的 HAL 左移地址。 */
#define LSM6DSV16X_WHO_AM_I_VALUE       0x70U         /* LSM6DSV16X 芯片身份寄存器期望值。 */
#define LSM6DSV16X_I2C_TIMEOUT_MS       10U           /* 单次阻塞式 I2C 访问超时，单位为毫秒。 */

#define LSM6DSV16X_REG_WHO_AM_I         0x0FU /* 芯片身份寄存器地址。 */
#define LSM6DSV16X_REG_CTRL1            0x10U /* 加速度计控制寄存器地址。 */
#define LSM6DSV16X_REG_CTRL2            0x11U /* 陀螺仪控制寄存器地址。 */
#define LSM6DSV16X_REG_CTRL3            0x12U /* 数据锁存与地址递增控制寄存器地址。 */
#define LSM6DSV16X_REG_STATUS           0x1EU /* 数据就绪状态寄存器地址。 */
#define LSM6DSV16X_REG_OUT_TEMP_L       0x20U /* 温度、陀螺仪和加速度连续输出区起始地址。 */

#define LSM6DSV16X_STATUS_GDA           0x02U    /* 陀螺仪新数据可用标志。 */
#define LSM6DSV16X_CTRL1_XL_60HZ_2G     0x05U    /* 加速度计 60 Hz、±2 g 配置值。 */
#define LSM6DSV16X_CTRL2_G_60HZ_250DPS  0x05U    /* 陀螺仪 60 Hz、±250 dps 配置值。 */
#define LSM6DSV16X_CTRL3_BDU_IF_INC     0x44U    /* 开启块数据锁存和寄存器地址自动递增。 */
#define LSM6DSV16X_GYRO_250_SENS_DPS    0.00875f /* ±250 dps 量程的每 LSB 角速度。 */
#define IMU_HEADING_FRESH_MS            100U     /* 航向数据允许的最大陈旧时间，单位为毫秒。 */

typedef struct {
    uint8_t ready;
    uint8_t address;
    uint8_t who_am_i;
    uint32_t last_sample_ms;
    uint32_t read_count;
    uint32_t error_count;
    uint8_t init_step;
    float heading_deg;
    float gyro_z_dps;
} ImuHeadingState;

static ImuHeadingState s_imu;
volatile uint8_t g_imu_ready = 0U;
volatile uint8_t g_imu_who_am_i = 0U;
volatile uint8_t g_imu_i2c_address = 0U;
volatile uint8_t g_imu_init_step = 0xEEU;
volatile uint32_t g_imu_read_count = 0U;
volatile uint32_t g_imu_error_count = 0U;
volatile float g_imu_heading_deg = 0.0f;
volatile float g_imu_gyro_z_dps = 0.0f;
volatile uint32_t g_imu_last_sample_ms = 0U;

static void ImuHeading_UpdateDebug(void)
{
    g_imu_ready = s_imu.ready;
    g_imu_who_am_i = s_imu.who_am_i;
    g_imu_i2c_address = (uint8_t) (s_imu.address >> 1U);
    g_imu_init_step = s_imu.init_step;
    g_imu_read_count = s_imu.read_count;
    g_imu_error_count = s_imu.error_count;
    g_imu_heading_deg = s_imu.heading_deg;
    g_imu_gyro_z_dps = s_imu.gyro_z_dps;
    g_imu_last_sample_ms = s_imu.last_sample_ms;
}

static int16_t ImuHeading_ToInt16(uint8_t low, uint8_t high)
{
    return (int16_t) ((uint16_t) low | ((uint16_t) high << 8));
}

static uint8_t ImuHeading_ReadReg(uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0U)) {
        s_imu.error_count++;
        ImuHeading_UpdateDebug();
        return 0U;
    }

    if (HAL_I2C_Mem_Read(&hi2c2, s_imu.address, reg, I2C_MEMADD_SIZE_8BIT,
                         data, length, LSM6DSV16X_I2C_TIMEOUT_MS) != HAL_OK) {
        s_imu.error_count++;
        ImuHeading_UpdateDebug();
        return 0U;
    }

    return 1U;
}

static uint8_t ImuHeading_WriteReg(uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(&hi2c2, s_imu.address, reg, I2C_MEMADD_SIZE_8BIT,
                          &value, 1U, LSM6DSV16X_I2C_TIMEOUT_MS) != HAL_OK) {
        s_imu.error_count++;
        ImuHeading_UpdateDebug();
        return 0U;
    }

    return 1U;
}

static uint8_t ImuHeading_Probe(uint8_t address, uint8_t *who_am_i)
{
    uint8_t value = 0U;

    if (HAL_I2C_Mem_Read(&hi2c2, address, LSM6DSV16X_REG_WHO_AM_I,
                         I2C_MEMADD_SIZE_8BIT, &value, 1U,
                         LSM6DSV16X_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    if (who_am_i != 0) {
        *who_am_i = value;
    }
    return (uint8_t) (value == LSM6DSV16X_WHO_AM_I_VALUE);
}

static void ImuHeading_ResetState(void)
{
    s_imu.ready = 0U;
    s_imu.address = LSM6DSV16X_I2C_ADDRESS_LOW;
    s_imu.who_am_i = 0U;
    s_imu.last_sample_ms = 0U;
    s_imu.read_count = 0U;
    s_imu.error_count = 0U;
    s_imu.init_step = 1U;
    s_imu.heading_deg = 0.0f;
    s_imu.gyro_z_dps = 0.0f;
    ImuHeading_UpdateDebug();
}

void ImuHeading_Init(void)
{
    uint8_t who_am_i = 0U;

    ImuHeading_ResetState();
    s_imu.init_step = 2U;
    ImuHeading_UpdateDebug();
    if (ImuHeading_Probe(LSM6DSV16X_I2C_ADDRESS_LOW, &who_am_i) != 0U) {
        s_imu.address = LSM6DSV16X_I2C_ADDRESS_LOW;
        s_imu.init_step = 3U;
    } else if (ImuHeading_Probe(LSM6DSV16X_I2C_ADDRESS_HIGH, &who_am_i) != 0U) {
        s_imu.address = LSM6DSV16X_I2C_ADDRESS_HIGH;
        s_imu.init_step = 4U;
    } else {
        s_imu.who_am_i = who_am_i;
        s_imu.init_step = 10U;
        s_imu.error_count++;
        ImuHeading_UpdateDebug();
        return;
    }

    s_imu.who_am_i = who_am_i;
    s_imu.init_step = 5U;
    ImuHeading_UpdateDebug();
    if (ImuHeading_WriteReg(LSM6DSV16X_REG_CTRL3,
                            LSM6DSV16X_CTRL3_BDU_IF_INC) == 0U) {
        s_imu.init_step = 11U;
        ImuHeading_UpdateDebug();
        return;
    }
    if (ImuHeading_WriteReg(LSM6DSV16X_REG_CTRL1,
                            LSM6DSV16X_CTRL1_XL_60HZ_2G) == 0U) {
        s_imu.init_step = 12U;
        ImuHeading_UpdateDebug();
        return;
    }
    if (ImuHeading_WriteReg(LSM6DSV16X_REG_CTRL2,
                            LSM6DSV16X_CTRL2_G_60HZ_250DPS) == 0U) {
        s_imu.init_step = 13U;
        ImuHeading_UpdateDebug();
        return;
    }

    s_imu.ready = 1U;
    s_imu.init_step = 100U;
    s_imu.last_sample_ms = Board_TimeMs();
    ImuHeading_UpdateDebug();
}

void ImuHeading_Poll(void)
{
    uint8_t status;
    uint8_t raw[14];
    uint32_t now_ms;
    uint32_t delta_ms;
    int16_t gyro_z_raw;

    if (s_imu.ready == 0U) {
        return;
    }
    if (ImuHeading_ReadReg(LSM6DSV16X_REG_STATUS, &status, 1U) == 0U) {
        return;
    }
    if ((status & LSM6DSV16X_STATUS_GDA) == 0U) {
        return;
    }
    if (ImuHeading_ReadReg(LSM6DSV16X_REG_OUT_TEMP_L, raw,
                           (uint16_t) sizeof(raw)) == 0U) {
        return;
    }

    now_ms = Board_TimeMs();
    delta_ms = now_ms - s_imu.last_sample_ms;
    gyro_z_raw = ImuHeading_ToInt16(raw[6], raw[7]);
    s_imu.gyro_z_dps = (float) gyro_z_raw * LSM6DSV16X_GYRO_250_SENS_DPS;
    s_imu.heading_deg += s_imu.gyro_z_dps * ((float) delta_ms / 1000.0f);
    while (s_imu.heading_deg >= 360.0f) {
        s_imu.heading_deg -= 360.0f;
    }
    while (s_imu.heading_deg < 0.0f) {
        s_imu.heading_deg += 360.0f;
    }
    s_imu.last_sample_ms = now_ms;
    s_imu.read_count++;
    ImuHeading_UpdateDebug();
}

uint8_t ImuHeading_IsFresh(uint32_t now_ms)
{
    if (s_imu.ready == 0U) {
        return 0U;
    }
    return (uint8_t) ((uint32_t) (now_ms - s_imu.last_sample_ms) <=
                      IMU_HEADING_FRESH_MS);
}

float ImuHeading_GetDegrees(void)
{
    return s_imu.heading_deg;
}

uint8_t ImuHeading_IsReady(void)
{
    return s_imu.ready;
}

uint8_t ImuHeading_GetWhoAmI(void)
{
    return s_imu.who_am_i;
}

uint8_t ImuHeading_GetI2cAddress(void)
{
    return (uint8_t) (s_imu.address >> 1U);
}

float ImuHeading_GetGyroZDps(void)
{
    return s_imu.gyro_z_dps;
}

uint32_t ImuHeading_GetReadCount(void)
{
    return s_imu.read_count;
}

uint32_t ImuHeading_GetErrorCount(void)
{
    return s_imu.error_count;
}
