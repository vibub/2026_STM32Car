#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdint.h>

typedef struct {
    uint8_t valid;
    uint32_t timestamp_ms;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} ImuMotionSample;

/* IAR Watch 调试变量，仅用于观察 IMU 状态，不参与控制。 */
extern volatile uint8_t g_imu_ready;
extern volatile uint8_t g_imu_who_am_i;
extern volatile uint8_t g_imu_i2c_address;
extern volatile uint8_t g_imu_init_step;
extern volatile uint32_t g_imu_read_count;
extern volatile uint32_t g_imu_error_count;
extern volatile float g_imu_heading_deg;
extern volatile float g_imu_accel_x_g;
extern volatile float g_imu_accel_y_g;
extern volatile float g_imu_accel_z_g;
extern volatile float g_imu_gyro_x_dps;
extern volatile float g_imu_gyro_y_dps;
extern volatile float g_imu_gyro_z_dps;
extern volatile uint32_t g_imu_last_sample_ms;

void ImuHeading_Init(void);
void ImuHeading_Poll(void);
uint8_t ImuHeading_TakeSample(ImuMotionSample *sample);
uint8_t ImuHeading_GetLatestSample(ImuMotionSample *sample);
uint8_t ImuHeading_IsFresh(uint32_t now_ms);
float ImuHeading_GetDegrees(void);
uint8_t ImuHeading_IsReady(void);
uint8_t ImuHeading_GetWhoAmI(void);
uint8_t ImuHeading_GetI2cAddress(void);
float ImuHeading_GetGyroZDps(void);
uint32_t ImuHeading_GetReadCount(void);
uint32_t ImuHeading_GetErrorCount(void);

#endif /* IMU_HEADING_H */
