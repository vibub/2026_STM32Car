#ifndef CHASSIS_ACCEL_ESTIMATOR_H
#define CHASSIS_ACCEL_ESTIMATOR_H

#include <stdint.h>

typedef struct {
    uint8_t sample_ready;
    uint32_t last_sample_ms;
    float filtered_speed_cm_s;
    float raw_accel_cm_s2;
    float filtered_accel_cm_s2;
} ChassisAccelEstimator;

typedef struct {
    uint8_t valid;
    float raw_speed_cm_s;
    float filtered_speed_cm_s;
    float raw_accel_cm_s2;
    float accel_cm_s2;
} ChassisAccelOutput;

typedef struct {
    uint8_t sample_ready;
    uint8_t accel_bias_ready;
    uint8_t imu_sign_ready;
    uint8_t turn_active;
    uint16_t stationary_sample_count;
    uint16_t sign_sample_count;
    uint32_t last_sample_ms;
    int8_t imu_lateral_sign;
    float accel_x_bias_g;
    float sign_correlation;
    float filtered_yaw_rate_dps;
    float filtered_imu_accel_cm_s2;
    float filtered_lateral_accel_cm_s2;
} ChassisLateralEstimator;

typedef struct {
    uint8_t valid;
    uint8_t accel_bias_ready;
    uint8_t imu_accel_valid;
    uint8_t imu_sign_ready;
    uint8_t turn_active;
    int8_t imu_lateral_sign;
    float accel_x_bias_g;
    float yaw_rate_dps;
    float imu_accel_cm_s2;
    float kinematic_accel_cm_s2;
    float lateral_accel_cm_s2;
} ChassisLateralOutput;

void ChassisAccelEstimator_Init(ChassisAccelEstimator *estimator);
void ChassisAccelEstimator_Update(ChassisAccelEstimator *estimator,
                                  uint32_t now_ms,
                                  float left_speed_cm_s,
                                  float right_speed_cm_s,
                                  ChassisAccelOutput *output);
void ChassisLateralEstimator_Init(ChassisLateralEstimator *estimator);
void ChassisLateralEstimator_Update(ChassisLateralEstimator *estimator,
                                    uint32_t now_ms,
                                    float chassis_speed_cm_s,
                                    float accel_x_g,
                                    float gyro_z_dps,
                                    uint8_t imu_valid,
                                    ChassisLateralOutput *output);

#endif /* CHASSIS_ACCEL_ESTIMATOR_H */
