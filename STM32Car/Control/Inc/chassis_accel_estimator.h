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

void ChassisAccelEstimator_Init(ChassisAccelEstimator *estimator);
void ChassisAccelEstimator_Update(ChassisAccelEstimator *estimator,
                                  uint32_t now_ms,
                                  float left_speed_cm_s,
                                  float right_speed_cm_s,
                                  ChassisAccelOutput *output);

#endif /* CHASSIS_ACCEL_ESTIMATOR_H */
