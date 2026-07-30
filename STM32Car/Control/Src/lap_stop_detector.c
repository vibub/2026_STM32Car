#include "lap_stop_detector.h"

#include <string.h>

#define LAP_STOP_ARM_DISTANCE_CM     400.0f
#define LAP_STOP_FULL_BLACK_MASK     0xFFU
#define LAP_STOP_CONFIRM_SAMPLES     2U

void LapStopDetector_Init(LapStopDetector *detector)
{
    if (detector != 0) {
        memset(detector, 0, sizeof(*detector));
    }
}

uint8_t LapStopDetector_Update(LapStopDetector *detector,
                               float distance_delta_cm,
                               uint8_t sensor_sample_valid,
                               uint8_t sensor_mask)
{
    if (detector == 0) {
        return 0U;
    }
    if (detector->completed != 0U) {
        return 1U;
    }

    if (distance_delta_cm > 0.0f) {
        detector->distance_cm += distance_delta_cm;
    }

    /* 一圈约 614 cm，先行驶 400 cm，避免把起点附近的全黑误判为终点。 */
    if (detector->distance_cm >= LAP_STOP_ARM_DISTANCE_CM) {
        detector->armed = 1U;
    }
    if (detector->armed == 0U) {
        detector->full_black_count = 0U;
        return 0U;
    }

    /* 只接受连续有效的新采样，I2C 失败时的陈旧掩码不能用于停车。 */
    if ((sensor_sample_valid == 0U) ||
        (sensor_mask != LAP_STOP_FULL_BLACK_MASK)) {
        detector->full_black_count = 0U;
        return 0U;
    }

    if (detector->full_black_count < LAP_STOP_CONFIRM_SAMPLES) {
        detector->full_black_count++;
    }
    if (detector->full_black_count >= LAP_STOP_CONFIRM_SAMPLES) {
        detector->completed = 1U;
    }

    return detector->completed;
}
