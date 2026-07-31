#include "lap_stop_detector.h"

#include <string.h>

#define LAP_STOP_ARM_DISTANCE_CM       400.0f /* 累计达到该里程后允许识别返程 A 点。 */
#define LAP_STOP_MIN_ACTIVE_COUNT       4U    /* 每帧至少识别到 4 路黑线才参与横线确认。 */
#define LAP_STOP_LEAVE_CLEAR_SAMPLES    2U    /* 连续两帧低于横线门槛后确认驶离起点。 */
#define LAP_STOP_CONFIRM_SAMPLES        2U    /* 连续满足横线门槛的返程确认样本数。 */

static uint8_t LapStopDetector_CountActiveSensors(uint8_t sensor_mask)
{
    uint8_t index;
    uint8_t active_count = 0U;

    for (index = 0U; index < 8U; index++) {
        if ((sensor_mask & (uint8_t) (1U << index)) != 0U) {
            active_count++;
        }
    }
    return active_count;
}

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
    uint8_t active_count;

    if (detector == 0) {
        return 0U;
    }
    if (detector->completed != 0U) {
        return 1U;
    }

    if (distance_delta_cm > 0.0f) {
        detector->distance_cm += distance_delta_cm;
    }

    /* 只接受连续有效的新采样，I2C 失败时的陈旧掩码不能推进状态。 */
    if (sensor_sample_valid == 0U) {
        detector->leave_clear_count = 0U;
        detector->full_black_count = 0U;
        return 0U;
    }

    active_count = LapStopDetector_CountActiveSensors(sensor_mask);
    if (detector->left_start_line == 0U) {
        if (active_count < LAP_STOP_MIN_ACTIVE_COUNT) {
            if (detector->leave_clear_count <
                LAP_STOP_LEAVE_CLEAR_SAMPLES) {
                detector->leave_clear_count++;
            }
            if (detector->leave_clear_count >=
                LAP_STOP_LEAVE_CLEAR_SAMPLES) {
                detector->left_start_line = 1U;
            }
        } else {
            detector->leave_clear_count = 0U;
        }
        detector->full_black_count = 0U;
        return 0U;
    }

    /* 一圈约 614 cm，先行驶 400 cm，再允许确认返程 A 点横线。 */
    if (detector->distance_cm >= LAP_STOP_ARM_DISTANCE_CM) {
        detector->armed = 1U;
    }
    if (detector->armed == 0U) {
        detector->full_black_count = 0U;
        return 0U;
    }

    if (active_count < LAP_STOP_MIN_ACTIVE_COUNT) {
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
