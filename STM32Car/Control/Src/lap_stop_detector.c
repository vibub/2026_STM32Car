#include "lap_stop_detector.h"

#include <string.h>

#define LAP_STOP_FALLBACK_ARM_DISTANCE_CM 400.0f /* 未配置名义里程时的兼容门槛。 */
#define LAP_STOP_EARLY_WINDOW_CM        40.0f /* 距名义整圈不足该值后才允许识别 A 点。 */
#define LAP_STOP_MAX_OVERRUN_CM         80.0f /* 超过名义里程过多时按编码器兜底结束。 */
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

void LapStopDetector_SetExpectedDistance(LapStopDetector *detector,
                                         float expected_distance_cm)
{
    if ((detector != 0) && (expected_distance_cm > 0.0f)) {
        detector->expected_distance_cm = expected_distance_cm;
    }
}

uint8_t LapStopDetector_Update(LapStopDetector *detector,
                               float distance_delta_cm,
                               uint8_t sensor_sample_valid,
                               uint8_t sensor_mask)
{
    uint8_t active_count;
    float arm_distance_cm;

    if (detector == 0) {
        return 0U;
    }
    if (detector->completed != 0U) {
        return 1U;
    }

    if (distance_delta_cm > 0.0f) {
        detector->distance_cm += distance_delta_cm;
    }

    /* If the A-line sensor is stuck or missed completely, never drive an
     * additional lap.  This fallback is intentionally well beyond the
     * normal recognition window and relies only on encoder distance. */
    if ((detector->left_start_line != 0U) &&
        (detector->expected_distance_cm > 0.0f) &&
        (detector->distance_cm >=
         detector->expected_distance_cm + LAP_STOP_MAX_OVERRUN_CM)) {
        detector->armed = 1U;
        detector->full_black_count = 0U;
        detector->completed = 1U;
        return 1U;
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

    arm_distance_cm = LAP_STOP_FALLBACK_ARM_DISTANCE_CM;
    if (detector->expected_distance_cm > 0.0f) {
        arm_distance_cm = detector->expected_distance_cm;
        if (arm_distance_cm > LAP_STOP_EARLY_WINDOW_CM) {
            arm_distance_cm -= LAP_STOP_EARLY_WINDOW_CM;
        }
    }
    /* Before the encoder reaches the nominal-lap window, ignore every
     * apparent wide line.  A stuck side sensor must not end the lap early. */
    if (detector->distance_cm >= arm_distance_cm) {
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
