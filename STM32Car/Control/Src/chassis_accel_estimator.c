#include "chassis_accel_estimator.h"

#include <string.h>

#define CHASSIS_ACCEL_SPEED_FILTER_TAU_S   0.060f /* 编码器平均速度低通滤波时间常数。 */
#define CHASSIS_ACCEL_FILTER_TAU_S         0.120f /* 纵向加速度低通滤波时间常数。 */
#define CHASSIS_ACCEL_MAX_ABS_CM_S2      250.0f   /* 编码器差分加速度的物理保护限幅。 */
#define CHASSIS_ACCEL_MAX_SAMPLE_GAP_MS   100U    /* 超过该间隔后重新建立速度基线。 */

static float ChassisAccelEstimator_Limit(float value,
                                         float minimum,
                                         float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void ChassisAccelEstimator_FillOutput(
    const ChassisAccelEstimator *estimator,
    uint8_t valid,
    float raw_speed_cm_s,
    ChassisAccelOutput *output)
{
    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    output->raw_speed_cm_s = raw_speed_cm_s;
    if (estimator == 0) {
        return;
    }

    output->valid = valid;
    output->filtered_speed_cm_s = estimator->filtered_speed_cm_s;
    output->raw_accel_cm_s2 = estimator->raw_accel_cm_s2;
    output->accel_cm_s2 = estimator->filtered_accel_cm_s2;
}

void ChassisAccelEstimator_Init(ChassisAccelEstimator *estimator)
{
    if (estimator != 0) {
        memset(estimator, 0, sizeof(*estimator));
    }
}

void ChassisAccelEstimator_Update(ChassisAccelEstimator *estimator,
                                  uint32_t now_ms,
                                  float left_speed_cm_s,
                                  float right_speed_cm_s,
                                  ChassisAccelOutput *output)
{
    uint32_t elapsed_ms;
    float raw_speed_cm_s;
    float previous_filtered_speed;
    float dt_s;
    float alpha;

    raw_speed_cm_s = 0.5f * (left_speed_cm_s + right_speed_cm_s);
    if (estimator == 0) {
        ChassisAccelEstimator_FillOutput(0, 0U, raw_speed_cm_s, output);
        return;
    }

    if (estimator->sample_ready == 0U) {
        estimator->sample_ready = 1U;
        estimator->last_sample_ms = now_ms;
        estimator->filtered_speed_cm_s = raw_speed_cm_s;
        estimator->raw_accel_cm_s2 = 0.0f;
        estimator->filtered_accel_cm_s2 = 0.0f;
        ChassisAccelEstimator_FillOutput(estimator, 0U,
                                         raw_speed_cm_s, output);
        return;
    }

    elapsed_ms = (uint32_t) (now_ms - estimator->last_sample_ms);
    estimator->last_sample_ms = now_ms;
    if ((elapsed_ms == 0U) ||
        (elapsed_ms > CHASSIS_ACCEL_MAX_SAMPLE_GAP_MS)) {
        if (elapsed_ms > CHASSIS_ACCEL_MAX_SAMPLE_GAP_MS) {
            estimator->filtered_speed_cm_s = raw_speed_cm_s;
            estimator->raw_accel_cm_s2 = 0.0f;
            estimator->filtered_accel_cm_s2 = 0.0f;
        }
        ChassisAccelEstimator_FillOutput(estimator, 0U,
                                         raw_speed_cm_s, output);
        return;
    }

    dt_s = (float) elapsed_ms * 0.001f;
    previous_filtered_speed = estimator->filtered_speed_cm_s;
    alpha = dt_s / (CHASSIS_ACCEL_SPEED_FILTER_TAU_S + dt_s);
    estimator->filtered_speed_cm_s += alpha *
        (raw_speed_cm_s - estimator->filtered_speed_cm_s);

    estimator->raw_accel_cm_s2 = ChassisAccelEstimator_Limit(
        (estimator->filtered_speed_cm_s - previous_filtered_speed) / dt_s,
        -CHASSIS_ACCEL_MAX_ABS_CM_S2,
        CHASSIS_ACCEL_MAX_ABS_CM_S2);
    alpha = dt_s / (CHASSIS_ACCEL_FILTER_TAU_S + dt_s);
    estimator->filtered_accel_cm_s2 += alpha *
        (estimator->raw_accel_cm_s2 - estimator->filtered_accel_cm_s2);

    ChassisAccelEstimator_FillOutput(estimator, 1U,
                                     raw_speed_cm_s, output);
}
