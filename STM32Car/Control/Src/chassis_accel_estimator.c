#include "chassis_accel_estimator.h"

#include <string.h>

#define CHASSIS_ACCEL_SPEED_FILTER_TAU_S   0.060f /* 编码器平均速度低通滤波时间常数。 */
#define CHASSIS_ACCEL_FILTER_TAU_S         0.120f /* 纵向加速度低通滤波时间常数。 */
#define CHASSIS_ACCEL_MAX_ABS_CM_S2      250.0f   /* 编码器差分加速度的物理保护限幅。 */
#define CHASSIS_ACCEL_MAX_SAMPLE_GAP_MS   100U    /* 超过该间隔后重新建立速度基线。 */

#define CHASSIS_LATERAL_GRAVITY_CM_S2              980.665f
#define CHASSIS_LATERAL_DEG_TO_RAD                    0.01745329252f
#define CHASSIS_LATERAL_MAX_SAMPLE_GAP_MS           100U
#define CHASSIS_LATERAL_MAX_ABS_ACCEL_CM_S2        100.0f
#define CHASSIS_LATERAL_STATIONARY_SPEED_CM_S         2.0f
#define CHASSIS_LATERAL_STATIONARY_YAW_DPS            3.0f
#define CHASSIS_LATERAL_BIAS_SAMPLE_COUNT             30U
#define CHASSIS_LATERAL_SIGN_SAMPLE_COUNT              8U
#define CHASSIS_LATERAL_SIGN_MIN_KIN_ACCEL_CM_S2       4.0f
#define CHASSIS_LATERAL_SIGN_MIN_IMU_ACCEL_CM_S2       2.0f
#define CHASSIS_LATERAL_YAW_FILTER_TAU_S               0.080f
#define CHASSIS_LATERAL_IMU_FILTER_TAU_S               0.120f
#define CHASSIS_LATERAL_OUTPUT_FILTER_TAU_S            0.100f
#define CHASSIS_LATERAL_IMU_BLEND                      0.25f
#define CHASSIS_LATERAL_TURN_ENTER_YAW_DPS             8.0f
#define CHASSIS_LATERAL_TURN_EXIT_YAW_DPS              5.0f
#define CHASSIS_LATERAL_TURN_MIN_SPEED_CM_S             8.0f

static float ChassisAccelEstimator_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

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

static void ChassisLateralEstimator_FillOutput(
    const ChassisLateralEstimator *estimator,
    uint8_t valid,
    float imu_accel_cm_s2,
    float kinematic_accel_cm_s2,
    ChassisLateralOutput *output)
{
    if (output == 0) {
        return;
    }
    memset(output, 0, sizeof(*output));
    if (estimator == 0) {
        return;
    }

    output->valid = valid;
    output->accel_bias_ready = estimator->accel_bias_ready;
    output->imu_accel_valid = (uint8_t) (
        (estimator->accel_bias_ready != 0U) &&
        (estimator->imu_sign_ready != 0U));
    output->imu_sign_ready = estimator->imu_sign_ready;
    output->turn_active = estimator->turn_active;
    output->imu_lateral_sign = estimator->imu_lateral_sign;
    output->accel_x_bias_g = estimator->accel_x_bias_g;
    output->yaw_rate_dps = estimator->filtered_yaw_rate_dps;
    output->imu_accel_cm_s2 = imu_accel_cm_s2;
    output->kinematic_accel_cm_s2 = kinematic_accel_cm_s2;
    output->lateral_accel_cm_s2 =
        estimator->filtered_lateral_accel_cm_s2;
}

void ChassisLateralEstimator_Init(ChassisLateralEstimator *estimator)
{
    if (estimator != 0) {
        memset(estimator, 0, sizeof(*estimator));
    }
}

void ChassisLateralEstimator_Update(ChassisLateralEstimator *estimator,
                                    uint32_t now_ms,
                                    float chassis_speed_cm_s,
                                    float accel_x_g,
                                    float gyro_z_dps,
                                    uint8_t imu_valid,
                                    ChassisLateralOutput *output)
{
    uint32_t elapsed_ms;
    float dt_s;
    float alpha;
    float raw_imu_accel_cm_s2 = 0.0f;
    float signed_imu_accel_cm_s2 = 0.0f;
    float kinematic_accel_cm_s2 = 0.0f;
    float target_lateral_accel_cm_s2;
    uint8_t stationary;

    if ((estimator == 0) || (imu_valid == 0U)) {
        ChassisLateralEstimator_FillOutput(estimator, 0U, 0.0f, 0.0f,
                                           output);
        return;
    }

    stationary = (uint8_t) (
        (ChassisAccelEstimator_Abs(chassis_speed_cm_s) <=
         CHASSIS_LATERAL_STATIONARY_SPEED_CM_S) &&
        (ChassisAccelEstimator_Abs(gyro_z_dps) <=
         CHASSIS_LATERAL_STATIONARY_YAW_DPS));
    if (stationary != 0U) {
        if (estimator->stationary_sample_count == 0U) {
            estimator->accel_x_bias_g = accel_x_g;
        } else {
            float sample_count = (float) estimator->stationary_sample_count;
            estimator->accel_x_bias_g +=
                (accel_x_g - estimator->accel_x_bias_g) /
                (sample_count + 1.0f);
        }
        if (estimator->stationary_sample_count <
            CHASSIS_LATERAL_BIAS_SAMPLE_COUNT) {
            estimator->stationary_sample_count++;
        }
        if (estimator->stationary_sample_count >=
            CHASSIS_LATERAL_BIAS_SAMPLE_COUNT) {
            estimator->accel_bias_ready = 1U;
        }
    }

    if (estimator->sample_ready == 0U) {
        estimator->sample_ready = 1U;
        estimator->last_sample_ms = now_ms;
        estimator->filtered_yaw_rate_dps = gyro_z_dps;
        ChassisLateralEstimator_FillOutput(estimator, 0U, 0.0f, 0.0f,
                                           output);
        return;
    }

    elapsed_ms = (uint32_t) (now_ms - estimator->last_sample_ms);
    estimator->last_sample_ms = now_ms;
    if ((elapsed_ms == 0U) ||
        (elapsed_ms > CHASSIS_LATERAL_MAX_SAMPLE_GAP_MS)) {
        if (elapsed_ms > CHASSIS_LATERAL_MAX_SAMPLE_GAP_MS) {
            estimator->filtered_yaw_rate_dps = gyro_z_dps;
            estimator->filtered_imu_accel_cm_s2 = 0.0f;
            estimator->filtered_lateral_accel_cm_s2 = 0.0f;
            estimator->turn_active = 0U;
        }
        ChassisLateralEstimator_FillOutput(estimator, 0U, 0.0f, 0.0f,
                                           output);
        return;
    }

    dt_s = (float) elapsed_ms * 0.001f;
    alpha = dt_s / (CHASSIS_LATERAL_YAW_FILTER_TAU_S + dt_s);
    estimator->filtered_yaw_rate_dps += alpha *
        (gyro_z_dps - estimator->filtered_yaw_rate_dps);
    kinematic_accel_cm_s2 = ChassisAccelEstimator_Limit(
        chassis_speed_cm_s * estimator->filtered_yaw_rate_dps *
        CHASSIS_LATERAL_DEG_TO_RAD,
        -CHASSIS_LATERAL_MAX_ABS_ACCEL_CM_S2,
        CHASSIS_LATERAL_MAX_ABS_ACCEL_CM_S2);

    if (estimator->accel_bias_ready != 0U) {
        raw_imu_accel_cm_s2 = ChassisAccelEstimator_Limit(
            (accel_x_g - estimator->accel_x_bias_g) *
            CHASSIS_LATERAL_GRAVITY_CM_S2,
            -CHASSIS_LATERAL_MAX_ABS_ACCEL_CM_S2,
            CHASSIS_LATERAL_MAX_ABS_ACCEL_CM_S2);
        alpha = dt_s / (CHASSIS_LATERAL_IMU_FILTER_TAU_S + dt_s);
        estimator->filtered_imu_accel_cm_s2 += alpha *
            (raw_imu_accel_cm_s2 -
             estimator->filtered_imu_accel_cm_s2);

        if ((estimator->imu_sign_ready == 0U) &&
            (ChassisAccelEstimator_Abs(kinematic_accel_cm_s2) >=
             CHASSIS_LATERAL_SIGN_MIN_KIN_ACCEL_CM_S2) &&
            (ChassisAccelEstimator_Abs(
                 estimator->filtered_imu_accel_cm_s2) >=
             CHASSIS_LATERAL_SIGN_MIN_IMU_ACCEL_CM_S2)) {
            estimator->sign_correlation +=
                estimator->filtered_imu_accel_cm_s2 *
                kinematic_accel_cm_s2;
            if (estimator->sign_sample_count <
                CHASSIS_LATERAL_SIGN_SAMPLE_COUNT) {
                estimator->sign_sample_count++;
            }
            if (estimator->sign_sample_count >=
                CHASSIS_LATERAL_SIGN_SAMPLE_COUNT) {
                estimator->imu_lateral_sign =
                    (estimator->sign_correlation >= 0.0f) ? 1 : -1;
                estimator->imu_sign_ready = 1U;
            }
        }
    }

    target_lateral_accel_cm_s2 = kinematic_accel_cm_s2;
    if ((estimator->accel_bias_ready != 0U) &&
        (estimator->imu_sign_ready != 0U)) {
        signed_imu_accel_cm_s2 =
            (float) estimator->imu_lateral_sign *
            estimator->filtered_imu_accel_cm_s2;
        target_lateral_accel_cm_s2 =
            (1.0f - CHASSIS_LATERAL_IMU_BLEND) *
                kinematic_accel_cm_s2 +
            CHASSIS_LATERAL_IMU_BLEND * signed_imu_accel_cm_s2;
    }
    alpha = dt_s / (CHASSIS_LATERAL_OUTPUT_FILTER_TAU_S + dt_s);
    estimator->filtered_lateral_accel_cm_s2 += alpha *
        (target_lateral_accel_cm_s2 -
         estimator->filtered_lateral_accel_cm_s2);

    if (estimator->turn_active == 0U) {
        estimator->turn_active = (uint8_t) (
            (ChassisAccelEstimator_Abs(chassis_speed_cm_s) >=
             CHASSIS_LATERAL_TURN_MIN_SPEED_CM_S) &&
            (ChassisAccelEstimator_Abs(
                 estimator->filtered_yaw_rate_dps) >=
             CHASSIS_LATERAL_TURN_ENTER_YAW_DPS));
    } else if ((ChassisAccelEstimator_Abs(chassis_speed_cm_s) <
                CHASSIS_LATERAL_TURN_MIN_SPEED_CM_S) ||
               (ChassisAccelEstimator_Abs(
                    estimator->filtered_yaw_rate_dps) <=
                CHASSIS_LATERAL_TURN_EXIT_YAW_DPS)) {
        estimator->turn_active = 0U;
    }

    ChassisLateralEstimator_FillOutput(
        estimator, 1U, signed_imu_accel_cm_s2,
        kinematic_accel_cm_s2, output);
}
