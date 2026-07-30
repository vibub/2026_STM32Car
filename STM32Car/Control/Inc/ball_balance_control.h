#ifndef BALL_BALANCE_CONTROL_H
#define BALL_BALANCE_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t valid;
    int32_t position_d10_mm;
    uint8_t quality;
    uint16_t sequence;
} BallBalanceMeasurement;

typedef struct {
    uint8_t enabled;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t filter_ready;
    uint8_t sequence_ready;
    uint8_t task_time_ready;
    uint16_t last_sequence;
    uint32_t enabled_since_ms;
    uint32_t last_valid_ms;
    uint32_t last_task_ms;
    float target_mm;
    float filtered_position_mm;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float desired_angle_deg;
    float command_angle_deg;
} BallBalanceControl;

typedef struct {
    uint8_t enabled;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t measurement_accepted;
    float target_mm;
    float filtered_position_mm;
    float velocity_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float desired_angle_deg;
    float command_angle_deg;
} BallBalanceOutput;

void BallBalanceControl_Init(BallBalanceControl *control);
void BallBalanceControl_SetEnabled(BallBalanceControl *control,
                                   uint8_t enabled,
                                   uint32_t now_ms);
uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control);
void BallBalanceControl_SetTargetMm(BallBalanceControl *control,
                                    float target_mm);
float BallBalanceControl_GetNeutralAngleDeg(void);
void BallBalanceControl_Update(BallBalanceControl *control,
                               uint32_t now_ms,
                               const BallBalanceMeasurement *measurement,
                               BallBalanceOutput *output);

#endif /* BALL_BALANCE_CONTROL_H */
