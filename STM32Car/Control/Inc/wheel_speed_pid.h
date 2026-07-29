#ifndef WHEEL_SPEED_PID_H
#define WHEEL_SPEED_PID_H

#include <stdint.h>

typedef struct {
    float target_cm_s;
    float integral;
    float last_error;
    float command;
    float p_term;
    float i_term;
    float d_term;
    uint8_t startup_kick_windows;
    uint16_t zero_feedback_windows;
    uint8_t fault;
} WheelSpeedPid;

void WheelSpeedPid_Init(WheelSpeedPid *pid);
void WheelSpeedPid_Reset(WheelSpeedPid *pid);
void WheelSpeedPid_SetTarget(WheelSpeedPid *pid, float target_cm_s);
float WheelSpeedPid_Step(WheelSpeedPid *pid, float measured_cm_s);
uint8_t WheelSpeedPid_HasFault(const WheelSpeedPid *pid);

#endif /* WHEEL_SPEED_PID_H */
