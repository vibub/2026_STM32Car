#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

/* SG90 舵机允许设置的机械角度范围。 */
#define SERVO_DRIVER_MIN_ANGLE_DEG 0.0f
#define SERVO_DRIVER_MAX_ANGLE_DEG 180.0f

void ServoDriver_Init(void);
void ServoDriver_SetAngle(float angle_deg);
float ServoDriver_GetAngle(void);
void ServoDriver_StartContinuousSweep(void);
void ServoDriver_StartPowerOnClockwiseSweep(void);
void ServoDriver_Task(uint32_t now_ms);

#endif /* SERVO_DRIVER_H */
