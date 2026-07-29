#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

void MotorDriver_Init(void);
void MotorDriver_Set(float left_ratio, float right_ratio);
void MotorDriver_Stop(void);
float MotorDriver_GetLeftRatio(void);
float MotorDriver_GetRightRatio(void);

#endif /* MOTOR_DRIVER_H */
