#ifndef LINKAGE_KINEMATICS_H
#define LINKAGE_KINEMATICS_H

#include <stdint.h>

#define LINKAGE_KINEMATICS_MIN_PIPE_PITCH_DEG       (-6.0f) /* 连杆模型允许的最小水管俯仰角。 */
#define LINKAGE_KINEMATICS_MAX_PIPE_PITCH_DEG         6.0f  /* 连杆模型允许的最大水管俯仰角。 */
#define LINKAGE_KINEMATICS_NOMINAL_SERVO_CENTER_DEG 126.2058f /* 实机反向安装下水管水平时的名义舵机内部角。 */

uint8_t LinkageKinematics_PipePitchToHornAngle(float pipe_pitch_deg,
                                                float *horn_angle_deg);
uint8_t LinkageKinematics_PipePitchToServoAngle(float pipe_pitch_deg,
                                                 float *servo_angle_deg);
uint8_t LinkageKinematics_ServoAngleToPipePitch(float servo_angle_deg,
                                                 float *pipe_pitch_deg);
float LinkageKinematics_GetNominalServoCenterDeg(void);

#endif /* LINKAGE_KINEMATICS_H */
