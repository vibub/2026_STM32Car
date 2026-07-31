#ifndef LINKAGE_KINEMATICS_H
#define LINKAGE_KINEMATICS_H

#include <stdint.h>

#define LINKAGE_KINEMATICS_MIN_PIPE_PITCH_DEG      (-10.0f) /* 与负向几何死点保留约0.83°余量。 */
#define LINKAGE_KINEMATICS_MAX_PIPE_PITCH_DEG         6.8f  /* 避开正向几何死点的最大模型俯仰角。 */
#define LINKAGE_KINEMATICS_NOMINAL_SERVO_CENTER_DEG 122.0f  /* 重新调整连接件后水管水平对应的舵机内部角。 */
#define LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT    9U     /* 实测映射使用的舵机采样点数量。 */
#define LINKAGE_KINEMATICS_LEVEL_POINT_INDEX          3U     /* 水管水平对应的实测映射点下标。 */

uint8_t LinkageKinematics_PipePitchToHornAngle(float pipe_pitch_deg,
                                                float *horn_angle_deg);
uint8_t LinkageKinematics_PipePitchToServoAngle(float pipe_pitch_deg,
                                                 float *servo_angle_deg);
uint8_t LinkageKinematics_ServoAngleToPipePitch(float servo_angle_deg,
                                                 float *pipe_pitch_deg);
uint8_t LinkageKinematics_GetCalibrationPoint(uint8_t index,
                                              float *model_pitch_deg,
                                              float *servo_angle_deg);
float LinkageKinematics_GetNominalServoCenterDeg(void);

#endif /* LINKAGE_KINEMATICS_H */
