#include "linkage_kinematics.h"

#include <math.h>

#define LINKAGE_DEG_TO_RAD                     0.01745329252f /* 角度转换为弧度的比例。 */
#define LINKAGE_RAD_TO_DEG                    57.29577951f    /* 弧度转换为角度的比例。 */
#define LINKAGE_POINT_B_Y_CM                   9.0f           /* 固定点 B 相对 A 的竖直高度。 */
#define LINKAGE_PIPE_BD_CM                    25.0f           /* 水管转轴 B 到端点 D 的中心距。 */
#define LINKAGE_BRACKET_CD_CM                  2.5f           /* 水管末端 D 到连杆点 C 的中心距。 */
#define LINKAGE_SERVO_G_X_CM                  19.0f           /* 舵机轴 G 相对 A 的水平坐标。 */
#define LINKAGE_SERVO_G_Y_CM                   3.0f           /* 舵机轴 G 相对 A 的竖直坐标。 */
#define LINKAGE_HORN_GF_CM                     1.9f           /* 舵机摇臂轴心到连接孔的中心距。 */
#define LINKAGE_CONNECTING_CF_CM               7.0716718f     /* 单连杆 C-F 的名义中心距。 */
#define LINKAGE_HORN_INSTALL_CENTER_DEG      107.4401f        /* 内部 135°对应的摇臂绝对安装角。 */
#define LINKAGE_SERVO_INTERNAL_CENTER_DEG    135.0f           /* 270°舵机的内部位置中点。 */
#define LINKAGE_SERVO_DIRECTION                1.0f           /* 内部角增大时摇臂绝对角也增大。 */
#define LINKAGE_REACH_TOLERANCE_CM             0.0001f        /* 可达条件允许的浮点计算误差。 */
#define LINKAGE_ANGLE_TOLERANCE_DEG            0.01f          /* 反解工作范围允许的浮点角度误差。 */

static float LinkageKinematics_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float LinkageKinematics_Clamp(float value,
                                     float minimum,
                                     float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t LinkageKinematics_ValueIsValid(float value)
{
    /* NaN 是唯一不等于自身的浮点值；范围判断同时拒绝无穷大。 */
    return (uint8_t) ((value == value) &&
                      (value >= -1000000.0f) &&
                      (value <= 1000000.0f));
}

uint8_t LinkageKinematics_PipePitchToHornAngle(float pipe_pitch_deg,
                                                float *horn_angle_deg)
{
    float theta;
    float point_c_x;
    float point_c_y;
    float delta_x;
    float delta_y;
    float rho;
    float minimum_link;
    float maximum_link;
    float acos_argument;
    float horn_angle_rad;

    if ((horn_angle_deg == 0) ||
        (LinkageKinematics_ValueIsValid(pipe_pitch_deg) == 0U) ||
        (pipe_pitch_deg < LINKAGE_KINEMATICS_MIN_PIPE_PITCH_DEG) ||
        (pipe_pitch_deg > LINKAGE_KINEMATICS_MAX_PIPE_PITCH_DEG)) {
        return 0U;
    }

    theta = pipe_pitch_deg * LINKAGE_DEG_TO_RAD;
    point_c_x = LINKAGE_PIPE_BD_CM * cosf(theta) +
                LINKAGE_BRACKET_CD_CM * sinf(theta);
    point_c_y = LINKAGE_POINT_B_Y_CM +
                LINKAGE_PIPE_BD_CM * sinf(theta) -
                LINKAGE_BRACKET_CD_CM * cosf(theta);
    delta_x = point_c_x - LINKAGE_SERVO_G_X_CM;
    delta_y = point_c_y - LINKAGE_SERVO_G_Y_CM;
    rho = sqrtf(delta_x * delta_x + delta_y * delta_y);
    if (rho <= 0.0f) {
        return 0U;
    }

    minimum_link = LinkageKinematics_Abs(rho - LINKAGE_HORN_GF_CM);
    maximum_link = rho + LINKAGE_HORN_GF_CM;
    if ((LINKAGE_CONNECTING_CF_CM <
         minimum_link - LINKAGE_REACH_TOLERANCE_CM) ||
        (LINKAGE_CONNECTING_CF_CM >
         maximum_link + LINKAGE_REACH_TOLERANCE_CM)) {
        return 0U;
    }

    acos_argument =
        (rho * rho + LINKAGE_HORN_GF_CM * LINKAGE_HORN_GF_CM -
         LINKAGE_CONNECTING_CF_CM * LINKAGE_CONNECTING_CF_CM) /
        (2.0f * rho * LINKAGE_HORN_GF_CM);
    acos_argument = LinkageKinematics_Clamp(acos_argument, -1.0f, 1.0f);
    horn_angle_rad = atan2f(delta_y, delta_x) + acosf(acos_argument);
    *horn_angle_deg = horn_angle_rad * LINKAGE_RAD_TO_DEG;
    return LinkageKinematics_ValueIsValid(*horn_angle_deg);
}

uint8_t LinkageKinematics_PipePitchToServoAngle(float pipe_pitch_deg,
                                                 float *servo_angle_deg)
{
    float horn_angle_deg;

    if ((servo_angle_deg == 0) ||
        (LinkageKinematics_PipePitchToHornAngle(pipe_pitch_deg,
                                                &horn_angle_deg) == 0U)) {
        return 0U;
    }

    *servo_angle_deg = LINKAGE_SERVO_INTERNAL_CENTER_DEG +
        LINKAGE_SERVO_DIRECTION *
        (horn_angle_deg - LINKAGE_HORN_INSTALL_CENTER_DEG);
    return LinkageKinematics_ValueIsValid(*servo_angle_deg);
}

uint8_t LinkageKinematics_ServoAngleToPipePitch(float servo_angle_deg,
                                                 float *pipe_pitch_deg)
{
    float horn_angle_deg;
    float horn_angle_rad;
    float point_f_x;
    float point_f_y;
    float vector_x;
    float vector_y;
    float vector_length;
    float equivalent_radius;
    float equivalent_offset;
    float beta;
    float acos_argument;
    float theta_deg;

    if ((pipe_pitch_deg == 0) ||
        (LinkageKinematics_ValueIsValid(servo_angle_deg) == 0U)) {
        return 0U;
    }

    horn_angle_deg = LINKAGE_HORN_INSTALL_CENTER_DEG +
        (servo_angle_deg - LINKAGE_SERVO_INTERNAL_CENTER_DEG) /
        LINKAGE_SERVO_DIRECTION;
    horn_angle_rad = horn_angle_deg * LINKAGE_DEG_TO_RAD;
    point_f_x = LINKAGE_SERVO_G_X_CM +
                LINKAGE_HORN_GF_CM * cosf(horn_angle_rad);
    point_f_y = LINKAGE_SERVO_G_Y_CM +
                LINKAGE_HORN_GF_CM * sinf(horn_angle_rad);
    vector_x = point_f_x;
    vector_y = point_f_y - LINKAGE_POINT_B_Y_CM;
    vector_length = sqrtf(vector_x * vector_x + vector_y * vector_y);
    equivalent_radius = sqrtf(LINKAGE_PIPE_BD_CM * LINKAGE_PIPE_BD_CM +
                              LINKAGE_BRACKET_CD_CM *
                              LINKAGE_BRACKET_CD_CM);
    if ((vector_length <= 0.0f) || (equivalent_radius <= 0.0f)) {
        return 0U;
    }

    equivalent_offset = atan2f(-LINKAGE_BRACKET_CD_CM,
                               LINKAGE_PIPE_BD_CM);
    beta = atan2f(vector_y, vector_x);
    acos_argument =
        (equivalent_radius * equivalent_radius +
         vector_length * vector_length -
         LINKAGE_CONNECTING_CF_CM * LINKAGE_CONNECTING_CF_CM) /
        (2.0f * equivalent_radius * vector_length);
    if ((acos_argument < -1.0f - LINKAGE_REACH_TOLERANCE_CM) ||
        (acos_argument > 1.0f + LINKAGE_REACH_TOLERANCE_CM)) {
        return 0U;
    }
    acos_argument = LinkageKinematics_Clamp(acos_argument, -1.0f, 1.0f);
    theta_deg = (beta - equivalent_offset + acosf(acos_argument)) *
                LINKAGE_RAD_TO_DEG;
    if ((LinkageKinematics_ValueIsValid(theta_deg) == 0U) ||
        (theta_deg < LINKAGE_KINEMATICS_MIN_PIPE_PITCH_DEG -
                     LINKAGE_ANGLE_TOLERANCE_DEG) ||
        (theta_deg > LINKAGE_KINEMATICS_MAX_PIPE_PITCH_DEG +
                     LINKAGE_ANGLE_TOLERANCE_DEG)) {
        return 0U;
    }

    *pipe_pitch_deg = LinkageKinematics_Clamp(
        theta_deg,
        LINKAGE_KINEMATICS_MIN_PIPE_PITCH_DEG,
        LINKAGE_KINEMATICS_MAX_PIPE_PITCH_DEG);
    return 1U;
}

float LinkageKinematics_GetNominalServoCenterDeg(void)
{
    return LINKAGE_KINEMATICS_NOMINAL_SERVO_CENTER_DEG;
}
