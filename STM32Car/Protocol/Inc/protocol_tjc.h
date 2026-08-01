#ifndef PROTOCOL_TJC_H
#define PROTOCOL_TJC_H

#include <stdint.h>

#define TJC_REQUEST_HEADER_0       0x5AU /* TJC 请求帧第一个帧头字节。 */
#define TJC_REQUEST_HEADER_1       0xA5U /* TJC 请求帧第二个帧头字节。 */
#define TJC_REQUEST_FRAME_SIZE     4U    /* TJC 请求帧固定字节数。 */
#define TJC_COMMAND_STOP           0x00U /* TJC 全局停车命令编号。 */
#define TJC_COMMAND_BASIC_LINE     0x01U /* TJC 基础八路红外巡线命令编号。 */
#define TJC_COMMAND_BALL_ENABLE    0x02U /* TJC 中心钢球平衡启用命令编号。 */
#define TJC_COMMAND_BALL_DISABLE   0x03U /* TJC 钢球平衡禁用命令编号。 */
#define TJC_COMMAND_PIPE_CALIBRATE 0x04U /* TJC 水管IMU单点水平标定命令编号。 */
#define TJC_COMMAND_BALL_SHUTTLE   0x05U /* TJC 第三题 O 到正负 5 cm 往返命令编号。 */
#define TJC_COMMAND_AB             0x06U /* TJC 第四项 AB 中心稳定通过命令。 */
#define TJC_COMMAND_ABCD           0x07U /* TJC 第五项 ABCD 整圈中心稳定通过命令。 */
#define TJC_COMMAND_QUERY          0x0EU /* TJC 当前底盘模式查询命令编号。 */
#define TJC_RESPONSE_HEADER        0xA5U /* STM32 返回帧的帧头字节。 */

#define TJC_COMMAND_SHUTTLE_SAVE   0x10U /* Save three open-loop phase durations. */
#define TJC_COMMAND_SHUTTLE_QUERY  0x11U /* Query the three saved phase durations. */

typedef enum {
    TJC_RESULT_STATE = 0x10U,
    TJC_RESULT_ACCEPTED_BRAKING = 0x11U,
    TJC_RESULT_SWITCH_OK = 0x12U,
    TJC_RESULT_STOPPED = 0x13U,
    TJC_RESULT_ALREADY_ACTIVE = 0x14U,
    TJC_RESULT_INVALID_COMMAND = 0xE0U,
    TJC_RESULT_ENTER_FAILED = 0xE1U
} Tjc_Result;

typedef enum {
    TJC_STATUS_CALIBRATION_READY = 0,
    TJC_STATUS_CALIBRATION_LEVEL,
    TJC_STATUS_CALIBRATION_NEGATIVE,
    TJC_STATUS_CALIBRATION_CENTER,
    TJC_STATUS_CALIBRATION_POSITIVE,
    TJC_STATUS_CALIBRATION_RETURN,
    TJC_STATUS_CALIBRATION_SUCCESS,
    TJC_STATUS_CALIBRATION_FAILED_IMU,
    TJC_STATUS_CALIBRATION_FAILED_MOTION,
    TJC_STATUS_CALIBRATION_FAILED_RANGE,
    TJC_STATUS_CALIBRATION_ABORTED,
    TJC_STATUS_BALANCE_CHECKING,
    TJC_STATUS_BALANCE_ENABLED,
    TJC_STATUS_BALANCE_IMU_FAULT,
    TJC_STATUS_BALANCE_RECOVERED,
    TJC_STATUS_BALANCE_DISABLED,
    TJC_STATUS_SHUTTLE_PREPARING,
    TJC_STATUS_SHUTTLE_READY,
    TJC_STATUS_SHUTTLE_NOT_READY,
    TJC_STATUS_SHUTTLE_RUNNING,
    TJC_STATUS_SHUTTLE_TURNING,
    TJC_STATUS_SHUTTLE_SUCCESS,
    TJC_STATUS_SHUTTLE_FAILED,
    TJC_STATUS_AB_PREPARING,
    TJC_STATUS_AB_RUNNING,
    TJC_STATUS_AB_PASSED,
    TJC_STATUS_AB_FAILED,
    TJC_STATUS_ABCD_PREPARING,
    TJC_STATUS_ABCD_RUNNING,
    TJC_STATUS_ABCD_PASSED,
    TJC_STATUS_ABCD_FAILED,
    TJC_STATUS_RUN_FAILED_IMU,
    TJC_STATUS_RUN_FAILED_MAPPING,
    TJC_STATUS_RUN_FAILED_READY,
    TJC_STATUS_RUN_FAILED_MOTION,
    TJC_STATUS_RUN_FAILED_BALANCE,
    TJC_STATUS_RUN_FAILED_VISION,
    TJC_STATUS_RUN_FAILED_SENSOR,
    TJC_STATUS_RUN_FAILED_LINE,
    TJC_STATUS_RUN_FAILED_MOTOR,
    TJC_STATUS_RUN_COMMAND_LOCKED,
    TJC_STATUS_RUN_REJECT_STOP,
    TJC_STATUS_RUN_REJECT_CALIBRATING,
    TJC_STATUS_RUN_REJECT_CALIBRATION,
    TJC_STATUS_RUN_REJECT_BUSY,
    TJC_STATUS_RUN_REJECT_BALANCE
} Tjc_StatusText;

extern volatile uint32_t g_tjc_rx_byte_count;
extern volatile uint32_t g_tjc_valid_command_count;
extern volatile uint32_t g_tjc_invalid_command_count;
extern volatile uint32_t g_tjc_invalid_frame_count;
extern volatile uint32_t g_tjc_response_count;
extern volatile uint32_t g_tjc_status_text_count;
extern volatile uint32_t g_tjc_rx_overflow_count;
extern volatile uint32_t g_tjc_rx_error_count;
extern volatile uint8_t g_tjc_last_command;
extern volatile uint8_t g_tjc_last_result;

void Protocol_Tjc_Init(void);
void Protocol_Tjc_Task(void);
void Protocol_Tjc_SendResult(Tjc_Result result,
                             uint8_t current_mode,
                             uint8_t request);
void Protocol_Tjc_SendStatusText(Tjc_StatusText status);
void Protocol_Tjc_SendShuttlePhase(uint8_t phase,
                                   float target_pipe_pitch_deg);
void Protocol_Tjc_SendDynamicRunResult(uint8_t run_mode,
                                       uint8_t run_result,
                                       uint32_t elapsed_ms,
                                       float maximum_abs_error_mm);
void Protocol_Tjc_SendCalibrationPoint(uint8_t point_index,
                                       uint8_t point_count);
void Protocol_Tjc_SendCalibrationSuccess(int16_t scale_x100,
                                         int16_t offset_x100);
void Protocol_Tjc_SendCalibrationRangeFailure(int16_t scale_x100,
                                              int16_t offset_x100,
                                              int16_t return_error_x100);
void Protocol_Tjc_SetBallLimitWarning(uint8_t active);

#endif /* PROTOCOL_TJC_H */
