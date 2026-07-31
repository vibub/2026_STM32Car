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
#define TJC_COMMAND_PIPE_CALIBRATE 0x04U /* TJC 水管 IMU 和连杆模型九点标定命令编号。 */
#define TJC_COMMAND_BALL_SHUTTLE   0x05U /* TJC 第三题 O 到正负 5 cm 往返命令编号。 */
#define TJC_COMMAND_QUERY          0x0EU /* TJC 当前底盘模式查询命令编号。 */
#define TJC_RESPONSE_HEADER        0xA5U /* STM32 返回帧的帧头字节。 */

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
    TJC_STATUS_SHUTTLE_RUNNING,
    TJC_STATUS_SHUTTLE_TURNING,
    TJC_STATUS_SHUTTLE_SUCCESS,
    TJC_STATUS_SHUTTLE_FAILED
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
void Protocol_Tjc_SendCalibrationPoint(uint8_t point_index,
                                       uint8_t point_count);
void Protocol_Tjc_SendCalibrationSuccess(int16_t scale_x100,
                                         int16_t offset_x100);
void Protocol_Tjc_SendCalibrationRangeFailure(int16_t scale_x100,
                                              int16_t offset_x100,
                                              int16_t return_error_x100);

#endif /* PROTOCOL_TJC_H */
