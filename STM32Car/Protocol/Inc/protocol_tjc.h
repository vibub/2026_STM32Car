#ifndef PROTOCOL_TJC_H
#define PROTOCOL_TJC_H
#include <stdint.h>
#define TJC_REQUEST_HEADER_0   0x5AU /* TJC 请求帧第一个帧头字节。 */
#define TJC_REQUEST_HEADER_1   0xA5U /* TJC 请求帧第二个帧头字节。 */
#define TJC_REQUEST_FRAME_SIZE 4U    /* TJC 请求帧固定字节数。 */
#define TJC_COMMAND_STOP         0x00U /* TJC 全局停车命令编号。 */
#define TJC_COMMAND_BASIC_LINE   0x01U /* TJC 基础八路巡线命令编号。 */
#define TJC_COMMAND_BALL_ENABLE  0x02U /* TJC 中心钢球平衡启用命令编号。 */
#define TJC_COMMAND_BALL_DISABLE 0x03U /* TJC 钢球平衡禁用命令编号。 */
#define TJC_COMMAND_QUERY        0x0EU /* TJC 当前底盘模式查询命令编号。 */
#define TJC_RESPONSE_HEADER    0xA5U /* STM32 返回帧的帧头字节。 */
typedef enum {
    TJC_RESULT_STATE = 0x10U,
    TJC_RESULT_ACCEPTED_BRAKING = 0x11U,
    TJC_RESULT_SWITCH_OK = 0x12U,
    TJC_RESULT_STOPPED = 0x13U,
    TJC_RESULT_ALREADY_ACTIVE = 0x14U,
    TJC_RESULT_INVALID_COMMAND = 0xE0U,
    TJC_RESULT_ENTER_FAILED = 0xE1U
} Tjc_Result;
extern volatile uint32_t g_tjc_rx_byte_count;
extern volatile uint32_t g_tjc_valid_command_count;
extern volatile uint32_t g_tjc_invalid_command_count;
extern volatile uint32_t g_tjc_invalid_frame_count;
extern volatile uint32_t g_tjc_response_count;
extern volatile uint32_t g_tjc_rx_overflow_count;
extern volatile uint32_t g_tjc_rx_error_count;
extern volatile uint8_t g_tjc_last_command;
extern volatile uint8_t g_tjc_last_result;
void Protocol_Tjc_Init(void);
void Protocol_Tjc_Task(void);
void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode, uint8_t request);
#endif /* PROTOCOL_TJC_H */
