#include "protocol_tjc.h"

#include "app_main.h"
#include "tjc_uart.h"

#include <string.h>

typedef enum {
    TJC_RX_WAIT_HEADER_0 = 0,
    TJC_RX_WAIT_HEADER_1,
    TJC_RX_WAIT_COMMAND,
    TJC_RX_WAIT_CHECKSUM
} Tjc_RxState;

#define TJC_STATUS_FRAME_BUFFER_SIZE 64U /* TJC 文本状态命令的最大发送缓存。 */

static const uint8_t s_tjc_text_prefix[] = "page2.t0.txt=\"";
static const uint8_t s_tjc_text_suffix[] = {'\"', 0xFFU, 0xFFU, 0xFFU};

/* 以下固定中文均使用 GBK 字节，避免 IAR 源文件编码影响串口屏显示。 */
static const uint8_t s_text_calibration_ready[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xD7U, 0xBCU, 0xB1U, 0xB8U
};
static const uint8_t s_text_calibration_level[] = {
    0xB2U, 0xC9U, 0xBCU, 0xAFU, 0xD6U, 0xD0U, 0xCEU, 0xBBU
};
static const uint8_t s_text_calibration_negative[] = {
    0xB2U, 0xC9U, 0xBCU, 0xAFU, 0x38U, 0x38U, 0xB6U, 0xC8U
};
static const uint8_t s_text_calibration_center[] = {
    0xB8U, 0xB4U, 0xBAU, 0xCBU, 0x39U, 0x30U, 0xB6U, 0xC8U
};
static const uint8_t s_text_calibration_positive[] = {
    0xB2U, 0xC9U, 0xBCU, 0xAFU, 0x39U, 0x32U, 0xB6U, 0xC8U
};
static const uint8_t s_text_calibration_return[] = {
    0xBCU, 0xECU, 0xB2U, 0xE9U, 0xBBU, 0xD8U, 0xD6U, 0xD0U
};
static const uint8_t s_text_calibration_success[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xB3U, 0xC9U, 0xB9U, 0xA6U,
    0x20U, 0x4BU, 0x3DU
};
static const uint8_t s_text_direction_positive[] = {
    0x20U, 0xD5U, 0xFDU, 0xCFU, 0xF2U
};
static const uint8_t s_text_direction_negative[] = {
    0x20U, 0xB7U, 0xB4U, 0xCFU, 0xF2U
};
static const uint8_t s_text_calibration_failed_imu[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xCAU, 0xA7U, 0xB0U, 0xDCU,
    0x20U, 0x49U, 0x4DU, 0x55U
};
static const uint8_t s_text_calibration_failed_motion[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xCAU, 0xA7U, 0xB0U, 0xDCU,
    0x20U, 0xCEU, 0xB4U, 0xBEU, 0xB2U, 0xD6U, 0xB9U
};
static const uint8_t s_text_calibration_failed_range[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xCAU, 0xA7U, 0xB0U, 0xDCU,
    0x20U, 0xD0U, 0xD0U, 0xB3U, 0xCCU
};
static const uint8_t s_text_calibration_aborted[] = {
    0xB1U, 0xEAU, 0xB6U, 0xA8U, 0xD2U, 0xD1U, 0xD6U, 0xD0U,
    0xB6U, 0xCFU
};
static const uint8_t s_text_balance_checking[] = {
    0xC6U, 0xBDU, 0xBAU, 0xE2U, 0xB8U, 0xB4U, 0xBAU, 0xCBU,
    0xD6U, 0xD0U
};
static const uint8_t s_text_balance_enabled[] = {
    0xC6U, 0xBDU, 0xBAU, 0xE2U, 0xD2U, 0xD1U, 0xC6U, 0xF4U,
    0xD3U, 0xC3U
};
static const uint8_t s_text_balance_imu_fault[] = {
    0x49U, 0x4DU, 0x55U, 0xD2U, 0xECU, 0xB3U, 0xA3U, 0xBBU,
    0xD8U, 0xD6U, 0xD0U
};
static const uint8_t s_text_balance_recovered[] = {
    0xC6U, 0xBDU, 0xBAU, 0xE2U, 0xD2U, 0xD1U, 0xBBU, 0xD6U,
    0xB8U, 0xB4U
};
static const uint8_t s_text_balance_disabled[] = {
    0xC6U, 0xBDU, 0xBAU, 0xE2U, 0xD2U, 0xD1U, 0xBDU, 0xFBU,
    0xD3U, 0xC3U
};

static Tjc_RxState s_rx_state;
static uint8_t s_rx_command;

volatile uint32_t g_tjc_rx_byte_count;
volatile uint32_t g_tjc_valid_command_count;
volatile uint32_t g_tjc_invalid_command_count;
volatile uint32_t g_tjc_invalid_frame_count;
volatile uint32_t g_tjc_response_count;
volatile uint32_t g_tjc_status_text_count;
volatile uint32_t g_tjc_rx_overflow_count;
volatile uint32_t g_tjc_rx_error_count;
volatile uint8_t g_tjc_last_command;
volatile uint8_t g_tjc_last_result;

static void Protocol_Tjc_ResetRxParser(void)
{
    s_rx_state = TJC_RX_WAIT_HEADER_0;
    s_rx_command = TJC_COMMAND_STOP;
}

static uint8_t Protocol_Tjc_CalculateRequestChecksum(uint8_t command)
{
    return (uint8_t) (TJC_REQUEST_HEADER_0 ^ TJC_REQUEST_HEADER_1 ^ command);
}

static uint16_t Protocol_Tjc_AppendBytes(uint8_t *buffer,
                                         uint16_t length,
                                         const uint8_t *data,
                                         uint16_t data_length)
{
    uint16_t available;

    if ((buffer == 0) || (data == 0) ||
        (length >= TJC_STATUS_FRAME_BUFFER_SIZE)) {
        return length;
    }

    available = (uint16_t) (TJC_STATUS_FRAME_BUFFER_SIZE - length);
    if (data_length > available) {
        data_length = available;
    }
    memcpy(&buffer[length], data, data_length);
    return (uint16_t) (length + data_length);
}

static uint16_t Protocol_Tjc_AppendRatio(uint8_t *buffer,
                                         uint16_t length,
                                         int16_t ratio_x100)
{
    uint16_t magnitude;
    uint16_t integer_part;
    uint16_t fraction_part;

    magnitude = (ratio_x100 < 0) ?
        (uint16_t) (-ratio_x100) : (uint16_t) ratio_x100;
    integer_part = (uint16_t) (magnitude / 100U);
    fraction_part = (uint16_t) (magnitude % 100U);

    if (integer_part >= 10U) {
        buffer[length++] = (uint8_t) ('0' + (integer_part / 10U) % 10U);
    }
    buffer[length++] = (uint8_t) ('0' + integer_part % 10U);
    buffer[length++] = (uint8_t) '.';
    buffer[length++] = (uint8_t) ('0' + fraction_part / 10U);
    buffer[length++] = (uint8_t) ('0' + fraction_part % 10U);
    return length;
}

void Protocol_Tjc_Init(void)
{
    g_tjc_rx_byte_count = 0U;
    g_tjc_valid_command_count = 0U;
    g_tjc_invalid_command_count = 0U;
    g_tjc_invalid_frame_count = 0U;
    g_tjc_response_count = 0U;
    g_tjc_status_text_count = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    g_tjc_last_command = TJC_COMMAND_STOP;
    g_tjc_last_result = TJC_RESULT_STATE;
    Protocol_Tjc_ResetRxParser();
    TjcUart_FlushRx();
}

void Protocol_Tjc_SendResult(Tjc_Result result,
                             uint8_t current_mode,
                             uint8_t request)
{
    uint8_t frame[5];

    frame[0] = TJC_RESPONSE_HEADER;
    frame[1] = (uint8_t) result;
    frame[2] = current_mode;
    frame[3] = request;
    frame[4] = (uint8_t) (frame[0] ^ frame[1] ^ frame[2] ^ frame[3]);
    TjcUart_SendData(frame, (uint16_t) sizeof(frame));
    g_tjc_last_result = (uint8_t) result;
    g_tjc_response_count++;
}

void Protocol_Tjc_SendStatusText(Tjc_StatusText status,
                                 int16_t ratio_x100)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    const uint8_t *text = 0;
    uint16_t text_length = 0U;
    uint16_t length = 0U;

    switch (status) {
        case TJC_STATUS_CALIBRATION_READY:
            text = s_text_calibration_ready;
            text_length = (uint16_t) sizeof(s_text_calibration_ready);
            break;
        case TJC_STATUS_CALIBRATION_LEVEL:
            text = s_text_calibration_level;
            text_length = (uint16_t) sizeof(s_text_calibration_level);
            break;
        case TJC_STATUS_CALIBRATION_NEGATIVE:
            text = s_text_calibration_negative;
            text_length = (uint16_t) sizeof(s_text_calibration_negative);
            break;
        case TJC_STATUS_CALIBRATION_CENTER:
            text = s_text_calibration_center;
            text_length = (uint16_t) sizeof(s_text_calibration_center);
            break;
        case TJC_STATUS_CALIBRATION_POSITIVE:
            text = s_text_calibration_positive;
            text_length = (uint16_t) sizeof(s_text_calibration_positive);
            break;
        case TJC_STATUS_CALIBRATION_RETURN:
            text = s_text_calibration_return;
            text_length = (uint16_t) sizeof(s_text_calibration_return);
            break;
        case TJC_STATUS_CALIBRATION_FAILED_IMU:
            text = s_text_calibration_failed_imu;
            text_length = (uint16_t) sizeof(s_text_calibration_failed_imu);
            break;
        case TJC_STATUS_CALIBRATION_FAILED_MOTION:
            text = s_text_calibration_failed_motion;
            text_length = (uint16_t) sizeof(s_text_calibration_failed_motion);
            break;
        case TJC_STATUS_CALIBRATION_FAILED_RANGE:
            text = s_text_calibration_failed_range;
            text_length = (uint16_t) sizeof(s_text_calibration_failed_range);
            break;
        case TJC_STATUS_CALIBRATION_ABORTED:
            text = s_text_calibration_aborted;
            text_length = (uint16_t) sizeof(s_text_calibration_aborted);
            break;
        case TJC_STATUS_BALANCE_CHECKING:
            text = s_text_balance_checking;
            text_length = (uint16_t) sizeof(s_text_balance_checking);
            break;
        case TJC_STATUS_BALANCE_ENABLED:
            text = s_text_balance_enabled;
            text_length = (uint16_t) sizeof(s_text_balance_enabled);
            break;
        case TJC_STATUS_BALANCE_IMU_FAULT:
            text = s_text_balance_imu_fault;
            text_length = (uint16_t) sizeof(s_text_balance_imu_fault);
            break;
        case TJC_STATUS_BALANCE_RECOVERED:
            text = s_text_balance_recovered;
            text_length = (uint16_t) sizeof(s_text_balance_recovered);
            break;
        case TJC_STATUS_BALANCE_DISABLED:
            text = s_text_balance_disabled;
            text_length = (uint16_t) sizeof(s_text_balance_disabled);
            break;
        case TJC_STATUS_CALIBRATION_SUCCESS:
        default:
            break;
    }

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    if (status == TJC_STATUS_CALIBRATION_SUCCESS) {
        length = Protocol_Tjc_AppendBytes(
            frame,
            length,
            s_text_calibration_success,
            (uint16_t) sizeof(s_text_calibration_success));
        length = Protocol_Tjc_AppendRatio(frame, length, ratio_x100);
        if (ratio_x100 >= 0) {
            length = Protocol_Tjc_AppendBytes(
                frame,
                length,
                s_text_direction_positive,
                (uint16_t) sizeof(s_text_direction_positive));
        } else {
            length = Protocol_Tjc_AppendBytes(
                frame,
                length,
                s_text_direction_negative,
                (uint16_t) sizeof(s_text_direction_negative));
        }
    } else if (text != 0) {
        length = Protocol_Tjc_AppendBytes(frame, length, text, text_length);
    }
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

static void Protocol_Tjc_ProcessCommand(uint8_t command)
{
    uint8_t current_mode;
    uint8_t accepted;

    g_tjc_last_command = command;
    current_mode = App_GetCurrentMode();

    if (command == TJC_COMMAND_QUERY) {
        g_tjc_valid_command_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_STATE, current_mode, command);
    } else if (command == TJC_COMMAND_STOP) {
        g_tjc_valid_command_count++;
        accepted = App_RequestMode(command);
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_STOPPED :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else if (command == TJC_COMMAND_BASIC_LINE) {
        g_tjc_valid_command_count++;
        if (command == current_mode) {
            Protocol_Tjc_SendResult(TJC_RESULT_ALREADY_ACTIVE, current_mode,
                                    command);
        } else {
            accepted = App_RequestMode(command);
            Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                    TJC_RESULT_ENTER_FAILED,
                                    App_GetCurrentMode(), command);
        }
    } else if (command == TJC_COMMAND_BALL_ENABLE) {
        g_tjc_valid_command_count++;
        accepted = App_SetBallBalanceEnabled(1U);
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else if (command == TJC_COMMAND_BALL_DISABLE) {
        g_tjc_valid_command_count++;
        accepted = App_SetBallBalanceEnabled(0U);
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else if (command == TJC_COMMAND_PIPE_CALIBRATE) {
        g_tjc_valid_command_count++;
        accepted = App_StartPipeCalibration();
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else {
        g_tjc_invalid_command_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_INVALID_COMMAND, current_mode,
                                command);
    }
}

static void Protocol_Tjc_ConsumeByte(uint8_t byte)
{
    switch (s_rx_state) {
        case TJC_RX_WAIT_HEADER_0:
            if (byte == TJC_REQUEST_HEADER_0) {
                s_rx_state = TJC_RX_WAIT_HEADER_1;
            }
            break;

        case TJC_RX_WAIT_HEADER_1:
            if (byte == TJC_REQUEST_HEADER_1) {
                s_rx_state = TJC_RX_WAIT_COMMAND;
            } else if (byte != TJC_REQUEST_HEADER_0) {
                s_rx_state = TJC_RX_WAIT_HEADER_0;
            }
            break;

        case TJC_RX_WAIT_COMMAND:
            if (byte == TJC_REQUEST_HEADER_0) {
                s_rx_state = TJC_RX_WAIT_HEADER_1;
            } else {
                s_rx_command = byte;
                s_rx_state = TJC_RX_WAIT_CHECKSUM;
            }
            break;

        case TJC_RX_WAIT_CHECKSUM:
            if (byte == Protocol_Tjc_CalculateRequestChecksum(s_rx_command)) {
                Protocol_Tjc_ProcessCommand(s_rx_command);
                Protocol_Tjc_ResetRxParser();
            } else {
                g_tjc_invalid_frame_count++;
                s_rx_state = (byte == TJC_REQUEST_HEADER_0) ?
                    TJC_RX_WAIT_HEADER_1 : TJC_RX_WAIT_HEADER_0;
            }
            break;

        default:
            Protocol_Tjc_ResetRxParser();
            break;
    }
}

void Protocol_Tjc_Task(void)
{
    uint8_t byte;
    uint32_t overflow_count = TjcUart_GetOverflowCount();
    uint32_t error_count = TjcUart_GetErrorCount();

    if ((overflow_count != g_tjc_rx_overflow_count) ||
        (error_count != g_tjc_rx_error_count)) {
        Protocol_Tjc_ResetRxParser();
    }

    g_tjc_rx_overflow_count = overflow_count;
    g_tjc_rx_error_count = error_count;

    while (TjcUart_ReadByte(&byte) != 0U) {
        g_tjc_rx_byte_count++;
        Protocol_Tjc_ConsumeByte(byte);
    }
}
