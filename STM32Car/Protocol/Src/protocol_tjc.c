#include "protocol_tjc.h"

#include "app_main.h"
#include "tjc_uart.h"

typedef enum {
    TJC_RX_WAIT_HEADER_0 = 0,
    TJC_RX_WAIT_HEADER_1,
    TJC_RX_WAIT_COMMAND,
    TJC_RX_WAIT_CHECKSUM
} Tjc_RxState;

static Tjc_RxState s_rx_state;
static uint8_t s_rx_command;

volatile uint32_t g_tjc_rx_byte_count;
volatile uint32_t g_tjc_valid_command_count;
volatile uint32_t g_tjc_invalid_command_count;
volatile uint32_t g_tjc_invalid_frame_count;
volatile uint32_t g_tjc_response_count;
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

void Protocol_Tjc_Init(void)
{
    g_tjc_rx_byte_count = 0U;
    g_tjc_valid_command_count = 0U;
    g_tjc_invalid_command_count = 0U;
    g_tjc_invalid_frame_count = 0U;
    g_tjc_response_count = 0U;
    g_tjc_rx_overflow_count = 0U;
    g_tjc_rx_error_count = 0U;
    g_tjc_last_command = TJC_COMMAND_STOP;
    g_tjc_last_result = TJC_RESULT_STATE;
    Protocol_Tjc_ResetRxParser();
    TjcUart_FlushRx();
}

void Protocol_Tjc_SendResult(Tjc_Result result, uint8_t current_mode,
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
