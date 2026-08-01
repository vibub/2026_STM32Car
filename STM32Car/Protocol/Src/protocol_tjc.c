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

#define TJC_STATUS_FRAME_BUFFER_SIZE 128U /* TJC 中文状态和动态结果命令的最大发送缓存。 */

static const uint8_t s_tjc_text_prefix[] = "page0.t0.txt=\"";
static const uint8_t s_tjc_text_suffix[] = {'\"', 0xFFU, 0xFFU, 0xFFU};

/* 串口屏工程使用 UTF-8，固定中文显式写成 UTF-8 字节以避免编译器转码。 */
static const uint8_t s_text_calibration_ready[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0x87U, 0x86U, 0xE5U, 0xA4U, 0x87U
};
static const uint8_t s_text_calibration_level[] = {
    0xE9U, 0x87U, 0x87U, 0xE9U, 0x9BU, 0x86U,
    0xE4U, 0xB8U, 0xADU, 0xE4U, 0xBDU, 0x8DU
};
static const uint8_t s_text_calibration_point[] = {
    0xE9U, 0x87U, 0x87U, 0xE9U, 0x9BU, 0x86U
};
static const uint8_t s_text_calibration_negative[] = {
    0xE9U, 0x87U, 0x87U, 0xE9U, 0x9BU, 0x86U,
    0x2DU, 0x31U, 0x30U, 0xE5U, 0xBAU, 0xA6U
};
static const uint8_t s_text_calibration_center[] = {
    0xE5U, 0xA4U, 0x8DU, 0xE6U, 0xA0U, 0xB8U,
    0x30U, 0xE5U, 0xBAU, 0xA6U
};
static const uint8_t s_text_calibration_positive[] = {
    0xE9U, 0x87U, 0x87U, 0xE9U, 0x9BU, 0x86U,
    0x2BU, 0x36U, 0x2EU, 0x38U, 0xE5U, 0xBAU, 0xA6U
};
static const uint8_t s_text_calibration_return[] = {
    0xE6U, 0xA3U, 0x80U, 0xE6U, 0x9FU, 0xA5U,
    0xE5U, 0x9BU, 0x9EU, 0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_calibration_success[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE6U, 0x88U, 0x90U, 0xE5U, 0x8AU, 0x9FU,
    0x20U, 0x53U, 0x3DU
};
static const uint8_t s_text_calibration_offset[] = {
    0x20U, 0x4FU, 0x3DU
};
static const uint8_t s_text_calibration_failed_imu[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U,
    0x20U, 0x49U, 0x4DU, 0x55U
};
static const uint8_t s_text_calibration_failed_motion[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U,
    0x20U, 0xE6U, 0x9CU, 0xAAU, 0xE9U, 0x9DU, 0x99U,
    0xE6U, 0xADU, 0xA2U
};
static const uint8_t s_text_calibration_failed_range[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U,
    0x20U, 0xE8U, 0xA1U, 0x8CU, 0xE7U, 0xA8U, 0x8BU
};
static const uint8_t s_text_calibration_failed_values[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U,
    0x20U, 0x53U, 0x3DU
};
static const uint8_t s_text_calibration_return_error[] = {
    0x20U, 0x52U, 0x3DU
};
static const uint8_t s_text_calibration_aborted[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU,
    0xE5U, 0xB7U, 0xB2U, 0xE4U, 0xB8U, 0xADU,
    0xE6U, 0x96U, 0xADU
};
static const uint8_t s_text_balance_checking[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U,
    0xE5U, 0xA4U, 0x8DU, 0xE6U, 0xA0U, 0xB8U,
    0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_balance_enabled[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U,
    0xE5U, 0xB7U, 0xB2U, 0xE5U, 0x90U, 0xAFU,
    0xE7U, 0x94U, 0xA8U
};
static const uint8_t s_text_balance_imu_fault[] = {
    0x49U, 0x4DU, 0x55U, 0xE5U, 0xBCU, 0x82U,
    0xE5U, 0xB8U, 0xB8U, 0xE5U, 0x9BU, 0x9EU,
    0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_balance_recovered[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U,
    0xE5U, 0xB7U, 0xB2U, 0xE6U, 0x81U, 0xA2U,
    0xE5U, 0xA4U, 0x8DU
};
static const uint8_t s_text_balance_disabled[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U,
    0xE5U, 0xB7U, 0xB2U, 0xE7U, 0xA6U, 0x81U,
    0xE7U, 0x94U, 0xA8U
};
static const uint8_t s_text_shuttle_preparing[] = {
    0xE4U, 0xB8U, 0x89U, 0xE9U, 0xA2U, 0x98U,
    0xE5U, 0x87U, 0x86U, 0xE5U, 0xA4U, 0x87U
};
static const uint8_t s_text_shuttle_ready[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xB8U, 0x89U,
    0xE9U, 0xA2U, 0x98U, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0xB0U, 0xB1U, 0xE7U, 0xBBU, 0xAAU
};
static const uint8_t s_text_shuttle_not_ready[] = {
    0xE8U, 0xAFU, 0xB7U, 0xE5U, 0xB0U, 0x86U,
    0xE7U, 0x90U, 0x83U, 0xE6U, 0x94U, 0xBEU,
    0xE5U, 0x9BU, 0x9EU, 0xE4U, 0xB8U, 0xADU,
    0xE5U, 0xBFU, 0x83U
};
static const uint8_t s_text_shuttle_running[] = {
    0xE4U, 0xB8U, 0x89U, 0xE9U, 0xA2U, 0x98U,
    0xE8U, 0xBFU, 0x90U, 0xE8U, 0xA1U, 0x8CU
};
static const uint8_t s_text_shuttle_turning[] = {
    0x2BU, 0x35U, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0x88U, 0xB0U, 0xE8U, 0xBEU, 0xBEU
};
static const uint8_t s_text_shuttle_success[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xB8U, 0x89U,
    0xE9U, 0xA2U, 0x98U, 0xE6U, 0xB0U, 0xB4U,
    0xE5U, 0xB9U, 0xB3U, 0xE4U, 0xBFU, 0x9DU,
    0xE6U, 0x8CU, 0x81U
};
static const uint8_t s_text_shuttle_phase[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xB8U, 0x89U,
    0xE9U, 0xA2U, 0x98U, 0xE9U, 0x98U, 0xB6U,
    0xE6U, 0xAEU, 0xB5U
};
static const uint8_t s_text_degree[] = {
    0xE5U, 0xBAU, 0xA6U
};
static const uint8_t s_text_shuttle_failed[] = {
    0xE4U, 0xB8U, 0x89U, 0xE9U, 0xA2U, 0x98U,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U
};
static const uint8_t s_text_ab_preparing[] = {
    0xE7U, 0xACU, 0xACU, 0xE5U, 0x9BU, 0x9BU, 0xE9U, 0xA2U, 0x98U,
    0xE7U, 0xADU, 0x89U, 0xE5U, 0xBEU, 0x85U, 0xE9U, 0x92U, 0xA2U,
    0xE7U, 0x90U, 0x83U, 0xE7U, 0xA8U, 0xB3U, 0xE5U, 0xAEU, 0x9AU
};
static const uint8_t s_text_ab_running[] = {
    0xE7U, 0xACU, 0xACU, 0xE5U, 0x9BU, 0x9BU, 0xE9U, 0xA2U, 0x98U,
    0xE8U, 0xBFU, 0x90U, 0xE8U, 0xA1U, 0x8CU, 0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_ab_passed[] = {
    0xE7U, 0xACU, 0xACU, 0xE5U, 0x9BU, 0x9BU, 0xE9U, 0xA2U, 0x98U,
    0xE9U, 0x80U, 0x9AU, 0xE8U, 0xBFU, 0x87U
};
static const uint8_t s_text_ab_failed[] = {
    0xE7U, 0xACU, 0xACU, 0xE5U, 0x9BU, 0x9BU, 0xE9U, 0xA2U, 0x98U,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U
};
static const uint8_t s_text_abcd_preparing[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xBAU, 0x94U, 0xE9U, 0xA2U, 0x98U,
    0xE7U, 0xADU, 0x89U, 0xE5U, 0xBEU, 0x85U, 0xE9U, 0x92U, 0xA2U,
    0xE7U, 0x90U, 0x83U, 0xE7U, 0xA8U, 0xB3U, 0xE5U, 0xAEU, 0x9AU
};
static const uint8_t s_text_abcd_running[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xBAU, 0x94U, 0xE9U, 0xA2U, 0x98U,
    0xE8U, 0xBFU, 0x90U, 0xE8U, 0xA1U, 0x8CU, 0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_abcd_passed[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xBAU, 0x94U, 0xE9U, 0xA2U, 0x98U,
    0xE9U, 0x80U, 0x9AU, 0xE8U, 0xBFU, 0x87U
};
static const uint8_t s_text_abcd_failed[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xBAU, 0x94U, 0xE9U, 0xA2U, 0x98U,
    0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U
};
static const uint8_t s_text_run_failed_imu[] = {
    0x49U, 0x4DU, 0x55U, 0xE6U, 0x95U, 0x85U, 0xE9U, 0x9AU, 0x9CU,
    0xEFU, 0xBCU, 0x8CU, 0xE5U, 0xB7U, 0xB2U, 0xE5U, 0x81U, 0x9CU,
    0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_failed_mapping[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU, 0xE6U, 0x98U, 0xA0U,
    0xE5U, 0xB0U, 0x84U, 0xE6U, 0x95U, 0x85U, 0xE9U, 0x9AU, 0x9CU
};
static const uint8_t s_text_run_failed_ready[] = {
    0xE9U, 0x92U, 0xA2U, 0xE7U, 0x90U, 0x83U, 0xE6U, 0x9CU, 0xAAU,
    0xE7U, 0xA8U, 0xB3U, 0xE5U, 0xAEU, 0x9AU
};
static const uint8_t s_text_run_failed_motion[] = {
    0xE9U, 0x80U, 0x9FU, 0xE5U, 0xBAU, 0xA6U, 0xE8U, 0xA7U, 0x84U,
    0xE5U, 0x88U, 0x92U, 0xE5U, 0xA4U, 0xB1U, 0xE8U, 0xB4U, 0xA5U
};
static const uint8_t s_text_run_failed_balance[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U, 0xE8U, 0xA2U, 0xABU,
    0xE7U, 0xA6U, 0x81U, 0xE7U, 0x94U, 0xA8U, 0xEFU, 0xBCU, 0x8CU,
    0xE5U, 0xB7U, 0xB2U, 0xE5U, 0x81U, 0x9CU, 0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_failed_vision[] = {
    0xE8U, 0xA7U, 0x86U, 0xE8U, 0xA7U, 0x89U, 0xE8U, 0xB6U, 0x85U,
    0xE6U, 0x97U, 0xB6U, 0xEFU, 0xBCU, 0x8CU, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0x81U, 0x9CU, 0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_failed_sensor[] = {
    0xE7U, 0xBAU, 0xA2U, 0xE5U, 0xA4U, 0x96U, 0xE6U, 0x95U, 0x85U,
    0xE9U, 0x9AU, 0x9CU, 0xEFU, 0xBCU, 0x8CU, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0x81U, 0x9CU, 0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_failed_line[] = {
    0xE6U, 0x8CU, 0x81U, 0xE7U, 0xBBU, 0xADU, 0xE4U, 0xB8U, 0xA2U,
    0xE7U, 0xBAU, 0xBFU, 0xEFU, 0xBCU, 0x8CU, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0x81U, 0x9CU, 0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_failed_motor[] = {
    0xE7U, 0x94U, 0xB5U, 0xE6U, 0x9CU, 0xBAU, 0xE6U, 0x95U, 0x85U,
    0xE9U, 0x9AU, 0x9CU, 0xEFU, 0xBCU, 0x8CU, 0xE5U, 0xB7U, 0xB2U,
    0xE5U, 0x81U, 0x9CU, 0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_command_locked[] = {
    0xE5U, 0x8AU, 0xA8U, 0xE6U, 0x80U, 0x81U, 0xE4U, 0xBBU, 0xBBU,
    0xE5U, 0x8AU, 0xA1U, 0xE8U, 0xBFU, 0x90U, 0xE8U, 0xA1U, 0x8CU,
    0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_run_reject_stop[] = {
    0xE8U, 0xAFU, 0xB7U, 0xE5U, 0x85U, 0x88U, 0xE5U, 0x81U, 0x9CU,
    0xE8U, 0xBDU, 0xA6U
};
static const uint8_t s_text_run_reject_calibrating[] = {
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU, 0xE6U, 0xADU, 0xA3U,
    0xE5U, 0x9CU, 0xA8U, 0xE8U, 0xBFU, 0x9BU, 0xE8U, 0xA1U, 0x8CU
};
static const uint8_t s_text_run_reject_calibration[] = {
    0xE8U, 0xAFU, 0xB7U, 0xE5U, 0x85U, 0x88U, 0xE5U, 0xAEU, 0x8CU,
    0xE6U, 0x88U, 0x90U, 0xE4U, 0xB9U, 0x9DU, 0xE7U, 0x82U, 0xB9U,
    0xE6U, 0xA0U, 0x87U, 0xE5U, 0xAEU, 0x9AU
};
static const uint8_t s_text_run_reject_busy[] = {
    0xE5U, 0x85U, 0xB6U, 0xE4U, 0xBBU, 0x96U, 0xE4U, 0xBBU, 0xBBU,
    0xE5U, 0x8AU, 0xA1U, 0xE8U, 0xBFU, 0x90U, 0xE8U, 0xA1U, 0x8CU,
    0xE4U, 0xB8U, 0xADU
};
static const uint8_t s_text_run_reject_balance[] = {
    0xE5U, 0xB9U, 0xB3U, 0xE8U, 0xA1U, 0xA1U, 0xE6U, 0x97U, 0xA0U,
    0xE6U, 0xB3U, 0x95U, 0xE5U, 0x90U, 0xAFU, 0xE7U, 0x94U, 0xA8U
};
static const uint8_t s_text_result_ab[] = {
    0xE7U, 0xACU, 0xACU, 0xE5U, 0x9BU, 0x9BU, 0xE9U, 0xA2U, 0x98U
};
static const uint8_t s_text_result_abcd[] = {
    0xE7U, 0xACU, 0xACU, 0xE4U, 0xBAU, 0x94U, 0xE9U, 0xA2U, 0x98U
};
static const uint8_t s_text_result_passed[] = {
    0xE9U, 0x80U, 0x9AU, 0xE8U, 0xBFU, 0x87U
};
static const uint8_t s_text_result_overrange[] = {
    0xE5U, 0xAEU, 0x8CU, 0xE6U, 0x88U, 0x90U, 0xE4U, 0xBDU, 0x86U,
    0xE8U, 0xB6U, 0x85U, 0xE5U, 0xB7U, 0xAEU
};
static const uint8_t s_text_result_timeout[] = {
    0xE8U, 0xB6U, 0x85U, 0xE6U, 0x97U, 0xB6U
};
static const uint8_t s_text_result_max_error[] = {
    0x20U, 0xE6U, 0x9CU, 0x80U, 0xE5U, 0xA4U, 0xA7U,
    0xE8U, 0xAFU, 0xAFU, 0xE5U, 0xB7U, 0xAEU
};

static const uint8_t s_tjc_command_suffix[] = {0xFFU, 0xFFU, 0xFFU};
static const uint8_t s_tjc_warning_circle_100[] =
    "cir 160,120,100,63488";
static const uint8_t s_tjc_warning_circle_99[] =
    "cir 160,120,99,63488";
static const uint8_t s_tjc_warning_circle_98[] =
    "cir 160,120,98,63488";
static const uint8_t s_tjc_warning_circle_97[] =
    "cir 160,120,97,63488";
static const uint8_t s_tjc_refresh_page[] = "ref 0";

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

static uint16_t Protocol_Tjc_AppendFixedX100(uint8_t *buffer,
                                             uint16_t length,
                                             int16_t value_x100,
                                             uint8_t always_sign)
{
    uint32_t magnitude;
    uint16_t integer_part;
    uint16_t fraction_part;

    if (value_x100 < 0) {
        buffer[length++] = (uint8_t) '-';
        magnitude = (uint32_t) (-(int32_t) value_x100);
    } else {
        if (always_sign != 0U) {
            buffer[length++] = (uint8_t) '+';
        }
        magnitude = (uint32_t) value_x100;
    }
    integer_part = (uint16_t) (magnitude / 100U);
    fraction_part = (uint16_t) (magnitude % 100U);

    if (integer_part >= 100U) {
        buffer[length++] = (uint8_t) ('0' + (integer_part / 100U) % 10U);
    }
    if (integer_part >= 10U) {
        buffer[length++] = (uint8_t) ('0' + (integer_part / 10U) % 10U);
    }
    buffer[length++] = (uint8_t) ('0' + integer_part % 10U);
    buffer[length++] = (uint8_t) '.';
    buffer[length++] = (uint8_t) ('0' + fraction_part / 10U);
    buffer[length++] = (uint8_t) ('0' + fraction_part % 10U);
    return length;
}

static uint16_t Protocol_Tjc_AppendUnsignedX10(uint8_t *buffer,
                                               uint16_t length,
                                               uint32_t value_x10)
{
    uint32_t integer_part = value_x10 / 10U;
    uint32_t divisor = 1U;

    while ((integer_part / divisor) >= 10U) {
        divisor *= 10U;
    }
    do {
        buffer[length++] = (uint8_t) ('0' +
            (uint8_t) ((integer_part / divisor) % 10U));
        divisor /= 10U;
    } while (divisor != 0U);
    buffer[length++] = (uint8_t) '.';
    buffer[length++] = (uint8_t) ('0' + value_x10 % 10U);
    return length;
}

static void Protocol_Tjc_SendTextFrame(const uint8_t *text,
                                       uint16_t text_length)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(frame, length, text, text_length);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

static void Protocol_Tjc_SendAsciiCommand(const uint8_t *command,
                                          uint16_t command_length)
{
    if ((command == 0) || (command_length == 0U)) {
        return;
    }
    TjcUart_SendData(command, command_length);
    TjcUart_SendData(s_tjc_command_suffix,
                     (uint16_t) sizeof(s_tjc_command_suffix));
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

void Protocol_Tjc_SendStatusText(Tjc_StatusText status)
{
    const uint8_t *text = 0;
    uint16_t text_length = 0U;

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
        case TJC_STATUS_CALIBRATION_SUCCESS:
            text = s_text_calibration_success;
            text_length = (uint16_t) sizeof(s_text_calibration_success);
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
        case TJC_STATUS_SHUTTLE_PREPARING:
            text = s_text_shuttle_preparing;
            text_length = (uint16_t) sizeof(s_text_shuttle_preparing);
            break;
        case TJC_STATUS_SHUTTLE_READY:
            text = s_text_shuttle_ready;
            text_length = (uint16_t) sizeof(s_text_shuttle_ready);
            break;
        case TJC_STATUS_SHUTTLE_NOT_READY:
            text = s_text_shuttle_not_ready;
            text_length = (uint16_t) sizeof(s_text_shuttle_not_ready);
            break;
        case TJC_STATUS_SHUTTLE_RUNNING:
            text = s_text_shuttle_running;
            text_length = (uint16_t) sizeof(s_text_shuttle_running);
            break;
        case TJC_STATUS_SHUTTLE_TURNING:
            text = s_text_shuttle_turning;
            text_length = (uint16_t) sizeof(s_text_shuttle_turning);
            break;
        case TJC_STATUS_SHUTTLE_SUCCESS:
            text = s_text_shuttle_success;
            text_length = (uint16_t) sizeof(s_text_shuttle_success);
            break;
        case TJC_STATUS_SHUTTLE_FAILED:
            text = s_text_shuttle_failed;
            text_length = (uint16_t) sizeof(s_text_shuttle_failed);
            break;
        case TJC_STATUS_AB_PREPARING:
            text = s_text_ab_preparing;
            text_length = (uint16_t) sizeof(s_text_ab_preparing);
            break;
        case TJC_STATUS_AB_RUNNING:
            text = s_text_ab_running;
            text_length = (uint16_t) sizeof(s_text_ab_running);
            break;
        case TJC_STATUS_AB_PASSED:
            text = s_text_ab_passed;
            text_length = (uint16_t) sizeof(s_text_ab_passed);
            break;
        case TJC_STATUS_AB_FAILED:
            text = s_text_ab_failed;
            text_length = (uint16_t) sizeof(s_text_ab_failed);
            break;
        case TJC_STATUS_ABCD_PREPARING:
            text = s_text_abcd_preparing;
            text_length = (uint16_t) sizeof(s_text_abcd_preparing);
            break;
        case TJC_STATUS_ABCD_RUNNING:
            text = s_text_abcd_running;
            text_length = (uint16_t) sizeof(s_text_abcd_running);
            break;
        case TJC_STATUS_ABCD_PASSED:
            text = s_text_abcd_passed;
            text_length = (uint16_t) sizeof(s_text_abcd_passed);
            break;
        case TJC_STATUS_ABCD_FAILED:
            text = s_text_abcd_failed;
            text_length = (uint16_t) sizeof(s_text_abcd_failed);
            break;
        case TJC_STATUS_RUN_FAILED_IMU:
            text = s_text_run_failed_imu;
            text_length = (uint16_t) sizeof(s_text_run_failed_imu);
            break;
        case TJC_STATUS_RUN_FAILED_MAPPING:
            text = s_text_run_failed_mapping;
            text_length = (uint16_t) sizeof(s_text_run_failed_mapping);
            break;
        case TJC_STATUS_RUN_FAILED_READY:
            text = s_text_run_failed_ready;
            text_length = (uint16_t) sizeof(s_text_run_failed_ready);
            break;
        case TJC_STATUS_RUN_FAILED_MOTION:
            text = s_text_run_failed_motion;
            text_length = (uint16_t) sizeof(s_text_run_failed_motion);
            break;
        case TJC_STATUS_RUN_FAILED_BALANCE:
            text = s_text_run_failed_balance;
            text_length = (uint16_t) sizeof(s_text_run_failed_balance);
            break;
        case TJC_STATUS_RUN_FAILED_VISION:
            text = s_text_run_failed_vision;
            text_length = (uint16_t) sizeof(s_text_run_failed_vision);
            break;
        case TJC_STATUS_RUN_FAILED_SENSOR:
            text = s_text_run_failed_sensor;
            text_length = (uint16_t) sizeof(s_text_run_failed_sensor);
            break;
        case TJC_STATUS_RUN_FAILED_LINE:
            text = s_text_run_failed_line;
            text_length = (uint16_t) sizeof(s_text_run_failed_line);
            break;
        case TJC_STATUS_RUN_FAILED_MOTOR:
            text = s_text_run_failed_motor;
            text_length = (uint16_t) sizeof(s_text_run_failed_motor);
            break;
        case TJC_STATUS_RUN_COMMAND_LOCKED:
            text = s_text_run_command_locked;
            text_length = (uint16_t) sizeof(s_text_run_command_locked);
            break;
        case TJC_STATUS_RUN_REJECT_STOP:
            text = s_text_run_reject_stop;
            text_length = (uint16_t) sizeof(s_text_run_reject_stop);
            break;
        case TJC_STATUS_RUN_REJECT_CALIBRATING:
            text = s_text_run_reject_calibrating;
            text_length = (uint16_t) sizeof(s_text_run_reject_calibrating);
            break;
        case TJC_STATUS_RUN_REJECT_CALIBRATION:
            text = s_text_run_reject_calibration;
            text_length = (uint16_t) sizeof(s_text_run_reject_calibration);
            break;
        case TJC_STATUS_RUN_REJECT_BUSY:
            text = s_text_run_reject_busy;
            text_length = (uint16_t) sizeof(s_text_run_reject_busy);
            break;
        case TJC_STATUS_RUN_REJECT_BALANCE:
            text = s_text_run_reject_balance;
            text_length = (uint16_t) sizeof(s_text_run_reject_balance);
            break;
        default:
            break;
    }

    if (text != 0) {
        Protocol_Tjc_SendTextFrame(text, text_length);
    }
}

void Protocol_Tjc_SendDynamicRunResult(uint8_t run_mode,
                                       uint8_t run_result,
                                       uint32_t elapsed_ms,
                                       float maximum_abs_error_mm)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;
    uint32_t elapsed_x10;
    uint32_t maximum_error_x10;
    const uint8_t *mode_text;
    uint16_t mode_text_length;
    const uint8_t *result_text;
    uint16_t result_text_length;

    if (run_mode == (uint8_t) DYNAMIC_RUN_MODE_AB) {
        mode_text = s_text_result_ab;
        mode_text_length = (uint16_t) sizeof(s_text_result_ab);
    } else if (run_mode == (uint8_t) DYNAMIC_RUN_MODE_ABCD) {
        mode_text = s_text_result_abcd;
        mode_text_length = (uint16_t) sizeof(s_text_result_abcd);
    } else {
        return;
    }

    if (run_result == (uint8_t) DYNAMIC_RUN_RESULT_PASSED) {
        result_text = s_text_result_passed;
        result_text_length = (uint16_t) sizeof(s_text_result_passed);
    } else if (run_result ==
               (uint8_t) DYNAMIC_RUN_RESULT_PASSED_OVERRANGE) {
        result_text = s_text_result_overrange;
        result_text_length = (uint16_t) sizeof(s_text_result_overrange);
    } else if (run_result == (uint8_t) DYNAMIC_RUN_RESULT_TIMEOUT) {
        result_text = s_text_result_timeout;
        result_text_length = (uint16_t) sizeof(s_text_result_timeout);
    } else {
        return;
    }

    elapsed_x10 = (elapsed_ms + 50U) / 100U;
    if (maximum_abs_error_mm < 0.0f) {
        maximum_abs_error_mm = 0.0f;
    }
    if (maximum_abs_error_mm > 999.9f) {
        maximum_abs_error_mm = 999.9f;
    }
    maximum_error_x10 = (uint32_t) (maximum_abs_error_mm * 10.0f + 0.5f);

    length = Protocol_Tjc_AppendBytes(
        frame, length, s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(
        frame, length, mode_text, mode_text_length);
    length = Protocol_Tjc_AppendBytes(
        frame, length, result_text, result_text_length);
    frame[length++] = (uint8_t) ' ';
    length = Protocol_Tjc_AppendUnsignedX10(frame, length, elapsed_x10);
    frame[length++] = (uint8_t) 's';
    length = Protocol_Tjc_AppendBytes(
        frame, length, s_text_result_max_error,
        (uint16_t) sizeof(s_text_result_max_error));
    length = Protocol_Tjc_AppendUnsignedX10(
        frame, length, maximum_error_x10);
    frame[length++] = (uint8_t) 'm';
    frame[length++] = (uint8_t) 'm';
    length = Protocol_Tjc_AppendBytes(
        frame, length, s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

void Protocol_Tjc_SendShuttlePhase(uint8_t phase,
                                   float target_pipe_pitch_deg)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;
    float scaled = target_pipe_pitch_deg * 100.0f;
    int16_t target_x100;

    if ((phase == 0U) || (phase > 3U)) {
        return;
    }
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32767.0f) {
        scaled = -32767.0f;
    }
    target_x100 = (int16_t) ((scaled >= 0.0f) ?
                             (scaled + 0.5f) : (scaled - 0.5f));

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_shuttle_phase,
        (uint16_t) sizeof(s_text_shuttle_phase));
    frame[length++] = (uint8_t) ('0' + phase);
    frame[length++] = (uint8_t) ' ';
    length = Protocol_Tjc_AppendFixedX100(
        frame, length, target_x100, 1U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_degree,
        (uint16_t) sizeof(s_text_degree));
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

void Protocol_Tjc_SetBallLimitWarning(uint8_t active)
{
    if (active != 0U) {
        Protocol_Tjc_SendAsciiCommand(
            s_tjc_warning_circle_100,
            (uint16_t) (sizeof(s_tjc_warning_circle_100) - 1U));
        Protocol_Tjc_SendAsciiCommand(
            s_tjc_warning_circle_99,
            (uint16_t) (sizeof(s_tjc_warning_circle_99) - 1U));
        Protocol_Tjc_SendAsciiCommand(
            s_tjc_warning_circle_98,
            (uint16_t) (sizeof(s_tjc_warning_circle_98) - 1U));
        Protocol_Tjc_SendAsciiCommand(
            s_tjc_warning_circle_97,
            (uint16_t) (sizeof(s_tjc_warning_circle_97) - 1U));
    } else {
        Protocol_Tjc_SendAsciiCommand(
            s_tjc_refresh_page,
            (uint16_t) (sizeof(s_tjc_refresh_page) - 1U));
    }
}

void Protocol_Tjc_SendCalibrationPoint(uint8_t point_index,
                                       uint8_t point_count)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;

    if ((point_index == 0U) || (point_index > 9U) ||
        (point_count == 0U) || (point_count > 9U)) {
        return;
    }

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_point,
        (uint16_t) sizeof(s_text_calibration_point));
    frame[length++] = (uint8_t) ('0' + point_index);
    frame[length++] = (uint8_t) '/';
    frame[length++] = (uint8_t) ('0' + point_count);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

void Protocol_Tjc_SendCalibrationSuccess(int16_t scale_x100,
                                         int16_t offset_x100)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_success,
        (uint16_t) sizeof(s_text_calibration_success));
    length = Protocol_Tjc_AppendFixedX100(frame, length, scale_x100, 0U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_offset,
        (uint16_t) sizeof(s_text_calibration_offset));
    length = Protocol_Tjc_AppendFixedX100(frame, length, offset_x100, 1U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

void Protocol_Tjc_SendCalibrationRangeFailure(int16_t scale_x100,
                                              int16_t offset_x100,
                                              int16_t return_error_x100)
{
    uint8_t frame[TJC_STATUS_FRAME_BUFFER_SIZE];
    uint16_t length = 0U;

    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_prefix,
        (uint16_t) (sizeof(s_tjc_text_prefix) - 1U));
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_failed_values,
        (uint16_t) sizeof(s_text_calibration_failed_values));
    length = Protocol_Tjc_AppendFixedX100(frame, length, scale_x100, 0U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_offset,
        (uint16_t) sizeof(s_text_calibration_offset));
    length = Protocol_Tjc_AppendFixedX100(frame, length, offset_x100, 1U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_text_calibration_return_error,
        (uint16_t) sizeof(s_text_calibration_return_error));
    length = Protocol_Tjc_AppendFixedX100(
        frame, length, return_error_x100, 0U);
    length = Protocol_Tjc_AppendBytes(
        frame,
        length,
        s_tjc_text_suffix,
        (uint16_t) sizeof(s_tjc_text_suffix));
    TjcUart_SendData(frame, length);
    g_tjc_status_text_count++;
}

static uint8_t Protocol_Tjc_IsModeCommand(uint8_t command)
{
    return (uint8_t) (
        (command == TJC_COMMAND_BASIC_LINE) ||
        (command == TJC_COMMAND_BALL_ENABLE) ||
        (command == TJC_COMMAND_BALL_DISABLE) ||
        (command == TJC_COMMAND_PIPE_CALIBRATE) ||
        (command == TJC_COMMAND_BALL_SHUTTLE) ||
        (command == TJC_COMMAND_AB) ||
        (command == TJC_COMMAND_ABCD));
}

static void Protocol_Tjc_ProcessCommand(uint8_t command)
{
    uint8_t current_mode;
    uint8_t accepted;

    g_tjc_last_command = command;
    current_mode = App_GetCurrentMode();

    if ((App_GetHostCalibrationEnabled() != 0U) &&
        (command != TJC_COMMAND_QUERY) &&
        (command != TJC_COMMAND_STOP)) {
        /* 上位机标定模式：禁止通过串口屏启动任何任务。 */
        g_tjc_valid_command_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_ENTER_FAILED, current_mode,
                                command);
        return;
    }

    if (command == TJC_COMMAND_QUERY) {
        g_tjc_valid_command_count++;
        Protocol_Tjc_SendResult(TJC_RESULT_STATE, current_mode, command);
    } else if (command == TJC_COMMAND_STOP) {
        g_tjc_valid_command_count++;
        accepted = App_RequestMode(command);
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_STOPPED :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else if ((App_IsDynamicRunCommandLocked() != 0U) &&
               (Protocol_Tjc_IsModeCommand(command) != 0U)) {
        g_tjc_valid_command_count++;
        if (command == TJC_COMMAND_BALL_DISABLE) {
            accepted = App_SetBallBalanceEnabled(0U);
            Protocol_Tjc_SendResult(accepted ? TJC_RESULT_STOPPED :
                                    TJC_RESULT_ENTER_FAILED,
                                    App_GetCurrentMode(), command);
        } else {
            Protocol_Tjc_SendResult(TJC_RESULT_ENTER_FAILED,
                                    App_GetCurrentMode(), command);
            Protocol_Tjc_SendStatusText(TJC_STATUS_RUN_COMMAND_LOCKED);
        }
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
    } else if (command == TJC_COMMAND_BALL_SHUTTLE) {
        g_tjc_valid_command_count++;
        accepted = App_StartBallShuttle();
        Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                TJC_RESULT_ENTER_FAILED,
                                App_GetCurrentMode(), command);
    } else if (command == TJC_COMMAND_AB) {
        g_tjc_valid_command_count++;
        if (command == current_mode) {
            Protocol_Tjc_SendResult(TJC_RESULT_ALREADY_ACTIVE,
                                    current_mode, command);
        } else {
            accepted = App_StartAbRun();
            Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                    TJC_RESULT_ENTER_FAILED,
                                    App_GetCurrentMode(), command);
        }
    } else if (command == TJC_COMMAND_ABCD) {
        g_tjc_valid_command_count++;
        if (command == current_mode) {
            Protocol_Tjc_SendResult(TJC_RESULT_ALREADY_ACTIVE,
                                    current_mode, command);
        } else {
            accepted = App_StartAbcdRun();
            Protocol_Tjc_SendResult(accepted ? TJC_RESULT_SWITCH_OK :
                                    TJC_RESULT_ENTER_FAILED,
                                    App_GetCurrentMode(), command);
        }
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
