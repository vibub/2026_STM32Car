#include "app_main.h"
#include "protocol_tjc.h"
#include "tjc_uart.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_RX_CAPACITY 64U   /* 单次协议测试允许注入的最大接收字节数。 */
#define TEST_TX_CAPACITY 1024U /* 单次协议测试允许记录的最大发送字节数。 */

static uint8_t s_rx_data[TEST_RX_CAPACITY];
static uint16_t s_rx_length;
static uint16_t s_rx_index;
static uint8_t s_tx_data[TEST_TX_CAPACITY];
static uint16_t s_tx_length;
static uint8_t s_zero_adjust_active;
static uint32_t s_zero_pwm_us = 1070U;
static uint32_t s_last_set_pwm_us;
static uint32_t s_last_confirm_pwm_us;
static uint8_t s_start_adjust_count;
static uint32_t s_last_saved_times_ms[3];
static uint8_t s_save_count;

static void test_reset_tx(void)
{
    s_tx_length = 0U;
    memset(s_tx_data, 0, sizeof(s_tx_data));
}

static void test_feed(const uint8_t *data, uint16_t length)
{
    assert(length <= TEST_RX_CAPACITY);
    memcpy(s_rx_data, data, length);
    s_rx_length = length;
    s_rx_index = 0U;
    Protocol_Tjc_Task();
    assert(s_rx_index == s_rx_length);
}

static uint8_t test_tx_contains(const char *text)
{
    uint16_t text_length = (uint16_t) strlen(text);
    uint16_t index;

    if (text_length > s_tx_length) {
        return 0U;
    }
    for (index = 0U; index <= (uint16_t) (s_tx_length - text_length); index++) {
        if (memcmp(&s_tx_data[index], text, text_length) == 0) {
            return 1U;
        }
    }
    return 0U;
}

void TjcUart_Init(void)
{
}

void TjcUart_SendByte(uint8_t byte)
{
    TjcUart_SendData(&byte, 1U);
}

void TjcUart_SendData(const uint8_t *data, uint16_t length)
{
    assert((uint32_t) s_tx_length + length <= TEST_TX_CAPACITY);
    memcpy(&s_tx_data[s_tx_length], data, length);
    s_tx_length = (uint16_t) (s_tx_length + length);
}

uint8_t TjcUart_ReadByte(uint8_t *byte)
{
    if ((byte == 0) || (s_rx_index >= s_rx_length)) {
        return 0U;
    }
    *byte = s_rx_data[s_rx_index++];
    return 1U;
}

void TjcUart_FlushRx(void)
{
    s_rx_length = 0U;
    s_rx_index = 0U;
}

uint32_t TjcUart_GetRxByteCount(void)
{
    return s_rx_index;
}

uint32_t TjcUart_GetOverflowCount(void)
{
    return 0U;
}

uint32_t TjcUart_GetErrorCount(void)
{
    return 0U;
}

void TjcUart_HandleRxComplete(UART_HandleTypeDef *huart)
{
    (void) huart;
}

void TjcUart_HandleError(UART_HandleTypeDef *huart)
{
    (void) huart;
}

uint8_t App_GetHostCalibrationEnabled(void)
{
    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return APP_MODE_STOP;
}

uint8_t App_StartPipeZeroAdjustment(void)
{
    s_zero_adjust_active = 1U;
    s_start_adjust_count++;
    return 1U;
}

uint8_t App_SetPipeZeroPwm(uint32_t pwm_us)
{
    if ((s_zero_adjust_active == 0U) || (pwm_us < 500U) ||
        (pwm_us > 1600U)) {
        return 0U;
    }
    s_zero_pwm_us = pwm_us;
    s_last_set_pwm_us = pwm_us;
    return 1U;
}

uint8_t App_ConfirmPipeZeroAndStartCalibration(uint32_t pwm_us)
{
    if (App_SetPipeZeroPwm(pwm_us) == 0U) {
        return 0U;
    }
    s_last_confirm_pwm_us = pwm_us;
    s_zero_adjust_active = 0U;
    return 1U;
}

uint16_t App_GetPipeZeroPwmUs(void)
{
    return (uint16_t) s_zero_pwm_us;
}

uint8_t App_RequestMode(uint8_t mode)
{
    (void) mode;
    s_zero_adjust_active = 0U;
    return 1U;
}

uint8_t App_IsDynamicRunCommandLocked(void)
{
    return 0U;
}

uint8_t App_SetBallBalanceEnabled(uint8_t enabled)
{
    (void) enabled;
    return 1U;
}

uint8_t App_StartBallShuttle(void)
{
    return 1U;
}

uint8_t App_StartAbRun(void)
{
    return 1U;
}

uint8_t App_StartAbcdRun(void)
{
    return 1U;
}

uint8_t App_StartAbcdTargetRun(void)
{
    return 1U;
}

uint8_t App_SaveBallShuttleTimes(uint32_t phase_1_time_ms,
                                 uint32_t phase_2_time_ms,
                                 uint32_t phase_3_time_ms)
{
    s_last_saved_times_ms[0] = phase_1_time_ms;
    s_last_saved_times_ms[1] = phase_2_time_ms;
    s_last_saved_times_ms[2] = phase_3_time_ms;
    s_save_count++;
    return 1U;
}

void App_GetBallShuttleConfig(float *phase_1_angle_deg,
                              uint32_t *phase_1_time_ms,
                              float *phase_2_angle_deg,
                              uint32_t *phase_2_time_ms,
                              float *phase_3_angle_deg,
                              uint32_t *phase_3_time_ms)
{
    if (phase_1_angle_deg != 0) {
        *phase_1_angle_deg = 0.0f;
    }
    if (phase_1_time_ms != 0) {
        *phase_1_time_ms = (s_save_count != 0U) ?
                           s_last_saved_times_ms[0] : 300U;
    }
    if (phase_2_angle_deg != 0) {
        *phase_2_angle_deg = 0.0f;
    }
    if (phase_2_time_ms != 0) {
        *phase_2_time_ms = (s_save_count != 0U) ?
                           s_last_saved_times_ms[1] : 600U;
    }
    if (phase_3_angle_deg != 0) {
        *phase_3_angle_deg = 0.0f;
    }
    if (phase_3_time_ms != 0) {
        *phase_3_time_ms = (s_save_count != 0U) ?
                           s_last_saved_times_ms[2] : 300U;
    }
}

int main(void)
{
    static const uint8_t shuttle_query_frame[] = {
        0x5AU, 0xA5U, 0x11U, 0xEEU
    };
    static const uint8_t shuttle_save_frame[] = {
        0x5AU, 0xA5U, 0x10U,
        0xBCU, 0x02U, /* 第一阶段700 ms。 */
        0x50U, 0x03U, /* 第二阶段848 ms。 */
        0x21U, 0x02U, /* 第三阶段545 ms。 */
        0x48U, 0x78U  /* 前9字节的Modbus CRC16，小端发送。 */
    };
    static const uint8_t enter_frame[] = {0x5AU, 0xA5U, 0x04U, 0xFBU};
    static const uint8_t set_frame[] = {
        0x5AU, 0xA5U, 0x12U, 0x7EU, 0x04U, 0x00U, 0x00U
    };
    static const uint8_t confirm_frame[] = {
        0x5AU, 0xA5U, 0x13U, 0x7EU, 0x04U, 0x00U, 0x00U
    };

    Protocol_Tjc_Init();

    test_reset_tx();
    Protocol_Tjc_SendDynamicRunResult(
        (uint8_t) DYNAMIC_RUN_MODE_AB,
        (uint8_t) DYNAMIC_RUN_RESULT_PASSED_OVERRANGE,
        7500U,
        12.5f);
    assert(s_tx_length == 0U);

    test_reset_tx();
    Protocol_Tjc_SendDynamicRunResult(
        (uint8_t) DYNAMIC_RUN_MODE_ABCD,
        (uint8_t) DYNAMIC_RUN_RESULT_TIMEOUT,
        30000U,
        8.0f);
    assert(s_tx_length == 0U);

    test_reset_tx();
    Protocol_Tjc_SendDynamicRunResult(
        (uint8_t) DYNAMIC_RUN_MODE_AB,
        (uint8_t) DYNAMIC_RUN_RESULT_PASSED,
        6500U,
        5.0f);
    assert(test_tx_contains("page0.t0.txt=\"") != 0U);

    test_reset_tx();
    test_feed(shuttle_query_frame,
              (uint16_t) sizeof(shuttle_query_frame));
    assert(test_tx_contains("page3.n0.val=300") != 0U);
    assert(test_tx_contains("page3.n1.val=600") != 0U);
    assert(test_tx_contains("page3.n2.val=300") != 0U);

    test_reset_tx();
    test_feed(shuttle_save_frame,
              (uint16_t) sizeof(shuttle_save_frame));
    assert(s_save_count == 1U);
    assert(s_last_saved_times_ms[0] == 700U);
    assert(s_last_saved_times_ms[1] == 848U);
    assert(s_last_saved_times_ms[2] == 545U);
    assert(test_tx_contains("page3.n0.val=700") != 0U);
    assert(test_tx_contains("page3.n1.val=848") != 0U);
    assert(test_tx_contains("page3.n2.val=545") != 0U);
    assert(test_tx_contains("page3.t0.txt=\"") != 0U);

    test_reset_tx();
    test_feed(enter_frame, (uint16_t) sizeof(enter_frame));
    assert(s_start_adjust_count == 1U);
    assert(test_tx_contains("page2.n0.val=1070") != 0U);
    assert(test_tx_contains("page2.t0.txt=\"") != 0U);

    test_reset_tx();
    test_feed(set_frame, (uint16_t) sizeof(set_frame));
    assert(s_last_set_pwm_us == 1150U);
    assert(s_zero_adjust_active != 0U);
    assert(test_tx_contains("page2.n0.val=1150") != 0U);

    test_reset_tx();
    test_feed(confirm_frame, (uint16_t) sizeof(confirm_frame));
    assert(s_last_confirm_pwm_us == 1150U);
    assert(s_zero_adjust_active == 0U);
    assert(test_tx_contains("page2.n0.val=1150") != 0U);

    puts("TJC页面协议与未达标提示抑制主机测试通过");
    return 0;
}
