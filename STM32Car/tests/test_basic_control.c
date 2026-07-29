#include "k230_line_protocol.h"
#include "line_follow_control.h"
#include "wheel_speed_pid.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const char *s_rx_text;
static unsigned s_rx_index;
static uint32_t s_now_ms = 100U;

uint32_t Board_TimeMs(void)
{
    return s_now_ms;
}

uint8_t K230Uart_ReadByte(uint8_t *byte)
{
    if ((s_rx_text == NULL) || (s_rx_text[s_rx_index] == '\0')) {
        return 0U;
    }
    *byte = (uint8_t) s_rx_text[s_rx_index++];
    return 1U;
}

uint32_t K230Uart_GetRxByteCount(void)
{
    return 0U;
}

uint32_t K230Uart_GetOverflowCount(void)
{
    return 0U;
}

uint32_t K230Uart_GetErrorCount(void)
{
    return 0U;
}

static void test_protocol(void)
{
    K230LineFrame frame;

    assert(K230LineProtocol_Parse("L,1,20,100,80,2", &frame) != 0U);
    assert(frame.valid == 1U);
    assert(frame.error_x == 20);
    assert(frame.angle_d10 == 100);
    assert(frame.quality == 80U);
    assert(frame.direction_mask == 2U);
    assert(K230LineProtocol_Parse("L,0,1,0,80,0", &frame) == 0U);

    K230LineProtocol_Init();
    s_rx_text = "L,1,20,100,80,2\r\n";
    s_rx_index = 0U;
    K230LineProtocol_Poll();
    assert(K230LineProtocol_HasFrame() != 0U);
    assert(K230LineProtocol_TakeLatest(&frame) != 0U);
    assert(frame.error_x == 20);
}

static void test_line_control(void)
{
    LineFollowControl control;
    LineFollowOutput output;
    K230LineFrame frame = {1U, 0, 0, 90U, 2U};

    LineFollowControl_Init(&control);
    LineFollowControl_Update(&control, 0U, &frame, 1U, 1U, &output);
    assert(output.command_valid != 0U);
    assert(output.left_target_cm_s > 0.0f);
    assert(output.right_target_cm_s > 0.0f);

    frame.error_x = 60;
    LineFollowControl_Update(&control, 20U, &frame, 1U, 1U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);

    frame.valid = 0U;
    frame.error_x = 0;
    frame.angle_d10 = 0;
    LineFollowControl_Update(&control, 141U, &frame, 1U, 1U, &output);
    assert(output.command_valid == 0U);
}

static void test_speed_pid(void)
{
    WheelSpeedPid pid;

    WheelSpeedPid_Init(&pid);
    WheelSpeedPid_SetTarget(&pid, 10.0f);
    assert(WheelSpeedPid_Step(&pid, 0.0f) > 0.0f);
    assert(WheelSpeedPid_Step(&pid, 10.0f) <= 0.50f);
    WheelSpeedPid_SetTarget(&pid, 0.0f);
    assert(WheelSpeedPid_Step(&pid, 10.0f) == 0.0f);
}

int main(void)
{
    test_protocol();
    test_line_control();
    test_speed_pid();
    puts("基础巡线主机测试通过");
    return 0;
}
