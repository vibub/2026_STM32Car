#include "host_calibration_protocol.h"

#include "app_main.h"
#include "board_time.h"
#include "host_link_uart.h"
#include "imu_heading.h"
#include "servo_driver.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define HOST_PROTOCOL_LINE_BUFFER_SIZE  80U   /* 单行命令最大长度。 */
#define HOST_PROTOCOL_REPLY_BUFFER_SIZE 96U   /* 应答行缓冲大小。 */
#define HOST_PROTOCOL_NUMBER_BUFFER_SIZE 16U  /* 单个浮点数字符串缓冲大小。 */
#define HOST_PROTOCOL_VERSION           "pclink-1.0"
#define HOST_PROTOCOL_PWM_MIN_US        500U
#define HOST_PROTOCOL_PWM_MAX_US        2500U
#define HOST_PROTOCOL_SERVO_SPAN_DEG    270.0f
#define HOST_PROTOCOL_PULSE_SPAN_US     2000.0f
#define HOST_PROTOCOL_SAMPLE_COUNT_MAX  200U
#define HOST_PROTOCOL_SAMPLE_DT_MIN_MS  10U
#define HOST_PROTOCOL_SAMPLE_DT_MAX_MS  100U
#define HOST_PROTOCOL_STREAM_PERIOD_MS  100U
#define HOST_PROTOCOL_RAD_TO_DEG        57.2957795f

typedef struct {
    char data[HOST_PROTOCOL_LINE_BUFFER_SIZE];
    uint8_t length;
    uint8_t discarding;
    uint8_t stream_enabled;
    uint32_t last_stream_ms;
    uint32_t last_rx_overflow;
    uint32_t last_rx_error;
    uint16_t last_servo_pwm_us;
} HostCalibrationProtocolState;

static HostCalibrationProtocolState s_protocol;

volatile uint8_t g_host_calibration_enabled;
volatile uint32_t g_host_calibration_rx_byte_count;
volatile uint32_t g_host_calibration_valid_command_count;
volatile uint32_t g_host_calibration_invalid_command_count;
volatile uint32_t g_host_calibration_stream_count;
volatile char g_host_calibration_last_command[16];

static uint32_t HostCalib_Pow10(uint8_t decimals)
{
    static const uint32_t powers[] = {1U, 10U, 100U, 1000U, 10000U};

    if (decimals > 4U) {
        decimals = 4U;
    }
    return powers[decimals];
}

static uint16_t HostCalib_AppendU32(char *buffer, uint16_t size, uint32_t value)
{
    char digits[12];
    uint8_t count = 0U;
    uint16_t index;
    uint16_t written = 0U;

    if ((buffer == 0) || (size < 2U)) {
        return 0U;
    }
    if (value == 0U) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1U;
    }
    while (value != 0U) {
        digits[count] = (char) ('0' + (value % 10U));
        value /= 10U;
        count++;
    }
    for (index = count; index > 0U; index--) {
        if (written + 1U < size) {
            buffer[written] = digits[index - 1U];
            written++;
        }
    }
    buffer[written] = '\0';
    return written;
}

/* 不依赖 printf 浮点支持，手动把浮点按指定小数位转成字符串。 */
static void HostCalib_FormatFloat(char *buffer, uint16_t buffer_size,
                                  float value, uint8_t decimals)
{
    uint32_t scale;
    int64_t fixed;
    uint32_t magnitude;
    uint32_t integer_part;
    uint32_t fraction_part;
    uint16_t written = 0U;
    uint8_t digit;

    if ((buffer == 0) || (buffer_size < 2U)) {
        return;
    }
    scale = HostCalib_Pow10(decimals);
    if (value < 0.0f) {
        fixed = (int64_t) (value * (float) scale - 0.5f);
    } else {
        fixed = (int64_t) (value * (float) scale + 0.5f);
    }
    if (fixed < 0) {
        buffer[0] = '-';
        written = 1U;
        magnitude = (uint32_t) (-fixed);
    } else {
        magnitude = (uint32_t) fixed;
    }
    integer_part = magnitude / scale;
    fraction_part = magnitude % scale;
    written += HostCalib_AppendU32(buffer + written,
                                   (uint16_t) (buffer_size - written),
                                   integer_part);
    if (decimals > 0U) {
        if (written + 1U < buffer_size) {
            buffer[written] = '.';
            written++;
        }
        for (digit = decimals; digit > 0U; digit--) {
            uint32_t power = HostCalib_Pow10((uint8_t) (digit - 1U));
            if (written + 1U < buffer_size) {
                buffer[written] = (char) ('0' + (fraction_part / power) % 10U);
                written++;
            }
        }
    }
    buffer[written] = '\0';
}

static uint8_t HostCalib_ParseInt(const char *text, int32_t *value)
{
    const char *cursor = text;
    uint8_t negative = 0U;
    uint32_t magnitude = 0U;
    uint8_t digits = 0U;

    if ((text == 0) || (*text == '\0') || (value == 0)) {
        return 0U;
    }
    if (*cursor == '-') {
        negative = 1U;
        cursor++;
    } else if (*cursor == '+') {
        cursor++;
    }
    while ((*cursor >= '0') && (*cursor <= '9')) {
        if (magnitude > 214748364U) {
            return 0U;
        }
        magnitude = magnitude * 10U + (uint32_t) (*cursor - '0');
        digits++;
        cursor++;
    }
    if ((digits == 0U) || (*cursor != '\0')) {
        return 0U;
    }
    if (magnitude > 2147483647U) {
        return 0U;
    }
    *value = (negative != 0U) ? -(int32_t) magnitude : (int32_t) magnitude;
    return 1U;
}

static uint8_t HostCalib_ParseTwoPositiveInts(const char *text,
                                              uint32_t *first,
                                              uint32_t *second)
{
    const char *cursor = text;
    uint32_t value = 0U;
    uint32_t values[2];
    uint8_t count = 0U;
    uint8_t has_digits = 0U;

    if ((text == 0) || (first == 0) || (second == 0)) {
        return 0U;
    }
    while (*cursor != '\0') {
        if ((*cursor >= '0') && (*cursor <= '9')) {
            value = value * 10U + (uint32_t) (*cursor - '0');
            has_digits = 1U;
        } else if ((*cursor == ' ') || (*cursor == '\t')) {
            if (has_digits != 0U) {
                if (count >= 2U) {
                    return 0U;
                }
                values[count] = value;
                count++;
                value = 0U;
                has_digits = 0U;
            }
        } else {
            return 0U;
        }
        cursor++;
    }
    if (has_digits != 0U) {
        if (count >= 2U) {
            return 0U;
        }
        values[count] = value;
        count++;
    }
    if (count != 2U) {
        return 0U;
    }
    *first = values[0];
    *second = values[1];
    return 1U;
}

static void HostCalib_SplitLine(const char *line,
                                char *command, uint8_t command_size,
                                char *argument, uint8_t argument_size)
{
    uint16_t index = 0U;
    uint8_t command_length = 0U;
    uint8_t argument_length = 0U;
    uint8_t reading_command = 1U;

    command[0] = '\0';
    argument[0] = '\0';
    while ((line[index] != '\0') &&
           (index < HOST_PROTOCOL_LINE_BUFFER_SIZE)) {
        char ch = line[index];
        if (reading_command != 0U) {
            if ((ch == ' ') || (ch == '\t')) {
                if (command_length != 0U) {
                    /* 命令结束，后续全部作为参数。 */
                    reading_command = 0U;
                }
            } else {
                if (command_length + 1U < command_size) {
                    command[command_length] = ch;
                    command_length++;
                }
            }
        } else {
            /* 参数必须原样保留内部空格，例如 "SAMPLE 100 20"。 */
            if (argument_length + 1U < argument_size) {
                argument[argument_length] = ch;
                argument_length++;
            }
        }
        index++;
    }
    command[command_length] = '\0';
    argument[argument_length] = '\0';
}

static void HostCalib_SendLine(const char *line)
{
    uint16_t length = (uint16_t) strlen(line);

    HostLinkUart_SendData((const uint8_t *) line, length);
    HostLinkUart_SendData((const uint8_t *) "\r\n", 2U);
}

/* SAMPLE 会阻塞主循环，期间 ImuHeading_Poll 得不到执行。
 * 这里在循环内直接轮询 IMU 并现场计算原始加速度俯仰角，
 * 保证每个样本都是新的真实读数（否则 100 个样本全是同一个冻结值）。 */
static float HostCalib_ReadRawPitchDeg(void)
{
    ImuMotionSample sample;

    ImuHeading_Poll();
    if (ImuHeading_GetLatestSample(&sample) != 0U) {
        return atan2f(sample.accel_y_g, sample.accel_z_g) *
               HOST_PROTOCOL_RAD_TO_DEG;
    }
    return App_GetPipeRawAccelPitchDeg();
}

static uint8_t HostCalib_RequireCalibrationMode(void)
{
    if (App_GetHostCalibrationEnabled() == 0U) {
        HostCalib_SendLine("ERR disabled");
        return 0U;
    }
    return 1U;
}

static void HostCalib_ProcessSample(const char *argument)
{
    uint32_t count;
    uint32_t dt_ms;
    uint32_t index;
    double sum = 0.0;
    double sum_squared = 0.0;
    double variance;
    float mean;
    float std;
    char mean_text[HOST_PROTOCOL_NUMBER_BUFFER_SIZE];
    char std_text[HOST_PROTOCOL_NUMBER_BUFFER_SIZE];
    char reply[HOST_PROTOCOL_REPLY_BUFFER_SIZE];

    if (HostCalib_ParseTwoPositiveInts(argument, &count, &dt_ms) == 0U) {
        HostCalib_SendLine("ERR parse");
        g_host_calibration_invalid_command_count++;
        return;
    }
    if ((count == 0U) || (count > HOST_PROTOCOL_SAMPLE_COUNT_MAX) ||
        (dt_ms < HOST_PROTOCOL_SAMPLE_DT_MIN_MS) ||
        (dt_ms > HOST_PROTOCOL_SAMPLE_DT_MAX_MS)) {
        HostCalib_SendLine("ERR range");
        g_host_calibration_invalid_command_count++;
        return;
    }

    for (index = 0U; index < count; index++) {
        float pitch = HostCalib_ReadRawPitchDeg();

        sum += (double) pitch;
        sum_squared += (double) pitch * (double) pitch;
        if (index + 1U < count) {
            HAL_Delay(dt_ms);
        }
    }
    mean = (float) (sum / (double) count);
    variance = (sum_squared - sum * sum / (double) count) /
               (double) count;
    if (variance < 0.0) {
        variance = 0.0;
    }
    std = (float) sqrt(variance);

    HostCalib_FormatFloat(mean_text, sizeof(mean_text), mean, 4U);
    HostCalib_FormatFloat(std_text, sizeof(std_text), std, 4U);
    (void) snprintf(reply, sizeof(reply), "SAMPLE_OK %s %s %lu %lu",
                    mean_text, std_text, (unsigned long) count,
                    (unsigned long) Board_TimeMs());
    HostCalib_SendLine(reply);
    g_host_calibration_valid_command_count++;
}

static void HostCalib_ProcessLine(const char *line)
{
    char command[16];
    char argument[32];
    char reply[HOST_PROTOCOL_REPLY_BUFFER_SIZE];
    char number_text[HOST_PROTOCOL_NUMBER_BUFFER_SIZE];
    int32_t parsed;
    uint8_t enabled;
    float angle;
    float pitch;
    float rate;
    uint32_t ts;

    HostCalib_SplitLine(line, command, sizeof(command),
                        argument, sizeof(argument));
    if (command[0] == '\0') {
        return;
    }
    {
        uint8_t index;

        /* g_host_calibration_last_command 是 volatile 调试变量，
         * 不能直接传给 strncpy，逐字节写入以保持 volatile 语义。 */
        for (index = 0U;
             index < sizeof(g_host_calibration_last_command) - 1U;
             index++) {
            g_host_calibration_last_command[index] = command[index];
            if (command[index] == '\0') {
                break;
            }
        }
    }
    g_host_calibration_last_command[
        sizeof(g_host_calibration_last_command) - 1U] = '\0';

    if (strcmp(command, "PING") == 0) {
        (void) snprintf(reply, sizeof(reply), "PONG %s",
                        HOST_PROTOCOL_VERSION);
        HostCalib_SendLine(reply);
        g_host_calibration_valid_command_count++;
        return;
    }

    if (strcmp(command, "CAL") == 0) {
        if (HostCalib_ParseInt(argument, &parsed) == 0U) {
            HostCalib_SendLine("ERR parse");
            g_host_calibration_invalid_command_count++;
            return;
        }
        enabled = (parsed != 0) ? 1U : 0U;
        if (App_SetHostCalibrationEnabled(enabled) == 0U) {
            HostCalib_SendLine("ERR busy");
        } else {
            HostCalib_SendLine("OK");
        }
        g_host_calibration_valid_command_count++;
        return;
    }

    if (HostCalib_RequireCalibrationMode() == 0U) {
        g_host_calibration_invalid_command_count++;
        return;
    }

    if (strcmp(command, "SERVO") == 0) {
        if ((HostCalib_ParseInt(argument, &parsed) == 0U) ||
            (parsed < (int32_t) HOST_PROTOCOL_PWM_MIN_US) ||
            (parsed > (int32_t) HOST_PROTOCOL_PWM_MAX_US)) {
            HostCalib_SendLine("ERR range");
            g_host_calibration_invalid_command_count++;
            return;
        }
        s_protocol.last_servo_pwm_us = (uint16_t) parsed;
        angle = ((float) parsed - (float) HOST_PROTOCOL_PWM_MIN_US) /
                HOST_PROTOCOL_PULSE_SPAN_US * HOST_PROTOCOL_SERVO_SPAN_DEG;
        ServoDriver_SetAngle(angle);
        HostCalib_SendLine("OK");
        g_host_calibration_valid_command_count++;
        return;
    }

    if (strcmp(command, "PWM") == 0) {
        (void) snprintf(reply, sizeof(reply), "PWM %u",
                        (unsigned) s_protocol.last_servo_pwm_us);
        HostCalib_SendLine(reply);
        g_host_calibration_valid_command_count++;
        return;
    }

    if (strcmp(command, "IMU") == 0) {
        pitch = App_GetPipeRawAccelPitchDeg();
        rate = App_GetPipeRawPitchRateDps();
        ts = Board_TimeMs();
        HostCalib_FormatFloat(number_text, sizeof(number_text), pitch, 4U);
        (void) snprintf(reply, sizeof(reply), "IMU %s", number_text);
        HostCalib_FormatFloat(number_text, sizeof(number_text), rate, 3U);
        (void) snprintf(reply + strlen(reply),
                        sizeof(reply) - strlen(reply), " %s %lu",
                        number_text, (unsigned long) ts);
        HostCalib_SendLine(reply);
        g_host_calibration_valid_command_count++;
        return;
    }

    if (strcmp(command, "SAMPLE") == 0) {
        HostCalib_ProcessSample(argument);
        return;
    }

    if (strcmp(command, "STREAM") == 0) {
        if (HostCalib_ParseInt(argument, &parsed) == 0U) {
            HostCalib_SendLine("ERR parse");
            g_host_calibration_invalid_command_count++;
            return;
        }
        if ((parsed != 0) && (parsed != 1)) {
            HostCalib_SendLine("ERR range");
            g_host_calibration_invalid_command_count++;
            return;
        }
        s_protocol.stream_enabled = (uint8_t) parsed;
        s_protocol.last_stream_ms = Board_TimeMs();
        HostCalib_SendLine("OK");
        g_host_calibration_valid_command_count++;
        return;
    }

    if (strcmp(command, "VIN") == 0) {
        HostCalib_SendLine("ERR unsupported");
        g_host_calibration_invalid_command_count++;
        return;
    }

    HostCalib_SendLine("ERR unknown");
    g_host_calibration_invalid_command_count++;
}

static void HostCalib_FeedByte(uint8_t byte)
{
    g_host_calibration_rx_byte_count++;
    if (byte == (uint8_t) '\r') {
        return;
    }
    if (byte == (uint8_t) '\n') {
        if (s_protocol.discarding != 0U) {
            s_protocol.length = 0U;
            s_protocol.discarding = 0U;
            return;
        }
        if (s_protocol.length == 0U) {
            return;
        }
        s_protocol.data[s_protocol.length] = '\0';
        HostCalib_ProcessLine(s_protocol.data);
        s_protocol.length = 0U;
        return;
    }
    if (s_protocol.discarding != 0U) {
        return;
    }
    if (s_protocol.length >= (HOST_PROTOCOL_LINE_BUFFER_SIZE - 1U)) {
        s_protocol.length = 0U;
        s_protocol.discarding = 1U;
        return;
    }
    s_protocol.data[s_protocol.length] = (char) byte;
    s_protocol.length++;
}

void HostCalibrationProtocol_Init(void)
{
    memset(&s_protocol, 0, sizeof(s_protocol));
    s_protocol.last_servo_pwm_us = 1500U;
    s_protocol.last_stream_ms = Board_TimeMs();
    s_protocol.last_rx_overflow = HostLinkUart_GetOverflowCount();
    s_protocol.last_rx_error = HostLinkUart_GetErrorCount();
    g_host_calibration_enabled = App_GetHostCalibrationEnabled();
    g_host_calibration_rx_byte_count = 0U;
    g_host_calibration_valid_command_count = 0U;
    g_host_calibration_invalid_command_count = 0U;
    g_host_calibration_stream_count = 0U;
    g_host_calibration_last_command[0] = '\0';
    HostLinkUart_FlushRx();
}

void HostCalibrationProtocol_Task(void)
{
    uint8_t byte;
    uint32_t overflow_count;
    uint32_t error_count;
    uint32_t now_ms;
    char number_text[HOST_PROTOCOL_NUMBER_BUFFER_SIZE];
    char reply[HOST_PROTOCOL_REPLY_BUFFER_SIZE];

    overflow_count = HostLinkUart_GetOverflowCount();
    error_count = HostLinkUart_GetErrorCount();
    if ((overflow_count != s_protocol.last_rx_overflow) ||
        (error_count != s_protocol.last_rx_error)) {
        s_protocol.length = 0U;
        s_protocol.discarding = 0U;
    }
    s_protocol.last_rx_overflow = overflow_count;
    s_protocol.last_rx_error = error_count;

    while (HostLinkUart_ReadByte(&byte) != 0U) {
        HostCalib_FeedByte(byte);
    }

    if ((s_protocol.stream_enabled != 0U) &&
        (App_GetHostCalibrationEnabled() != 0U)) {
        now_ms = Board_TimeMs();
        if ((uint32_t) (now_ms - s_protocol.last_stream_ms) >=
            HOST_PROTOCOL_STREAM_PERIOD_MS) {
            s_protocol.last_stream_ms = now_ms;
            HostCalib_FormatFloat(number_text, sizeof(number_text),
                                  App_GetPipeRawAccelPitchDeg(), 4U);
            (void) snprintf(reply, sizeof(reply), "S %lu %s",
                            (unsigned long) now_ms, number_text);
            HostCalib_FormatFloat(number_text, sizeof(number_text),
                                  App_GetPipeRawPitchRateDps(), 3U);
            (void) snprintf(reply + strlen(reply),
                            sizeof(reply) - strlen(reply), " %s",
                            number_text);
            HostCalib_SendLine(reply);
            g_host_calibration_stream_count++;
        }
    }
}
