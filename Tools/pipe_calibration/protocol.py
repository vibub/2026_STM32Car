# -*- coding: utf-8 -*-
"""
摆杆角度标定 —— 串口协议的机器可读常量与解析函数。

协议权威定义见 PROTOCOL.md（v1.1）；本文件只存放两端共用的常量，
改动协议时必须先改 PROTOCOL.md 再同步本文件。
"""

CMD_PING = "PING"
CMD_SERVO = "SERVO"
CMD_PWM = "PWM"
CMD_IMU = "IMU"
CMD_SAMPLE = "SAMPLE"
CMD_STREAM = "STREAM"
CMD_VIN = "VIN"

RESP_PONG = "PONG"
RESP_OK = "OK"
RESP_ERR = "ERR"
RESP_IMU = "IMU"
RESP_SAMPLE_OK = "SAMPLE_OK"
RESP_PWM = "PWM"
RESP_VIN = "VIN"
RESP_STREAM = "S"

PWM_MIN_US = 500
PWM_CENTER_US = 1050
PWM_MAX_US = 1600


def parse_imu(line):
    """'IMU <pitch_deg> <rate_deg_s> <ts_ms>' -> (pitch, rate, ts)"""
    parts = line.split()
    if len(parts) != 4:
        raise ValueError(f"IMU 响应格式错误: {line!r}")
    return float(parts[1]), float(parts[2]), int(parts[3])


def parse_sample(line):
    """'SAMPLE_OK <mean_deg> <std_deg> <n> <ts_ms>' -> (mean, std, n, ts)"""
    parts = line.split()
    if len(parts) != 5:
        raise ValueError(f"SAMPLE 响应格式错误: {line!r}")
    return float(parts[1]), float(parts[2]), int(parts[3]), int(parts[4])


def parse_pwm(line):
    """'PWM <pwm_us>' -> pwm_us"""
    parts = line.split()
    if len(parts) != 2:
        raise ValueError(f"PWM 响应格式错误: {line!r}")
    return int(parts[1])
