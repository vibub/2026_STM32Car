# -*- coding: utf-8 -*-
"""
摆杆角度标定 —— 串口协议的机器可读常量与解析函数。

协议权威定义见 PROTOCOL.md（v1.2）；本文件只存放两端共用的常量，
改动协议时必须先改 PROTOCOL.md 再同步本文件。
"""

CMD_PING = "PING"
CMD_SERVO = "SERVO"
CMD_PWM = "PWM"
CMD_IMU = "IMU"
CMD_SAMPLE = "SAMPLE"
CMD_STREAM = "STREAM"
CMD_PID = "PID"
CMD_BALANCE = "BALANCE"
CMD_TARGET = "TARGET"
CMD_TUNE_STREAM = "TSTREAM"
CMD_VIN = "VIN"

RESP_PONG = "PONG"
RESP_OK = "OK"
RESP_ERR = "ERR"
RESP_IMU = "IMU"
RESP_SAMPLE_OK = "SAMPLE_OK"
RESP_PWM = "PWM"
RESP_VIN = "VIN"
RESP_STREAM = "S"
RESP_PID = "PID"
RESP_TUNE_STREAM = "T"

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


def parse_pid(line):
    """'PID <kp> <ki> <kd> <pipe_kp> <pipe_kd_s>' -> 五个增益。"""
    parts = line.split()
    if len(parts) != 6 or parts[0] != RESP_PID:
        raise ValueError(f"PID 响应格式错误: {line!r}")
    return tuple(float(value) for value in parts[1:])


def parse_tune_stream(line):
    """解析20 Hz钢球平衡PID遥测行，返回便于绘图和保存的字典。"""
    parts = line.split()
    if len(parts) != 17 or parts[0] != RESP_TUNE_STREAM:
        raise ValueError(f"PID 遥测格式错误: {line!r}")
    return {
        "ts_ms": int(parts[1]),
        "flags": int(parts[2]),
        "phase": int(parts[3]),
        "target_mm": float(parts[4]),
        "position_mm": float(parts[5]),
        "velocity_mm_s": float(parts[6]),
        "error_mm": float(parts[7]),
        "p_term_deg": float(parts[8]),
        "i_term_deg": float(parts[9]),
        "d_term_deg": float(parts[10]),
        "desired_pipe_pitch_deg": float(parts[11]),
        "measured_pipe_pitch_deg": float(parts[12]),
        "measured_pipe_rate_dps": float(parts[13]),
        "pipe_p_term_deg": float(parts[14]),
        "pipe_d_term_deg": float(parts[15]),
        "servo_pwm_us": int(parts[16]),
    }
