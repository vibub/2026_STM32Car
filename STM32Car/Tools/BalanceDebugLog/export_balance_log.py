#!/usr/bin/env python3
"""把 IAR 导出的钢球平衡诊断日志 RAM 转换为 CSV。"""

from __future__ import annotations

import argparse
import csv
import struct
from pathlib import Path


LOG_MAGIC = 0x424C4F47
LOG_FORMAT_VERSION = 1
HEADER_FORMAT = "<8I"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
INTEGER_FORMAT = "IIiIIIII"
FLOAT_NAMES = [
    "target_mm",
    "raw_position_mm",
    "filtered_position_mm",
    "predicted_position_mm",
    "predicted_accel_mm_s2",
    "velocity_mm_s",
    "integral_mm_s",
    "error_mm",
    "p_term_deg",
    "i_term_deg",
    "d_term_deg",
    "brake_term_deg",
    "desired_pipe_pitch_deg",
    "command_pipe_pitch_deg",
    "pipe_command_rate_dps",
    "measured_pipe_pitch_deg",
    "measured_pipe_rate_dps",
    "pipe_pitch_error_deg",
    "pipe_p_term_deg",
    "pipe_d_term_deg",
    "minimum_pipe_pitch_deg",
    "maximum_pipe_pitch_deg",
    "feedforward_model_pitch_deg",
    "desired_model_pitch_deg",
    "servo_feedforward_deg",
    "desired_angle_deg",
    "command_angle_deg",
    "actual_servo_angle_deg",
    "raw_accel_pitch_deg",
    "accel_pitch_deg",
    "raw_pitch_deg",
    "raw_pitch_rate_dps",
    "imu_accel_y_g",
    "imu_accel_z_g",
    "imu_gyro_x_dps",
    "transport_command_accel_cm_s2",
    "transport_imu_disturbance_deg",
    "transport_feedforward_pitch_deg",
    "transport_feedback_pitch_deg",
    "phase_pitch_deg",
    "stop_distance_mm",
]
SAMPLE_FORMAT = INTEGER_FORMAT + f"{len(FLOAT_NAMES)}f"
SAMPLE_SIZE = struct.calcsize("<" + SAMPLE_FORMAT)
FLAG_COLUMNS = [
    (0, "enable_requested"),
    (1, "enabled"),
    (2, "enable_pending"),
    (3, "ball_sample_valid"),
    (4, "tracking_valid"),
    (5, "timed_out"),
    (6, "measurement_accepted"),
    (7, "attitude_valid"),
    (8, "imu_fault"),
    (9, "mapping_valid"),
    (10, "mapping_fault"),
    (11, "integral_active"),
    (12, "braking_active"),
    (13, "zero_crossed"),
    (14, "turnaround_freeze_active"),
    (15, "pipe_settled"),
    (16, "endpoint_hold_enabled"),
    (17, "transport_mode_enabled"),
    (18, "accel_trusted"),
    (19, "servo_override_active"),
    (20, "calibration_valid"),
]
STATE_NAMES = {
    0: "idle",
    1: "recording",
    2: "full",
    3: "stopped",
}


def _combine_records(records: list[tuple[int, bytes]]) -> bytes:
    if not records:
        raise ValueError("导出文件中没有数据记录")

    result = bytearray()
    expected_address = None
    for address, payload in sorted(records):
        if not payload:
            continue
        if expected_address is None:
            expected_address = address
        if address != expected_address:
            raise ValueError(
                f"导出数据不连续：期望地址 0x{expected_address:X}，"
                f"实际地址 0x{address:X}"
            )
        result.extend(payload)
        expected_address += len(payload)
    return bytes(result)


def _parse_motorola_srecord(text: str) -> bytes:
    address_sizes = {"1": 2, "2": 3, "3": 4}
    records = []
    for line_number, source_line in enumerate(text.splitlines(), 1):
        line = source_line.strip()
        if not line:
            continue
        if len(line) < 4 or line[0] != "S":
            raise ValueError(f"Motorola S-record 第 {line_number} 行格式错误")
        record_type = line[1]
        try:
            count = int(line[2:4], 16)
            record = bytes.fromhex(line[4:])
        except ValueError as error:
            raise ValueError(
                f"Motorola S-record 第 {line_number} 行包含非法十六进制数据"
            ) from error
        if len(record) != count:
            raise ValueError(f"Motorola S-record 第 {line_number} 行长度错误")
        if ((count + sum(record)) & 0xFF) != 0xFF:
            raise ValueError(f"Motorola S-record 第 {line_number} 行校验失败")
        if record_type not in address_sizes:
            continue
        address_size = address_sizes[record_type]
        if len(record) <= address_size:
            raise ValueError(f"Motorola S-record 第 {line_number} 行没有校验字节")
        address = int.from_bytes(record[:address_size], "big")
        records.append((address, record[address_size:-1]))
    return _combine_records(records)


def _parse_intel_hex(text: str) -> bytes:
    records = []
    base_address = 0
    for line_number, source_line in enumerate(text.splitlines(), 1):
        line = source_line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"Intel HEX 第 {line_number} 行格式错误")
        try:
            record = bytes.fromhex(line[1:])
        except ValueError as error:
            raise ValueError(
                f"Intel HEX 第 {line_number} 行包含非法十六进制数据"
            ) from error
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"Intel HEX 第 {line_number} 行长度错误")
        if (sum(record) & 0xFF) != 0:
            raise ValueError(f"Intel HEX 第 {line_number} 行校验失败")

        byte_count = record[0]
        offset = int.from_bytes(record[1:3], "big")
        record_type = record[3]
        payload = record[4 : 4 + byte_count]
        if record_type == 0x00:
            records.append((base_address + offset, payload))
        elif record_type == 0x02:
            base_address = int.from_bytes(payload, "big") << 4
        elif record_type == 0x04:
            base_address = int.from_bytes(payload, "big") << 16
        elif record_type == 0x01:
            break
    return _combine_records(records)


def _read_memory_export(path: Path) -> bytes:
    file_data = path.read_bytes()
    stripped_data = file_data.lstrip()
    if stripped_data.startswith(b"S") or stripped_data.startswith(b":"):
        try:
            text = file_data.decode("ascii")
        except UnicodeDecodeError as error:
            raise ValueError(f"无法识别导出文件格式：{path}") from error
        if stripped_data.startswith(b"S"):
            return _parse_motorola_srecord(text)
        return _parse_intel_hex(text)
    return file_data


def _decode_direction(state_word: int) -> int:
    value = (state_word >> 24) & 0xFF
    return value - 256 if value >= 128 else value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="把 g_balance_debug_log 的 IAR 内存导出转换为 CSV"
    )
    parser.add_argument("input", type=Path, help="IAR 导出的 RAM 文件")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="输出 CSV；默认与输入文件同名",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    output_path = args.output or args.input.with_suffix(".csv")
    data = _read_memory_export(args.input)
    if len(data) < HEADER_SIZE:
        raise ValueError(f"文件不足 {HEADER_SIZE} 字节，不包含完整日志头")

    (
        magic,
        version,
        sample_size,
        capacity,
        count,
        state,
        period_ms,
        buffer_size,
    ) = struct.unpack_from(HEADER_FORMAT, data, 0)

    if magic != LOG_MAGIC:
        raise ValueError(
            f"日志标识错误：读取到 0x{magic:08X}，期望 0x{LOG_MAGIC:08X}；"
            "请从 &g_balance_debug_log 起始地址导出"
        )
    if version != LOG_FORMAT_VERSION:
        raise ValueError(
            f"不支持日志版本 {version}，当前脚本只支持 {LOG_FORMAT_VERSION}"
        )
    if sample_size != SAMPLE_SIZE:
        raise ValueError(
            f"样本大小不匹配：固件为 {sample_size} 字节，脚本为 {SAMPLE_SIZE} 字节"
        )
    expected_buffer_size = HEADER_SIZE + capacity * sample_size
    if buffer_size != expected_buffer_size:
        raise ValueError(
            f"日志总大小错误：头部声明 {buffer_size}，按容量计算应为 "
            f"{expected_buffer_size}"
        )
    if len(data) < buffer_size:
        raise ValueError(
            f"导出文件不完整：需要 {buffer_size} 字节，实际只有 {len(data)} 字节"
        )
    if count > capacity:
        raise ValueError(f"有效样本数 {count} 超过容量 {capacity}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    integer_names = [
        "timestamp_ms",
        "imu_timestamp_ms",
        "raw_position_d10_mm",
        "ball_sequence",
        "ball_quality",
        "servo_pulse_us",
        "flags",
        "state_word",
    ]
    metadata_names = [
        "index",
        "elapsed_ms",
        *integer_names,
        *(name for _, name in FLAG_COLUMNS),
        "trajectory_phase",
        "moving_sample_count",
        "kick_attempt_count",
        "moving_direction",
    ]

    first_timestamp = None
    with output_path.open("w", newline="", encoding="utf-8-sig") as output_file:
        writer = csv.writer(output_file)
        writer.writerow([*metadata_names, *FLOAT_NAMES])
        for index in range(count):
            offset = HEADER_SIZE + index * sample_size
            values = struct.unpack_from("<" + SAMPLE_FORMAT, data, offset)
            integer_values = list(values[:8])
            float_values = values[8:]
            timestamp_ms = integer_values[0]
            flags = integer_values[6]
            state_word = integer_values[7]
            if first_timestamp is None:
                first_timestamp = timestamp_ms
            elapsed_ms = (timestamp_ms - first_timestamp) & 0xFFFFFFFF
            flag_values = [(flags >> bit) & 1 for bit, _ in FLAG_COLUMNS]
            state_values = [
                state_word & 0xFF,
                (state_word >> 8) & 0xFF,
                (state_word >> 16) & 0xFF,
                _decode_direction(state_word),
            ]
            writer.writerow(
                [
                    index,
                    elapsed_ms,
                    *integer_values,
                    *flag_values,
                    *state_values,
                    *(f"{value:.7g}" for value in float_values),
                ]
            )

    state_name = STATE_NAMES.get(state, f"unknown({state})")
    print(
        f"已生成 {count} 点 CSV：{output_path.resolve()}\n"
        f"采样周期：{period_ms} ms，日志状态：{state_name}，"
        f"导出缓冲区：{buffer_size} 字节"
    )


if __name__ == "__main__":
    main()
