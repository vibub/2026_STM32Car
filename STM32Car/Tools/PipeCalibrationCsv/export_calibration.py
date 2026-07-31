#!/usr/bin/env python3
"""Convert IAR memory exports of the 50-point pipe calibration to CSV."""

from __future__ import annotations

import argparse
import csv
import math
import struct
from pathlib import Path


DEFAULT_POINT_COUNT = 50
SERVO_PERIOD_US = 20_000.0
SERVO_MIN_PULSE_US = 500.0
SERVO_PULSE_SPAN_US = 2_000.0
SERVO_ANGLE_SPAN_DEG = 270.0


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


def _read_memory_export(
    path: Path, expected_size: int, description: str
) -> bytes:
    file_data = path.read_bytes()
    if len(file_data) == expected_size:
        data = file_data
    else:
        stripped_data = file_data.lstrip()
        try:
            text = file_data.decode("ascii")
        except UnicodeDecodeError as error:
            raise ValueError(f"无法识别 {description} 文件格式：{path}") from error
        if stripped_data.startswith(b"S"):
            data = _parse_motorola_srecord(text)
        elif stripped_data.startswith(b":"):
            data = _parse_intel_hex(text)
        else:
            raise ValueError(
                f"无法识别 {description} 文件格式；请使用 Motorola、"
                f"Intel HEX 或 Raw Binary：{path}"
            )

    if len(data) != expected_size:
        raise ValueError(
            f"{description} 文件大小应为 {expected_size} 字节，"
            f"解析后实际为 {len(data)} 字节：{path}"
        )
    return data


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "把 IAR 从 g_app_pipe_mapping_pwm_us 和 "
            "g_app_pipe_mapping_pitch_deg 导出的 Motorola、Intel HEX "
            "或原始二进制数据转成 CSV。"
        )
    )
    parser.add_argument("pwm_file", type=Path, help="50 个 uint16_t PWM(us)，共 100 字节")
    parser.add_argument("imu_file", type=Path, help="50 个 float32 IMU 角度，共 200 字节")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("pipe_calibration_50.csv"),
        help="输出 CSV 路径",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=DEFAULT_POINT_COUNT,
        help="采样点数量，默认 50",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.count <= 1:
        raise ValueError("采样点数量必须大于 1")

    pwm_data = _read_memory_export(args.pwm_file, args.count * 2, "PWM")
    imu_data = _read_memory_export(args.imu_file, args.count * 4, "IMU")
    pwm_values = struct.unpack(f"<{args.count}H", pwm_data)
    imu_values = struct.unpack(f"<{args.count}f", imu_data)

    for index, pitch_deg in enumerate(imu_values):
        if not math.isfinite(pitch_deg):
            raise ValueError(f"第 {index} 点 IMU 角度不是有限数：{pitch_deg!r}")
    for index, pulse_us in enumerate(pwm_values):
        if pulse_us < 500 or pulse_us > 2500:
            raise ValueError(f"第 {index} 点 PWM 超出 500~2500 us：{pulse_us}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8-sig") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(
            ["index", "pwm_us", "duty_percent", "servo_angle_deg", "imu_pitch_deg"]
        )
        for index, (pulse_us, pitch_deg) in enumerate(zip(pwm_values, imu_values)):
            duty_percent = pulse_us / SERVO_PERIOD_US * 100.0
            servo_angle_deg = (
                (pulse_us - SERVO_MIN_PULSE_US)
                / SERVO_PULSE_SPAN_US
                * SERVO_ANGLE_SPAN_DEG
            )
            writer.writerow(
                [
                    index,
                    pulse_us,
                    f"{duty_percent:.4f}",
                    f"{servo_angle_deg:.4f}",
                    f"{pitch_deg:.7f}",
                ]
            )

    print(f"已生成 {args.count} 点 CSV：{args.output.resolve()}")


if __name__ == "__main__":
    main()
