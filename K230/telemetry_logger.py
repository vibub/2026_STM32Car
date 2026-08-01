"""STM32 ABCD telemetry receiver and deferred SD-card logger."""

import os


FRAME_SYNC = b"\xA5\x5A"
FRAME_VERSION = 1
FRAME_SIZE = 44
TYPE_RUN_START = 1
TYPE_SAMPLE = 2
TYPE_RUN_END = 3


def _u16(data, offset):
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def _i16(data, offset):
    value = _u16(data, offset)
    return value - 65536 if value >= 32768 else value


def _u32(data, offset):
    return (
        int(data[offset])
        | (int(data[offset + 1]) << 8)
        | (int(data[offset + 2]) << 16)
        | (int(data[offset + 3]) << 24)
    )


def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= int(value) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_frame(data):
    if len(data) != FRAME_SIZE:
        return None
    if bytes(data[0:2]) != FRAME_SYNC:
        return None
    if int(data[2]) != FRAME_VERSION or int(data[4]) != FRAME_SIZE:
        return None
    expected_crc = _u16(data, 42)
    if crc16_ccitt(data[:42]) != expected_crc:
        return None
    frame_type = int(data[3])
    if frame_type not in (TYPE_RUN_START, TYPE_SAMPLE, TYPE_RUN_END):
        return None
    return {
        "type": frame_type,
        "sequence": _u16(data, 6),
        "timestamp_ms": _u32(data, 8),
        "run_mode": int(data[12]),
        "run_state": int(data[13]),
        "flags": _u16(data, 14),
        "speed_cm_s": _i16(data, 16) / 100.0,
        "yaw_rate_dps": _i16(data, 18) / 100.0,
        "accel_x_g": _i16(data, 20) / 1000.0,
        "lateral_kinematic_cm_s2": _i16(data, 22) / 100.0,
        "lateral_imu_cm_s2": _i16(data, 24) / 100.0,
        "lateral_fused_cm_s2": _i16(data, 26) / 100.0,
        "ball_error_mm": _i16(data, 28) / 10.0,
        "ball_velocity_mm_s": _i16(data, 30) / 10.0,
        "pipe_command_deg": _i16(data, 32) / 1000.0,
        "wall_accel_cm_s2": _i16(data, 34) / 100.0,
        "wall_pitch_deg": _i16(data, 36) / 1000.0,
        "travelled_distance_cm": _i16(data, 38) / 10.0,
        "transport_feedback_pitch_deg": _i16(data, 40) / 1000.0,
    }


class TelemetryStreamParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_error_count = 0
        self.discarded_byte_count = 0

    def feed(self, data):
        frames = []
        if data:
            self.buffer.extend(data)
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0xA5 or self.buffer[1] != 0x5A:
                del self.buffer[0]
                self.discarded_byte_count += 1
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            candidate = bytes(self.buffer[:FRAME_SIZE])
            frame = decode_frame(candidate)
            if frame is None:
                del self.buffer[0]
                self.crc_error_count += 1
                continue
            del self.buffer[:FRAME_SIZE]
            frames.append(frame)
        return frames


class TelemetryRunLogger:
    COLUMNS = (
        "type", "sequence", "timestamp_ms", "k230_timestamp_ms",
        "run_mode", "run_state", "flags", "speed_cm_s",
        "yaw_rate_dps", "accel_x_g", "lateral_kinematic_cm_s2",
        "lateral_imu_cm_s2", "lateral_fused_cm_s2",
        "ball_error_mm", "ball_velocity_mm_s", "pipe_command_deg",
        "wall_accel_cm_s2", "wall_pitch_deg", "travelled_distance_cm",
        "transport_feedback_pitch_deg",
        "k230_ball_valid", "k230_ball_position_mm", "k230_ball_quality",
    )

    def __init__(self, log_dir="/sdcard/logs"):
        self.log_dir = log_dir
        self.active = False
        self.records = []
        self.saved_count = 0
        self.last_path = None

    def _append(self, frame, ball_result, receive_ms):
        record = dict(frame)
        record["k230_timestamp_ms"] = int(receive_ms)
        if ball_result is None:
            record["k230_ball_valid"] = 0
            record["k230_ball_position_mm"] = 0.0
            record["k230_ball_quality"] = 0
        else:
            record["k230_ball_valid"] = 1 if ball_result.valid else 0
            record["k230_ball_position_mm"] = (
                float(ball_result.position_d10_mm) / 10.0
            )
            record["k230_ball_quality"] = int(ball_result.quality)
        self.records.append(record)

    def handle(self, frame, ball_result=None, receive_ms=0):
        frame_type = frame["type"]
        if frame_type == TYPE_RUN_START:
            self.records = []
            self.active = True
            self._append(frame, ball_result, receive_ms)
            return None
        if not self.active:
            return None
        self._append(frame, ball_result, receive_ms)
        if frame_type == TYPE_RUN_END:
            self.active = False
            return self.save_csv()
        return None

    def _ensure_log_dir(self):
        try:
            os.stat(self.log_dir)
        except OSError:
            os.mkdir(self.log_dir)

    def _next_path(self):
        self._ensure_log_dir()
        run_mode = "abcd"
        if self.records:
            mode_value = int(self.records[0].get("run_mode", 2))
            run_mode = "ab" if mode_value == 1 else "abcd"
        for index in range(1, 10000):
            path = "%s/%s_%04d.csv" % (self.log_dir, run_mode, index)
            try:
                os.stat(path)
            except OSError:
                return path
        return "%s/%s_latest.csv" % (self.log_dir, run_mode)

    def save_csv(self):
        if not self.records:
            return None
        path = self._next_path()
        with open(path, "w") as output:
            output.write(",".join(self.COLUMNS) + "\n")
            for record in self.records:
                output.write(",".join(
                    str(record.get(column, "")) for column in self.COLUMNS
                ) + "\n")
        self.saved_count += 1
        self.last_path = path
        self.records = []
        return path
