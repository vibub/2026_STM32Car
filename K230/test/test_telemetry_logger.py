import pathlib
import sys
import tempfile


K230_DIR = pathlib.Path(__file__).resolve().parents[1]
if str(K230_DIR) not in sys.path:
    sys.path.insert(0, str(K230_DIR))

from telemetry_logger import (  # noqa: E402
    FRAME_SIZE,
    TelemetryRunLogger,
    TelemetryStreamParser,
    crc16_ccitt,
)


def put_u16(frame, offset, value):
    frame[offset] = value & 0xFF
    frame[offset + 1] = (value >> 8) & 0xFF


def put_u32(frame, offset, value):
    put_u16(frame, offset, value & 0xFFFF)
    put_u16(frame, offset + 2, value >> 16)


def build_frame(frame_type=2, sequence=7):
    frame = bytearray(FRAME_SIZE)
    frame[0:2] = b"\xA5\x5A"
    frame[2] = 1
    frame[3] = frame_type
    frame[4] = FRAME_SIZE
    put_u16(frame, 6, sequence)
    put_u32(frame, 8, 123456)
    frame[12] = 2
    frame[13] = 2
    put_u16(frame, 14, 0x00FF)
    put_u16(frame, 16, 2600)
    put_u16(frame, 18, 3000)
    put_u16(frame, 20, 14)
    put_u16(frame, 22, 1360)
    put_u16(frame, 24, 1280)
    put_u16(frame, 26, 1340)
    put_u16(frame, 28, (-25) & 0xFFFF)
    put_u16(frame, 30, 120)
    put_u16(frame, 32, (-350) & 0xFFFF)
    put_u16(frame, 34, 68)
    put_u16(frame, 36, (-40) & 0xFFFF)
    put_u16(frame, 38, 3050)
    put_u16(frame, 40, 120)
    put_u16(frame, 42, crc16_ccitt(frame[:42]))
    return bytes(frame)


def test_fragmented_frame():
    parser = TelemetryStreamParser()
    raw = build_frame()
    assert parser.feed(raw[:13]) == []
    frames = parser.feed(raw[13:])
    assert len(frames) == 1
    frame = frames[0]
    assert frame["sequence"] == 7
    assert frame["timestamp_ms"] == 123456
    assert frame["speed_cm_s"] == 26.0
    assert frame["yaw_rate_dps"] == 30.0
    assert frame["ball_error_mm"] == -2.5
    assert frame["wall_pitch_deg"] == -0.04


def test_resynchronizes_after_noise_and_bad_crc():
    parser = TelemetryStreamParser()
    bad = bytearray(build_frame(sequence=8))
    bad[20] ^= 0x10
    good = build_frame(sequence=9)
    frames = parser.feed(b"noise" + bytes(bad) + good)
    assert len(frames) == 1
    assert frames[0]["sequence"] == 9
    assert parser.crc_error_count >= 1
    assert parser.discarded_byte_count >= 5


def test_run_is_written_only_after_end():
    parser = TelemetryStreamParser()
    with tempfile.TemporaryDirectory() as directory:
        logger = TelemetryRunLogger(directory)
        start = parser.feed(build_frame(frame_type=1, sequence=1))[0]
        sample = parser.feed(build_frame(frame_type=2, sequence=2))[0]
        end = parser.feed(build_frame(frame_type=3, sequence=3))[0]
        assert logger.handle(start, receive_ms=100) is None
        assert logger.handle(sample, receive_ms=150) is None
        assert list(pathlib.Path(directory).glob("*.csv")) == []
        saved = logger.handle(end, receive_ms=200)
        assert saved is not None
        lines = pathlib.Path(saved).read_text().splitlines()
        assert len(lines) == 4
        assert lines[0].startswith("type,sequence,timestamp_ms")


if __name__ == "__main__":
    test_fragmented_frame()
    test_resynchronizes_after_noise_and_bad_crc()
    test_run_is_written_only_after_end()
    print("telemetry logger tests passed")
