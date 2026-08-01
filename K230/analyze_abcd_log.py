"""Summarize an ABCD CSV and estimate an initial wall-coupling gain."""

import csv
import sys


ACCEL_TO_DEG = 0.058405
FLAG_TURN_ACTIVE = 1 << 1
FLAG_BALL_VALID = 1 << 4


def _load_turn_samples(path):
    samples = []
    with open(path, "r", newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            if int(row["type"]) != 2:
                continue
            flags = int(row["flags"])
            if not (flags & FLAG_TURN_ACTIVE) or not (flags & FLAG_BALL_VALID):
                continue
            lateral = float(row["lateral_fused_cm_s2"])
            if abs(lateral) < 3.0:
                continue
            samples.append({
                "time": int(row["timestamp_ms"]),
                "lateral": lateral,
                "error": float(row["ball_error_mm"]),
                "feedback": float(row["transport_feedback_pitch_deg"]),
            })
    return samples


def _split_turns(samples):
    turns = []
    current = []
    for sample in samples:
        if current and sample["time"] - current[-1]["time"] > 300:
            turns.append(current)
            current = []
        current.append(sample)
    if current:
        turns.append(current)
    return turns


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def analyze(path):
    samples = _load_turn_samples(path)
    if not samples:
        raise RuntimeError("no valid turn samples in " + path)

    for index, turn in enumerate(_split_turns(samples), 1):
        errors = [sample["error"] for sample in turn]
        print(
            "turn {}: n={} mean_lat={:.3f} cm/s^2 "
            "mean_error={:.3f} mm max_abs_error={:.3f} mm "
            "mean_feedback={:.4f} deg".format(
                index,
                len(turn),
                _mean([sample["lateral"] for sample in turn]),
                _mean(errors),
                max(abs(value) for value in errors),
                _mean([sample["feedback"] for sample in turn]),
            )
        )

    numerator = sum(
        sample["lateral"] * sample["feedback"] for sample in samples
    )
    denominator = ACCEL_TO_DEG * sum(
        sample["lateral"] * sample["lateral"] for sample in samples
    )
    suggested_gain = -numerator / denominator if denominator else 0.0
    print("suggested APP_ABCD_WALL_COUPLING_GAIN={:.4f}".format(
        suggested_gain
    ))
    print("Use this only as a conservative initial value; verify with a second run.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python analyze_abcd_log.py abcd_0001.csv")
    analyze(sys.argv[1])
