# -*- coding: utf-8 -*-
"""
标定表数据层：点数据、多轮合并、线性插值（正向/反向）、JSON/C 头文件导出、验证统计。
"""

import bisect
import csv
import json
import math
import os
from dataclasses import dataclass, field
from datetime import datetime


@dataclass
class CalibPoint:
    pwm_us: int
    angle_deg: float
    std_deg: float
    direction: str  # 'up' 升序扫描 / 'down' 降序扫描
    ts_ms: int = 0


def interp1d(xs, ys, x):
    """单调递增 xs 上的线性插值，端点外钳位。"""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x) - 1
    x0, x1 = xs[i], xs[i + 1]
    if x1 == x0:
        return ys[i]
    t = (x - x0) / (x1 - x0)
    return ys[i] + (ys[i + 1] - ys[i]) * t


def write_round_csv(path, meta, points):
    """每轮数据存一个 CSV，文件头写入 # 注释元信息。"""
    with open(path, "w", newline="", encoding="utf-8") as f:
        for k, v in meta.items():
            f.write(f"# {k}: {v}\n")
        w = csv.writer(f)
        w.writerow(["pwm_us", "angle_deg", "std_deg", "direction", "ts_ms"])
        for p in points:
            w.writerow([p.pwm_us, f"{p.angle_deg:.4f}", f"{p.std_deg:.4f}", p.direction, p.ts_ms])


def read_round_csv(path):
    meta = {}
    points = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("# "):
                k, _, v = line[2:].strip().partition(": ")
                meta[k] = v
            else:
                break
    with open(path, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            points.append(
                CalibPoint(
                    pwm_us=int(row["pwm_us"]),
                    angle_deg=float(row["angle_deg"]),
                    std_deg=float(row["std_deg"]),
                    direction=row["direction"],
                    ts_ms=int(row["ts_ms"]),
                )
            )
    return meta, points


class CalibTable:
    """标定结果表。pwms 升序；up/down/mean 对齐；mean 用于前馈，up/down 用于回差分析。"""

    def __init__(self, pwms, up, down, mean, meta=None):
        self.pwms = list(pwms)
        self.up = list(up)      # 升序扫描均值（无数据时为 None）
        self.down = list(down)  # 降序扫描均值
        self.mean = list(mean)  # up/down 平均（单方向时取可用者）
        self.meta = meta or {}
        self.hysteresis = [
            abs(d - u) if (u is not None and d is not None) else 0.0
            for u, d in zip(self.up, self.down)
        ]
        # 反向插值表（按角度升序）
        self._inv = {
            "up": sorted((a, p) for p, a in zip(self.pwms, self.up) if a is not None),
            "down": sorted((a, p) for p, a in zip(self.pwms, self.down) if a is not None),
            "mean": sorted((a, p) for p, a in zip(self.pwms, self.mean) if a is not None),
        }

    @property
    def hysteresis_min(self):
        return min(self.hysteresis) if self.hysteresis else 0.0

    @property
    def hysteresis_max(self):
        return max(self.hysteresis) if self.hysteresis else 0.0

    def pwm_to_angle(self, pwm):
        return interp1d(self.pwms, self.mean, float(pwm))

    def angle_to_pwm(self, angle_deg, direction="mean"):
        """目标角度 -> PWM。direction: 'mean' | 'up' | 'down'。"""
        pairs = self._inv.get(direction) or self._inv["mean"]
        if not pairs:
            raise ValueError("标定表为空")
        xs = [a for a, _ in pairs]
        ys = [p for _, p in pairs]
        return int(round(interp1d(xs, ys, float(angle_deg))))

    def to_dict(self):
        return {
            "meta": self.meta,
            "points": [
                [p, u, d, m, h]
                for p, u, d, m, h in zip(self.pwms, self.up, self.down, self.mean, self.hysteresis)
            ],
        }

    def save_json(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)

    @classmethod
    def load_json(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        pwms, up, down, mean = [], [], [], []
        for row in d["points"]:
            p, u, dn, m = row[0], row[1], row[2], row[3]
            pwms.append(p)
            up.append(u)
            down.append(dn)
            mean.append(m)
        return cls(pwms, up, down, mean, d.get("meta", {}))

    def export_c(self, path):
        """导出嵌入式端可直接使用的 C 数组（角度升序 + 三套 PWM）。"""
        inv = sorted(zip(self.mean, self.pwms))
        ang = [a for a, _ in inv]
        pwm = [p for _, p in inv]
        # 在统一的平均角度网格上分别反插值升序和降序PWM，
        # 不能把up/down角度值误写入uint16_t PWM数组。
        up_arr = [self.angle_to_pwm(a, "up") for a in ang]
        down_arr = [self.angle_to_pwm(a, "down") for a in ang]

        def fmt(vals, width=8):
            return ", ".join(f"{v:>{width}}" for v in vals)

        lines = [
            "/* 摆杆角度标定表 - 由上位机自动生成，请勿手改 */",
            f"/* 生成时间: {datetime.now().isoformat(timespec='seconds')} */",
            f"/* 点数: {len(ang)}  回差: {self.hysteresis_min:.3f}~{self.hysteresis_max:.3f} deg */",
            "",
            f"#define CALIB_TABLE_POINTS {len(ang)}",
            "",
            "/* 角度（度，升序） */",
            f"static const float calib_angle_deg[CALIB_TABLE_POINTS] = {{ {fmt(ang)} }};",
            "",
            "/* 平均表：角度 -> PWM (us) */",
            f"static const uint16_t calib_pwm_mean_us[CALIB_TABLE_POINTS] = {{ {fmt(pwm)} }};",
            "",
            "/* 升序扫描表（可选，用于方向感知前馈） */",
            f"static const uint16_t calib_pwm_up_us[CALIB_TABLE_POINTS] = {{ {fmt(up_arr)} }};",
            "",
            "/* 降序扫描表（可选） */",
            f"static const uint16_t calib_pwm_down_us[CALIB_TABLE_POINTS] = {{ {fmt(down_arr)} }};",
            "",
        ]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))


def build_table(rounds, meta=None):
    """
    合并多轮标定数据。
    rounds: list[list[CalibPoint]]，每轮内可能包含 up/down 两个方向。
    同一 PWM 同方向跨轮取均值。
    """
    up_map, down_map = {}, {}
    for pts in rounds:
        for p in pts:
            m = up_map if p.direction == "up" else down_map
            m.setdefault(p.pwm_us, []).append(p.angle_deg)
    pwms = sorted(set(up_map) | set(down_map))
    up, down, mean = [], [], []
    for p in pwms:
        u = sum(up_map[p]) / len(up_map[p]) if p in up_map else None
        d = sum(down_map[p]) / len(down_map[p]) if p in down_map else None
        if u is not None and d is not None:
            m = (u + d) / 2.0
        else:
            m = u if u is not None else d
        up.append(u)
        down.append(d)
        mean.append(m)
    return CalibTable(pwms, up, down, mean, meta=meta)


def recommended_grid(max_angle=10.0, near_step=0.5, near_radius=3.0):
    """推荐目标角度网格：零位附近加密，远处放宽。"""
    angles = {0.0}
    a = near_step
    while a <= near_radius + 1e-9:
        angles.add(round(a, 4))
        angles.add(round(-a, 4))
        a += near_step
    a = math.ceil(near_radius / 2.0) * 2.0 + 2.0
    while a < max_angle - 1e-9:
        angles.add(round(a, 4))
        angles.add(round(-a, 4))
        a += 2.0
    angles.add(float(max_angle))
    angles.add(float(-max_angle))
    return sorted(angles)


def validation_stats(results):
    """results: list[dict(target, actuals:list)] -> 汇总统计。"""
    errs = []
    for r in results:
        for a in r["actuals"]:
            errs.append(a - r["target"])
    if not errs:
        return {"n": 0, "mean": 0.0, "max_abs": 0.0, "std": 0.0, "rms": 0.0}
    n = len(errs)
    mean = sum(errs) / n
    var = sum((e - mean) ** 2 for e in errs) / n
    return {
        "n": n,
        "mean": mean,
        "max_abs": max(abs(e) for e in errs),
        "std": math.sqrt(var),
        "rms": math.sqrt(sum(e * e for e in errs) / n),
    }


def write_validation_csv(path, results):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["target_deg", "pwm_us", "actual1_deg", "actual2_deg", "actual_mean_deg",
                    "err_mean_deg", "err_max_abs_deg"])
        for r in results:
            actuals = r["actuals"]
            am = sum(actuals) / len(actuals)
            w.writerow([
                f"{r['target']:.4f}", r["pwm"],
                *[f"{a:.4f}" for a in actuals],
                f"{am:.4f}",
                f"{am - r['target']:.4f}",
                f"{max(abs(a - r['target']) for a in actuals):.4f}",
            ])
