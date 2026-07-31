#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摆杆角度标定上位机

三个功能区：
  基础区  —— 实时显示水管 IMU 俯仰角，手动控制舵机 PWM，记录零位/上下限；
  标定区  —— 设置扫描参数（起点/终点/方向/角度网格/稳定时间/采样数/轮数），
             逐轮采集并保存 CSV，多轮合并后构建标定表（升序/降序/平均 + 回差），
             导出 JSON 与嵌入式端 C 头文件；
  验证区  —— 输入目标角度，查表反插值得到 PWM 并控制到位，记录实际 IMU 角度，
             统计误差验证标定准确度。

通信协议见 PROTOCOL.md；串口默认 USART1 / 115200 8N1。
"""

import argparse
import csv
import json
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, ttk

import matplotlib

matplotlib.use("TkAgg")
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from links import LinkError, SerialLink
from protocol import (
    CMD_PING,
    CMD_SAMPLE,
    CMD_SERVO,
    CMD_STREAM,
    RESP_OK,
    RESP_PONG,
    RESP_SAMPLE_OK,
    parse_sample,
)
import table as tbl

APP_TITLE = "摆杆角度标定上位机"
PWM_MIN_US = 500
PWM_MAX_US = 2500
STREAM_PERIOD_S = 0.1
ROLLING_WINDOW_S = 15.0
DEFAULT_GRID_TEXT = "0, ±0.5, ±1, ±1.5, ±2, ±2.5, ±3, ±5, ±8, ±10"


def parse_angle_list(text):
    """解析角度网格文本：支持逗号/空格分隔与 ± 前缀展开。"""
    values = []
    for token in text.replace("，", ",").replace(";", ",").replace(" ", ",").split(","):
        token = token.strip()
        if not token:
            continue
        if token.startswith("±"):
            value = float(token[1:])
            values.extend([value, -value])
        else:
            values.append(float(token))
    return sorted(set(round(v, 4) for v in values))


def clamp_pwm(value):
    return max(PWM_MIN_US, min(PWM_MAX_US, int(round(value))))


class CalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("1320x880")
        self.link = None
        self.table = None
        self.job_q = queue.Queue()
        self.data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        os.makedirs(self.data_dir, exist_ok=True)

        self._build_connection_bar()
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=(6, 0))
        self.base_tab = BaseTab(self.notebook, self)
        self.calib_tab = CalibrationTab(self.notebook, self)
        self.validate_tab = ValidationTab(self.notebook, self)
        self.notebook.add(self.base_tab.frame, text="基础区")
        self.notebook.add(self.calib_tab.frame, text="标定区")
        self.notebook.add(self.validate_tab.frame, text="验证区")

        log_frame = ttk.LabelFrame(self.root, text="日志")
        log_frame.pack(fill="both", padx=6, pady=6)
        self.log_text = tk.Text(log_frame, height=9, state="disabled", wrap="none")
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_configure("error", foreground="#c62828")

        self._poll_count = 0
        self.root.after(50, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.log("上位机就绪。请连接串口后勾选“标定模式”开始操作。")

    # ---------------- 连接栏 ----------------
    def _build_connection_bar(self):
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=6, pady=(6, 0))

        ttk.Label(bar, text="串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            bar, textvariable=self.port_var, width=14, state="readonly"
        )
        self.port_combo.pack(side="left", padx=(2, 4))
        ttk.Button(bar, text="刷新", width=6, command=self.refresh_ports).pack(
            side="left"
        )

        ttk.Label(bar, text="波特率:").pack(side="left", padx=(10, 2))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(
            bar,
            textvariable=self.baud_var,
            values=("9600", "115200", "230400", "460800"),
            width=8,
            state="readonly",
        ).pack(side="left")

        self.connect_btn = ttk.Button(
            bar, text="连接", width=8, command=self.toggle_connect
        )
        self.connect_btn.pack(side="left", padx=(12, 4))

        self.cal_mode_var = tk.BooleanVar(value=False)
        self.cal_mode_check = ttk.Checkbutton(
            bar,
            text="标定模式 (CAL)",
            variable=self.cal_mode_var,
            command=self.toggle_cal_mode,
        )
        self.cal_mode_check.pack(side="left", padx=(14, 4))

        self.conn_status_var = tk.StringVar(value="未连接")
        ttk.Label(bar, textvariable=self.conn_status_var, foreground="#1565c0").pack(
            side="left", padx=10
        )
        self.refresh_ports()

    def refresh_ports(self):
        try:
            from serial.tools import list_ports

            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.link is not None:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self.log("请先选择串口。", error=True)
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = 115200
        self.log(f"正在连接 {port} @ {baud} ...")
        try:
            link = SerialLink(port, baud)
            link.open()
        except Exception as exc:
            self.log(f"串口打开失败: {exc}", error=True)
            self.conn_status_var.set("连接失败")
            return
        ok = False
        for _ in range(3):
            try:
                resp = link.request(CMD_PING, expect=RESP_PONG, timeout=1.0)
                self.log(f"PING -> {resp}")
                ok = True
                break
            except LinkError:
                time.sleep(0.1)
        if not ok:
            self.log("PING 失败：设备未应答，请确认固件已开启标定链路。", error=True)
            link.close()
            self.conn_status_var.set("PING 失败")
            return
        self.link = link
        self.connect_btn.config(text="断开")
        self.conn_status_var.set(f"已连接 {port} @ {baud}")
        self.log("串口已连接。")
        # 设备可能刚重启（标定模式默认关闭），若复选框仍勾选则自动重新进入标定模式
        if self.cal_mode_var.get():
            resp = self.send("CAL 1")
            if resp is not None:
                self.log("已自动进入标定模式（车辆已停车）")
                self.conn_status_var.set(
                    f"标定模式 ON | {port} @ {baud}"
                )
            else:
                self.cal_mode_var.set(False)

    def disconnect(self):
        if self.link is None:
            return
        try:
            if self.base_tab.stream_var.get():
                self.base_tab.set_stream(False)
            if self.cal_mode_var.get():
                self.send("CAL 0")
        except Exception:
            pass
        try:
            self.link.close()
        finally:
            self.link = None
        self.connect_btn.config(text="连接")
        self.conn_status_var.set("未连接")
        self.log("串口已断开。")

    def toggle_cal_mode(self):
        enabled = self.cal_mode_var.get()
        resp = self.send(f"CAL {1 if enabled else 0}")
        if resp is None:
            self.cal_mode_var.set(False)
            return
        self.log(("已进入标定模式（车辆已停车）" if enabled else "已退出标定模式"))
        self.conn_status_var.set(
            f"标定模式 {'ON' if enabled else 'OFF'} | "
            f"{self.port_var.get()} @ {self.baud_var.get()}"
        )

    # ---------------- 通用工具 ----------------
    def send(self, cmd, expect=None, timeout=2.0):
        if self.link is None:
            self.log("未连接设备。", error=True)
            return None
        try:
            return self.link.request(cmd, expect=expect, timeout=timeout)
        except LinkError as exc:
            message = str(exc)
            if "ERR disabled" in message:
                self.log(
                    "命令被拒绝：标定模式未开启，请勾选连接栏的“标定模式 (CAL)”。",
                    error=True,
                )
            else:
                self.log(f"[ERR] {message}", error=True)
            return None

    def log(self, message, error=False):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{stamp}] {message}\n")
        if error:
            end = self.log_text.index("end-1c")
            self.log_text.tag_add("error", f"{end} linestart", end)
        self.log_text.see("end")
        self.log_text.config(state="disabled")
        # 限制日志行数，避免长时间运行内存膨胀
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 2000:
            self.log_text.config(state="normal")
            self.log_text.delete("1.0", f"{lines - 1500}.0")
            self.log_text.config(state="disabled")

    def set_stream(self, enabled):
        resp = self.send(f"{CMD_STREAM} {1 if enabled else 0}")
        return resp is not None

    def _poll(self):
        try:
            if self.link is not None:
                lines = self.link.read_available()
                for line in lines:
                    self.base_tab.on_stream_line(line)
            while True:
                try:
                    message = self.job_q.get_nowait()
                except queue.Empty:
                    break
                self._handle_job_message(message)
            self._poll_count += 1
            if self._poll_count % 10 == 0:
                self.base_tab.refresh_realtime()
        finally:
            self.root.after(50, self._poll)

    def _handle_job_message(self, message):
        kind = message[0]
        if kind == "log":
            self.log(message[1], error=len(message) > 2 and message[2])
        elif kind == "progress":
            self.calib_tab.progress_var.set(max(0.0, min(1.0, message[1])))
        elif kind == "round_done":
            self.calib_tab.on_round_done(message[1])
        elif kind == "calib_done":
            self.calib_tab.on_calibration_done()
        elif kind == "validate_done":
            self.validate_tab.on_validation_done(message[1])
        elif kind == "result":
            target, pwm, actuals_text, err_mean, err_max = message[1]
            self.validate_tab.result_tree.insert(
                "",
                "end",
                values=(
                    f"{target:+.3f}",
                    pwm,
                    actuals_text,
                    f"{err_mean:+.4f}",
                    f"{err_max:.4f}",
                ),
            )
        elif kind == "err":
            self.log(f"[任务失败] {message[1]}", error=True)
            self.calib_tab.on_task_failed()
            self.validate_tab.on_task_failed()

    def _on_close(self):
        try:
            self.disconnect()
        except Exception:
            pass
        self.root.destroy()


class BaseTab:
    """基础区：实时 IMU + 手动舵机控制 + 零位/限位标记。"""

    def __init__(self, parent, app):
        self.app = app
        self.frame = ttk.Frame(parent)

        left = ttk.LabelFrame(self.frame, text="舵机控制")
        left.pack(side="left", fill="y", padx=(6, 3), pady=6)
        right = ttk.LabelFrame(self.frame, text="实时 IMU（水管俯仰角）")
        right.pack(side="left", fill="both", expand=True, padx=(3, 6), pady=6)

        # ---- PWM 控制 ----
        row = ttk.Frame(left)
        row.pack(fill="x", padx=6, pady=(8, 2))
        ttk.Label(row, text="PWM(us):").pack(side="left")
        self.pwm_var = tk.StringVar(value="1500")
        ttk.Spinbox(
            row,
            textvariable=self.pwm_var,
            from_=PWM_MIN_US,
            to=PWM_MAX_US,
            increment=1,
            width=8,
        ).pack(side="left", padx=4)
        ttk.Button(row, text="发送", width=6, command=self.send_pwm).pack(side="left")

        row = ttk.Frame(left)
        row.pack(fill="x", padx=6, pady=2)
        for delta in (-50, -10, -1, 1, 10, 50):
            ttk.Button(
                row,
                text=f"{delta:+d}",
                width=5,
                command=lambda d=delta: self.step_pwm(d),
            ).pack(side="left", padx=1)

        row = ttk.Frame(left)
        row.pack(fill="x", padx=6, pady=(10, 2))
        ttk.Label(row, text="角度(°):").pack(side="left")
        self.angle_var = tk.StringVar(value="0")
        ttk.Entry(row, textvariable=self.angle_var, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="发送角度", width=8, command=self.send_angle).pack(
            side="left"
        )
        ttk.Button(row, text="发送并测量", width=10, command=self.send_angle_measure).pack(
            side="left", padx=2
        )
        ttk.Label(left, text="（角度命令需先构建标定表）", foreground="#757575").pack(
            anchor="w", padx=6
        )

        row = ttk.Frame(left)
        row.pack(fill="x", padx=6, pady=(10, 2))
        ttk.Label(row, text="舵机角(°):").pack(side="left")
        self.servo_angle_var = tk.StringVar(value="122")
        ttk.Spinbox(
            row,
            textvariable=self.servo_angle_var,
            from_=0,
            to=270,
            increment=0.5,
            width=8,
        ).pack(side="left", padx=4)
        ttk.Button(row, text="发送", width=6, command=self.send_servo_angle).pack(
            side="left"
        )
        ttk.Label(left, text="（直接换算 PWM，无需标定表，用于找零位/限位）", foreground="#757575").pack(
            anchor="w", padx=6
        )

        # ---- 标记 ----
        ttk.Label(left, text="零位 / 限位标记", font=("", 10, "bold")).pack(
            anchor="w", padx=6, pady=(12, 2)
        )
        row = ttk.Frame(left)
        row.pack(fill="x", padx=6)
        ttk.Button(row, text="记录下限", command=lambda: self.record_marker("low")).pack(
            side="left"
        )
        ttk.Button(row, text="记录零位", command=lambda: self.record_marker("zero")).pack(
            side="left", padx=2
        )
        ttk.Button(row, text="记录上限", command=lambda: self.record_marker("high")).pack(
            side="left"
        )
        self.marker_list = tk.Listbox(left, height=5, width=34)
        self.marker_list.pack(fill="x", padx=6, pady=4)
        row = ttk.Frame(left)
        row.pack(fill="x", padx=6)
        ttk.Button(row, text="保存标记", command=self.save_markers).pack(side="left")
        ttk.Button(row, text="载入标记", command=self.load_markers).pack(side="left", padx=2)
        ttk.Button(row, text="清除", command=self.clear_markers).pack(side="left")
        self.markers = {"low": None, "zero": None, "high": None}

        # ---- 实时 IMU ----
        row = ttk.Frame(right)
        row.pack(fill="x", padx=8, pady=(8, 2))
        self.stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            row, text="实时显示 (STREAM 10Hz)", variable=self.stream_var,
            command=self.toggle_stream,
        ).pack(side="left")
        self.info_var = tk.StringVar(value="俯仰角: --   角速度: --   时间戳: --")
        ttk.Label(right, textvariable=self.info_var, font=("", 11)).pack(
            anchor="w", padx=8, pady=2
        )
        self.attitude_var = tk.StringVar(value="姿态有效: --")
        ttk.Label(right, textvariable=self.attitude_var, foreground="#1565c0").pack(
            anchor="w", padx=8
        )

        record_frame = ttk.LabelFrame(right, text="录制（静止稳定性分析）")
        record_frame.pack(fill="x", padx=8, pady=(4, 2))
        row = ttk.Frame(record_frame)
        row.pack(fill="x", padx=6, pady=4)
        self.record_start_btn = ttk.Button(
            row, text="开始录制", command=self.start_recording, width=10
        )
        self.record_start_btn.pack(side="left")
        self.record_stop_btn = ttk.Button(
            row,
            text="停止并保存",
            command=self.stop_recording,
            width=10,
            state="disabled",
        )
        self.record_stop_btn.pack(side="left", padx=4)
        self.record_plot_btn = ttk.Button(
            row,
            text="查看曲线",
            command=self.show_record_plot,
            width=9,
            state="disabled",
        )
        self.record_plot_btn.pack(side="left")
        self.record_status_var = tk.StringVar(value="未录制")
        ttk.Label(record_frame, textvariable=self.record_status_var).pack(
            side="left", padx=8
        )
        self.recording = False
        self.record_buffer = []
        self.record_start_host = 0.0
        self.last_record = None
        self._last_sent_pwm = 1500

        self.figure = Figure(figsize=(6, 3.2), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("水管俯仰角（最近 15 秒）")
        self.ax.set_xlabel("时间 (s)")
        self.ax.set_ylabel("俯仰角 (°)")
        self.ax.grid(True, alpha=0.3)
        (self.line,) = self.ax.plot([], [], "b-", linewidth=1.2)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)
        self.buffer = deque(maxlen=int(ROLLING_WINDOW_S / STREAM_PERIOD_S) + 4)
        self._last_draw = 0.0

    def toggle_stream(self):
        if self.app.link is None:
            self.stream_var.set(False)
            self.app.log("未连接设备。", error=True)
            return
        if self.app.set_stream(self.stream_var.get()):
            self.app.log(f"STREAM {'开' if self.stream_var.get() else '关'}")
        else:
            self.stream_var.set(False)

    def on_stream_line(self, line):
        parts = line.split()
        if len(parts) < 4 or not parts[0].startswith("S"):
            return
        try:
            ts = int(parts[1])
            pitch = float(parts[2])
            rate = float(parts[3])
        except ValueError:
            return
        now = time.monotonic()
        self.buffer.append((now, pitch))
        self.info_var.set(
            f"俯仰角: {pitch:+.4f}°   角速度: {rate:+.3f}°/s   时间戳: {ts} ms"
        )
        self.attitude_var.set("姿态有效: 收到实时帧")
        if self.recording:
            self.record_buffer.append(
                (
                    time.monotonic() - self.record_start_host,
                    ts,
                    self._last_sent_pwm,
                    pitch,
                    rate,
                )
            )
            elapsed = self.record_buffer[-1][0]
            self.record_status_var.set(
                f"录制中 {elapsed:.0f}s  样本 {len(self.record_buffer)}"
            )

    # ---- 录制 ----
    def start_recording(self):
        if self.app.link is None:
            self.app.log("未连接设备，无法录制。", error=True)
            return
        if not self.stream_var.get():
            if not self.app.set_stream(True):
                self.app.log("开启 STREAM 失败，无法录制。", error=True)
                return
            self.stream_var.set(True)
        self.recording = True
        self.record_buffer = []
        self.record_start_host = time.monotonic()
        self.record_start_btn.config(state="disabled")
        self.record_stop_btn.config(state="normal")
        self.record_status_var.set("录制中 0s  样本 0")
        self.app.log("录制开始（10Hz，记录 舵机PWM + IMU俯仰角/角速度）。")

    def stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        self.record_start_btn.config(state="normal")
        self.record_stop_btn.config(state="disabled")
        if not self.record_buffer:
            self.record_status_var.set("无数据，未保存")
            self.app.log("录制结束，但没有采集到数据。", error=True)
            return
        self.record_status_var.set("保存中...")
        path = self._save_record_csv()
        self._log_record_stats(path)
        self.record_plot_btn.config(state="normal")
        self.record_status_var.set("已保存")

    def _save_record_csv(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.app.data_dir, f"record_{stamp}.csv")
        duration = self.record_buffer[-1][0]
        with open(path, "w", newline="", encoding="utf-8") as f:
            f.write("# 摆杆角度录制数据\n")
            f.write(f"# created: {datetime.now().isoformat(timespec='seconds')}\n")
            f.write(f"# duration_s: {duration:.3f}\n")
            f.write(f"# samples: {len(self.record_buffer)}\n")
            w = csv.writer(f)
            w.writerow(["host_time_s", "device_ts_ms", "pwm_us",
                        "pitch_deg", "rate_deg_s"])
            for host_t, ts, pwm, pitch, rate in self.record_buffer:
                w.writerow([f"{host_t:.3f}", ts, pwm,
                            f"{pitch:.4f}", f"{rate:.3f}"])
        self.last_record = tuple(zip(*self.record_buffer))
        return path

    def _log_record_stats(self, path):
        pitches = [r[3] for r in self.record_buffer]
        n = len(pitches)
        mean = sum(pitches) / n
        var = sum((p - mean) ** 2 for p in pitches) / n
        std = var ** 0.5
        duration = self.record_buffer[-1][0]
        self.app.log(f"录制已保存: {path}（{duration:.1f}s, {n} 样本）")
        self.app.log(
            f"整体俯仰角: 均值 {mean:+.4f}°  标准差 {std:.4f}°  "
            f"范围 [{min(pitches):+.4f}, {max(pitches):+.4f}]°"
        )
        segments = {}
        for _, _, pwm, pitch, _ in self.record_buffer:
            segments.setdefault(pwm, []).append(pitch)
        if len(segments) > 1:
            self.app.log("各 PWM 段稳定性（样本数 / 均值 / 标准差）:")
            for pwm in sorted(segments):
                values = segments[pwm]
                m = sum(values) / len(values)
                s = (sum((v - m) ** 2 for v in values) / len(values)) ** 0.5
                self.app.log(
                    f"  PWM {pwm}us: n={len(values)}  均值 {m:+.4f}°  "
                    f"标准差 {s:.4f}°"
                )

    def show_record_plot(self):
        if self.last_record is None:
            return
        host_times, _ts, pwms, pitches, _rates = self.last_record
        window = tk.Toplevel(self.app.root)
        window.title("录制数据分析")
        window.geometry("900x520")
        figure = Figure(figsize=(9, 5), dpi=100)
        ax1 = figure.add_subplot(211)
        ax2 = figure.add_subplot(212, sharex=ax1)
        ax1.plot(host_times, pitches, "b-", linewidth=1.0, label="俯仰角")
        ax1.set_ylabel("俯仰角 (°)")
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=9)
        ax2.step(host_times, pwms, "r-", where="post", linewidth=1.0, label="舵机PWM")
        ax2.set_xlabel("时间 (s)")
        ax2.set_ylabel("PWM (us)")
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9)
        figure.tight_layout()
        canvas = FigureCanvasTkAgg(figure, master=window)
        canvas.get_tk_widget().pack(fill="both", expand=True)

    def refresh_realtime(self):
        if not self.buffer:
            return
        now = time.monotonic()
        cutoff = now - ROLLING_WINDOW_S
        while self.buffer and self.buffer[0][0] < cutoff:
            self.buffer.popleft()
        if now - self._last_draw < 0.4:
            return
        self._last_draw = now
        xs = [t - now for t, _ in self.buffer]
        ys = [p for _, p in self.buffer]
        self.line.set_data(xs, ys)
        self.ax.set_xlim(-ROLLING_WINDOW_S, 0)
        self.ax.set_ylim(min(ys) - 0.3, max(ys) + 0.3) if ys else None
        self.canvas.draw_idle()

    def _current_pwm(self):
        try:
            return clamp_pwm(int(float(self.pwm_var.get())))
        except ValueError:
            self.app.log("PWM 输入无效。", error=True)
            return None

    def send_pwm(self):
        pwm = self._current_pwm()
        if pwm is None:
            return
        resp = self.app.send(f"{CMD_SERVO} {pwm}", expect=RESP_OK)
        if resp is not None:
            self.app.log(f"SERVO {pwm}us -> OK")
            self.pwm_var.set(str(pwm))
            self._last_sent_pwm = pwm

    def send_servo_angle(self):
        """直接按舵机电气角(0~270°)换算 PWM 发送，用于找零位/上下限。"""
        try:
            angle = float(self.servo_angle_var.get())
        except ValueError:
            self.app.log("舵机角输入无效。", error=True)
            return
        if angle < 0.0 or angle > 270.0:
            self.app.log("舵机角范围应为 0~270°。", error=True)
            return
        pwm = clamp_pwm(500.0 + angle / 270.0 * 2000.0)
        self.pwm_var.set(str(pwm))
        self.send_pwm()

    def step_pwm(self, delta):
        try:
            current = int(float(self.pwm_var.get()))
        except ValueError:
            current = 1500
        self.pwm_var.set(str(clamp_pwm(current + delta)))
        self.send_pwm()

    def send_angle(self):
        if self.app.table is None:
            self.app.log("尚未构建/加载标定表，无法按角度发送。", error=True)
            return
        try:
            angle = float(self.angle_var.get())
        except ValueError:
            self.app.log("角度输入无效。", error=True)
            return
        pwm = self.app.table.angle_to_pwm(angle)
        self.pwm_var.set(str(pwm))
        self.send_pwm()

    def send_angle_measure(self):
        if self.app.table is None:
            self.app.log("尚未构建/加载标定表，无法按角度发送。", error=True)
            return

        def worker():
            try:
                angle = float(self.angle_var.get())
            except ValueError:
                self.app.job_q.put(("log", "角度输入无效。", True))
                return
            pwm = self.app.table.angle_to_pwm(angle)
            resp = self.app.send(f"{CMD_SERVO} {pwm}", expect=RESP_OK)
            if resp is None:
                return
            time.sleep(1.0)
            resp = self.app.send(
                f"{CMD_SAMPLE} 50 20",
                expect=RESP_SAMPLE_OK,
                timeout=4.0,
            )
            if resp is None:
                return
            mean, std, count, ts = parse_sample(resp)
            self.app.job_q.put(
                (
                    "log",
                    f"目标 {angle:+.2f}° -> PWM {pwm}us -> "
                    f"实测 {mean:+.4f}° (std {std:.4f}°, n={count})",
                )
            )

        threading.Thread(target=worker, daemon=True).start()

    # ---- 标记 ----
    def record_marker(self, kind):
        pwm = self._current_pwm()
        if pwm is None:
            return
        self.markers[kind] = pwm
        self._refresh_marker_list()
        self.app.log(f"标记 {kind}: PWM={pwm}us")

    def _refresh_marker_list(self):
        self.marker_list.delete(0, "end")
        labels = {"low": "下限", "zero": "零位", "high": "上限"}
        for kind, pwm in self.markers.items():
            self.marker_list.insert(
                "end", f"{labels[kind]}: {pwm if pwm is not None else '--'} us"
            )

    def clear_markers(self):
        self.markers = {"low": None, "zero": None, "high": None}
        self._refresh_marker_list()

    def save_markers(self):
        path = os.path.join(self.app.data_dir, "markers.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.markers, f, ensure_ascii=False, indent=2)
        self.app.log(f"标记已保存: {path}")

    def load_markers(self):
        path = os.path.join(self.app.data_dir, "markers.json")
        if not os.path.exists(path):
            self.app.log("没有已保存的标记文件。", error=True)
            return
        with open(path, "r", encoding="utf-8") as f:
            self.markers = json.load(f)
        self._refresh_marker_list()
        self.app.log(f"标记已载入: {path}")


class CalibrationTab:
    """标定区：参数化扫描、多轮采集、建表与导出。"""

    def __init__(self, parent, app):
        self.app = app
        self.frame = ttk.Frame(parent)
        self.rounds = []
        self.stop_event = threading.Event()
        self.worker = None

        params = ttk.LabelFrame(self.frame, text="扫描参数")
        params.pack(fill="x", padx=6, pady=6)

        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=2)
        self.mode_var = tk.StringVar(value="uniform")
        ttk.Radiobutton(row, text="均匀 PWM", variable=self.mode_var, value="uniform").pack(
            side="left"
        )
        ttk.Radiobutton(
            row, text="角度网格", variable=self.mode_var, value="angle_grid"
        ).pack(side="left", padx=8)

        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=2)
        ttk.Label(row, text="起点PWM:").pack(side="left")
        self.start_var = tk.StringVar(value="1200")
        ttk.Spinbox(row, textvariable=self.start_var, from_=PWM_MIN_US, to=PWM_MAX_US, width=7).pack(side="left", padx=2)
        ttk.Label(row, text="终点PWM:").pack(side="left")
        self.end_var = tk.StringVar(value="1800")
        ttk.Spinbox(row, textvariable=self.end_var, from_=PWM_MIN_US, to=PWM_MAX_US, width=7).pack(side="left", padx=2)
        ttk.Label(row, text="点数:").pack(side="left")
        self.num_points_var = tk.StringVar(value="21")
        ttk.Spinbox(row, textvariable=self.num_points_var, from_=2, to=200, width=5).pack(side="left", padx=2)
        ttk.Button(row, text="使用标记范围", command=self.use_marker_range).pack(
            side="left", padx=(10, 2)
        )
        ttk.Label(row, text="角度网格:").pack(side="left", padx=(10, 2))
        self.grid_var = tk.StringVar(value=DEFAULT_GRID_TEXT)
        ttk.Entry(row, textvariable=self.grid_var, width=42).pack(side="left")
        ttk.Button(row, text="推荐网格", command=self.fill_recommended_grid).pack(side="left", padx=4)

        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=2)
        ttk.Label(row, text="方向:").pack(side="left")
        self.direction_var = tk.StringVar(value="双向")
        ttk.Combobox(
            row, textvariable=self.direction_var, values=("双向", "仅升序", "仅降序"),
            width=8, state="readonly",
        ).pack(side="left", padx=2)
        self.flip_sign_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            row,
            text="翻转俯仰符号",
            variable=self.flip_sign_var,
            command=self._on_flip_sign_changed,
        ).pack(side="left", padx=8)
        self.flip_hint_var = tk.StringVar(value="")
        ttk.Label(row, textvariable=self.flip_hint_var, foreground="#b26a00").pack(
            side="left"
        )
        ttk.Label(row, text="稳定(s):").pack(side="left")
        self.settle_var = tk.StringVar(value="1.0")
        ttk.Spinbox(row, textvariable=self.settle_var, from_=0.1, to=10.0, increment=0.1, width=6).pack(side="left", padx=2)
        ttk.Label(row, text="采样数:").pack(side="left")
        self.sample_count_var = tk.StringVar(value="100")
        ttk.Spinbox(row, textvariable=self.sample_count_var, from_=1, to=200, width=6).pack(side="left", padx=2)
        ttk.Label(row, text="间隔(ms):").pack(side="left")
        self.sample_dt_var = tk.StringVar(value="20")
        ttk.Spinbox(row, textvariable=self.sample_dt_var, from_=10, to=100, width=5).pack(side="left", padx=2)
        ttk.Label(row, text="轮数:").pack(side="left")
        self.rounds_var = tk.StringVar(value="3")
        ttk.Spinbox(row, textvariable=self.rounds_var, from_=1, to=10, width=4).pack(side="left", padx=2)

        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=4)
        self.start_btn = ttk.Button(row, text="开始标定", command=self.start_calibration)
        self.start_btn.pack(side="left")
        self.stop_btn = ttk.Button(row, text="停止", command=self.stop_calibration, state="disabled")
        self.stop_btn.pack(side="left", padx=4)
        self.progress_var = tk.DoubleVar(value=0.0)
        ttk.Progressbar(row, variable=self.progress_var, maximum=1.0, length=260).pack(side="left", padx=8)
        self.status_var = tk.StringVar(value="未开始")
        ttk.Label(row, textvariable=self.status_var).pack(side="left")

        bottom = ttk.Frame(self.frame)
        bottom.pack(fill="both", expand=True, padx=6, pady=(0, 6))

        left = ttk.Frame(bottom)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(bottom)
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        rounds_frame = ttk.LabelFrame(left, text="已采集轮次")
        rounds_frame.pack(fill="both", expand=True)
        columns = ("round", "direction", "points", "file")
        self.round_tree = ttk.Treeview(rounds_frame, columns=columns, show="headings", height=7)
        for col, title, width in (
            ("round", "轮次", 50),
            ("direction", "方向", 70),
            ("points", "点数", 60),
            ("file", "文件", 300),
        ):
            self.round_tree.heading(col, text=title)
            self.round_tree.column(col, width=width)
        self.round_tree.pack(fill="both", expand=True, padx=4, pady=4)
        row = ttk.Frame(rounds_frame)
        row.pack(fill="x", padx=4, pady=(0, 4))
        self.build_btn = ttk.Button(row, text="构建标定表", command=self.build_table, state="disabled")
        self.build_btn.pack(side="left")
        self.load_btn = ttk.Button(row, text="加载标定表", command=self.load_table)
        self.load_btn.pack(side="left", padx=4)
        self.table_info_var = tk.StringVar(value="标定表: 未加载")
        ttk.Label(row, textvariable=self.table_info_var).pack(side="left", padx=8)

        result_frame = ttk.LabelFrame(right, text="标定结果")
        result_frame.pack(fill="both", expand=True)
        self.summary_text = tk.Text(result_frame, height=12, state="disabled", wrap="none")
        self.summary_text.pack(fill="both", expand=True, padx=4, pady=4)

        self.figure = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("PWM -> 水管俯仰角")
        self.ax.set_xlabel("PWM (us)")
        self.ax.set_ylabel("俯仰角 (°)")
        self.ax.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=False, padx=6, pady=(0, 6))

    def fill_recommended_grid(self):
        grid = tbl.recommended_grid(max_angle=10.0, near_step=0.5, near_radius=3.0)
        text = ", ".join(f"{v:g}" for v in grid)
        self.grid_var.set(text)

    def use_marker_range(self):
        markers = self.app.base_tab.markers
        low = markers.get("low")
        high = markers.get("high")
        zero = markers.get("zero")
        if low is None or high is None:
            self.app.log("请先在基础区记录下限和上限。", error=True)
            return
        low, high = sorted((low, high))
        self.start_var.set(str(low))
        self.end_var.set(str(high))
        self.app.log(
            f"已使用标记范围: {low} ~ {high} us"
            + (f"（零位 {zero} us）" if zero is not None else "")
        )

    def _on_flip_sign_changed(self):
        if self.flip_sign_var.get():
            self.flip_hint_var.set("（右端升高读数减小 → 勾选）")
        else:
            self.flip_hint_var.set("")

    def _read_params(self):
        try:
            start = clamp_pwm(int(float(self.start_var.get())))
            end = clamp_pwm(int(float(self.end_var.get())))
            num = max(2, min(200, int(self.num_points_var.get())))
            settle = float(self.settle_var.get())
            count = max(1, min(200, int(self.sample_count_var.get())))
            dt = max(10, min(100, int(self.sample_dt_var.get())))
            rounds = max(1, min(10, int(self.rounds_var.get())))
        except ValueError as exc:
            raise ValueError(f"参数无效: {exc}")
        if settle <= 0:
            raise ValueError("稳定时间必须大于 0")
        return start, end, num, settle, count, dt, rounds

    def _make_pwm_list(self, start, end, num):
        if self.mode_var.get() == "uniform":
            lo, hi = min(start, end), max(start, end)
            pwms = sorted({int(round(lo + (hi - lo) * i / (num - 1))) for i in range(num)})
            return pwms
        if self.app.table is None:
            raise ValueError("角度网格模式需要先构建/加载标定表")
        angles = parse_angle_list(self.grid_var.get())
        if not angles:
            raise ValueError("角度网格为空")
        pwms = sorted({self.app.table.angle_to_pwm(a) for a in angles})
        return pwms

    def start_calibration(self):
        if self.app.link is None:
            self.app.log("未连接设备。", error=True)
            return
        if self.worker is not None and self.worker.is_alive():
            self.app.log("标定已在运行。", error=True)
            return
        self.app.cal_mode_var.set(True)
        try:
            params = self._read_params()
            pwms = self._make_pwm_list(*params[:3])
        except ValueError as exc:
            self.app.log(str(exc), error=True)
            return

        self.rounds = []
        self.round_tree.delete(*self.round_tree.get_children())
        self.stop_event.clear()
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_var.set("准备中...")
        self.progress_var.set(0.0)
        self.app.log(
            f"标定开始: {len(pwms)} 点, 方向={self.direction_var.get()}, "
            f"稳定={params[3]}s, SAMPLE {params[4]}x{params[5]}ms, {params[6]} 轮"
        )

        def worker():
            try:
                resp = self.app.send("STREAM 0")
                if resp is None:
                    raise RuntimeError("关闭 STREAM 失败")
                resp = self.app.send("CAL 1")
                if resp is None:
                    raise RuntimeError("进入标定模式失败，请先确认串口与固件")
                self._run_rounds(pwms, params)
                self.app.job_q.put(("calib_done", None))
            except Exception as exc:
                self.app.job_q.put(("err", str(exc)))

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def _run_rounds(self, pwms, params):
        start, end, num, settle, count, dt, rounds = params
        direction_map = {"双向": ("up", "down"), "仅升序": ("up",), "仅降序": ("down",)}
        directions = direction_map[self.direction_var.get()]
        total_steps = rounds * len(directions) * len(pwms)
        step_index = 0

        for round_index in range(1, rounds + 1):
            if self.stop_event.is_set():
                break
            for direction in directions:
                if self.stop_event.is_set():
                    break
                sequence = pwms if direction == "up" else list(reversed(pwms))
                points = []
                for index, pwm in enumerate(sequence, 1):
                    if self.stop_event.is_set():
                        break
                    step_index += 1
                    resp = self.app.send(f"{CMD_SERVO} {pwm}", expect=RESP_OK)
                    if resp is None:
                        raise RuntimeError(f"第{round_index}轮 {direction} 第{index}点 SERVO 失败")
                    time.sleep(settle)
                    resp = self.app.send(
                        f"{CMD_SAMPLE} {count} {dt}",
                        expect=RESP_SAMPLE_OK,
                        timeout=count * dt / 1000.0 + 3.0,
                    )
                    if resp is None:
                        raise RuntimeError(f"第{round_index}轮 {direction} 第{index}点 SAMPLE 失败")
                    mean, std, n, ts = parse_sample(resp)
                    points.append(tbl.CalibPoint(pwm, mean, std, direction, ts))
                    self.app.job_q.put(
                        (
                            "log",
                            f"第{round_index}轮 {direction} {index}/{len(sequence)}: "
                            f"PWM={pwm}us -> {mean:+.4f}° (std {std:.4f}°)",
                        )
                    )
                    self.app.job_q.put(
                        ("progress", step_index / max(1, total_steps))
                    )
                if not points:
                    continue
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                path = os.path.join(
                    self.app.data_dir,
                    f"calib_r{round_index}_{direction}_{stamp}.csv",
                )
                meta = {
                    "round": round_index,
                    "direction": direction,
                    "points": len(points),
                    "settle_s": settle,
                    "sample_count": count,
                    "sample_dt_ms": dt,
                    "time": stamp,
                }
                tbl.write_round_csv(path, meta, points)
                self.rounds.append(points)
                self.app.job_q.put(
                    (
                        "round_done",
                        {
                            "round": round_index,
                            "direction": direction,
                            "points": len(points),
                            "file": path,
                        },
                    )
                )
        self.app.job_q.put(("progress", 1.0))

    def stop_calibration(self):
        self.stop_event.set()
        self.status_var.set("停止中...")

    def on_round_done(self, info):
        self.round_tree.insert(
            "",
            "end",
            values=(info["round"], info["direction"], info["points"], info["file"]),
        )
        self.status_var.set(f"已采集 {len(self.rounds)} 个方向轮次")
        if len(self.rounds) >= 1:
            self.build_btn.config(state="normal")

    def on_calibration_done(self):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("标定完成，可构建标定表")
        self.app.log("标定完成。")

    def on_task_failed(self):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.status_var.set("任务失败")

    def build_table(self):
        if not self.rounds:
            self.app.log("没有可用的标定数据。", error=True)
            return
        meta = {
            "created": datetime.now().isoformat(timespec="seconds"),
            "points": sum(len(r) for r in self.rounds),
            "rounds": len(self.rounds),
            "pitch_sign": -1.0 if self.flip_sign_var.get() else 1.0,
        }
        sign = -1.0 if self.flip_sign_var.get() else 1.0
        flipped_rounds = [
            [
                tbl.CalibPoint(
                    p.pwm_us,
                    p.angle_deg * sign,
                    p.std_deg,
                    p.direction,
                    p.ts_ms,
                )
                for p in one_round
            ]
            for one_round in self.rounds
        ]
        table = tbl.build_table(flipped_rounds, meta=meta)
        self.app.table = table
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = os.path.join(self.app.data_dir, f"calib_table_{stamp}.json")
        header_path = os.path.join(self.app.data_dir, f"calib_table_{stamp}.h")
        table.save_json(json_path)
        table.export_c(header_path)
        self.app.log(
            f"标定表已构建: {len(table.pwms)} 点, "
            f"回差 {table.hysteresis_min:.3f}~{table.hysteresis_max:.3f}°"
        )
        self.app.log(f"JSON: {json_path}")
        self.app.log(f"C头文件: {header_path}")
        self.table_info_var.set(f"标定表: {len(table.pwms)} 点, 回差 {table.hysteresis_max:.3f}°")
        self._show_table_summary(table)
        self._plot_table()

    def load_table(self):
        path = filedialog.askopenfilename(
            title="选择标定表 JSON",
            initialdir=self.app.data_dir,
            filetypes=(("JSON 标定表", "*.json"), ("所有文件", "*.*")),
        )
        if not path:
            return
        try:
            table = tbl.CalibTable.load_json(path)
        except Exception as exc:
            self.app.log(f"加载标定表失败: {exc}", error=True)
            return
        self.app.table = table
        self.table_info_var.set(
            f"标定表: {len(table.pwms)} 点, 回差 {table.hysteresis_max:.3f}°"
        )
        self.app.log(f"已加载标定表: {path}")
        self._show_table_summary(table)
        self._plot_table()

    def _show_table_summary(self, table):
        self.summary_text.config(state="normal")
        self.summary_text.delete("1.0", "end")
        self.summary_text.insert(
            "end",
            f"{'PWM(us)':>8} {'up(°)':>9} {'down(°)':>9} {'mean(°)':>9} {'hyst(°)':>8}\n",
        )
        for pwm, up, down, mean, hyst in zip(
            table.pwms, table.up, table.down, table.mean, table.hysteresis
        ):
            up_s = f"{up:+.4f}" if up is not None else "  --  "
            down_s = f"{down:+.4f}" if down is not None else "  --  "
            self.summary_text.insert(
                "end",
                f"{pwm:>8} {up_s:>9} {down_s:>9} {mean:+.4f} {hyst:.4f}\n",
            )
        self.summary_text.config(state="disabled")

    def _plot_table(self):
        self.ax.clear()
        for index, points in enumerate(self.rounds):
            for direction in ("up", "down"):
                subset = [p for p in points if p.direction == direction]
                if not subset:
                    continue
                xs = [p.pwm_us for p in subset]
                ys = [p.angle_deg for p in subset]
                color = "tab:red" if direction == "up" else "tab:blue"
                marker = "^" if direction == "up" else "v"
                self.ax.plot(xs, ys, marker, markersize=3, alpha=0.6, linestyle="",
                             color=color, label=f"r{index+1} {direction}")
        if self.app.table is not None:
            self.ax.plot(self.app.table.pwms, self.app.table.mean, "k-", linewidth=1.5,
                         label="mean")
        self.ax.set_xlabel("PWM (us)")
        self.ax.set_ylabel("俯仰角 (°)")
        self.ax.set_title("PWM -> 水管俯仰角")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(fontsize=8, ncol=2)
        self.canvas.draw_idle()


class ValidationTab:
    """验证区：目标角度 -> 查表 PWM -> 实测误差统计。"""

    def __init__(self, parent, app):
        self.app = app
        self.frame = ttk.Frame(parent)
        self.stop_event = threading.Event()
        self.worker = None

        params = ttk.LabelFrame(self.frame, text="验证参数")
        params.pack(fill="x", padx=6, pady=6)
        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=2)
        ttk.Label(row, text="目标角度(°):").pack(side="left")
        self.targets_var = tk.StringVar(value=DEFAULT_GRID_TEXT)
        ttk.Entry(row, textvariable=self.targets_var, width=46).pack(side="left", padx=4)
        ttk.Label(row, text="重复:").pack(side="left")
        self.repeats_var = tk.StringVar(value="1")
        ttk.Spinbox(row, textvariable=self.repeats_var, from_=1, to=10, width=4).pack(side="left", padx=2)
        ttk.Label(row, text="稳定(s):").pack(side="left")
        self.settle_var = tk.StringVar(value="1.0")
        ttk.Spinbox(row, textvariable=self.settle_var, from_=0.1, to=10.0, increment=0.1, width=6).pack(side="left", padx=2)
        ttk.Label(row, text="方向:").pack(side="left")
        self.direction_var = tk.StringVar(value="平均表")
        ttk.Combobox(
            row, textvariable=self.direction_var,
            values=("平均表", "升序表", "降序表", "方向感知"),
            width=8, state="readonly",
        ).pack(side="left", padx=2)

        row = ttk.Frame(params)
        row.pack(fill="x", padx=6, pady=4)
        self.load_btn = ttk.Button(row, text="加载标定表", command=self.load_table)
        self.load_btn.pack(side="left")
        self.start_btn = ttk.Button(row, text="开始验证", command=self.start_validation)
        self.start_btn.pack(side="left", padx=4)
        self.stop_btn = ttk.Button(row, text="停止", command=self.stop_validation, state="disabled")
        self.stop_btn.pack(side="left")
        self.table_info_var = tk.StringVar(value="标定表: 未加载")
        ttk.Label(row, textvariable=self.table_info_var).pack(side="left", padx=8)
        self.stats_var = tk.StringVar(value="统计: --")
        ttk.Label(row, textvariable=self.stats_var, foreground="#1565c0").pack(side="left", padx=8)

        body = ttk.Frame(self.frame)
        body.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        left = ttk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(body)
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        columns = ("target", "pwm", "actuals", "err_mean", "err_max")
        self.result_tree = ttk.Treeview(left, columns=columns, show="headings", height=14)
        for col, title, width in (
            ("target", "目标(°)", 80),
            ("pwm", "PWM(us)", 80),
            ("actuals", "实测(°)", 160),
            ("err_mean", "误差均值(°)", 110),
            ("err_max", "最大|误差|(°)", 120),
        ):
            self.result_tree.heading(col, text=title)
            self.result_tree.column(col, width=width, anchor="center")
        self.result_tree.pack(fill="both", expand=True, padx=4, pady=4)
        row = ttk.Frame(left)
        row.pack(fill="x", padx=4)
        self.save_btn = ttk.Button(row, text="保存CSV", command=self.save_csv, state="disabled")
        self.save_btn.pack(side="left")

        self.figure = Figure(figsize=(5, 3.2), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("验证误差")
        self.ax.set_xlabel("目标角度 (°)")
        self.ax.set_ylabel("误差 (°)")
        self.ax.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.results = []

    def load_table(self):
        path = filedialog.askopenfilename(
            title="选择标定表 JSON",
            initialdir=self.app.data_dir,
            filetypes=(("JSON 标定表", "*.json"), ("所有文件", "*.*")),
        )
        if not path:
            return
        try:
            table = tbl.CalibTable.load_json(path)
        except Exception as exc:
            self.app.log(f"加载标定表失败: {exc}", error=True)
            return
        self.app.table = table
        self.table_info_var.set(
            f"标定表: {len(table.pwms)} 点, 回差 {table.hysteresis_max:.3f}°"
        )
        self.app.log(f"已加载标定表: {path}")

    def start_validation(self):
        if self.app.link is None:
            self.app.log("未连接设备。", error=True)
            return
        if self.app.table is None:
            self.app.log("请先加载标定表。", error=True)
            return
        if self.worker is not None and self.worker.is_alive():
            self.app.log("验证已在运行。", error=True)
            return
        self.app.cal_mode_var.set(True)
        try:
            targets = parse_angle_list(self.targets_var.get())
            repeats = max(1, min(10, int(self.repeats_var.get())))
            settle = float(self.settle_var.get())
        except ValueError as exc:
            self.app.log(f"参数无效: {exc}", error=True)
            return
        if not targets:
            self.app.log("目标角度列表为空。", error=True)
            return
        self.results = []
        self.result_tree.delete(*self.result_tree.get_children())
        self.stop_event.clear()
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.stats_var.set("运行中...")
        self.app.log(f"验证开始: {len(targets)} 个目标 x {repeats} 次")

        def worker():
            try:
                resp = self.app.send("STREAM 0")
                if resp is None:
                    raise RuntimeError("关闭 STREAM 失败")
                resp = self.app.send("CAL 1")
                if resp is None:
                    raise RuntimeError("进入标定模式失败")
                self._run_targets(targets, repeats, settle)
                stats = tbl.validation_stats(self.results)
                self.app.job_q.put(("validate_done", stats))
            except Exception as exc:
                self.app.job_q.put(("err", str(exc)))

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def _run_targets(self, targets, repeats, settle):
        table = self.app.table
        direction_mode = self.direction_var.get()
        current_angle = None

        for target in targets:
            if self.stop_event.is_set():
                break
            if direction_mode == "方向感知":
                direction = "up" if current_angle is None or target >= current_angle else "down"
            elif direction_mode == "升序表":
                direction = "up"
            elif direction_mode == "降序表":
                direction = "down"
            else:
                direction = "mean"
            pwm = table.angle_to_pwm(target, direction)
            actuals = []
            for rep in range(1, repeats + 1):
                if self.stop_event.is_set():
                    break
                resp = self.app.send(f"{CMD_SERVO} {pwm}", expect=RESP_OK)
                if resp is None:
                    raise RuntimeError(f"目标 {target}° 第{rep}次 SERVO 失败")
                time.sleep(settle)
                resp = self.app.send(
                    f"{CMD_SAMPLE} 50 20",
                    expect=RESP_SAMPLE_OK,
                    timeout=4.0,
                )
                if resp is None:
                    raise RuntimeError(f"目标 {target}° 第{rep}次 SAMPLE 失败")
                mean, std, count, ts = parse_sample(resp)
                actuals.append(mean)
                current_angle = mean
                self.app.job_q.put(
                    (
                        "log",
                        f"验证 {target:+.2f}° -> PWM {pwm}us -> 实测 {mean:+.4f}° "
                        f"(std {std:.4f}°, 方向 {direction})",
                    )
                )
            if not actuals:
                continue
            err_mean = sum(actuals) / len(actuals) - target
            err_max = max(abs(a - target) for a in actuals)
            self.results.append(
                {
                    "target": target,
                    "pwm": pwm,
                    "actuals": actuals,
                    "err_mean": err_mean,
                    "err_max": err_max,
                }
            )
            actuals_text = ", ".join(f"{a:+.3f}" for a in actuals)
            self.app.job_q.put(
                (
                    "result",
                    (target, pwm, actuals_text, err_mean, err_max),
                )
            )

    def on_validation_done(self, stats):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self.save_btn.config(state="normal")
        self.stats_var.set(
            f"n={stats['n']} 均值误差 {stats['mean']:+.4f}° "
            f"最大|误差| {stats['max_abs']:.4f}° RMS {stats['rms']:.4f}°"
        )
        self.app.log(
            f"验证完成: n={stats['n']}, 均值误差 {stats['mean']:+.4f}°, "
            f"最大|误差| {stats['max_abs']:.4f}°, RMS {stats['rms']:.4f}°"
        )
        self._plot_errors()

    def on_task_failed(self):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def stop_validation(self):
        self.stop_event.set()

    def _plot_errors(self):
        self.ax.clear()
        targets = [r["target"] for r in self.results]
        errors = [r["err_mean"] for r in self.results]
        self.ax.plot(targets, errors, "o-", color="tab:red", markersize=5, label="误差均值")
        self.ax.axhline(0.0, color="gray", linewidth=0.8)
        self.ax.set_xlabel("目标角度 (°)")
        self.ax.set_ylabel("误差 (°)")
        self.ax.set_title("验证误差")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(fontsize=9)
        self.canvas.draw_idle()

    def save_csv(self):
        if not self.results:
            self.app.log("没有验证结果。", error=True)
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.app.data_dir, f"validation_{stamp}.csv")
        tbl.write_validation_csv(path, self.results)
        self.app.log(f"验证结果已保存: {path}")


def main():
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--data-dir", help="数据保存目录（默认 Tools/pipe_calibration/data）")
    args = parser.parse_args()

    root = tk.Tk()
    app = CalibrationApp(root)
    if args.data_dir:
        app.data_dir = args.data_dir
        os.makedirs(app.data_dir, exist_ok=True)
    root.mainloop()


if __name__ == "__main__":
    main()
