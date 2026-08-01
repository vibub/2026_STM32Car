# -*- coding: utf-8 -*-
"""
PC <-> STM32 串口链路（USB-TTL 转串口模块）。

协议定义见 PROTOCOL.md，机器可读常量见 protocol.py。
本文件只做两件事：
    1. 后台读线程 + 环形队列，把设备应答行与 STREAM 实时行分开；
    2. 提供 request() 同步请求 / read_available() 拉取实时行。
"""

import queue
import threading
import time

from protocol import RESP_ERR, RESP_STREAM, RESP_TUNE_STREAM


class LinkError(Exception):
    """通信错误（超时 / 设备 ERR / 未连接等）。"""


class SerialLink:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self._ser = None
        self._rx_q = queue.Queue()
        self._stream_q = queue.Queue()
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def open(self):
        import serial

        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                line = self._ser.readline().decode("utf-8", errors="replace").strip()
            except Exception:
                break
            if not line:
                continue
            if (line.startswith(RESP_STREAM + " ") or
                    line.startswith(RESP_TUNE_STREAM + " ")):
                self._stream_q.put(line)
            else:
                self._rx_q.put(line)

    def close(self):
        self._stop.set()
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass

    def send(self, cmd):
        if self._ser is None:
            raise LinkError("串口未打开")
        with self._wlock:
            self._ser.write((cmd + "\n").encode("utf-8"))
            self._ser.flush()

    def request(self, cmd, expect=None, timeout=1.5):
        """发送命令并等待下一条非流式应答行。
        expect 为期望响应首词；不匹配的行丢弃（单请求串行模式，无并发）。"""
        self.send(cmd)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self._rx_q.get(timeout=0.05)
            except queue.Empty:
                continue
            if line.startswith(RESP_ERR):
                raise LinkError(f"设备返回错误: {line}")
            if expect is None or line.startswith(expect):
                return line
        raise LinkError(f"命令超时: {cmd}")

    def read_available(self):
        lines = []
        while True:
            try:
                lines.append(self._stream_q.get_nowait())
            except queue.Empty:
                break
        return lines
