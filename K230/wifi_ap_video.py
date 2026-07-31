"""Wi-Fi AP MJPEG video streamer for CanMV K230.

Open a browser on a phone or PC connected to the board hotspot:
    http://192.168.4.1/
"""

import gc
import socket
import time

import network


_BOUNDARY = b"k230frame"
_INDEX_HTML = b"""\
HTTP/1.0 200 OK
Content-Type: text/html; charset=utf-8
Connection: close

<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>K230 Video</title>
  <style>
    html,body{margin:0;height:100%;background:#111;color:#eee;font-family:sans-serif}
    body{display:flex;align-items:center;justify-content:center}
    img{max-width:100vw;max-height:100vh;image-rendering:auto}
  </style>
</head>
<body><img src="/stream"></body>
</html>
"""


def _ticks_diff(now_ms, old_ms):
    try:
        return time.ticks_diff(now_ms, old_ms)
    except AttributeError:
        return now_ms - old_ms


def _send_all(sock, data):
    view = memoryview(data)
    total = 0
    while total < len(view):
        sent = sock.send(view[total:])
        if sent is None:
            sent = 0
        if sent <= 0:
            raise OSError("socket send failed")
        total += sent


def _image_bytes(img):
    try:
        return memoryview(img)
    except TypeError:
        pass

    for name in ("bytearray", "bytes"):
        method = getattr(img, name, None)
        if method is not None:
            return method()

    return bytes(img)


class WifiApVideoStreamer:
    """Serve camera frames as a simple MJPEG stream without blocking vision logic."""

    def __init__(
        self,
        ssid="kk22330",
        key="czhhqgyyb",
        ip="192.168.4.1",
        port=80,
        fps=6,
        jpeg_quality=40,
        channel=6,
        max_clients=1,
    ):
        self.ssid = ssid
        self.key = key
        self.ip = ip
        self.port = int(port)
        self.fps = max(1, int(fps))
        self.jpeg_quality = max(1, min(100, int(jpeg_quality)))
        self.channel = int(channel)
        self.max_clients = max(1, int(max_clients))

        self.ap = None
        self.server = None
        self.clients = []
        self.frame_interval_ms = max(1, 1000 // self.fps)
        self.last_frame_ms = 0
        self.started = False

    def start(self):
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self._config_ap()
        self.ap.ifconfig((self.ip, "255.255.255.0", self.ip, self.ip))
        self._print_ap_status()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", self.port))
        self.server.listen(self.max_clients)
        self._set_nonblocking(self.server)
        self.started = True

        print(
            "video AP started: ssid={} key={} url=http://{}:{}/".format(
                self.ssid,
                self.key,
                self.ip,
                self.port,
            )
        )

    def _config_ap(self):
        configs = (
            ({"ssid": self.ssid, "channel": self.channel, "key": self.key}, "ssid/channel/key"),
            ({"ssid": self.ssid, "key": self.key}, "ssid/key"),
            ({"ssid": self.ssid, "channel": self.channel, "password": self.key}, "ssid/channel/password"),
            ({"ssid": self.ssid, "password": self.key}, "ssid/password"),
            ({"ssid": self.ssid, "channel": self.channel}, "ssid/channel"),
            ({"ssid": self.ssid}, "ssid only"),
        )
        last_error = None
        for kwargs, name in configs:
            try:
                self.ap.config(**kwargs)
                print("video AP config:", name)
                return
            except TypeError as error:
                last_error = error

        try:
            self.ap.config(ssid=self.ssid)
            config_name = "ssid"
            for kwargs, name in (
                ({"key": self.key}, "/key"),
                ({"password": self.key}, "/password"),
                ({"channel": self.channel}, "/channel"),
            ):
                try:
                    self.ap.config(**kwargs)
                    config_name += name
                except TypeError:
                    pass
            print("video AP config:", config_name)
            return
        except TypeError as error:
            last_error = error

        raise last_error

    def _print_ap_status(self):
        for label, call in (
            ("active", self.ap.active),
            ("status", self.ap.status),
            ("ifconfig", self.ap.ifconfig),
            ("info", getattr(self.ap, "info", None)),
        ):
            if call is None:
                continue
            try:
                print("video AP", label + ":", call())
            except Exception as error:
                print("video AP", label, "read failed:", error)

    def _set_nonblocking(self, sock):
        try:
            sock.setblocking(False)
        except AttributeError:
            sock.settimeout(0)

    def needs_frame(self, now_ms=None):
        """仅在存在客户端且达到发送周期时请求一张 CV 帧。"""
        if not self.started or not self.clients:
            return False
        if now_ms is None:
            now_ms = time.ticks_ms()
        return _ticks_diff(now_ms, self.last_frame_ms) >= self.frame_interval_ms

    def tick(self, frame=None, now_ms=None):
        if not self.started:
            return

        if now_ms is None:
            now_ms = time.ticks_ms()

        # 接受客户端不依赖图像，主循环可以在绝大多数 YOLO 帧中跳过 CV snapshot。
        self._accept_clients()
        if frame is None or not self.needs_frame(now_ms):
            return
        self.last_frame_ms = now_ms

        jpeg = None
        payload = None
        try:
            jpeg = frame.compressed(quality=self.jpeg_quality)
            payload = _image_bytes(jpeg)
            self._send_frame(payload)
        except BaseException as error:
            print("video frame send failed:", error)
            self._close_all_clients()
        finally:
            payload = None
            jpeg = None

    def _accept_clients(self):
        while True:
            try:
                client, addr = self.server.accept()
            except OSError:
                return

            try:
                client.settimeout(0.2)
            except AttributeError:
                pass
            try:
                request = client.recv(256)
            except OSError:
                request = b""
            self._set_nonblocking(client)

            if len(self.clients) >= self.max_clients:
                old = self.clients.pop(0)
                self._close_socket(old)

            if b"/stream" in request:
                self._send_stream_header(client)
                self.clients.append(client)
                print("video stream client:", addr)
            elif (not request) or (b"GET / " in request) or (b"GET /HTTP" in request):
                try:
                    _send_all(client, _INDEX_HTML)
                    print("video index client:", addr)
                finally:
                    self._close_socket(client)
            else:
                try:
                    _send_all(client, _INDEX_HTML)
                    print("video http client:", addr)
                finally:
                    self._close_socket(client)

    def _send_stream_header(self, client):
        header = (
            b"HTTP/1.0 200 OK\r\n"
            b"Server: CanMV-K230\r\n"
            b"Cache-Control: no-cache\r\n"
            b"Pragma: no-cache\r\n"
            b"Connection: close\r\n"
            b"Content-Type: multipart/x-mixed-replace; boundary="
            + _BOUNDARY
            + b"\r\n\r\n"
        )
        _send_all(client, header)

    def _send_frame(self, payload):
        header = (
            b"--"
            + _BOUNDARY
            + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(payload)).encode()
            + b"\r\n\r\n"
        )
        next_clients = []
        for client in self.clients:
            try:
                _send_all(client, header)
                _send_all(client, payload)
                _send_all(client, b"\r\n")
                next_clients.append(client)
            except OSError:
                self._close_socket(client)
        self.clients = next_clients

    def _close_socket(self, sock):
        try:
            sock.close()
        except Exception:
            pass

    def _close_all_clients(self):
        for client in self.clients:
            self._close_socket(client)
        self.clients = []

    def stop(self):
        self._close_all_clients()
        if self.server is not None:
            self._close_socket(self.server)
            self.server = None
        if self.ap is not None:
            try:
                self.ap.active(False)
            except Exception:
                pass
            self.ap = None
        self.started = False
        gc.collect()
