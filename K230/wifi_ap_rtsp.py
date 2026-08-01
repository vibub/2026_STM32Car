"""CanMV K230 AP 模式下的 WBC/H.264 RTSP 图传。"""

import _thread
import gc
import os
import time

import multimedia as mm
import network
from _media import Display
from media.vencoder import *
from mpp import *


class _RtspH264Server:
    """把 Display WBC 帧送入硬件 H.264 编码器和 RTSP 服务器。"""

    def __init__(
        self,
        width,
        height,
        port=8554,
        session_name="test",
        bitrate_kbps=2048,
    ):
        self.width = ALIGN_UP(int(width), 16)
        self.height = int(height)
        self.port = int(port)
        self.session_name = session_name
        self.bitrate_kbps = max(256, int(bitrate_kbps))
        self.venc_channel = VENC_CHN_ID_0
        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            self.venc_channel,
            16,
            self.width,
            self.height,
        )
        self.server = mm.rtsp_server()
        self.started = False

    def start(self):
        if self.started:
            return
        channel_attr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            self.width,
            self.height,
            bit_rate=self.bitrate_kbps,
        )
        self.encoder.Create(self.venc_channel, channel_attr)
        self.server.rtspserver_init(self.port)
        self.server.rtspserver_createsession(
            self.session_name,
            mm.multi_media_type.media_h264,
            False,
        )
        self.server.rtspserver_start()
        self.encoder.Start(self.venc_channel)
        self.started = True

    def send_frame(self, frame_info):
        if not self.started:
            return False
        # 后台线程也不能无限阻塞在编码器中，否则会长期占用 MicroPython VM。
        if self.encoder.SendFrame(
            self.venc_channel,
            frame_info,
            timeout=100,
        ) != 0:
            return False

        stream_data = StreamData()
        if self.encoder.GetStream(
            self.venc_channel,
            stream_data,
            timeout=100,
        ) != 0:
            return False

        timestamp_ms = time.ticks_ms()
        for pack_index in range(stream_data.pack_cnt):
            self.server.rtspserver_sendvideodata_byphyaddr(
                self.session_name,
                stream_data.phy_addr[pack_index],
                stream_data.data_size[pack_index],
                timestamp_ms,
            )
        self.encoder.ReleaseStream(self.venc_channel, stream_data)
        return True

    def stop(self):
        if not self.started:
            return

        while True:
            stream_data = StreamData()
            result = self.encoder.GetStream(
                self.venc_channel,
                stream_data,
                timeout=0,
            )
            if result != 0:
                break
            self.encoder.ReleaseStream(self.venc_channel, stream_data)

        self.encoder.Stop(self.venc_channel)
        self.encoder.Destroy(self.venc_channel)
        self.server.rtspserver_stop()
        self.server.rtspserver_deinit()
        self.started = False


class WifiApRtspStreamer:
    """创建 K230 Wi-Fi AP，并在后台线程推送 Display 的 H.264 RTSP。"""

    def __init__(
        self,
        ssid="K230_VIDEO",
        key="12345678",
        ip="192.168.4.1",
        port=8554,
        session_name="test",
        fps=15,
        bitrate_kbps=2048,
        channel=6,
    ):
        self.ssid = ssid
        self.key = key
        self.ip = ip
        self.port = int(port)
        self.session_name = session_name
        self.fps = max(1, int(fps))
        self.bitrate_kbps = max(256, int(bitrate_kbps))
        self.channel = int(channel)
        self.frame_interval_ms = max(1, 1000 // self.fps)

        self.ap = None
        self.rtsp = None
        self.started = False
        self._running = False
        self._thread_over = True

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
        for kwargs, config_name in configs:
            try:
                self.ap.config(**kwargs)
                print("RTSP AP config:", config_name)
                return
            except TypeError as error:
                last_error = error
        raise last_error

    def _print_ap_status(self):
        for label, call in (
            ("status", self.ap.status),
            ("ifconfig", self.ap.ifconfig),
            ("info", getattr(self.ap, "info", None)),
        ):
            if call is None:
                continue
            try:
                print("RTSP AP", label + ":", call())
            except Exception as error:
                print("RTSP AP", label, "read failed:", error)

    def _stream_loop(self):
        self._thread_over = False
        last_frame_ms = time.ticks_ms() - self.frame_interval_ms
        encoded_frames = 0
        try:
            while self._running:
                os.exitpoint()
                now_ms = time.ticks_ms()
                if (
                    time.ticks_diff(now_ms, last_frame_ms)
                    < self.frame_interval_ms
                ):
                    time.sleep_ms(1)
                    continue

                frame_info = Display.writeback_dump(100)
                if frame_info:
                    self.rtsp.send_frame(frame_info)
                    # 主动解除WBC帧对象引用，并定期回收其MicroPython包装对象，
                    # 防止长时间图传后媒体缓冲区因延迟回收而耗尽。
                    frame_info = None
                    encoded_frames += 1
                    if encoded_frames % 30 == 0:
                        gc.collect()
                    last_frame_ms = now_ms
        except BaseException as error:
            print("RTSP stream thread stopped by error:")
            try:
                import sys
                sys.print_exception(error)
            except Exception:
                print(error)
        finally:
            self._thread_over = True
            print("RTSP stream thread over")

    def start(self):
        if self.started:
            return
        if not Display.inited():
            raise RuntimeError("Display must be initialized before RTSP")

        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self._config_ap()
        self.ap.ifconfig((self.ip, "255.255.255.0", self.ip, self.ip))
        self._print_ap_status()

        self.rtsp = _RtspH264Server(
            Display.width(),
            Display.height(),
            port=self.port,
            session_name=self.session_name,
            bitrate_kbps=self.bitrate_kbps,
        )
        if not Display.writeback(True):
            try:
                self.ap.active(False)
            except Exception:
                pass
            self.ap = None
            self.rtsp = None
            raise RuntimeError("Display WBC start failed")

        try:
            self.rtsp.start()
            self._running = True
            self._thread_over = False
            _thread.start_new_thread(self._stream_loop, ())
            self.started = True
        except BaseException:
            Display.writeback(False)
            self.rtsp.stop()
            self.rtsp = None
            raise

        print(
            "RTSP AP started: ssid={} key={} url=rtsp://{}:{}/{}".format(
                self.ssid,
                self.key,
                self.ip,
                self.port,
                self.session_name,
            )
        )

    def needs_frame(self, _now_ms=None):
        """WBC线程自行获取显示帧，主循环不需要提供CV图像。"""
        return False

    def tick(self, _frame=None, _now_ms=None):
        """保持与旧 MJPEG 推流器相同接口；RTSP 模式无需主循环轮询。"""
        return

    def stop(self):
        self._running = False
        wait_start_ms = time.ticks_ms()
        while (
            not self._thread_over
            and time.ticks_diff(time.ticks_ms(), wait_start_ms) < 2000
        ):
            time.sleep_ms(20)

        try:
            if Display.inited() and not Display.writeback(False):
                print("Display WBC stop failed")
        except Exception as error:
            print("Display WBC stop error:", error)

        if self.rtsp is not None:
            try:
                self.rtsp.stop()
            except Exception as error:
                print("RTSP server stop error:", error)
            self.rtsp = None

        if self.ap is not None:
            try:
                self.ap.active(False)
            except Exception:
                pass
            self.ap = None

        self.started = False
        gc.collect()
