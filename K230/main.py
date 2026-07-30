"""CanMV K230 红色位置标记 + 圆环/钢珠识别独立联调入口。"""

import gc
import os
import sys
import time

# CanMV IDE 的“运行当前脚本”会把 main.py 作为临时代码执行，部分固件不会
# 自动把 SD 卡根目录加入模块搜索路径。显式加入后，同卡的依赖文件才能导入。
for module_dir in ("/sdcard/app", "/sdcard"):
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

from machine import UART, FPIOA

try:
    from ball_position import (
        BallPositionDetector,
        RING_TARGET_THRESHOLDS,
        STEEL_BALL_TARGET_THRESHOLDS,
        format_ball_message,
        select_best_yolo_box,
    )
    from libs.YOLO import YOLO11
    from vision_pipeline import VisionPipeline
    try:
        from wifi_ap_video import WifiApVideoStreamer
    except ImportError:
        WifiApVideoStreamer = None
except ImportError:
    print("K230 module import failed")
    print("sys.path:", sys.path)
    try:
        print("/sdcard files:", os.listdir("/sdcard"))
    except Exception as list_error:
        print("cannot read /sdcard:", list_error)
    print("required: ball_position.py, vision_pipeline.py and /sdcard/libs/YOLO.py")
    raise


CV_SIZE = (640, 360)
DISPLAY_SIZE = (800, 480)
AI_SIZE = (640, 360)
STEEL_BALL_MODEL_INPUT_SIZE = [320, 320]
STEEL_BALL_MODEL_PATHS = (
    "/sdcard/models/steel_ball_yolo11n_det_320.kmodel",
    "/sdcard/yolo11n_det_320.kmodel",
)
STEEL_BALL_LABELS = {0: "steel_ball"}
STEEL_BALL_CONFIDENCE_THRESHOLD = 0.60
STEEL_BALL_NMS_THRESHOLD = 0.45

K230_UART_TX_IO = 44
K230_UART_RX_IO = 45
SEND_INTERVAL_MS = 40
OSD_INTERVAL_MS = 100
GC_INTERVAL_FRAMES = 30

# K230 creates a Wi-Fi AP and serves MJPEG video for PC/phone browser.
VIDEO_AP_ENABLE = True
VIDEO_AP_SSID = "K230_VIDEO"
VIDEO_AP_KEY = "12345678"
VIDEO_AP_IP = "192.168.4.1"
VIDEO_AP_PORT = 80
VIDEO_AP_FPS = 6
VIDEO_AP_JPEG_QUALITY = 40
VIDEO_AP_START_DELAY_FRAMES = 20

# STEEL_BALL 使用现成 YOLO；临时圆环联调时改回 RING，使用传统颜色检测。
TARGET_PROFILE = "STEEL_BALL"
RING_DIAMETER_MM = 18.0
STEEL_BALL_DIAMETER_MM = 10.0


def _target_diameter_mm():
    if TARGET_PROFILE == "RING":
        return RING_DIAMETER_MM
    if TARGET_PROFILE == "STEEL_BALL":
        return STEEL_BALL_DIAMETER_MM
    raise ValueError("unknown TARGET_PROFILE: " + str(TARGET_PROFILE))


def _target_thresholds():
    if TARGET_PROFILE == "RING":
        return RING_TARGET_THRESHOLDS
    if TARGET_PROFILE == "STEEL_BALL":
        return STEEL_BALL_TARGET_THRESHOLDS
    raise ValueError("unknown TARGET_PROFILE: " + str(TARGET_PROFILE))


def _resolve_steel_ball_model_path():
    for path in STEEL_BALL_MODEL_PATHS:
        try:
            os.stat(path)
            return path
        except Exception:
            pass
    raise OSError(
        "steel-ball kmodel missing: " + ", ".join(STEEL_BALL_MODEL_PATHS)
    )


def _create_steel_ball_yolo(pipeline):
    model_path = _resolve_steel_ball_model_path()
    print("loading steel-ball YOLO:", model_path)
    detector = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=model_path,
        labels=STEEL_BALL_LABELS,
        rgb888p_size=list(AI_SIZE),
        model_input_size=STEEL_BALL_MODEL_INPUT_SIZE,
        display_size=list(pipeline.display_size),
        conf_thresh=STEEL_BALL_CONFIDENCE_THRESHOLD,
        nms_thresh=STEEL_BALL_NMS_THRESHOLD,
        max_boxes_num=20,
        debug_mode=0,
    )
    detector.config_preprocess()
    return detector


def _scale_rect(rect):
    x, y, width, height = rect
    return (
        int(x * DISPLAY_SIZE[0] // CV_SIZE[0]),
        int(y * DISPLAY_SIZE[1] // CV_SIZE[1]),
        max(1, int(width * DISPLAY_SIZE[0] // CV_SIZE[0])),
        max(1, int(height * DISPLAY_SIZE[1] // CV_SIZE[1])),
    )


def _draw_overlay(pipeline, detector, result, fps, yolo=None, yolo_result=None):
    osd = pipeline.osd_img
    pipeline.clear_osd()
    if yolo is not None and yolo_result is not None:
        yolo.draw_result(yolo_result, osd)
    green = (0, 255, 0)
    red = (255, 0, 0)
    yellow = (255, 255, 0)
    cyan = (0, 255, 255)
    white = (255, 255, 255)

    calibration = detector.calibration
    if calibration is not None:
        if hasattr(calibration, "marker_columns"):
            for rect in calibration.marker_rects():
                x, y, width, height = _scale_rect(rect)
                osd.draw_rectangle(x, y, width, height, color=cyan, thickness=2)
        else:
            for row in (calibration.upper, calibration.lower):
                x, y, width, height = _scale_rect(row.rect)
                osd.draw_rectangle(x, y, width, height, color=cyan, thickness=2)

    if result.target_rect is not None:
        x, y, width, height = _scale_rect(result.target_rect)
        osd.draw_rectangle(x, y, width, height, color=green, thickness=3)
        center_x = x + width // 2
        center_y = y + height // 2
        osd.draw_line(center_x - 10, center_y, center_x + 10, center_y, color=green, thickness=2)
        osd.draw_line(center_x, center_y - 10, center_x, center_y + 10, color=green, thickness=2)

    scale_ok = calibration is not None
    marker_count = (
        calibration.marker_count
        if scale_ok and hasattr(calibration, "marker_count")
        else (2 if scale_ok else 0)
    )
    position_abs_d10 = abs(result.position_d10_mm)
    if result.valid:
        position_text = "{}{}.{} mm".format(
            "-" if result.position_d10_mm < 0 else "+",
            position_abs_d10 // 10,
            position_abs_d10 % 10,
        )
    else:
        position_text = "--.- mm"
    osd.draw_string_advanced(
        8, 8, 20,
        "MARK {}/5 Q{}".format(marker_count, result.scale_quality),
        color=cyan if scale_ok else red,
    )
    osd.draw_string_advanced(
        8, 34, 20,
        "TARGET {} V{} Q{}".format(
            TARGET_PROFILE,
            int(result.valid),
            result.quality,
        ),
        color=green if result.valid else yellow,
    )
    osd.draw_string_advanced(
        8, 60, 24,
        "BALL POS " + position_text,
        color=green if result.valid else yellow,
    )
    osd.draw_string_advanced(8, 92, 18, "FPS {:.1f}".format(fps), color=white)
    pipeline.show_osd()


def _create_uart():
    fpioa = FPIOA()
    fpioa.set_function(K230_UART_TX_IO, FPIOA.UART2_TXD)
    fpioa.set_function(K230_UART_RX_IO, FPIOA.UART2_RXD)
    return UART(
        UART.UART2,
        baudrate=115200,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0,
    )


def run():
    pipeline = None
    uart = None
    yolo = None
    video_streamer = None
    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        diameter_mm = _target_diameter_mm()
        print("启动双排刻度识别，目标模式:", TARGET_PROFILE, "直径/mm:", diameter_mm)
        pipeline = VisionPipeline(
            cv_size=CV_SIZE,
            ai_size=AI_SIZE,
            display_size=DISPLAY_SIZE,
        )
        pipeline.create()
        uart = _create_uart()
        detector = BallPositionDetector(
            thresholds=_target_thresholds(),
            target_diameter_mm=diameter_mm,
        )
        use_yolo = TARGET_PROFILE == "STEEL_BALL"
        if use_yolo:
            yolo = _create_steel_ball_yolo(pipeline)
        clock = time.clock()
        sequence = 0
        frame_count = 0
        last_send_ms = time.ticks_ms()
        last_osd_ms = last_send_ms
        video_ap_started = False

        while True:
            os.exitpoint()
            clock.tick()
            frame = pipeline.capture_vision_frame()
            now_ms = time.ticks_ms()
            if (
                VIDEO_AP_ENABLE and
                not video_ap_started and
                frame_count >= VIDEO_AP_START_DELAY_FRAMES
            ):
                video_ap_started = True
                if WifiApVideoStreamer is None:
                    print("wifi_ap_video.py missing, keep local display running")
                else:
                    try:
                        video_streamer = WifiApVideoStreamer(
                            ssid=VIDEO_AP_SSID,
                            key=VIDEO_AP_KEY,
                            ip=VIDEO_AP_IP,
                            port=VIDEO_AP_PORT,
                            fps=VIDEO_AP_FPS,
                            jpeg_quality=VIDEO_AP_JPEG_QUALITY,
                        )
                        video_streamer.start()
                    except BaseException as error:
                        print("video AP start failed, keep local display running:")
                        sys.print_exception(error)
                        video_streamer = None
            yolo_result = None
            ai_frame = None
            ai_array = None
            if use_yolo:
                detector.update_calibration(frame)
                if video_streamer is not None:
                    video_streamer.tick(frame, now_ms)
                frame = None
                ai_frame, ai_array = pipeline.capture_ai_frame()
                yolo_result = yolo.run(ai_array)
                detection = select_best_yolo_box(
                    yolo_result,
                    detector.previous_center_x,
                )
                result = detector.result_from_yolo_box(detection)
            else:
                result = detector.detect(frame)
                if video_streamer is not None:
                    video_streamer.tick(frame, now_ms)
            frame = None

            if time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS:
                uart.write(format_ball_message(result, sequence))
                sequence = (sequence + 1) & 0xFFFF
                last_send_ms = now_ms

            if time.ticks_diff(now_ms, last_osd_ms) >= OSD_INTERVAL_MS:
                _draw_overlay(
                    pipeline,
                    detector,
                    result,
                    clock.fps(),
                    yolo,
                    yolo_result,
                )
                last_osd_ms = now_ms

            ai_frame = None
            ai_array = None
            yolo_result = None

            frame_count += 1
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("用户停止钢珠位置识别")
    except BaseException as error:
        print("钢珠位置识别异常:")
        sys.print_exception(error)
    finally:
        if video_streamer is not None:
            try:
                video_streamer.stop()
            except BaseException as error:
                sys.print_exception(error)
        if yolo is not None:
            try:
                yolo.deinit()
            except BaseException as error:
                sys.print_exception(error)
        if pipeline is not None:
            try:
                pipeline.destroy()
            except BaseException as error:
                sys.print_exception(error)
        if uart is not None:
            try:
                uart.deinit()
            except BaseException as error:
                sys.print_exception(error)
        gc.collect()


if __name__ == "__main__":
    run()
