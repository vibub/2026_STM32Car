"""CanMV K230 钢球 AnchorBaseDet 摄像头识别与目标框显示测试。"""

import gc
import os
import sys
import time

# CanMV IDE 运行当前脚本时不一定自动加入 SD 卡模块目录。
for module_dir in ("/sdcard/app", "/sdcard"):
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

from libs.PipeLine import PipeLine
from libs.PlatTasks import DetectionApp
from media.sensor import Sensor


KMODEL_PATH = "/sdcard/models/ball_AnchorBaseDet_320.kmodel"  # 当前钢球模型路径。
LABELS = ["ball"]                                            # 单类别钢球标签。
MODEL_INPUT_SIZE = [320, 320]                                 # KModel 固定输入尺寸。
RGB888P_SIZE = [320, 180]                                     # 摄像头 AI 通道分辨率。
DISPLAY_SIZE = [800, 480]                                     # 显示与 OSD 分辨率。
DISPLAY_MODE = "st7701"                                      # 当前 LCD 显示接口。
CONFIDENCE_THRESHOLD = 0.40                                   # 钢球最低检测置信度。
NMS_THRESHOLD = 0.50                                          # 重叠检测框的 NMS 阈值。
ANCHORS = [                                                    # 三个检测层的九组锚框。
    12, 12, 14, 14, 16, 15,
    17, 14, 16, 18, 19, 16,
    18, 18, 22, 16, 21, 19,
]
GC_INTERVAL_FRAMES = 60                                       # 降低垃圾回收对帧周期的干扰。
FPS_REPORT_INTERVAL_MS = 1000                                 # 串口帧率打印周期。


def _model_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def run():
    pipeline = None
    detector = None
    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        if not _model_exists(KMODEL_PATH):
            raise OSError("kmodel missing: " + KMODEL_PATH)

        # 参照官方部署示例，显示通道由 PipeLine 直接绑定，
        # Python 主循环只获取 AI 帧、执行 AnchorBaseDet 和刷新检测框 OSD。
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create(sensor=Sensor(width=1920, height=1080))
        display_size = pipeline.get_display_size()

        # 使用模型部署配置中的 AnchorBaseDet 类型、阈值与九组锚框。
        detector = DetectionApp(
            "video",
            KMODEL_PATH,
            LABELS,
            MODEL_INPUT_SIZE,
            ANCHORS,
            "AnchorBaseDet",
            CONFIDENCE_THRESHOLD,
            NMS_THRESHOLD,
            RGB888P_SIZE,
            display_size,
            debug_mode=0,
        )
        detector.config_preprocess()

        frame_count = 0
        report_frame_count = 0
        report_start_ms = time.ticks_ms()
        display_fps = 0.0
        first_detection_reported = False

        print("steel-ball AnchorBaseDet camera test started:", KMODEL_PATH)
        while True:
            os.exitpoint()

            ai_frame = pipeline.get_frame()
            result = detector.run(ai_frame)

            frame_count += 1
            report_frame_count += 1
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, report_start_ms)
            if elapsed_ms >= FPS_REPORT_INTERVAL_MS:
                display_fps = report_frame_count * 1000.0 / float(elapsed_ms)
                print("AnchorBaseDet FPS: {:.1f}".format(display_fps))
                report_frame_count = 0
                report_start_ms = now_ms

            # DetectionApp 先绘制模型结果，再叠加帧率，避免帧率文字被清除。
            detector.draw_result(pipeline.osd_img, result)
            pipeline.osd_img.draw_string_advanced(
                8,
                8,
                24,
                "FPS {:.1f}".format(display_fps),
                color=(255, 255, 0),
            )
            pipeline.show_image()

            if result is not None and not first_detection_reported:
                try:
                    has_detection = len(result["scores"]) > 0
                except Exception:
                    has_detection = bool(result)
                if has_detection:
                    print("first detection:", result)
                    first_detection_reported = True

            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("用户停止钢球 AnchorBaseDet 摄像头测试")
    except BaseException as error:
        print("钢球 AnchorBaseDet 摄像头测试异常:")
        sys.print_exception(error)
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as error:
                sys.print_exception(error)
        if pipeline is not None:
            try:
                pipeline.destroy()
            except BaseException as error:
                sys.print_exception(error)
        gc.collect()


if __name__ == "__main__":
    run()
