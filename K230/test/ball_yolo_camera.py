"""CanMV K230 钢球 YOLO 摄像头识别与目标框显示测试。"""

import gc
import os
import sys
import time

# CanMV IDE 运行当前脚本时不一定自动加入 SD 卡模块目录。
for module_dir in ("/sdcard/app", "/sdcard"):
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

from libs.PipeLine import PipeLine
from libs.YOLO import YOLOv8
from media.sensor import Sensor


KMODEL_PATH = "/sdcard/models/ball_yolov5n_192.kmodel"  # 当前钢球 YOLOv5n 模型路径。
LABELS = ["ball"]                                       # 单类别钢球标签。
MODEL_INPUT_SIZE = [192, 192]                           # KModel 固定输入尺寸。
RGB888P_SIZE = [320, 180]                               # 摄像头 AI 通道分辨率。
DISPLAY_SIZE = [800, 480]                               # 3.5 英寸 ST7701 屏幕分辨率。
DISPLAY_MODE = "st7701"                                # 当前 LCD 显示接口。
CONFIDENCE_THRESHOLD = 0.40                             # 钢球最低检测置信度。
NMS_THRESHOLD = 0.50                                    # 重叠检测框的 NMS 阈值。
MAX_BOXES_NUM = 4                                       # 单钢球场景仅保留少量候选框。
GC_INTERVAL_FRAMES = 60                                 # 降低垃圾回收对帧周期的干扰。
FPS_REPORT_INTERVAL_MS = 1000                           # 串口帧率打印周期。


def _model_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def run():
    pipeline = None
    yolo = None
    os.exitpoint(os.EXITPOINT_ENABLE)

    try:
        if not _model_exists(KMODEL_PATH):
            raise OSError("kmodel missing: " + KMODEL_PATH)

        # 参照官方 demo-camera.py，显示通道由 PipeLine 直接绑定，
        # Python 主循环只获取 AI 帧、执行 YOLO 和刷新检测框 OSD。
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create(sensor=Sensor(width=1920, height=1080))
        display_size = pipeline.get_display_size()

        # 该 YOLOv5n KModel 使用 anchor-free 转置输出，需复用 YOLOv8 后处理。
        yolo = YOLOv8(
            task_type="detect",
            mode="video",
            kmodel_path=KMODEL_PATH,
            labels=LABELS,
            rgb888p_size=RGB888P_SIZE,
            model_input_size=MODEL_INPUT_SIZE,
            display_size=display_size,
            conf_thresh=CONFIDENCE_THRESHOLD,
            nms_thresh=NMS_THRESHOLD,
            max_boxes_num=MAX_BOXES_NUM,
            debug_mode=0,
        )
        yolo.config_preprocess()

        frame_count = 0
        report_frame_count = 0
        report_start_ms = time.ticks_ms()
        display_fps = 0.0
        first_detection_reported = False

        print("steel-ball YOLO camera test started:", KMODEL_PATH)
        while True:
            os.exitpoint()

            ai_frame = pipeline.get_frame()
            result = yolo.run(ai_frame)

            frame_count += 1
            report_frame_count += 1
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, report_start_ms)
            if elapsed_ms >= FPS_REPORT_INTERVAL_MS:
                display_fps = report_frame_count * 1000.0 / float(elapsed_ms)
                print("YOLO FPS: {:.1f}".format(display_fps))
                report_frame_count = 0
                report_start_ms = now_ms

            # YOLO 绘制函数会先清空 OSD，因此必须在目标框之后绘制帧率。
            yolo.draw_result(result, pipeline.osd_img)
            pipeline.osd_img.draw_string_advanced(
                8,
                8,
                24,
                "FPS {:.1f}".format(display_fps),
                color=(255, 255, 0),
            )
            pipeline.show_image()

            if result and result[0] and not first_detection_reported:
                print("first detection:", result)
                first_detection_reported = True

            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("用户停止钢球 YOLO 摄像头测试")
    except BaseException as error:
        print("钢球 YOLO 摄像头测试异常:")
        sys.print_exception(error)
    finally:
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
        gc.collect()


if __name__ == "__main__":
    run()
