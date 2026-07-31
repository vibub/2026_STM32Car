# K230 红色标记与钢珠位置识别联调

运行 `main.py`。程序使用 640×360 传统视觉通道低频识别红色标记，同时使用 320×180 AI 通道采集画面，并由 AI2D 缩放到 192×192 后运行钢球 YOLOv5n；检测框会转换到 640×360 标定坐标系。Display/OSD 合成画面通过 WBC 硬件 H.264 编码，以 RTSP 图传输出。导轨沿画面水平方向放置。

`main.py`、`ball_position.py`、`vision_pipeline.py` 和 `wifi_ap_rtsp.py` 应放在 SD 卡根目录。钢球模型放到 `/sdcard/models/ball_yolov5n_192.kmodel`，并确保 `/sdcard/libs/YOLO.py` 可用。该模型虽然命名为 YOLOv5n，但输出为 anchor-free 转置布局，运行时使用 `YOLOv8` 类完成匹配的后处理。入口会显式把 `/sdcard` 加入模块搜索路径。

## 红色位置标记

不再识别模糊的印刷数字和蓝色毫米刻线。在导轨上下边缘分别用红色记号笔画 5 组对齐短线：

```text
-10 cm     -5 cm       0 cm       +5 cm      +10 cm
  |          |          |           |           |    上排
====================================================
  |          |          |           |           |    下排
```

每条红线建议宽 3～5 mm、上下方向长 8～15 mm。上下两条必须尽量位于同一 x 坐标，背景不要出现其他红色物体。

当前只画出 `-5/0/+5 cm` 三组也可以工作：程序把画面中央附近的一组自动定义为 0，相邻组固定按 50 mm 间隔换算。以后补上 `±10 cm` 会自动识别为五组，提高边缘位置精度。

钢球遮住一组、画面只检测到两组时也会给出近似位置：若某一组最接近画面中心，就把它当作 0，另一组当作相邻的 ±5 cm；若两组的中点更接近画面中心，就认为 0 被遮住，两组分别为 -5 cm 和 +5 cm。两组模式的标定质量最高显示为 `Q65`，三组及以上仍是推荐状态。

LCD/IDE 叠加层中：

- 青色小框：识别到的上下红线。
- `MARK 2/5`：仅识别到两组标记，正在使用近似标定。
- `MARK 3/5`：识别到三组标记，已满足推荐要求。
- `MARK 5/5`：五组全部有效。
- 绿色框：识别到的圆环或钢珠。
- `BALL POS +xx.x mm`：钢球相对 0 标记的位置；识别无效时显示 `BALL POS --.- mm`。
- `BALL FPS x.x`：每秒实际完成的钢球 YOLO 识别次数，包含摄像头采集、图传和 OSD 对实时性的影响。

少于两组、上下红线无法配对或红线间距明显不合理时，位置结果立即无效，不沿用旧值。

## 识别与位置换算

1. 用红色 LAB 阈值提取短线 blob。
2. 将 x 坐标接近、上下间距合理的两个 blob 配成一组刻度列。
3. 按 x 排序；三组以上选择最靠近画面中心的一列作为 0，相邻列间隔为 50 mm；只有两组时按“中心标记可见”或“中心标记被遮挡”两种情况估算。
4. 根据目标所在高度对每组上下红线的 x 坐标插值，补偿轻微透视和相机歪斜。
5. 钢球模式由 YOLO 输出 `[x, y, width, height]` 框；取框中心并限制在上下红线之间，再在相邻刻度间分段插值位置。
6. 圆环模式仍使用金色 LAB 连通域，方便没有钢球时联调标定和屏幕显示。

## 钢球 YOLO 与圆环模式

默认配置使用 192×192 YOLOv5n 钢球模型：

```python
TARGET_PROFILE = "STEEL_BALL"
STEEL_BALL_MODEL_INPUT_SIZE = [192, 192]
STEEL_BALL_CONFIDENCE_THRESHOLD = 0.40
```

没有钢球、需要用金色圆环联调时改为：

```python
TARGET_PROFILE = "RING"
RING_DIAMETER_MM = 18.0
```

红色标记标定和 UART 协议无需修改。

## 现场阈值标定

- 红线漏检：用 CanMV IDE 阈值编辑器重新标定 `ball_position.py` 的 `SCALE_THRESHOLDS`，只保留红色记号笔。
- 红线粘连：缩小每条线宽，擦除附近其他红色污迹，或减小 `SCALE_MERGE_MARGIN`。
- YOLO 漏检钢球：先确认模型路径和 `/sdcard/libs/YOLO.py`，再把 `STEEL_BALL_CONFIDENCE_THRESHOLD` 从 0.40 逐步降至 0.30；误检时提高阈值。
- 金色圆环漏检：调整 `RING_TARGET_THRESHOLDS`；圆环被分成多块时把 `TARGET_MERGE_MARGIN` 从 5 增大到 6～8。
- 位置整体偏移：调整相机，使 0 标记位于画面中心附近；相机和导轨固定后锁定曝光、白平衡与焦距。

## H.264 RTSP 图传

K230 创建热点 `kk22330`，电脑连接后使用 VLC 打开：

```text
rtsp://192.168.4.1:8554/test
```

图传默认使用 800×480、15 FPS、2048 kbit/s H.264。WBC 和 VEncoder 在后台线程处理显示合成帧，主循环不再执行软件 JPEG 压缩，也不再为图传额外采集 CV 帧。浏览器不能直接播放该 RTSP 地址。

RTSP 和 CanMV IDE 虚拟帧缓冲都依赖唯一的 VO WBC 通道，不能同时开启。RTSP 模式下 `DISPLAY_MODE` 必须为 `"rtsp"`，画面请通过 VLC 查看；需要恢复 IDE 帧缓冲时，应关闭 `VIDEO_AP_ENABLE` 并把 `DISPLAY_MODE` 改回 `"ide"`。

## UART

UART1 使用 IO3 TX、IO4 RX，115200-8N1。每 40 ms 发送：

```text
B,valid,position_d10_mm,quality,sequence\r\n
```

例如 `position_d10_mm=-503` 表示 `-50.3 mm`。任何标记或目标失效时发送 `B,0,0,0,sequence`，下位机不得沿用旧位置。
